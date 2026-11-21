/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <metal/io.h>
#include <metal/sys.h>
#include <openamp/open_amp.h>
#include <string.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/sys_io.h>

/* Match the vdev0 carved out by the FRDM-i.MX95 Linux device tree. */
#define VRING_BASE  0x88000000U
#define VRING_SIZE  0x8000U
#define VRING_ALIGN 0x1000U
#define VRING_COUNT 256U
#define BUFFER_BASE 0x88020000U
#define BUFFER_SIZE 0x100000U
#define TABLE_BASE  0x88220000U
#define VDEV_NOTIFY 0xffU
#define VRING_RX    1U

struct rpmsg_table {
	struct resource_table header;
	uint32_t offset;
	struct fw_rsc_vdev vdev;
	struct fw_rsc_vdev_vring vrings[2];
};

static const struct rpmsg_table initial_table = {
	.header = {.ver = 1U, .num = 1U},
	.offset = offsetof(struct rpmsg_table, vdev),
	.vdev = {
		.type = RSC_VDEV,
		.id = VIRTIO_ID_RPMSG,
		.dfeatures = BIT(VIRTIO_RPMSG_F_NS),
		.num_of_vrings = ARRAY_SIZE(initial_table.vrings),
	},
	.vrings = {
		{.da = VRING_BASE, .align = VRING_ALIGN, .num = VRING_COUNT, .notifyid = 0U},
		{.da = VRING_BASE + VRING_SIZE, .align = VRING_ALIGN, .num = VRING_COUNT,
		 .notifyid = 1U},
	},
};

static const struct mbox_dt_spec mbox = {
	.dev = DEVICE_DT_GET(DT_NODELABEL(mu7)),
	.channel_id = 1U,
};
static K_SEM_DEFINE(kick_pending, 0, 1);

static void on_kick(const struct device *dev, mbox_channel_id_t channel, void *user_data,
		    struct mbox_msg *message)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel);
	ARG_UNUSED(user_data);
	ARG_UNUSED(message);

	k_sem_give(&kick_pending);
}

static int notify_linux(void *priv, uint32_t id)
{
	ARG_UNUSED(priv);

	/* The i.MX Linux remoteproc driver encodes the virtqueue ID in bits 31:16. */
	const uint32_t word = id << 16;
	const struct mbox_msg message = {.data = &word, .size = sizeof(word)};

	return mbox_send_dt(&mbox, &message);
}

static int echo(struct rpmsg_endpoint *endpoint, void *data, size_t size, uint32_t src,
		void *priv)
{
	ARG_UNUSED(priv);

	int ret = rpmsg_trysendto(endpoint, data, (int)size, src);

	if (ret < 0) {
		printk("RPMsg echo send failed: %d\n", ret);
	}

	return RPMSG_SUCCESS;
}

static void publish_table(struct rpmsg_table *table)
{
	const size_t version_size = sizeof(table->header.ver);

	/* Publish version 1 after the table entries are visible to Linux. */
	sys_write32(0U, TABLE_BASE);
	barrier_dmem_fence_full();
	memcpy((uint8_t *)table + version_size, (const uint8_t *)&initial_table + version_size,
	       sizeof(*table) - version_size);
	barrier_dmem_fence_full();
	sys_write32(initial_table.header.ver, TABLE_BASE);
	barrier_dmem_fence_full();
}

int main(void)
{
	static struct metal_io_region table_io;
	static struct metal_io_region vring_io;
	static struct metal_io_region buffer_io;
	static const metal_phys_addr_t table_phys = TABLE_BASE;
	static const metal_phys_addr_t vring_phys = VRING_BASE;
	static const metal_phys_addr_t buffer_phys = BUFFER_BASE;
	static struct rpmsg_virtio_device rpmsg;
	static struct rpmsg_endpoint endpoint;
	struct rpmsg_table *table = UINT_TO_POINTER(TABLE_BASE);
	const struct metal_init_params metal_params = METAL_INIT_DEFAULTS;
	struct virtio_device *vdev;
	int ret;

	if (!mbox_is_ready_dt(&mbox)) {
		printk("RPMsg mailbox is unavailable\n");
		return -ENODEV;
	}

	ret = metal_init(&metal_params);
	if (ret != 0) {
		printk("RPMsg metal_init failed: %d\n", ret);
		return ret;
	}

	metal_io_init(&table_io, table, &table_phys, sizeof(*table), -1, 0U, NULL);
	metal_io_init(&vring_io, UINT_TO_POINTER(VRING_BASE), &vring_phys, 2U * VRING_SIZE,
		      -1, 0U, NULL);
	metal_io_init(&buffer_io, UINT_TO_POINTER(BUFFER_BASE), &buffer_phys, BUFFER_SIZE,
		      -1, 0U, NULL);

	ret = mbox_register_callback_dt(&mbox, on_kick, NULL);
	if (ret == 0) {
		ret = mbox_set_enabled_dt(&mbox, true);
	}
	if (ret != 0) {
		printk("RPMsg mailbox setup failed: %d\n", ret);
		return ret;
	}

	publish_table(table);
	vdev = rproc_virtio_create_vdev(VIRTIO_DEV_DEVICE, VDEV_NOTIFY, &table->vdev,
				       &table_io, NULL, notify_linux, NULL);
	if (vdev == NULL) {
		printk("RPMsg virtio allocation failed\n");
		return -ENOMEM;
	}

	printk("RPMsg resource table published, waiting for Linux\n");
	rproc_virtio_wait_remote_ready(vdev);

	for (uint32_t i = 0U; i < ARRAY_SIZE(table->vrings); ++i) {
		const struct fw_rsc_vdev_vring *ring = &table->vrings[i];

		if (ring->da != initial_table.vrings[i].da || ring->align != VRING_ALIGN ||
		    ring->num != VRING_COUNT) {
			printk("Unexpected Linux RPMsg vring %u layout\n", i);
			return -EINVAL;
		}
		ret = rproc_virtio_init_vring(vdev, i, ring->notifyid, UINT_TO_POINTER(ring->da),
					    &vring_io, ring->num, ring->align);
		if (ret != 0) {
			printk("RPMsg vring %u setup failed: %d\n", i, ret);
			return ret;
		}
	}

	ret = rpmsg_init_vdev(&rpmsg, vdev, NULL, &buffer_io, NULL);
	if (ret != RPMSG_SUCCESS) {
		printk("RPMsg device initialization failed: %d\n", ret);
		return ret;
	}

	ret = rpmsg_create_ept(&endpoint, rpmsg_virtio_get_rpmsg_device(&rpmsg), "rpmsg-tty",
			       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY, echo, NULL);
	if (ret != RPMSG_SUCCESS) {
		printk("RPMsg TTY announcement failed: %d\n", ret);
		return ret;
	}

	printk("RPMsg TTY echo endpoint ready\n");
	for (;;) {
		k_sem_take(&kick_pending, K_FOREVER);
		rproc_virtio_notified(vdev, table->vrings[VRING_RX].notifyid);
	}
}
