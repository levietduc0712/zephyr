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
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/sys_io.h>

#include "led.h"

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

#define HEARTBEAT_ADDR       0x401U
#define HEARTBEAT_REQUEST    "HBQ1"
#define HEARTBEAT_REPLY      "HBR1"
#define HEARTBEAT_MAGIC_SIZE 4U
#define HEARTBEAT_PROBE_SIZE (HEARTBEAT_MAGIC_SIZE + sizeof(uint64_t))
#define HEARTBEAT_TIMEOUT_MS 3000
#define HOST_POLL_MS         50

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

static int heartbeat(struct rpmsg_endpoint *endpoint, void *data, size_t size, uint32_t src,
		     void *priv)
{
	int64_t *last_seen = priv;
	uint8_t reply[HEARTBEAT_PROBE_SIZE + sizeof(uint64_t)];
	int64_t now;
	int ret;

	if (size != HEARTBEAT_PROBE_SIZE ||
	    memcmp(data, HEARTBEAT_REQUEST, HEARTBEAT_MAGIC_SIZE) != 0) {
		return RPMSG_SUCCESS;
	}

	now = k_uptime_get();
	memcpy(reply, HEARTBEAT_REPLY, HEARTBEAT_MAGIC_SIZE);
	memcpy(reply + HEARTBEAT_MAGIC_SIZE, (uint8_t *)data + HEARTBEAT_MAGIC_SIZE,
	       sizeof(uint64_t));
	sys_put_le64((uint64_t)now, reply + HEARTBEAT_PROBE_SIZE);

	ret = rpmsg_trysendto(endpoint, reply, sizeof(reply), src);
	if (ret < 0) {
		printk("RPMsg heartbeat reply failed: %d\n", ret);
	}
	if (*last_seen < 0) {
		if (IS_ENABLED(CONFIG_PWM)) {
			d19_set_heartbeat(true);
		}
		printk("RPMsg heartbeat: A55 online\n");
	}
	*last_seen = now;
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

static bool host_ready(struct virtio_device *vdev)
{
	const uint8_t mask = VIRTIO_CONFIG_STATUS_DRIVER_OK | VIRTIO_CONFIG_STATUS_NEEDS_RESET |
			     VIRTIO_CONFIG_STATUS_FAILED;
	uint8_t status;

	/* Linux clears DRIVER_OK when it shuts down the remoteproc virtio device. */
	return virtio_get_status(vdev, &status) == 0 &&
	       (status & mask) == VIRTIO_CONFIG_STATUS_DRIVER_OK;
}

static void heartbeat_offline(int64_t *last_seen)
{
	*last_seen = -1;
	if (IS_ENABLED(CONFIG_PWM)) {
		d19_set_heartbeat(false);
	}
}

static int run_session(struct virtio_device *vdev, struct metal_io_region *buffer_io,
		       uint32_t rx_notify)
{
	static struct rpmsg_virtio_device rpmsg;
	static struct rpmsg_endpoint endpoint;
	static struct rpmsg_endpoint heartbeat_endpoint;
	struct rpmsg_device *rdev = rpmsg_virtio_get_rpmsg_device(&rpmsg);
	int64_t last_seen = -1;
	int ret;

	ret = rpmsg_init_vdev(&rpmsg, vdev, NULL, buffer_io, NULL);
	if (ret != RPMSG_SUCCESS) {
		printk("RPMsg device initialization failed: %d\n", ret);
		return ret;
	}

	ret = rpmsg_create_ept(&endpoint, rdev, "rpmsg-tty", RPMSG_ADDR_ANY,
			       RPMSG_ADDR_ANY, echo, NULL);
	if (ret != RPMSG_SUCCESS) {
		printk("RPMsg TTY announcement failed: %d\n", ret);
		goto teardown;
	}

	printk("RPMsg TTY echo endpoint ready\n");
	heartbeat_endpoint.priv = &last_seen;
	ret = rpmsg_create_ept(&heartbeat_endpoint, rdev, "rpmsg-tty", HEARTBEAT_ADDR,
			       RPMSG_ADDR_ANY, heartbeat, NULL);
	if (ret == RPMSG_SUCCESS) {
		printk("RPMsg heartbeat endpoint ready at 0x%x\n", HEARTBEAT_ADDR);
	} else {
		printk("RPMsg heartbeat endpoint announcement failed: %d\n", ret);
	}
	ret = 0;

	while (host_ready(vdev)) {
		int64_t wait_ms = HOST_POLL_MS;

		/* OpenAMP calls heartbeat() from this thread. */
		if (last_seen >= 0) {
			int64_t remaining = HEARTBEAT_TIMEOUT_MS - (k_uptime_get() - last_seen);

			if (remaining > 0) {
				wait_ms = MIN(wait_ms, remaining);
			} else {
				heartbeat_offline(&last_seen);
				printk("RPMsg heartbeat: A55 timed out\n");
			}
		}
		if (k_sem_take(&kick_pending, K_MSEC(wait_ms)) == 0 && host_ready(vdev)) {
			rproc_virtio_notified(vdev, rx_notify);
		}
	}

	heartbeat_offline(&last_seen);
	printk("RPMsg host reset, waiting for Linux\n");

teardown:
	/* The host owns the queues and may already have cleared them. */
	rdev->support_ns = false;
	rpmsg_deinit_vdev(&rpmsg);
	return ret;
}

int main(void)
{
	static struct metal_io_region table_io;
	static struct metal_io_region vring_io;
	static struct metal_io_region buffer_io;
	static const metal_phys_addr_t table_phys = TABLE_BASE;
	static const metal_phys_addr_t vring_phys = VRING_BASE;
	static const metal_phys_addr_t buffer_phys = BUFFER_BASE;
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
	printk("RPMsg resource table published, waiting for Linux\n");
	for (;;) {
		vdev = rproc_virtio_create_vdev(VIRTIO_DEV_DEVICE, VDEV_NOTIFY, &table->vdev,
					       &table_io, NULL, notify_linux, NULL);
		if (vdev == NULL) {
			printk("RPMsg virtio allocation failed\n");
			return -ENOMEM;
		}
		while (!host_ready(vdev)) {
			k_msleep(HOST_POLL_MS);
		}

		for (uint32_t i = 0U; i < ARRAY_SIZE(table->vrings); ++i) {
			const struct fw_rsc_vdev_vring *ring = &table->vrings[i];

			if (ring->da != initial_table.vrings[i].da || ring->align != VRING_ALIGN ||
			    ring->num != VRING_COUNT) {
				printk("Unexpected Linux RPMsg vring %u layout\n", i);
				rproc_virtio_remove_vdev(vdev);
				return -EINVAL;
			}
			ret = rproc_virtio_init_vring(vdev, i, ring->notifyid, UINT_TO_POINTER(ring->da),
						    &vring_io, ring->num, ring->align);
			if (ret != 0) {
				printk("RPMsg vring %u setup failed: %d\n", i, ret);
				rproc_virtio_remove_vdev(vdev);
				return ret;
			}
		}

		ret = run_session(vdev, &buffer_io, table->vrings[VRING_RX].notifyid);
		rproc_virtio_remove_vdev(vdev);
		if (ret != 0) {
			return ret;
		}
	}
}
