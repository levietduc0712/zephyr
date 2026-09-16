/*
 * Copyright (c) 2024 EXALT Technologies.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#ifdef CONFIG_I3C_STM32_DMA
#include <zephyr/drivers/dma/dma_stm32.h>
#include <zephyr/drivers/dma.h>
#endif
#include <zephyr/pm/policy.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/logging/log.h>

#include <stm32_bitops.h>
#include <stm32_ll_i3c.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_system.h>
#include <stm32_ll_cortex.h>
#include <math.h>

LOG_MODULE_REGISTER(i3c_stm32, CONFIG_I3C_LOG_LEVEL);

#define DT_DRV_COMPAT st_stm32_i3c

#ifdef CONFIG_STM32_HAL2
#define STM32_I3C_RXFIFO_THRESHOLD_1_BYTE	LL_I3C_RXFIFO_THRESHOLD_1_8
#define STM32_I3C_TXFIFO_THRESHOLD_1_BYTE	LL_I3C_TXFIFO_THRESHOLD_1_8
#else /* CONFIG_STM32_HAL2 */
#define STM32_I3C_RXFIFO_THRESHOLD_1_BYTE	LL_I3C_RXFIFO_THRESHOLD_1_4
#define STM32_I3C_TXFIFO_THRESHOLD_1_BYTE	LL_I3C_TXFIFO_THRESHOLD_1_4
#endif /* CONFIG_STM32_HAL2*/

#define STM32_I3C_SCLH_I2C_MIN_FM_NS  600ull
#define STM32_I3C_SCLH_I2C_MIN_FMP_NS 260ull
#define STM32_I3C_SCLL_OD_MIN_FM_NS   1320ull
#define STM32_I3C_SCLL_OD_MIN_FMP_NS  500ull
#define STM32_I3C_SCLL_OD_MIN_I3C_NS  200ull

#define STM32_I3C_SCLL_PP_MIN_NS  32ull
#define STM32_I3C_SCLH_I3C_MIN_NS 32ull

#define STM32_I3C_TBUF_FMP_MIN_NS 500ULL
#define STM32_I3C_TBUF_FM_MIN_NS  1300ULL
#define STM32_I3C_TCAS_MIN_NS     39ULL

#define STM32_I3C_TRANSFER_TIMEOUT K_MSEC(100)

#ifdef CONFIG_I3C_STM32_DMA
K_HEAP_DEFINE(stm32_i3c_fifo_heap, CONFIG_I3C_STM32_DMA_FIFO_HEAP_SIZE);
#endif

typedef void (*irq_config_func_t)(const struct device *port);

enum i3c_stm32_sf_state {
	STM32_I3C_SF_DAA,    /* Dynamic addressing state */
	STM32_I3C_SF_CCC,    /* First part of CCC command state*/
	STM32_I3C_SF_CCC_P2, /* Second part of CCC command state (used for direct commands)*/
	STM32_I3C_SF,        /* Private msg state */
	STM32_I2C_SF,        /* I2C legacy msg state */
	STM32_I3C_SF_IDLE,   /* Idle bus state */
	STM32_I3C_SF_ERR,    /* Error state */
	STM32_I3C_SF_INVAL,  /* Invalid state */
};

enum i3c_stm32_msg_state {
	STM32_I3C_MSG_DAA,    /* Dynamic addressing state */
	STM32_I3C_MSG_CCC,    /* First part of CCC command state*/
	STM32_I3C_MSG_CCC_P2, /* Second part of CCC command state (used for direct commands)*/
	STM32_I3C_MSG,        /* Private msg state */
	STM32_I3C_MSG_IDLE,   /* Idle bus state */
	STM32_I3C_MSG_ERR,    /* Error state */
	STM32_I3C_MSG_INVAL,  /* Invalid state */
};

#ifdef CONFIG_I3C_STM32_DMA
struct i3c_stm32_dma_stream {
	const struct device *dma_dev;
	uint32_t dma_channel;
	struct dma_config dma_cfg;
	uint8_t priority;
	bool src_addr_increment;
	bool dst_addr_increment;
	int fifo_threshold;
	struct dma_block_config blk_cfg;
};
#endif

/* Struct to hold the information about the current message on the bus */
struct i3c_stm32_msg {
	uint8_t target_addr;         /* Current target xfer address */
	struct i3c_msg *i3c_msg_ptr; /* Pointer to the current private message to send on the bus */
	struct i3c_msg *i3c_msg_ctrl_ptr; /* Pointer to the private message that will be used by the
					   * control FIFO
					   */
	struct i3c_msg *i3c_msg_status_ptr; /* Pointer to the private message that will be used by
					     * the status FIFO
					     */
	struct i2c_msg *i2c_msg_ptr; /* Pointer to the current legacy message to send on the bus */
	struct i2c_msg *i2c_msg_ctrl_ptr; /* Pointer to the I2C legavy message that will be used by
					   * the control FIFO
					   */
	size_t num_msgs;                  /* Number of messages */
	size_t ctrl_msg_idx;              /* Current control message index */
	size_t status_msg_idx;            /* Current status message index */
	size_t xfer_msg_idx;              /* Current transfer message index */
	size_t xfer_offset;               /* Current message transfer offset */
	uint32_t msg_type;                /* Either LL_I3C_CONTROLLER_MTYPE_PRIVATE or
					   * LL_I3C_CONTROLLER_MTYPE_LEGACY_I2C
					   */
};

struct i3c_stm32_config {
	struct i3c_driver_config drv_cfg;      /* I3C driver config */
	I3C_TypeDef *i3c;                      /* Pointer to I3C module base addr */
	irq_config_func_t irq_config_func;     /* IRQ config function */
	const struct stm32_pclken *pclken;     /* Pointer to peripheral clock configuration */
	size_t pclk_len;
	const struct pinctrl_dev_config *pcfg; /* Pointer to pin control configuration */
	struct reset_dt_spec reset;
#ifdef CONFIG_I3C_TARGET
	bool target_mode;
	uint8_t mipi_instance;
	uint8_t dcr;
	uint16_t mrl;
	uint16_t mwl;
#endif
};

struct i3c_stm32_data {
	struct i3c_driver_data drv_data;     /* I3C driver data */
	enum i3c_stm32_msg_state msg_state;  /* Current I3C bus state */
	enum i3c_stm32_sf_state sf_state;    /* Current I3C status FIFO state */
	struct i3c_ccc_payload *ccc_payload; /* Current CCC message payload */
	struct i3c_ccc_target_payload *
		ccc_target_payload; /* Current target addressed by 2nd part of direct CCC command */
	struct i3c_ccc_target_payload
		*ccc_target_payload_sf; /* Current target addressed
		* by 2nd part of direct CCC command used by the
		status FIFO
		*/
	size_t ccc_target_idx;        /* Current target index, used for filling C-FIFO */
	struct k_sem device_sync_sem; /* Sync between device communication messages */
	bool xfer_active;
	int xfer_result;
	struct k_mutex bus_mutex;     /* Sync between transfers */
	struct i3c_stm32_msg curr_msg;
	uint8_t target_addr;    /* Current target xfer address */
	uint8_t num_msgs;       /* Number of messages to send on bus */
#ifdef CONFIG_I3C_STM32_DMA
	struct i3c_stm32_dma_stream dma_rx; /* RX DMA channel config */
	struct i3c_stm32_dma_stream dma_tx; /* TX DMA channel config */
	struct i3c_stm32_dma_stream dma_tc; /* Control FIFO DMA channel config */
	struct i3c_stm32_dma_stream dma_rs; /* Status FIFO DMA channel config */
	uint32_t *status_fifo;  /* Pointer to the allocated region for status FIFO words */
	uint32_t *control_fifo; /* Pointer to the allocated region for control FIFO words */
	size_t fifo_len;        /* The size in bytes for the allocated region for each FIFO */
#endif
	uint64_t pid;      /* Current DAA target PID */
	size_t daa_rx_rcv; /* Number of RX bytes received during DAA */
	uint8_t target_id; /* Target id */
#ifdef CONFIG_I3C_USE_IBI
	uint32_t ibi_payload;      /* Received ibi payload */
	uint32_t ibi_payload_size; /* Received payload size */
	uint32_t ibi_target_addr;  /* Received target dynamic address */
	struct {
		uint8_t addr[4];  /* List of target addresses */
		uint8_t num_addr; /* Number of valid addresses */
	} ibi;
	bool hj_pm_lock;           /* Used as flag for setting pm */
#endif

#ifdef CONFIG_I3C_TARGET
	struct i3c_target_config *target_config;
#endif /*CONFIG_I3C_TARGET*/
};

static bool ll_i3c_is_in_controller_mode(I3C_TypeDef *i3c)
{
	/* LL_I3C_GetMode() does not return the same values depending on SoCs but always
	 * return 0 when in target mode and a non-0 value when in controller mode.
	 */
	return LL_I3C_GetMode(i3c) != 0;
}

#ifdef CONFIG_I3C_CONTROLLER
static int get_i3c_lvr_ic_mode(const struct i3c_dev_list *dev_list)
{
	for (int i = 0; i < dev_list->num_i2c; i++) {
		if (I3C_LVR_I2C_DEV_IDX(dev_list->i2c[i].lvr) == I3C_LVR_I2C_DEV_IDX_0) {
			if (I3C_LVR_I2C_MODE(dev_list->i2c[i].lvr) == I3C_LVR_I2C_FM_MODE) {
				return I3C_LVR_I2C_FM_MODE;
			}
		}
	}
	return I3C_LVR_I2C_FM_PLUS_MODE;
}

static bool i3c_stm32_curr_msg_is_i3c(const struct device *dev)
{
	struct i3c_stm32_data *data = dev->data;
	struct i3c_stm32_msg *curr_msg = &data->curr_msg;

	return (curr_msg->msg_type == LL_I3C_CONTROLLER_MTYPE_PRIVATE);
}

static void i3c_stm32_arbitration_header_config(const struct device *dev)
{
	struct i3c_stm32_data *data = dev->data;
	struct i3c_stm32_msg *curr_msg = &data->curr_msg;
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;

	if (i3c_stm32_curr_msg_is_i3c(dev)) {
		if (curr_msg->i3c_msg_ctrl_ptr->flags & I3C_MSG_NBCH) {
			/* Disable arbitration header for this transaction */
			LL_I3C_DisableArbitrationHeader(i3c);
		} else {
			/* Enable arbitration header for this transaction */
			LL_I3C_EnableArbitrationHeader(i3c);
		}
	}
}

static int i3c_stm32_curr_msg_init(const struct device *dev, struct i3c_msg *i3c_msgs,
				   struct i2c_msg *i2c_msgs, uint8_t num_msgs, uint8_t tgt_addr)
{
	struct i3c_stm32_data *data = dev->data;
	struct i3c_stm32_msg *curr_msg = &data->curr_msg;

	/* Either should be NULL */
	__ASSERT(!(i3c_msgs == NULL && i2c_msgs == NULL), "Both i3c_msgs and i2c_msgs are NULL");
	__ASSERT(!(i3c_msgs != NULL && i2c_msgs != NULL),
		 "Both i3c_msgs and i2c_msgs are not NULL");

	curr_msg->target_addr = tgt_addr;
	curr_msg->xfer_offset = 0;
	curr_msg->num_msgs = num_msgs;
	curr_msg->ctrl_msg_idx = 0;
	curr_msg->status_msg_idx = 0;
	curr_msg->xfer_msg_idx = 0;

	/* I3C private message */
	if (i2c_msgs == NULL) {
		curr_msg->msg_type = LL_I3C_CONTROLLER_MTYPE_PRIVATE;
		curr_msg->i3c_msg_ptr = i3c_msgs;
		curr_msg->i3c_msg_ctrl_ptr = i3c_msgs;
		curr_msg->i3c_msg_status_ptr = i3c_msgs;
	} else {
		/* Legacy I2C message */
		curr_msg->msg_type = LL_I3C_CONTROLLER_MTYPE_LEGACY_I2C;
		curr_msg->i2c_msg_ptr = i2c_msgs;
		curr_msg->i2c_msg_ctrl_ptr = i2c_msgs;
	}

	i3c_stm32_arbitration_header_config(dev);
	return 0;
}

static int i3c_stm32_curr_msg_control_get_dir(const struct device *dev)
{
	struct i3c_stm32_data *data = dev->data;
	struct i3c_stm32_msg *curr_msg = &data->curr_msg;

	if (i3c_stm32_curr_msg_is_i3c(dev)) {
		return (((curr_msg->i3c_msg_ctrl_ptr->flags & I3C_MSG_RW_MASK) == I3C_MSG_READ)
				? LL_I3C_DIRECTION_READ
				: LL_I3C_DIRECTION_WRITE);
	}

	return (((curr_msg->i2c_msg_ctrl_ptr->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ)
			? LL_I3C_DIRECTION_READ
			: LL_I3C_DIRECTION_WRITE);
}

static int i3c_stm32_curr_msg_control_get_len(const struct device *dev)
{
	struct i3c_stm32_data *data = dev->data;
	struct i3c_stm32_msg *curr_msg = &data->curr_msg;

	return (i3c_stm32_curr_msg_is_i3c(dev)) ? curr_msg->i3c_msg_ctrl_ptr->len
						: curr_msg->i2c_msg_ctrl_ptr->len;
}

static int i3c_stm32_curr_msg_control_get_end(const struct device *dev)
{
	struct i3c_stm32_data *data = dev->data;
	struct i3c_stm32_msg *curr_msg = &data->curr_msg;

	return ((curr_msg->ctrl_msg_idx < (curr_msg->num_msgs - 1)) ? LL_I3C_GENERATE_RESTART
								    : LL_I3C_GENERATE_STOP);
}

static int i3c_stm32_curr_msg_control_next(const struct device *dev)
{
	struct i3c_stm32_data *data = dev->data;
	struct i3c_stm32_msg *curr_msg = &data->curr_msg;

	if (curr_msg->ctrl_msg_idx >= curr_msg->num_msgs) {
		LOG_ERR("No more messages left");
		return -EFAULT;
	}

	if (i3c_stm32_curr_msg_is_i3c(dev)) {
		curr_msg->i3c_msg_ctrl_ptr++;
	} else {
		curr_msg->i2c_msg_ctrl_ptr++;
	}

	curr_msg->ctrl_msg_idx++;

	return 0;
}

static int i3c_stm32_curr_msg_status_update_num_xfer(const struct device *dev, size_t num_xfer)
{
	struct i3c_stm32_data *data = dev->data;
	struct i3c_stm32_msg *curr_msg = &data->curr_msg;

	if (curr_msg->status_msg_idx >= curr_msg->num_msgs) {
		LOG_ERR("No more messages left");
		return -EFAULT;
	}

	/* Legacy I2C messages do not have num_xfer */
	if (i3c_stm32_curr_msg_is_i3c(dev)) {
		curr_msg->i3c_msg_status_ptr->num_xfer = num_xfer;
	}

	return 0;
}

static int i3c_stm32_curr_msg_status_next(const struct device *dev)
{
	struct i3c_stm32_data *data = dev->data;
	struct i3c_stm32_msg *curr_msg = &data->curr_msg;

	if (curr_msg->status_msg_idx >= curr_msg->num_msgs) {
		LOG_ERR("No more messages left");
		return -EFAULT;
	}

	if (i3c_stm32_curr_msg_is_i3c(dev)) {
		curr_msg->i3c_msg_status_ptr++;
	}
	curr_msg->status_msg_idx++;

	return 0;
}

static int i3c_stm32_curr_msg_xfer_get_buf(const struct device *dev, uint8_t **buf, uint32_t *len,
					   size_t **offset)
{
	struct i3c_stm32_data *data = dev->data;
	struct i3c_stm32_msg *curr_msg = &data->curr_msg;

	if (curr_msg->xfer_msg_idx >= curr_msg->num_msgs) {
		LOG_ERR("No more messages left");
		return -EFAULT;
	}

	if (i3c_stm32_curr_msg_is_i3c(dev)) {
		*buf = curr_msg->i3c_msg_ptr->buf;
		*len = curr_msg->i3c_msg_ptr->len;
	} else {
		*buf = curr_msg->i2c_msg_ptr->buf;
		*len = curr_msg->i2c_msg_ptr->len;
	}

	*offset = &curr_msg->xfer_offset;

	return 0;
}

static bool i3c_stm32_curr_msg_xfer_is_read(const struct device *dev)
{
	struct i3c_stm32_data *data = dev->data;
	struct i3c_stm32_msg *curr_msg = &data->curr_msg;

	if (curr_msg->xfer_msg_idx >= curr_msg->num_msgs) {
		LOG_ERR("No more messages left");
		return false;
	}

	if (i3c_stm32_curr_msg_is_i3c(dev)) {
		return ((curr_msg->i3c_msg_ptr->flags & I3C_MSG_RW_MASK) == I3C_MSG_READ);
	}

	return ((curr_msg->i2c_msg_ptr->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ);
}

static int i3c_stm32_curr_msg_xfer_next(const struct device *dev)
{
	struct i3c_stm32_data *data = dev->data;
	struct i3c_stm32_msg *curr_msg = &data->curr_msg;

	if (curr_msg->xfer_msg_idx >= curr_msg->num_msgs) {
		LOG_ERR("No more messages left");
		return -EFAULT;
	}

	if (i3c_stm32_curr_msg_is_i3c(dev)) {
		curr_msg->i3c_msg_ptr++;
	} else {
		curr_msg->i2c_msg_ptr++;
	}

	curr_msg->xfer_msg_idx++;
	curr_msg->xfer_offset = 0;

	return 0;
}
#endif /*CONFIG_I3C_CONTROLLER*/

/* Activates the device I3C pinctrl and CLK */
static int i3c_stm32_activate(const struct device *dev)
{
	int ret;
	struct i3c_stm32_config *config = (struct i3c_stm32_config *)dev->config;
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	if (clock_control_on(clk, (clock_control_subsys_t)&config->pclken[0]) != 0) {
		return -EIO;
	}

	if (config->pclk_len > 1) {
		/* Enable I3C clock source */
		ret = clock_control_configure(clk, (clock_control_subsys_t)&config->pclken[1],
					      NULL);
		if (ret < 0) {
			return -EIO;
		}
	}

	return 0;
}

#ifdef CONFIG_I3C_CONTROLLER
static int i3c_stm32_calc_scll_od_sclh_i2c(const struct device *dev, uint32_t i2c_bus_freq,
					   uint32_t i3c_clock, uint8_t *scll_od, uint8_t *sclh_i2c)
{
	const struct i3c_stm32_config *config = dev->config;
	struct i3c_stm32_data *data = dev->data;
	uint64_t min_low = MAX(data->drv_data.ctrl_config.scl_od_min.low_ns,
			       STM32_I3C_SCLL_OD_MIN_I3C_NS);
	uint64_t min_high = 0U;
	uint32_t low;
	uint32_t high = 0U;

	if (i2c_bus_freq == 0U && config->drv_cfg.dev_list.num_i2c > 0U) {
		if (i3c_bus_mode(&config->drv_cfg.dev_list) != I3C_BUS_MODE_MIXED_FAST) {
			return -EINVAL;
		}
		i2c_bus_freq = get_i3c_lvr_ic_mode(&config->drv_cfg.dev_list) ==
			      I3C_LVR_I2C_FM_MODE ? 400000U : 1000000U;
	}

	if (i2c_bus_freq > 1000000U) {
		return -EINVAL;
	}
	if (i2c_bus_freq != 0U) {
		if (i2c_bus_freq > 400000U) {
			min_low = MAX(min_low, STM32_I3C_SCLL_OD_MIN_FMP_NS);
			min_high = STM32_I3C_SCLH_I2C_MIN_FMP_NS;
		} else {
			min_low = MAX(min_low, STM32_I3C_SCLL_OD_MIN_FM_NS);
			min_high = STM32_I3C_SCLH_I2C_MIN_FM_NS;
		}
	}

	low = DIV_ROUND_UP(min_low * i3c_clock, 1000000000ULL);
	if (i2c_bus_freq != 0U) {
		uint32_t period = DIV_ROUND_UP(i3c_clock, i2c_bus_freq);

		high = DIV_ROUND_UP(min_high * i3c_clock, 1000000000ULL);
		if (period > low) {
			high = MAX(high, period - low);
		}
	}
	if (low == 0U || low > 256U || high > 256U) {
		return -EINVAL;
	}

	*scll_od = low - 1U;
	*sclh_i2c = high == 0U ? 0U : high - 1U;
	LOG_DBG("TimingReg0: SCLL_OD = %d, SCLH_I2C = %d", *scll_od, *sclh_i2c);
	return 0;
}

static int i3c_stm32_calc_scll_pp_sclh_i3c(uint32_t i3c_bus_freq, uint32_t i3c_clock,
					   uint32_t min_high_ns, uint8_t *scll_pp,
					   uint8_t *sclh_i3c)
{
	uint32_t high = DIV_ROUND_UP(MAX((uint64_t)min_high_ns, STM32_I3C_SCLH_I3C_MIN_NS) *
				   i3c_clock, 1000000000ull);
	uint32_t low = DIV_ROUND_UP(STM32_I3C_SCLL_PP_MIN_NS * i3c_clock, 1000000000ull);
	uint32_t period = DIV_ROUND_UP(i3c_clock, i3c_bus_freq);

	/* SCLH_I3C is shared by OD and PP. Slow down to meet an OD constraint. */
	if (period > high) {
		low = MAX(low, period - high);
	}
	if (high == 0U || high > 256U || low == 0U || low > 256U) {
		return -EINVAL;
	}
	*sclh_i3c = high - 1U;
	*scll_pp = low - 1U;

	LOG_DBG("TimingReg0: SCLL_PP = %d, SCLH_I3C = %d", *scll_pp, *sclh_i3c);
	return 0;
}

static int i3c_stm32_config_clk_wave(const struct device *dev)
{
	const struct i3c_stm32_config *cfg = dev->config;
	struct i3c_stm32_data *data = dev->data;
	const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	I3C_TypeDef *i3c = cfg->i3c;
	uint32_t i3c_clock = 0;
	uint32_t i2c_bus_freq = data->drv_data.ctrl_config.scl.i2c;
	uint32_t i3c_bus_freq = data->drv_data.ctrl_config.scl.i3c;
	/* Set kern_clk_idx = 1, if independent kernel clock is used */
	uint32_t kern_clk_idx = (cfg->pclk_len > 1) ? 1 : 0;
	int ret;

	ret = clock_control_get_rate(clk, (clock_control_subsys_t)&cfg->pclken[kern_clk_idx],
				     &i3c_clock);
	if (ret < 0) {
		LOG_ERR("Failed call clock_control_get_rate(pclken[%d])", kern_clk_idx);
		return -EIO;
	}

	uint8_t scll_od = 0;
	uint8_t sclh_i2c = 0;
	uint8_t scll_pp = 0;
	uint8_t sclh_i3c = 0;
	uint32_t clk_wave = 0;

	LOG_DBG("I3C Clock = %u, I2C Bus Freq = %u, I3C Bus Freq = %u", i3c_clock, i2c_bus_freq,
		i3c_bus_freq);

	ret = i3c_stm32_calc_scll_od_sclh_i2c(dev, i2c_bus_freq, i3c_clock, &scll_od, &sclh_i2c);
	if (ret != 0) {
		LOG_ERR("Cannot calculate the timing for TimingReg0, err=%d", ret);
		return ret;
	}

	ret = i3c_stm32_calc_scll_pp_sclh_i3c(i3c_bus_freq, i3c_clock,
					   data->drv_data.ctrl_config.scl_od_min.high_ns,
					   &scll_pp, &sclh_i3c);
	if (ret != 0) {
		LOG_ERR("Cannot calculate the timing for TimingReg0, err=%d", ret);
		return ret;
	}

	clk_wave = ((uint32_t)sclh_i2c << 24) | ((uint32_t)scll_od << 16) |
		   ((uint32_t)sclh_i3c << 8) | (scll_pp);

	LOG_DBG("TimigReg0 = 0x%08x", clk_wave);

	LL_I3C_ConfigClockWaveForm(i3c, clk_wave);

	return 0;
}
#endif /*CONFIG_I3C_CONTROLLER*/

/**
 * @brief Get current configuration of the I3C hardware.
 *
 * @param[in] dev Pointer to controller device driver instance.
 * @param[in] type Type of configuration.
 * @param[in,out] config Pointer to the configuration parameters.
 *
 * @retval 0 If successful.
 * @retval -EIO General Input/Output errors.
 * @retval -ENOSYS If not implemented.
 */
static int i3c_stm32_config_get(const struct device *dev, enum i3c_config_type type, void *config)
{
	if (config == NULL) {
		return -EINVAL;
	}

	if (type == I3C_CONFIG_CONTROLLER) {
#ifdef CONFIG_I3C_CONTROLLER
		struct i3c_stm32_data *data = dev->data;

		(void)memcpy(config, &data->drv_data.ctrl_config,
			sizeof(data->drv_data.ctrl_config));
#else
		return -ENOTSUP;
#endif
	}

	if (type == I3C_CONFIG_TARGET) {
#ifdef CONFIG_I3C_TARGET
		const struct i3c_stm32_config *cfg = dev->config;
		I3C_TypeDef *i3c = cfg->i3c;
		struct i3c_config_target *config_target = config;

		(void)memset(config_target, 0, sizeof(*config_target));
		config_target->enabled = !ll_i3c_is_in_controller_mode(i3c);
		config_target->dynamic_addr =
			config_target->enabled ? LL_I3C_GetOwnDynamicAddress(i3c) : 0U;
		config_target->pid_random = (LL_I3C_GetIDTypeSelector(i3c) != 0U);
		config_target->pid = ((uint64_t)LL_I3C_GetMIPIManufacturerID(i3c) << 33);
		if (!config_target->pid_random) {
			config_target->pid |= ((uint64_t)LL_I3C_GetMIPIInstanceID(i3c) << 12);
		}
		config_target->bcr = (uint8_t)i3c->BCR;
		config_target->dcr = (uint8_t)LL_I3C_GetDeviceCharacteristics(i3c);
		config_target->max_read_len = (uint16_t)LL_I3C_GetMaxReadLength(i3c);
		config_target->max_write_len = (uint16_t)LL_I3C_GetMaxWriteLength(i3c);
#else
		return -ENOTSUP;
#endif
	}

	return 0;
}

#ifdef CONFIG_I3C_CONTROLLER
static uint32_t i3c_stm32_calc_free_timing(uint64_t min_ns, uint32_t i3c_clock)
{
	return DIV_ROUND_UP(min_ns * i3c_clock, 2000000000ULL);
}

static int i3c_stm32_configure_free_timing(const struct device *dev, uint32_t i3c_clock)
{
	const struct i3c_stm32_config *config = dev->config;
	uint32_t free_timing = 0;
	struct i3c_stm32_data *data = dev->data;
	uint32_t i2c_bus_freq = data->drv_data.ctrl_config.scl.i2c;
	I3C_TypeDef *i3c = config->i3c;

	if (i2c_bus_freq != 0) {
		if (i2c_bus_freq > 400000) {
			/* Mixed bus with I2C FM+ device */
			free_timing = i3c_stm32_calc_free_timing(STM32_I3C_TBUF_FMP_MIN_NS,
				i3c_clock);
		} else {
			/* Mixed bus with I2C FM device */
			free_timing = i3c_stm32_calc_free_timing(STM32_I3C_TBUF_FM_MIN_NS,
				i3c_clock);
		}
	} else {
		if (config->drv_cfg.dev_list.num_i2c > 0) {
			enum i3c_bus_mode mode = i3c_bus_mode(&config->drv_cfg.dev_list);

			if (mode == I3C_BUS_MODE_MIXED_FAST) {
				if (get_i3c_lvr_ic_mode(&config->drv_cfg.dev_list) ==
				    I3C_LVR_I2C_FM_MODE) {
					/* Mixed bus with I2C FM device */
					free_timing = i3c_stm32_calc_free_timing(
						STM32_I3C_TBUF_FM_MIN_NS,
						i3c_clock);
				} else {
					/* Mixed bus with I2C FM+ device */
					free_timing = i3c_stm32_calc_free_timing(
						STM32_I3C_TBUF_FMP_MIN_NS,
						i3c_clock);
				}
			} else {
				return -EINVAL;
			}
		} else {
			/* Pure I3C bus */
			free_timing = i3c_stm32_calc_free_timing(STM32_I3C_TCAS_MIN_NS, i3c_clock);
		}
	}
	if (free_timing > (I3C_TIMINGR1_FREE >> I3C_TIMINGR1_FREE_Pos)) {
		return -EINVAL;
	}
	LL_I3C_SetFreeTiming(i3c, free_timing);
	LL_I3C_SetDataHoldTime(i3c, LL_I3C_SDA_HOLD_TIME_1_5);
	return 0;
}
#endif /* CONFIG_I3C_CONTROLLER */

static int i3c_stm32_config_ctrl_bus_char(const struct device *dev, enum i3c_config_type type)
{
	const struct i3c_stm32_config *config = dev->config;
	const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	I3C_TypeDef *i3c = config->i3c;
	uint32_t i3c_clock = 0;
	uint32_t aval;
	uint32_t kern_clk_idx = (config->pclk_len > 1) ? 1 : 0;

	if (clock_control_get_rate(clk, (clock_control_subsys_t)&config->pclken[kern_clk_idx],
				   &i3c_clock) < 0) {
		LOG_ERR("Failed call clock_control_get_rate(pclken[%u])", kern_clk_idx);
		return -EIO;
	}
	aval = DIV_ROUND_UP(1000ULL * i3c_clock, 1000000000ULL);
	if (aval == 0U || aval > 256U) {
		return -EINVAL;
	}

	/* Satisfying I3C start timing min timing will satisfy the rest of the conditions */
#ifdef CONFIG_I3C_CONTROLLER
	if (type == I3C_CONFIG_CONTROLLER) {
		int ret = i3c_stm32_configure_free_timing(dev, i3c_clock);

		if (ret != 0) {
			return ret;
		}
	}
#endif /*CONFIG_I3C_CONTROLLER*/

	LL_I3C_SetAvalTiming(i3c, aval - 1U);

	LOG_DBG("TimingReg1 = 0x%08x", LL_I3C_GetCtrlBusCharacteristic(i3c));

	return 0;
}

/* Configures the I3C module in controller mode */
static int i3c_stm32_configure(const struct device *dev, enum i3c_config_type type, void *cfg)
{
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;
	struct i3c_stm32_data *data = dev->data;
	struct i3c_config_controller previous;
	uint32_t timing0;
	uint32_t timing1;
	bool was_enabled;
	int ret;
	int pm_ret;

	if (cfg == NULL) {
		return -EINVAL;
	}
	if (type == I3C_CONFIG_CUSTOM) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->bus_mutex, K_FOREVER);
	previous = data->drv_data.ctrl_config;

	if (type == I3C_CONFIG_CONTROLLER) {
		struct i3c_config_controller *ctrl_cfg = cfg;

		if (ctrl_cfg->scl.i3c == 0U || ctrl_cfg->scl.i3c > 12500000U) {
			ret = -EINVAL;
			goto unlock;
		}
		data->drv_data.ctrl_config.scl.i3c = ctrl_cfg->scl.i3c;
		data->drv_data.ctrl_config.scl.i2c = ctrl_cfg->scl.i2c;
		data->drv_data.ctrl_config.scl_od_min = ctrl_cfg->scl_od_min;
	}

	ret = pm_device_runtime_get(dev);
	if (ret < 0) {
		goto unlock;
	}
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	ret = i3c_stm32_activate(dev);
	if (ret != 0) {
		LOG_ERR("Clock and GPIO could not be initialized for the I3C module, err=%d", ret);
		goto put;
	}

	/* Timing registers are writable only while the peripheral is disabled. */
	was_enabled = LL_I3C_IsEnabled(i3c);
	LL_I3C_Disable(i3c);
	timing0 = LL_I3C_GetClockWaveForm(i3c);
	timing1 = LL_I3C_GetCtrlBusCharacteristic(i3c);

#ifdef CONFIG_I3C_CONTROLLER
	if (type == I3C_CONFIG_CONTROLLER) {
		ret = i3c_stm32_config_clk_wave(dev);
		if (ret != 0) {
			LOG_ERR("TimigReg0 timing could not be calculated, err=%d", ret);
			goto restore;
		}
	}
#endif /*CONFIG_I3C_CONTROLLER*/

	ret = i3c_stm32_config_ctrl_bus_char(dev, type);
	if (ret != 0) {
		LOG_ERR("TimingReg1 timing could not be calculated, err=%d", ret);
		goto restore;
	}

#ifdef CONFIG_I3C_TARGET
	if (type == I3C_CONFIG_TARGET) {
		struct i3c_config_target *targ_cfg = cfg;
		const struct i3c_stm32_config *config = dev->config;
		I3C_TypeDef *i3c = config->i3c;

		if (!was_enabled) {
			LL_I3C_SetMIPIInstanceID(i3c, (targ_cfg->pid >> 12U) & 0xFU);
			LL_I3C_SetDeviceCharacteristics(i3c, targ_cfg->dcr);
		}
		LL_I3C_SetMaxReadLength(i3c, targ_cfg->max_read_len);
		LL_I3C_SetMaxWriteLength(i3c, targ_cfg->max_write_len);
	}
#endif

restore:
	if (ret != 0) {
		LL_I3C_ConfigClockWaveForm(i3c, timing0);
		LL_I3C_SetCtrlBusCharacteristic(i3c, timing1);
	}
	if (was_enabled) {
		LL_I3C_Enable(i3c);
	}
put:
	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	pm_ret = pm_device_runtime_put(dev);
	if (ret == 0) {
		ret = pm_ret;
	}
unlock:
	if (ret != 0) {
		data->drv_data.ctrl_config = previous;
	}
	k_mutex_unlock(&data->bus_mutex);
	return ret;
}

#ifdef CONFIG_I3C_CONTROLLER
static int i3c_stm32_i2c_configure(const struct device *dev, uint32_t config)
{
	struct i3c_stm32_data *data = dev->data;
	struct i3c_config_controller *ctrl_config = &data->drv_data.ctrl_config;

	switch (I2C_SPEED_GET(config)) {
	case I2C_SPEED_FAST:
		ctrl_config->scl.i2c = 400000;
		break;
	case I2C_SPEED_FAST_PLUS:
		ctrl_config->scl.i2c = 1000000;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * @brief Find a registered I3C target device.
 *
 * This returns the I3C device descriptor of the I3C device
 * matching the incoming @p id.
 *
 * @param dev Pointer to controller device driver instance.
 * @param id Pointer to I3C device ID.
 *
 * @return @see i3c_device_find.
 */
static struct i3c_device_desc *i3c_stm32_device_find(const struct device *dev,
						     const struct i3c_device_id *id)
{
	const struct i3c_stm32_config *config = dev->config;

	return i3c_dev_list_find(&config->drv_cfg.dev_list, id);
}

#ifdef CONFIG_I3C_STM32_DMA

static void i3c_stm32_end_dma_requests(const struct device *dev)
{
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;

	LL_I3C_EnableIT_TXFNF(i3c);
	LL_I3C_EnableIT_RXFNE(i3c);
	LL_I3C_EnableIT_CFNF(i3c);
	LL_I3C_EnableIT_SFNE(i3c);

	LL_I3C_DisableDMAReq_TX(i3c);
	LL_I3C_DisableDMAReq_RX(i3c);
	LL_I3C_DisableDMAReq_Control(i3c);
	LL_I3C_DisableDMAReq_Status(i3c);
}

static void i3c_stm32_prepare_dma_requests(const struct device *dev)
{
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;

	LL_I3C_DisableIT_TXFNF(i3c);
	LL_I3C_DisableIT_RXFNE(i3c);
	LL_I3C_DisableIT_CFNF(i3c);
	LL_I3C_DisableIT_SFNE(i3c);

	LL_I3C_EnableDMAReq_TX(i3c);
	LL_I3C_EnableDMAReq_RX(i3c);
	LL_I3C_EnableDMAReq_Control(i3c);
	LL_I3C_EnableDMAReq_Status(i3c);
}

#endif /* CONFIG_I3C_STM32_DMA */

static void i3c_stm32_flush_all_fifo(const struct device *dev)
{
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;

	LL_I3C_RequestTxFIFOFlush(i3c);
	LL_I3C_RequestRxFIFOFlush(i3c);
	LL_I3C_RequestControlFIFOFlush(i3c);
	LL_I3C_RequestStatusFIFOFlush(i3c);
}
#endif /* CONFIG_I3C_CONTROLLER */

static void i3c_stm32_log_err_type(const struct device *dev)
{
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;

	if (LL_I3C_IsActiveFlag_ANACK(i3c)) {
		LOG_ERR("Address NACK");
	}

	if (LL_I3C_IsActiveFlag_COVR(i3c)) {
		LOG_ERR("Control/Status FIFO underrun/overrun");
	}

	if (LL_I3C_IsActiveFlag_DOVR(i3c)) {
		LOG_ERR("TX/RX FIFO underrun/overrun");
	}

	if (LL_I3C_IsActiveFlag_DNACK(i3c)) {
		LOG_ERR("Data NACK by target");
	}

	if (LL_I3C_IsActiveFlag_PERR(i3c)) {
		switch (LL_I3C_GetMessageErrorCode(i3c)) {
		case LL_I3C_CONTROLLER_ERROR_CE0:
			LOG_ERR("Illegally formatted CCC detected");
			break;
		case LL_I3C_CONTROLLER_ERROR_CE1:
			LOG_ERR("Data on bus is not as expected");
			break;
		case LL_I3C_CONTROLLER_ERROR_CE2:
			LOG_ERR("No response to broadcast address");
			break;
		default:
			LOG_ERR("Unsupported error detected");
			break;
		}
	}
}

#ifdef CONFIG_I3C_CONTROLLER
static void i3c_stm32_clear_err(const struct device *dev, bool is_i2c_xfer)
{
	struct i3c_stm32_data *data = dev->data;
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;

	i3c_stm32_flush_all_fifo(dev);

	/* Re-enable arbirtation header after exiting from error caused by legacy I2C msg */
	if (is_i2c_xfer) {
		LL_I3C_EnableArbitrationHeader(i3c);
	}

#ifdef CONFIG_I3C_STM32_DMA
	i3c_stm32_end_dma_requests(dev);

	k_heap_free(&stm32_i3c_fifo_heap, data->status_fifo);
	k_heap_free(&stm32_i3c_fifo_heap, data->control_fifo);
#endif

	data->msg_state = STM32_I3C_MSG_IDLE;
	data->sf_state = STM32_I3C_SF_IDLE;
}

/* Transfer references and the bus mutex belong to the calling thread. */
static int i3c_stm32_xfer_start(const struct device *dev, enum i3c_stm32_msg_state state)
{
	struct i3c_stm32_data *data = dev->data;
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;
	int ret = pm_device_runtime_get(dev);

	if (ret < 0) {
		return ret;
	}
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	k_sem_reset(&data->device_sync_sem);
	LL_I3C_ClearFlag_FC(i3c);
	LL_I3C_ClearFlag_ERR(i3c);
	LL_I3C_ClearFlag_RXTGTEND(i3c);
	data->xfer_result = 0;
	data->xfer_active = true;
	data->msg_state = state;
	return 0;
}

/* Arm interrupts only after the caller has initialized all ISR-visible state. */
static void i3c_stm32_xfer_arm(const struct device *dev)
{
	struct i3c_stm32_data *data = dev->data;
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;

	if (!IS_ENABLED(CONFIG_I3C_STM32_DMA) || data->msg_state != STM32_I3C_MSG) {
		LL_I3C_EnableIT_TXFNF(i3c);
		LL_I3C_EnableIT_RXFNE(i3c);
		LL_I3C_EnableIT_CFNF(i3c);
		LL_I3C_EnableIT_SFNE(i3c);
	}
	LL_I3C_EnableIT_FC(i3c);
	LL_I3C_EnableIT_ERR(i3c);
}

static void i3c_stm32_xfer_complete(const struct device *dev, int result)
{
	struct i3c_stm32_data *data = dev->data;
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;
	unsigned int key = irq_lock();

	LL_I3C_DisableIT_TXFNF(i3c);
	LL_I3C_DisableIT_RXFNE(i3c);
	LL_I3C_DisableIT_CFNF(i3c);
	LL_I3C_DisableIT_SFNE(i3c);
	LL_I3C_DisableIT_FC(i3c);
	LL_I3C_DisableIT_ERR(i3c);
	if (data->xfer_active) {
		data->xfer_active = false;
		data->xfer_result = result;
		k_sem_give(&data->device_sync_sem);
	}
	irq_unlock(key);
}

static int i3c_stm32_xfer_wait(const struct device *dev)
{
	struct i3c_stm32_data *data = dev->data;

	if (k_sem_take(&data->device_sync_sem, STM32_I3C_TRANSFER_TIMEOUT) != 0) {
		i3c_stm32_xfer_complete(dev, -ETIMEDOUT);
	}
	return data->xfer_result;
}

static int i3c_stm32_xfer_end(const struct device *dev, int result)
{
	struct i3c_stm32_data *data = dev->data;
	int ret;

	i3c_stm32_xfer_complete(dev, result);
	data->msg_state = STM32_I3C_MSG_IDLE;
	data->sf_state = STM32_I3C_SF_IDLE;
	data->ccc_payload = NULL;
	data->ccc_target_payload = NULL;
	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	ret = pm_device_runtime_put(dev);
	return result != 0 ? result : ret;
}

/**
 * @brief Fills the I3C TX FIFO from a given buffer
 *
 * @param buf The buffer to fill the TX FIFO from
 * @param len The total buffer length
 * @param offset Pointer to the offset from the beginning of buffer which will be incremented by the
 * number of bytes sent to the TX FIFO
 *
 * @return Returns true if last byte was sent (TXLAST flag was set)
 */
static bool i3c_stm32_fill_tx_fifo(const struct device *dev, uint8_t *buf, size_t len,
				   size_t *offset)
{
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;
	bool is_last = false;

	if (*offset >= len) {
		return 0;
	}

	while (*offset < len && LL_I3C_IsActiveFlag_TXFNF(i3c)) {
		if (LL_I3C_IsActiveFlag_TXLAST(i3c)) {
			is_last = true;
		}

		if (*offset < len) {
			LL_I3C_TransmitData8(i3c, buf[(*offset)++]);
		}

		if (is_last) {
			return is_last;
		}
	}

	return is_last;
}

/**
 * @brief Drains the I3C RX FIFO from a given buffer
 *
 * @param buf The buffer to drain the RX FIFO to
 * @param len The total buffer length
 * @param offset Pointer to the offset from the beginning of buffer which will be incremented by the
 * number of bytes drained from the RX FIFO
 *
 * @return Returns true if last byte was received (RXLAST flag was set)
 */
static bool i3c_stm32_drain_rx_fifo(const struct device *dev, uint8_t *buf, uint32_t len,
				    size_t *offset)
{
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;
	bool is_last = false;

	if (*offset >= len) {
		return 0;
	}

	while (*offset < len && LL_I3C_IsActiveFlag_RXFNE(i3c)) {
		if (LL_I3C_IsActiveFlag_RXLAST(i3c)) {
			is_last = true;
		}

		if (*offset < len) {
			buf[(*offset)++] = LL_I3C_ReceiveData8(i3c);
		}

		if (is_last) {
			return is_last;
		}
	}

	return is_last;
}

/* Handles broadcast/direct CCCs except for ENTDAA */
static int i3c_stm32_do_ccc(const struct device *dev, struct i3c_ccc_payload *payload)
{
	const struct i3c_stm32_config *config = dev->config;
	struct i3c_stm32_data *data = dev->data;
	I3C_TypeDef *i3c = config->i3c;
	int ret;

	__ASSERT(dev != NULL, "I3C Device is NULL.");
	__ASSERT(payload != NULL, "I3C Payload is NULL.");

	if (payload->ccc.id == I3C_CCC_ENTDAA) {
		return -EINVAL;
	}

	/* Check if payload has targets when sending a direct CCC */
	if (!i3c_ccc_is_payload_broadcast(payload) &&
	    (payload->targets.payloads == NULL || payload->targets.num_targets == 0)) {
		return -EINVAL;
	}

	if (payload->ccc.data_len > 0 && payload->ccc.data == NULL) {
		return -EINVAL;
	}
	for (size_t i = 0; i < payload->targets.num_targets; i++) {
		struct i3c_ccc_target_payload *target = &payload->targets.payloads[i];

		if (target->data_len > 0 && target->data == NULL) {
			return -EINVAL;
		}
	}

	k_mutex_lock(&data->bus_mutex, K_FOREVER);

	ret = i3c_stm32_xfer_start(dev, STM32_I3C_MSG_CCC);
	if (ret != 0) {
		goto unlock;
	}

	/* RXLAST identifies the target boundary, including short responses. */
	data->ccc_payload = payload;
	data->ccc_target_idx = 0;
	data->ccc_target_payload = payload->targets.payloads;

	payload->ccc.num_xfer = 0;

	for (size_t i = 0; i < payload->targets.num_targets; i++) {
		payload->targets.payloads[i].num_xfer = 0;
	}

	/* Start CCC transfer */
	i3c_stm32_xfer_arm(dev);
	LL_I3C_ControllerHandleCCC(i3c, payload->ccc.id, payload->ccc.data_len,
				   (i3c_ccc_is_payload_broadcast(payload)
					    ? LL_I3C_GENERATE_STOP
					    : LL_I3C_GENERATE_RESTART));

	ret = i3c_stm32_xfer_wait(dev);
	if (ret != 0) {
		i3c_stm32_clear_err(dev, false);
	}
	ret = i3c_stm32_xfer_end(dev, ret);
unlock:
	k_mutex_unlock(&data->bus_mutex);
	return ret;
}

/* Handles the ENTDAA CCC */
static int i3c_stm32_do_daa(const struct device *dev)
{
	const struct i3c_stm32_config *config = dev->config;
	struct i3c_stm32_data *data = dev->data;
	I3C_TypeDef *i3c = config->i3c;
	int ret = 0;

	k_mutex_lock(&data->bus_mutex, K_FOREVER);

	ret = i3c_stm32_xfer_start(dev, STM32_I3C_MSG_DAA);
	if (ret != 0) {
		goto unlock;
	}
	data->pid = 0;
	data->daa_rx_rcv = 0;
	i3c_stm32_xfer_arm(dev);

	/* Disable TXFNF interrupt, the RXFNE interrupt will enable it once all PID bytes are
	 * received
	 */
	LL_I3C_DisableIT_TXFNF(i3c);

	/* Start DAA */
	LL_I3C_ControllerHandleCCC(i3c, I3C_CCC_ENTDAA, 0, LL_I3C_GENERATE_STOP);

	ret = i3c_stm32_xfer_wait(dev);
	if (ret != 0) {
		i3c_stm32_clear_err(dev, false);
	}
	ret = i3c_stm32_xfer_end(dev, ret);
unlock:
	k_mutex_unlock(&data->bus_mutex);
	return ret;
}

#ifdef CONFIG_I3C_STM32_DMA

static int i3c_stm32_dma_msg_control_fifo_config(const struct device *dev)
{
	struct i3c_stm32_data *data = dev->data;
	int ret;

	data->dma_tc.blk_cfg.source_address = (uint32_t)data->control_fifo;
	data->dma_tc.blk_cfg.block_size = data->fifo_len;

	ret = dma_config(data->dma_tc.dma_dev, data->dma_tc.dma_channel, &data->dma_tc.dma_cfg);

	if (ret != 0) {
		LOG_ERR("Control DMA config error, err=%d", ret);
		return -EINVAL;
	}

	if (dma_start(data->dma_tc.dma_dev, data->dma_tc.dma_channel)) {
		LOG_ERR("Control DMA start failed");
		return -EFAULT;
	}

	return 0;
}

static int i3c_stm32_dma_msg_status_fifo_config(const struct device *dev)
{
	struct i3c_stm32_data *data = dev->data;
	int ret;

	data->dma_rs.blk_cfg.dest_address = (uint32_t)data->status_fifo;
	data->dma_rs.blk_cfg.block_size = data->fifo_len;

	ret = dma_config(data->dma_rs.dma_dev, data->dma_rs.dma_channel, &data->dma_rs.dma_cfg);

	if (ret != 0) {
		LOG_ERR("Status DMA config error, err=%d", ret);
		return -EINVAL;
	}

	if (dma_start(data->dma_rs.dma_dev, data->dma_rs.dma_channel)) {
		LOG_ERR("Status DMA start failed");
		return -EFAULT;
	}

	return 0;
}

static int i3c_stm32_dma_msg_config(const struct device *dev, uint32_t buf_addr, size_t buf_len)
{
	struct i3c_stm32_dma_stream *dma_stream;
	struct i3c_stm32_data *data = dev->data;
	int ret;

	if (i3c_stm32_curr_msg_xfer_is_read(dev)) {
		dma_stream = &(data->dma_rx);
		dma_stream->blk_cfg.dest_address = buf_addr;
	} else {
		dma_stream = &(data->dma_tx);
		dma_stream->blk_cfg.source_address = buf_addr;
	}

	i3c_stm32_arbitration_header_config(dev);

	dma_stream->blk_cfg.block_size = buf_len;
	ret = dma_config(dma_stream->dma_dev, dma_stream->dma_channel, &dma_stream->dma_cfg);

	if (ret != 0) {
		LOG_ERR("TX/RX DMA config error, err=%d", ret);
		return -EINVAL;
	}

	if (dma_start(dma_stream->dma_dev, dma_stream->dma_channel)) {
		LOG_ERR("TX/RX DMA start failed");
		return -EFAULT;
	}
	return 0;
}
#endif

static int i3c_stm32_transfer_begin(const struct device *dev)
{
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;

#ifdef CONFIG_I3C_STM32_DMA
	struct i3c_stm32_data *data = dev->data;
	struct i3c_stm32_msg *curr_msg = &data->curr_msg;

	data->fifo_len = curr_msg->num_msgs * sizeof(uint32_t);
	data->control_fifo = k_heap_alloc(&stm32_i3c_fifo_heap, data->fifo_len, K_FOREVER);
	data->status_fifo = k_heap_alloc(&stm32_i3c_fifo_heap, data->fifo_len, K_FOREVER);
	int ret;

	/* Prepare all control words for all messages on the transfer */
	for (size_t i = 0; i < curr_msg->num_msgs; i++) {
		stm32_reg_write(&data->control_fifo[i],
				((curr_msg->target_addr << I3C_CR_ADD_Pos) |
				 i3c_stm32_curr_msg_control_get_len(dev) |
				 i3c_stm32_curr_msg_control_get_dir(dev) | curr_msg->msg_type |
				 i3c_stm32_curr_msg_control_get_end(dev)) &
				(I3C_CR_ADD | I3C_CR_DCNT | I3C_CR_RNW | I3C_CR_MTYPE |
				 I3C_CR_MEND));

		i3c_stm32_curr_msg_control_next(dev);
	}

	/* Configure DMA for the first message only, DMA callback will take care of the rest */
	uint8_t *buf = NULL;
	size_t *offset = 0;
	uint32_t len = 0;

	i3c_stm32_curr_msg_xfer_get_buf(dev, &buf, &len, &offset);

	ret = i3c_stm32_dma_msg_config(dev, (uint32_t)buf, len);
	if (ret != 0) {
		return ret;
	}

	ret = i3c_stm32_dma_msg_control_fifo_config(dev);
	if (ret != 0) {
		return ret;
	}

	ret = i3c_stm32_dma_msg_status_fifo_config(dev);
	if (ret != 0) {
		return ret;
	}

	i3c_stm32_prepare_dma_requests(dev);
#endif

	/* Begin transmission */
	i3c_stm32_xfer_arm(dev);
	LL_I3C_RequestTransfer(i3c);

	return i3c_stm32_xfer_wait(dev);
}

/* Handles the controller private read/write transfers */
static int i3c_stm32_i3c_transfer(const struct device *dev, struct i3c_device_desc *target,
				  struct i3c_msg *msgs, uint8_t num_msgs)
{
	struct i3c_stm32_data *data = dev->data;
	int ret;

	/* Verify all messages */
	for (size_t i = 0; i < num_msgs; i++) {
		if (msgs[i].buf == NULL) {
			return -EINVAL;
		}
		if (msgs[i].len > UINT16_MAX) {
			return -EMSGSIZE;
		}
		if (msgs[i].flags & I3C_MSG_HDR) {
			return -ENOTSUP;
		}
	}

	k_mutex_lock(&data->bus_mutex, K_FOREVER);
	ret = i3c_stm32_xfer_start(dev, STM32_I3C_MSG);
	if (ret != 0) {
		goto unlock;
	}
	ret = i3c_stm32_curr_msg_init(dev, msgs, NULL, num_msgs, target->dynamic_addr);
	if (ret != 0) {
		LOG_ERR("Failed to initialize transfer messages, err=%d", ret);
		goto finish;
	}

	ret = i3c_stm32_transfer_begin(dev);
	if (ret != 0) {
		LOG_ERR("Failed to transfer messages, err=%d", ret);
		goto finish;
	}

#ifdef CONFIG_I3C_STM32_DMA
	/* Fill the num_xfer for each message from the status FIFO */
	for (size_t i = 0; i < num_msgs; i++) {
		msgs[i].num_xfer = stm32_reg_read_bits(&data->status_fifo[i], I3C_SR_XDCNT);
	}

	k_heap_free(&stm32_i3c_fifo_heap, data->control_fifo);
	k_heap_free(&stm32_i3c_fifo_heap, data->status_fifo);
	data->control_fifo = NULL;
	data->status_fifo = NULL;

	i3c_stm32_end_dma_requests(dev);
#endif

finish:
	if (ret != 0) {
		i3c_stm32_clear_err(dev, false);
	}
	ret = i3c_stm32_xfer_end(dev, ret);
unlock:
	k_mutex_unlock(&data->bus_mutex);
	return ret;
}

static int i3c_stm32_i2c_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
				  uint16_t addr)
{
	struct i3c_stm32_data *data = dev->data;
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;
	int ret;

	/* Verify all messages */
	for (size_t i = 0; i < num_msgs; i++) {
		if (msgs[i].buf == NULL) {
			return -EINVAL;
		}
		if (msgs[i].len > UINT16_MAX) {
			return -EMSGSIZE;
		}
		if (msgs[i].flags & I2C_MSG_ADDR_10_BITS) {
			LOG_ERR("10-bit addressing mode is not supported");
			return -ENOTSUP;
		}
	}

	k_mutex_lock(&data->bus_mutex, K_FOREVER);
	ret = i3c_stm32_xfer_start(dev, STM32_I3C_MSG);
	if (ret != 0) {
		goto unlock;
	}

	/* Disable arbitration header for all I2C messages in case no I3C devices exist on bus */
	LL_I3C_DisableArbitrationHeader(i3c);

	ret = i3c_stm32_curr_msg_init(dev, NULL, msgs, num_msgs, addr);
	if (ret != 0) {
		LOG_ERR("Failed to initialize transfer messages, err=%d", ret);
		goto finish;
	}

	ret = i3c_stm32_transfer_begin(dev);
	if (ret != 0) {
		LOG_ERR("Failed to transfer messages, err=%d", ret);
		goto finish;
	}

	LL_I3C_EnableArbitrationHeader(i3c);

#ifdef CONFIG_I3C_STM32_DMA
	k_heap_free(&stm32_i3c_fifo_heap, data->control_fifo);
	k_heap_free(&stm32_i3c_fifo_heap, data->status_fifo);
	data->control_fifo = NULL;
	data->status_fifo = NULL;

	i3c_stm32_end_dma_requests(dev);
#endif

finish:
	if (ret != 0) {
		i3c_stm32_clear_err(dev, true);
	}
	ret = i3c_stm32_xfer_end(dev, ret);
unlock:
	k_mutex_unlock(&data->bus_mutex);
	return ret;
}
#endif /*CONFIG_I3C_CONTROLLER*/

#ifdef CONFIG_PM_DEVICE
static int i3c_stm32_suspend(const struct device *dev)
{
	int ret;
	const struct i3c_stm32_config *cfg = dev->config;
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	/* Disable device clock. */
	ret = clock_control_off(clk, (clock_control_subsys_t)&cfg->pclken[0]);
	if (ret < 0) {
		LOG_ERR("failure disabling I3C clock");
		return ret;
	}

	/* Move pins to sleep state */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_SLEEP);
	if (ret == -ENOENT) {
		/* Warn but don't block suspend */
		LOG_WRN("I3C pinctrl sleep state not available");
	} else if (ret < 0) {
		return ret;
	}

	return 0;
}

static int i3c_stm32_pm_action(const struct device *dev, enum pm_device_action action)
{
	int err;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		err = i3c_stm32_activate(dev);
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		err = i3c_stm32_suspend(dev);
		break;
	default:
		return -ENOTSUP;
	}

	return err;
}
#endif

#ifdef CONFIG_I3C_CONTROLLER
#ifdef CONFIG_I3C_STM32_DMA
static int i3c_stm32_dma_stream_config(const struct device *dev,
				       struct i3c_stm32_dma_stream *dma_stream, uint64_t src_addr,
				       uint64_t dst_addr)
{
	if (dma_stream->dma_dev != NULL) {
		if (!device_is_ready(dma_stream->dma_dev)) {
			return -ENODEV;
		}
	}

	memset(&dma_stream->blk_cfg, 0, sizeof(dma_stream->blk_cfg));

	dma_stream->blk_cfg.source_address = src_addr;

	dma_stream->blk_cfg.dest_address = dst_addr;

	if (dma_stream->src_addr_increment) {
		dma_stream->blk_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		dma_stream->blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	if (dma_stream->dst_addr_increment) {
		dma_stream->blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		dma_stream->blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	dma_stream->blk_cfg.source_reload_en = 0;
	dma_stream->blk_cfg.dest_reload_en = 0;
	dma_stream->blk_cfg.fifo_mode_control = dma_stream->fifo_threshold;

	dma_stream->dma_cfg.head_block = &dma_stream->blk_cfg;
	dma_stream->dma_cfg.user_data = (void *)dev;

	return 0;
}

/* Initializes the I3C DMA */
static int i3c_stm32_init_dma(const struct device *dev)
{
	struct i3c_stm32_data *data = dev->data;
	int err;
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;

	/*Configure DMA RX */
	err = i3c_stm32_dma_stream_config(
		dev, &data->dma_rx, LL_I3C_DMA_GetRegAddr(i3c, LL_I3C_DMA_REG_DATA_RECEIVE_BYTE),
		0);
	if (err != 0) {
		return err;
	}

	/*Configure DMA RS */
	err = i3c_stm32_dma_stream_config(dev, &data->dma_rs,
					  LL_I3C_DMA_GetRegAddr(i3c, LL_I3C_DMA_REG_STATUS), 0);
	if (err != 0) {
		return err;
	}

	/*Configure DMA TX */
	err = i3c_stm32_dma_stream_config(
		dev, &data->dma_tx, 0,
		LL_I3C_DMA_GetRegAddr(i3c, LL_I3C_DMA_REG_DATA_TRANSMIT_BYTE));
	if (err != 0) {
		return err;
	}

	/*Configure DMA TC */
	err = i3c_stm32_dma_stream_config(dev, &data->dma_tc, 0,
					  LL_I3C_DMA_GetRegAddr(i3c, LL_I3C_DMA_REG_CONTROL));
	if (err != 0) {
		return err;
	}

	return err;
}
#endif

static void i3c_stm32_controller_init(const struct device *dev)
{
	struct i3c_stm32_data *data = dev->data;
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;

	/* Configure FIFO */
	LL_I3C_SetRxFIFOThreshold(i3c, STM32_I3C_RXFIFO_THRESHOLD_1_BYTE);
	LL_I3C_SetTxFIFOThreshold(i3c, STM32_I3C_TXFIFO_THRESHOLD_1_BYTE);
	LL_I3C_EnableControlFIFO(i3c);
	LL_I3C_EnableStatusFIFO(i3c);

	/* I3C Initialization */
	LL_I3C_SetMode(i3c, LL_I3C_MODE_CONTROLLER);
	LL_I3C_SetStallTime(i3c, 0x00);
	LL_I3C_DisableStallACK(i3c);
	LL_I3C_DisableStallParityCCC(i3c);
	LL_I3C_DisableStallParityData(i3c);
	LL_I3C_DisableStallTbit(i3c);
	LL_I3C_DisableHighKeeperSDA(i3c);
	LL_I3C_SetControllerActivityState(i3c, LL_I3C_OWN_ACTIVITY_STATE_0);

	LL_I3C_Enable(i3c);

	LL_I3C_DisableIT_FC(i3c);
	LL_I3C_DisableIT_CFNF(i3c);
	LL_I3C_DisableIT_SFNE(i3c);
	LL_I3C_DisableIT_RXFNE(i3c);
	LL_I3C_DisableIT_TXFNF(i3c);
	LL_I3C_DisableIT_ERR(i3c);
	LL_I3C_EnableIT_WKP(i3c);

#ifdef CONFIG_I3C_USE_IBI
	LL_I3C_EnableIT_IBI(i3c);
	LL_I3C_EnableIT_HJ(i3c);
#endif

	/* Bus will be idle initially */
	data->msg_state = STM32_I3C_MSG_IDLE;
	data->sf_state = STM32_I3C_SF_IDLE;
	data->target_id = 0;
#ifdef CONFIG_I3C_USE_IBI
	data->ibi_payload = 0;
	data->ibi_payload_size = 0;
	data->ibi_target_addr = 0;
#endif
}
#endif /*CONFIG_I3C_CONTROLLER*/

#ifdef CONFIG_I3C_TARGET
static void i3c_stm32_target_init(const struct device *dev, uint8_t mipi_instance)
{
	struct i3c_stm32_data *data = dev->data;
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;

	/* Configure FIFO */
	LL_I3C_SetRxFIFOThreshold(i3c, STM32_I3C_RXFIFO_THRESHOLD_1_BYTE);
	LL_I3C_SetTxFIFOThreshold(i3c, STM32_I3C_RXFIFO_THRESHOLD_1_BYTE);
	LL_I3C_DisableControlFIFO(i3c);
	LL_I3C_DisableStatusFIFO(i3c);

	/* I3C Target Initialization */
	LL_I3C_SetMode(i3c, LL_I3C_MODE_TARGET);
	LL_I3C_SetDeviceCapabilityOnBus(i3c, LL_I3C_DEVICE_ROLE_AS_TARGET);

	/* Disable optional target-side features */
	LL_I3C_DisableControllerRoleReq(i3c);
	LL_I3C_DisableHotJoin(i3c);
	LL_I3C_DisableIBI(i3c);

	/* Configure baseline target capabilities and protocol defaults. */
	LL_I3C_SetDeviceIBIPayload(i3c, LL_I3C_IBI_NO_ADDITIONAL_DATA);
	LL_I3C_ConfigNbIBIAddData(i3c, LL_I3C_PAYLOAD_1_BYTE);
	LL_I3C_SetGrpAddrHandoffSupport(i3c, LL_I3C_HANDOFF_GRP_ADDR_NOT_SUPPORTED);
	LL_I3C_SetDataTurnAroundTime(i3c, LL_I3C_TURNAROUND_TIME_TSCO_LESS_12NS);
	LL_I3C_SetMiddleByteTurnAround(i3c, 0);
	LL_I3C_SetDataSpeedLimitation(i3c, LL_I3C_NO_DATA_SPEED_LIMITATION);
	LL_I3C_SetMaxDataSpeedFormat(i3c, LL_I3C_GETMXDS_FORMAT_1);
	LL_I3C_SetHandoffActivityState(i3c, LL_I3C_HANDOFF_ACTIVITY_STATE_0);
	LL_I3C_SetControllerHandoffDelayed(i3c, LL_I3C_HANDOFF_NOT_DELAYED);
	LL_I3C_SetPendingReadMDB(i3c, LL_I3C_MDB_NO_PENDING_READ_NOTIFICATION);

	/* Enable I3C block */
	LL_I3C_Enable(i3c);

	/* Disable Interrupts */
	LL_I3C_EnableIT_DAUPD(i3c);
	LL_I3C_EnableIT_FC(i3c);
	LL_I3C_EnableIT_RXFNE(i3c);
	LL_I3C_EnableIT_TXFNF(i3c);
	LL_I3C_EnableIT_ERR(i3c);
	LL_I3C_EnableIT_WKP(i3c);

	data->msg_state = STM32_I3C_MSG_IDLE;
	data->sf_state = STM32_I3C_SF_IDLE;
	data->target_id = 0;
}
#endif /*CONFIG_I3C_TARGET*/

#if defined(CONFIG_I3C_USE_IBI) && defined(CONFIG_I3C_CONTROLLER)
int i3c_stm32_ibi_hj_response(const struct device *dev, bool ack);
#endif

/* Initializes the I3C device and I3C bus */
static int i3c_stm32_init(const struct device *dev)
{
	const struct i3c_stm32_config *config = dev->config;
	struct i3c_stm32_data *data = dev->data;
	int ret;

	k_mutex_init(&data->bus_mutex);
#ifdef CONFIG_SOC_SERIES_STM32H7RSX
	if (!LL_FLASH_OptionBytes_IsI3CEnabled()) {
		LOG_ERR("I3C1 is disabled by the I2C_NI3C option byte");
		return -ENODEV;
	}
#endif
	if (!device_is_ready(config->reset.dev)) {
		return -ENODEV;
	}
	ret = reset_line_toggle_dt(&config->reset);
	if (ret != 0) {
		return ret;
	}
	config->irq_config_func(dev);

#ifdef CONFIG_I3C_TARGET
	if (config->target_mode) {
		struct i3c_config_target targ_cfg = {
			.pid = (uint64_t)config->mipi_instance << 12U,
			.dcr = config->dcr,
			.max_read_len = config->mrl,
			.max_write_len = config->mwl,
		};

		ret = i3c_stm32_configure(dev, I3C_CONFIG_TARGET, &targ_cfg);
		if (ret != 0) {
			LOG_ERR("Failed to configure I3C target mode, err=%d", ret);
			return ret;
		}

		i3c_stm32_target_init(dev, config->mipi_instance);
		return 0;
	}
#endif

#ifdef CONFIG_I3C_CONTROLLER

#ifdef CONFIG_I3C_STM32_DMA
	ret = i3c_stm32_init_dma(dev);

	if (ret != 0) {
		LOG_ERR("Failed to init I3C DMA, err=%d", ret);
		return ret;
	}
#endif

	k_sem_init(&data->device_sync_sem, 0, 1);

	ret = i3c_addr_slots_init(dev);
	if (ret != 0) {
		LOG_ERR("Addr slots init fail, err=%d", ret);
		return ret;
	}

	ret = i3c_stm32_configure(dev, I3C_CONFIG_CONTROLLER, &data->drv_data.ctrl_config);
	if (ret != 0) {
		return ret;
	}
	i3c_stm32_controller_init(dev);

	/* Perform bus initialization only if there are devices that already exist on the bus */
	if (config->drv_cfg.dev_list.num_i3c > 0 &&
	    !(config->drv_cfg.flags & I3C_CONTROLLER_FLAG_DISABLE_BUS_INIT)) {
		ret = i3c_bus_init(dev, &config->drv_cfg.dev_list);
		if (ret != 0) {
			LOG_ERR("Failed to do i3c bus init, err=%d", ret);
			return ret;
		}
	}

	/* Enable runtime PM before taking references for unsolicited bus events. */
	ret = pm_device_runtime_enable(dev);
	if (ret != 0) {
		return ret;
	}
#ifdef CONFIG_I3C_USE_IBI
	if (!(config->drv_cfg.flags & I3C_CONTROLLER_FLAG_DISABLE_HJ_AT_INIT)) {
		ret = i3c_stm32_ibi_hj_response(dev, true);
		if (ret != 0) {
			return ret;
		}
	}
#endif

	return 0;
#else  /*CONFIG_I3C_CONTROLLER*/
	return -ENOTSUP;
#endif /*CONFIG_I3C_CONTROLLER*/
}

#ifdef CONFIG_I3C_TARGET
static void i3c_stm32_process_target_txfnf(const struct device *dev)
{
	struct i3c_stm32_data *data = dev->data;
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;

	while (LL_I3C_IsActiveFlag_TXFNF(i3c)) {

		if (data->target_config != NULL && data->target_config->callbacks != NULL) {
			if (data->target_config->callbacks->read_processed_cb != NULL) {
				uint8_t byte = 0;

				data->target_config->callbacks->read_processed_cb(
					data->target_config, &byte);
				LL_I3C_TransmitData8(i3c, byte);
			}

			if (!LL_I3C_IsActiveTxPreload(i3c) &&
			    data->target_config->callbacks->read_requested_cb != NULL) {
				data->target_config->callbacks->read_requested_cb(
					data->target_config, NULL);
			}
		}
	}
}
#endif /*CONFIG_I3C_CONTROLLER*/

static void i3c_stm32_event_isr_tx(const struct device *dev)
{
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;

#if CONFIG_I3C_TARGET
	if (!ll_i3c_is_in_controller_mode(i3c)) {
		i3c_stm32_process_target_txfnf(dev);
	}
#endif /*CONFIG_I3C_TARGET*/
#if CONFIG_I3C_CONTROLLER
	struct i3c_stm32_data *data = dev->data;

	switch (data->msg_state) {
	case STM32_I3C_MSG: {
		uint8_t *buf = NULL;
		size_t *offset = NULL;
		uint32_t len = 0;

		if (i3c_stm32_curr_msg_xfer_get_buf(dev, &buf, &len, &offset) != 0 ||
		    i3c_stm32_curr_msg_xfer_is_read(dev)) {
			break;
		}

		if (i3c_stm32_fill_tx_fifo(dev, buf, len, offset)) {
			i3c_stm32_curr_msg_xfer_next(dev);
		}

		break;
	}
	case STM32_I3C_MSG_DAA: {
		struct i3c_device_desc *target;
		uint8_t bcr;
		uint8_t dcr;
		uint8_t dyn_addr = 0;
		int ret;

		bcr = (data->pid >> 8) & 0xFF;
		dcr = data->pid & 0xFF;
		data->pid >>= 16;

		/* Disable TXFNF interrupt, the RXFNE interrupt will enable it once all PID bytes
		 * are received for next I3C target, or in i3c_stm32_do_daa after frame complete
		 */
		LL_I3C_DisableIT_TXFNF(i3c);

		/* Find the device in the device list */
		ret = i3c_dev_list_daa_addr_helper(dev, data->pid, false, false, &target,
						   &dyn_addr);
		if (ret != 0) {
			/* TODO: figure out what is the correct sequence to exit form this error
			 * It is expected that a TX overrun error to occur which triggers err isr
			 */
			LOG_ERR("No dynamic address could be assigned to target");

			return;
		}

		/* Put the new dynamic address in TX FIFO for transmission */
		LL_I3C_TransmitData8(i3c, dyn_addr);

		if (target != NULL) {
			/* Update target descriptor */
			target->dynamic_addr = dyn_addr;
			target->bcr = bcr;
			target->dcr = dcr;

			int aret = i3c_attach_i3c_device(target);

			if (aret != 0 && aret != -EALREADY) {
				LOG_ERR("Failed to attach target");
			}
		}

		/* Mark the address as used */
		i3c_addr_slots_mark_i3c(&data->drv_data.attached_dev.addr_slots, dyn_addr);

		/* Mark the static address as free */
		if ((target != NULL) && (target->static_addr != 0) &&
		    (dyn_addr != target->static_addr)) {
			i3c_addr_slots_mark_free(&data->drv_data.attached_dev.addr_slots,
						 target->static_addr);
		}

		break;
	}
	case STM32_I3C_MSG_CCC: {
		struct i3c_ccc_payload *payload = data->ccc_payload;

		if (payload->ccc.num_xfer < payload->ccc.data_len) {
			LL_I3C_TransmitData8(i3c, payload->ccc.data[payload->ccc.num_xfer++]);
		}
		break;
	}
	case STM32_I3C_MSG_CCC_P2: {
		struct i3c_ccc_target_payload *target = data->ccc_target_payload;
		struct i3c_ccc_payload *payload = data->ccc_payload;

		if (target < payload->targets.payloads + payload->targets.num_targets &&
		    !target->rnw && target->num_xfer < target->data_len) {
			LL_I3C_TransmitData8(i3c, target->data[target->num_xfer++]);
			if (target->num_xfer == target->data_len) {
				data->ccc_target_payload++;
			}
		}
		break;
	}
	default:
		break;
	}
#endif /*CONFIG_I3C_CONTROLLER*/
}

static void i3c_stm32_event_isr_rx(const struct device *dev)
{
	const struct i3c_stm32_config *config = dev->config;
	struct i3c_stm32_data *data = dev->data;
	I3C_TypeDef *i3c = config->i3c;

#ifdef CONFIG_I3C_TARGET
	if (LL_I3C_IsActiveFlag_RXFNE(i3c) && !ll_i3c_is_in_controller_mode(i3c)) {
		if (data->target_config && data->target_config->callbacks != NULL &&
		    data->target_config->callbacks->write_requested_cb != NULL) {
			data->target_config->callbacks->write_requested_cb(data->target_config);
		}

		while (LL_I3C_IsActiveFlag_RXFNE(i3c)) {
			uint8_t rx_data = LL_I3C_ReceiveData8(i3c);

			if (data->target_config && data->target_config->callbacks != NULL &&
			    data->target_config->callbacks->write_received_cb != NULL) {
				data->target_config->callbacks->write_received_cb(
					data->target_config, rx_data);
			}
		}
	}
#endif /*CONFIG_I3C_TARGET*/

#ifdef CONFIG_I3C_CONTROLLER
	switch (data->msg_state) {
	case STM32_I3C_MSG: {
		while (LL_I3C_IsActiveFlag_RXFNE(i3c)) {
			uint8_t *buf = NULL;
			size_t *offset = NULL;
			uint32_t len = 0;

			if (i3c_stm32_curr_msg_xfer_get_buf(dev, &buf, &len, &offset) != 0 ||
			    !i3c_stm32_curr_msg_xfer_is_read(dev)) {
				break;
			}
			if (!i3c_stm32_drain_rx_fifo(dev, buf, len, offset)) {
				break;
			}
			i3c_stm32_curr_msg_xfer_next(dev);
		}
		break;
	}
	case STM32_I3C_MSG_DAA: {
		data->pid <<= 8;
		data->pid |= LL_I3C_ReceiveData8(i3c);

		data->daa_rx_rcv++;

		/* After receiving 8 PID bytes from DAA, enable TXFNF interrupt to send the dynamic
		 * address
		 */
		if (data->daa_rx_rcv == 8) {
			LL_I3C_EnableIT_TXFNF(i3c);
			data->daa_rx_rcv = 0;
		}
		break;
	}
	case STM32_I3C_MSG_CCC_P2: {
		struct i3c_ccc_payload *payload = data->ccc_payload;

		while (LL_I3C_IsActiveFlag_RXFNE(i3c)) {
			struct i3c_ccc_target_payload *target = data->ccc_target_payload;
			bool last = LL_I3C_IsActiveFlag_RXLAST(i3c);
			uint8_t byte = LL_I3C_ReceiveData8(i3c);

			if (target < payload->targets.payloads + payload->targets.num_targets &&
			    target->rnw && target->num_xfer < target->data_len) {
				target->data[target->num_xfer++] = byte;
				if (last) {
					data->ccc_target_payload++;
				}
			}
		}
		break;
	}
	default:
		break;
	}
#endif /*CONFIG_I3C_CONTROLLER*/
}

#ifdef CONFIG_I3C_CONTROLLER
static void i3c_stm32_event_isr_cf(const struct device *dev)
{
	const struct i3c_stm32_config *config = dev->config;
	struct i3c_stm32_data *data = dev->data;
	struct i3c_stm32_msg *curr_msg = &data->curr_msg;
	I3C_TypeDef *i3c = config->i3c;

	switch (data->msg_state) {
	case STM32_I3C_MSG: {
		if (curr_msg->ctrl_msg_idx >= curr_msg->num_msgs) {
			break;
		}
		LL_I3C_ControllerHandleMessage(
			i3c, curr_msg->target_addr, i3c_stm32_curr_msg_control_get_len(dev),
			i3c_stm32_curr_msg_control_get_dir(dev), curr_msg->msg_type,
			i3c_stm32_curr_msg_control_get_end(dev));

		i3c_stm32_curr_msg_control_next(dev);
		break;
	}
	case STM32_I3C_MSG_CCC:
	case STM32_I3C_MSG_CCC_P2: {
		struct i3c_ccc_payload *payload = data->ccc_payload;
		struct i3c_ccc_target_payload *target;

		if (data->ccc_target_idx < payload->targets.num_targets) {
			target = &payload->targets.payloads[data->ccc_target_idx++];

			LL_I3C_ControllerHandleMessage(
				i3c, target->addr, target->data_len,
				target->rnw ? LL_I3C_DIRECTION_READ : LL_I3C_DIRECTION_WRITE,
				LL_I3C_CONTROLLER_MTYPE_DIRECT,
				(data->ccc_target_idx == payload->targets.num_targets)
					? LL_I3C_GENERATE_STOP
					: LL_I3C_GENERATE_RESTART);

			/* Change state to second part of CCC communication */
			if (data->msg_state == STM32_I3C_MSG_CCC) {
				data->msg_state = STM32_I3C_MSG_CCC_P2;
			}
		}
		break;
	}
	default:
		break;
	}
}

#if CONFIG_I3C_USE_IBI
static void i3c_stm32_isr_controller_ibi(const struct device *dev)
{
	const struct i3c_stm32_config *config = dev->config;
	struct i3c_stm32_data *data = dev->data;
	I3C_TypeDef *i3c = config->i3c;

	if (LL_I3C_IsActiveFlag_IBI(i3c)) {
		/* Clear frame complete flag */
		LL_I3C_ClearFlag_IBI(i3c);
		data->ibi_payload = LL_I3C_GetIBIPayload(i3c);
		data->ibi_payload_size = LL_I3C_GetNbIBIAddData(i3c);
		data->ibi_target_addr = LL_I3C_GetIBITargetAddr(i3c);
		struct i3c_device_desc *target;

		target = i3c_dev_list_i3c_addr_find(dev, data->ibi_target_addr);
		if (target == NULL || data->ibi_payload_size > sizeof(data->ibi_payload)) {
			LOG_ERR("Invalid IBI from address 0x%x", data->ibi_target_addr);
		} else if (i3c_ibi_work_enqueue_target_irq(target, (uint8_t *)&data->ibi_payload,
							 data->ibi_payload_size) != 0) {
			LOG_ERR("Error enqueue IBI IRQ work");
		}
	}

	if (LL_I3C_IsActiveFlag_HJ(i3c)) {
		int ret;

		LL_I3C_ClearFlag_HJ(i3c);

		ret = i3c_ibi_work_enqueue_hotjoin(dev);
		if (ret != 0) {
			LOG_ERR("IBI Failed to enqueue hotjoin work");
		}
	}
}
#endif /* CONFIG_I3C_USE_IBI */
#endif /* CONFIG_I3C_CONTROLLER */

/* Handles the I3C event ISR */
static void i3c_stm32_event_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;

	const struct i3c_stm32_config *config = dev->config;
	struct i3c_stm32_data *data = dev->data;
	I3C_TypeDef *i3c = config->i3c;
	/* If FC arrives while draining FIFOs, finish on the next interrupt. */
	bool frame_complete = LL_I3C_IsActiveFlag_FC(i3c) && LL_I3C_IsEnabledIT_FC(i3c);

	/* TX FIFO not full handler */
	if (LL_I3C_IsActiveFlag_TXFNF(i3c) && LL_I3C_IsEnabledIT_TXFNF(i3c)) {
		i3c_stm32_event_isr_tx(dev);
	}

	/* RX FIFO not empty handler */
	if (LL_I3C_IsActiveFlag_RXFNE(i3c) && LL_I3C_IsEnabledIT_RXFNE(i3c)) {
		i3c_stm32_event_isr_rx(dev);
	}

#ifdef CONFIG_I3C_CONTROLLER
	/* Control FIFO not full handler */
	if (LL_I3C_IsActiveFlag_CFNF(i3c) && LL_I3C_IsEnabledIT_CFNF(i3c)) {
		i3c_stm32_event_isr_cf(dev);
	}

	/* Status FIFO not empty handler */
	while (LL_I3C_IsActiveFlag_SFNE(i3c) && LL_I3C_IsEnabledIT_SFNE(i3c)) {

		if (data->msg_state == STM32_I3C_MSG) {
			size_t num_xfer = LL_I3C_GetXferDataCount(i3c);

			i3c_stm32_curr_msg_status_update_num_xfer(dev, num_xfer);
			i3c_stm32_curr_msg_status_next(dev);
		} else {
			/* Read and discard the status FIFO word since it will not be used */
			uint32_t status_reg = i3c->SR;

			ARG_UNUSED(status_reg);
		}
	}
	if (LL_I3C_IsActiveFlag_RXTGTEND(i3c)) {
		LL_I3C_ClearFlag_RXTGTEND(i3c);
	}
#endif /*CONFIG_I3C_CONTROLLER*/

	/* Frame complete handler */
	if (frame_complete) {
		LL_I3C_ClearFlag_FC(i3c);
#ifdef CONFIG_I3C_CONTROLLER
		if (ll_i3c_is_in_controller_mode(i3c)) {
			i3c_stm32_xfer_complete(dev,
				LL_I3C_IsActiveFlag_ERR(i3c) ? -EIO : 0);
		}
#endif /*CONFIG_I3C_CONTROLLER*/
		/* Mark bus as idle after each frame complete */
		data->msg_state = STM32_I3C_MSG_IDLE;

#ifdef CONFIG_I3C_TARGET
		if (!ll_i3c_is_in_controller_mode(i3c) &&
		    data->target_config != NULL &&
		    data->target_config->callbacks != NULL &&
		    data->target_config->callbacks->stop_cb != NULL) {
			data->target_config->callbacks->stop_cb(data->target_config);
		}
#endif /*CONFIG_I3C_TARGET*/
	}

#ifdef CONFIG_I3C_USE_IBI
#ifdef CONFIG_I3C_CONTROLLER
	if (ll_i3c_is_in_controller_mode(i3c)) {
		i3c_stm32_isr_controller_ibi(dev);
	}
#endif /*CONFIG_I3C_CONTROLLER*/
#endif /*CONFIG_I3C_USE_IBI*/

	if (LL_I3C_IsActiveFlag_WKP(i3c)) {
		LL_I3C_ClearFlag_WKP(i3c);
	}

#ifdef CONFIG_I3C_TARGET
	if (LL_I3C_IsActiveFlag_DAUPD(i3c)) {
		LL_I3C_ClearFlag_DAUPD(i3c);
	}
#endif /*CONFIG_I3C_TARGET*/
}

/* Handles the I3C error ISR */
static int i3c_stm32_error(void *arg)
{
	const struct device *dev = (const struct device *)arg;

	const struct i3c_stm32_config *config = dev->config;
	struct i3c_stm32_data *data = dev->data;
	I3C_TypeDef *i3c = config->i3c;

	if (!LL_I3C_IsActiveFlag_ERR(i3c) || !LL_I3C_IsEnabledIT_ERR(i3c)) {
		return 0;
	}

	i3c_stm32_log_err_type(dev);

	LL_I3C_ClearFlag_ERR(i3c);

	data->msg_state = STM32_I3C_MSG_ERR;

#ifdef CONFIG_I3C_CONTROLLER
	if (ll_i3c_is_in_controller_mode(i3c)) {
		i3c_stm32_xfer_complete(dev, -EIO);
	}
#endif

	return 1;
}

#ifdef CONFIG_I3C_STM32_COMBINED_INTERRUPT
static void i3c_stm32_combined_isr(void *arg)
{
	if (!i3c_stm32_error(arg)) {
		i3c_stm32_event_isr(arg);
	}
}
#else /* CONFIG_I3C_STM32_COMBINED_INTERRUPT */
static void i3c_stm32_error_isr(void *arg)
{
	(void)i3c_stm32_error(arg);
}
#endif /* CONFIG_I3C_STM32_COMBINED_INTERRUPT */

#ifdef CONFIG_I3C_USE_IBI
#ifdef CONFIG_I3C_CONTROLLER
int i3c_stm32_ibi_hj_response(const struct device *dev, bool ack)
{
	const struct i3c_stm32_config *config = dev->config;
	struct i3c_stm32_data *data = dev->data;
	I3C_TypeDef *i3c = config->i3c;
	int ret = 0;

	k_mutex_lock(&data->bus_mutex, K_FOREVER);
	if (ack == data->hj_pm_lock) {
		goto unlock;
	}

	if (ack) {
		ret = pm_device_runtime_get(dev);
		if (ret < 0) {
			goto unlock;
		}
		pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
		LL_I3C_EnableHJAck(i3c);
		data->hj_pm_lock = true;
	} else {
		LL_I3C_DisableHJAck(i3c);
		ret = pm_device_runtime_put(dev);
		if (ret < 0) {
			LL_I3C_EnableHJAck(i3c);
			goto unlock;
		}
		data->hj_pm_lock = false;
		pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	}
unlock:
	k_mutex_unlock(&data->bus_mutex);
	return ret;
}

int i3c_stm32_ibi_enable(const struct device *dev, struct i3c_device_desc *target)
{
	struct i3c_stm32_data *data = dev->data;
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;
	struct i3c_ccc_events events = { .events = I3C_CCC_EVT_INTR };
	size_t slot = ARRAY_SIZE(data->ibi.addr);
	int ret;

	if (!i3c_device_is_ibi_capable(target) || target->dynamic_addr == 0U) {
		return -EINVAL;
	}

	k_mutex_lock(&data->bus_mutex, K_FOREVER);
	for (size_t i = 0; i < ARRAY_SIZE(data->ibi.addr); i++) {
		if (data->ibi.addr[i] == target->dynamic_addr) {
			ret = -EALREADY;
			goto unlock;
		}
		if (data->ibi.addr[i] == 0U) {
			slot = i;
		}
	}
	if (slot == ARRAY_SIZE(data->ibi.addr)) {
		ret = -ENOMEM;
		goto unlock;
	}

	/* Each enabled target owns one reference while unsolicited IBIs are possible. */
	ret = pm_device_runtime_get(dev);
	if (ret < 0) {
		goto unlock;
	}
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	LL_I3C_ConfigDeviceCapabilities(i3c, slot + 1U, target->dynamic_addr,
		LL_I3C_IBI_CAPABILITY,
		i3c_ibi_has_payload(target) ? LL_I3C_IBI_DATA_ENABLE : LL_I3C_IBI_DATA_DISABLE,
		LL_I3C_CR_NO_CAPABILITY);

	ret = i3c_ccc_do_events_set(target, true, &events);
	if (ret != 0) {
		LL_I3C_ConfigDeviceCapabilities(i3c, slot + 1U, 0U, LL_I3C_IBI_NO_CAPABILITY,
			LL_I3C_IBI_DATA_DISABLE, LL_I3C_CR_NO_CAPABILITY);
		pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
		(void)pm_device_runtime_put(dev);
		goto unlock;
	}
	data->ibi.addr[slot] = target->dynamic_addr;
	data->ibi.num_addr++;
unlock:
	k_mutex_unlock(&data->bus_mutex);
	return ret;
}

int i3c_stm32_ibi_disable(const struct device *dev, struct i3c_device_desc *target)
{
	struct i3c_stm32_data *data = dev->data;
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;
	struct i3c_ccc_events events = { .events = I3C_CCC_EVT_INTR };
	size_t slot;
	int ret;

	k_mutex_lock(&data->bus_mutex, K_FOREVER);
	for (slot = 0; slot < ARRAY_SIZE(data->ibi.addr); slot++) {
		if (data->ibi.addr[slot] != 0U &&
		    data->ibi.addr[slot] == target->dynamic_addr) {
			break;
		}
	}
	if (slot == ARRAY_SIZE(data->ibi.addr)) {
		ret = -ENODEV;
		goto unlock;
	}

	/* Preserve the registration and its reference if the target rejects DISEC. */
	ret = i3c_ccc_do_events_set(target, false, &events);
	if (ret != 0) {
		goto unlock;
	}
	LL_I3C_ConfigDeviceCapabilities(i3c, slot + 1U, 0U, LL_I3C_IBI_NO_CAPABILITY,
		LL_I3C_IBI_DATA_DISABLE, LL_I3C_CR_NO_CAPABILITY);
	ret = pm_device_runtime_put(dev);
	if (ret < 0) {
		goto unlock;
	}
	data->ibi.addr[slot] = 0U;
	data->ibi.num_addr--;
	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
unlock:
	k_mutex_unlock(&data->bus_mutex);
	return ret;
}
#endif /* CONFIG_I3C_CONTROLLER*/

#endif /* CONFIG_I3C_USE_IBI */

#ifdef CONFIG_I3C_STM32_DMA
static void i3c_stm32_tx_rx_msg_config(const struct device *dma_dev, void *user_data,
				       uint32_t channel, int status)
{
	const struct device *dev = (const struct device *)user_data;

	if (i3c_stm32_curr_msg_xfer_next(dev) != 0) {
		/* No more messages to transmit/receive */
		return;
	}

	uint8_t *buf = NULL;
	size_t *offset = 0;
	uint32_t len = 0;

	i3c_stm32_curr_msg_xfer_get_buf(dev, &buf, &len, &offset);
	i3c_stm32_dma_msg_config(dev, (uint32_t)buf, len);
}

static void i3c_stm32_dma_tx_cb(const struct device *dma_dev, void *user_data, uint32_t channel,
				int status)
{
	i3c_stm32_tx_rx_msg_config(dma_dev, user_data, channel, status);
}

static void i3c_stm32_dma_rx_cb(const struct device *dma_dev, void *user_data, uint32_t channel,
				int status)
{
	i3c_stm32_tx_rx_msg_config(dma_dev, user_data, channel, status);
}

static void i3c_stm32_dma_tc_cb(const struct device *dma_dev, void *user_data, uint32_t channel,
				int status)
{
}

static void i3c_stm32_dma_rs_cb(const struct device *dma_dev, void *user_data, uint32_t channel,
				int status)
{
}

#endif
#ifdef CONFIG_I3C_TARGET
static int i3c_stm32_target_register(const struct device *dev, struct i3c_target_config *cfg)
{
	struct i3c_stm32_data *data = dev->data;

	data->target_config = cfg;
	return 0;
}

static int i3c_stm32_target_unregister(const struct device *dev, struct i3c_target_config *cfg)
{
	ARG_UNUSED(cfg);
	const struct i3c_stm32_config *config = dev->config;
	struct i3c_stm32_data *data = dev->data;
	I3C_TypeDef *i3c = config->i3c;

	data->target_config = NULL;
	LL_I3C_Disable(i3c);

	return 0;
}

static int i3c_stm32_target_tx_write(const struct device *dev, uint8_t *buf, uint16_t len,
				     uint8_t hdr_mode)
{
	struct i3c_stm32_data *data = dev->data;
	const struct i3c_stm32_config *config = dev->config;
	I3C_TypeDef *i3c = config->i3c;

	if (data->target_config->callbacks->read_processed_cb == NULL) {
		return -ENOSYS;
	}

	if (LL_I3C_IsActiveTxPreload(i3c)) {
		LOG_ERR("Target preload in progress");
		k_mutex_unlock(&data->bus_mutex);
		return -EBUSY;
	}

	LL_I3C_ConfigTxPreload(i3c, len);

	k_mutex_unlock(&data->bus_mutex);
	return 0; /* we process 0 bytes here, and let the interrupt drive all bytes through */
}

#endif /*CONFIG_I3C_TARGET*/

static DEVICE_API(i3c, i3c_stm32_driver_api) = {
#ifdef CONFIG_I3C_CONTROLLER
	.i2c_api.configure = i3c_stm32_i2c_configure,
	.i2c_api.transfer = i3c_stm32_i2c_transfer,

#ifdef CONFIG_I2C_RTIO
	.i2c_api.iodev_submit = i2c_iodev_submit_fallback,
#endif /*CONFIG_I2C_RTIO*/
#endif /*CONFIG_I3C_CONTROLLER*/
	.configure = i3c_stm32_configure,
	.config_get = i3c_stm32_config_get,
#ifdef CONFIG_I3C_CONTROLLER
	.i3c_device_find = i3c_stm32_device_find,
	.i3c_xfers = i3c_stm32_i3c_transfer,
	.do_daa = i3c_stm32_do_daa,
	.do_ccc = i3c_stm32_do_ccc,
#endif
#ifdef CONFIG_I3C_USE_IBI
#ifdef CONFIG_I3C_CONTROLLER
	.ibi_hj_response = i3c_stm32_ibi_hj_response,
	.ibi_enable = i3c_stm32_ibi_enable,
	.ibi_disable = i3c_stm32_ibi_disable,
#endif /*CONFIG_I3C_CONTROLLER*/
#endif /*CONFIG_I3C_USE_IBI*/
#ifdef CONFIG_I3C_RTIO
	.iodev_submit = i3c_iodev_submit_fallback,
#endif

#ifdef CONFIG_I3C_TARGET
	.target_tx_write = i3c_stm32_target_tx_write,
	.target_register = i3c_stm32_target_register,
	.target_unregister = i3c_stm32_target_unregister,
#endif /* CONFIG_I3C_TARGET */
};

#ifdef CONFIG_I3C_STM32_DMA
#define STM32_I3C_DMA_CHANNEL_INIT(index, dir, dir_cap, src_dev, dest_dev)                         \
	.dma_dev = DEVICE_DT_GET(STM32_DMA_CTLR(index, dir)),                                      \
	.dma_channel = DT_INST_DMAS_CELL_BY_NAME(index, dir, channel),                             \
	.dma_cfg = {                                                                               \
		.dma_slot = STM32_DMA_SLOT(index, dir, slot),                                      \
		.channel_direction =                                                               \
				STM32_DMA_CONFIG_DIRECTION(STM32_DMA_CHANNEL_CONFIG(index, dir)),  \
		.channel_priority =                                                                \
				STM32_DMA_CONFIG_PRIORITY(STM32_DMA_CHANNEL_CONFIG(index, dir)),   \
		.source_data_size = STM32_DMA_CONFIG_##src_dev##_DATA_SIZE(                        \
				STM32_DMA_CHANNEL_CONFIG(index, dir)),                             \
		.dest_data_size = STM32_DMA_CONFIG_##dest_dev##_DATA_SIZE(                         \
				STM32_DMA_CHANNEL_CONFIG(index, dir)),                             \
		/* single transfers (burst length = data size) */                                  \
		.source_burst_length = STM32_DMA_CONFIG_##src_dev##_DATA_SIZE(                     \
				STM32_DMA_CHANNEL_CONFIG(index, dir)),                             \
		.dest_burst_length = STM32_DMA_CONFIG_##dest_dev##_DATA_SIZE(                      \
				STM32_DMA_CHANNEL_CONFIG(index, dir)),                             \
		.block_count = 1,                                                                  \
		.dma_callback = i3c_stm32_dma_##dir##_cb,                                          \
	},                                                                                         \
	.src_addr_increment =                                                                      \
		STM32_DMA_CONFIG_##src_dev##_ADDR_INC(STM32_DMA_CHANNEL_CONFIG(index, dir)),       \
	.dst_addr_increment =                                                                      \
		STM32_DMA_CONFIG_##dest_dev##_ADDR_INC(STM32_DMA_CHANNEL_CONFIG(index, dir)),      \
	.fifo_threshold = STM32_DMA_FEATURES_FIFO_THRESHOLD(STM32_DMA_FEATURES(index, dir)),
#endif

#ifdef CONFIG_I3C_STM32_DMA
#define STM32_I3C_DMA_CHANNEL(index, dir, DIR, src, dest)                                          \
	.dma_##dir = {                                                                             \
		COND_CODE_1(DT_INST_DMAS_HAS_NAME(index, dir),                                     \
			    (STM32_I3C_DMA_CHANNEL_INIT(index, dir, DIR, src, dest)),              \
			    (NULL))                                                                \
	},
#else
#define STM32_I3C_DMA_CHANNEL(index, dir, DIR, src, dest)
#endif

#ifdef CONFIG_I3C_STM32_COMBINED_INTERRUPT
#define STM32_I3C_IRQ_CONNECT_AND_ENABLE(index)                                                    \
	do {                                                                                       \
		IRQ_CONNECT(DT_INST_IRQN(index), DT_INST_IRQ(index, priority),                     \
			    i3c_stm32_combined_isr, DEVICE_DT_INST_GET(index), 0);                 \
		irq_enable(DT_INST_IRQN(index));                                                   \
	} while (false)
#else  /* CONFIG_I3C_STM32_COMBINED_INTERRUPT */
#define STM32_I3C_IRQ_CONNECT_AND_ENABLE(index)                                                    \
	do {                                                                                       \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(index, event, irq),                                \
			    DT_INST_IRQ_BY_NAME(index, event, priority), i3c_stm32_event_isr,      \
			    DEVICE_DT_INST_GET(index), 0);                                         \
		irq_enable(DT_INST_IRQ_BY_NAME(index, event, irq));                                \
                                                                                                   \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(index, error, irq),                                \
			    DT_INST_IRQ_BY_NAME(index, error, priority), i3c_stm32_error_isr,      \
			    DEVICE_DT_INST_GET(index), 0);                                         \
		irq_enable(DT_INST_IRQ_BY_NAME(index, error, irq));                                \
	} while (false)
#endif /* CONFIG_I3C_STM32_COMBINED_INTERRUPT */

#define STM32_I3C_IRQ_HANDLER_DECL(index)                                                          \
	static void i3c_stm32_irq_config_func_##index(const struct device *dev)

#define STM32_I3C_IRQ_HANDLER(index)                                                               \
	static void i3c_stm32_irq_config_func_##index(const struct device *dev)                    \
	{                                                                                          \
		STM32_I3C_IRQ_CONNECT_AND_ENABLE(index);                                           \
	}

#define I3C_STM32_INIT(index)                                                                      \
	STM32_I3C_IRQ_HANDLER_DECL(index);                                                         \
                                                                                                   \
	static const struct stm32_pclken pclken_##index[] = STM32_DT_INST_CLOCKS(index);           \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
	IF_ENABLED(UTIL_AND(IS_ENABLED(CONFIG_I3C_CONTROLLER),                                     \
			UTIL_NOT(DT_INST_PROP(index, target_mode))), (                             \
	static struct i3c_device_desc i3c_stm32_dev_arr_##index[] =                                \
		I3C_DEVICE_ARRAY_DT_INST(index);                                                   \
	static struct i3c_i2c_device_desc i3c_i2c_stm32_dev_arr_##index[] =                        \
		I3C_I2C_DEVICE_ARRAY_DT_INST(index);))                                             \
                                                                                                   \
	static const struct i3c_stm32_config i3c_stm32_cfg_##index = {                             \
		.i3c = (I3C_TypeDef *)DT_INST_REG_ADDR(index),                                     \
		.irq_config_func = i3c_stm32_irq_config_func_##index,                              \
		.pclken = pclken_##index,                                                          \
		.pclk_len = DT_INST_NUM_CLOCKS(index),                                             \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                     \
		.reset = RESET_DT_SPEC_INST_GET(index),                                           \
		IF_ENABLED(UTIL_AND(IS_ENABLED(CONFIG_I3C_TARGET),                                 \
				    DT_INST_PROP(index, target_mode)), (                           \
		.target_mode = DT_INST_PROP(index, target_mode),                                   \
		.mipi_instance = DT_INST_PROP_OR(index, mipi_instance, 0),                         \
		.dcr = DT_INST_PROP_OR(index, dcr, 0xC6),                                          \
		.mrl = DT_INST_PROP_OR(index, mrl, 0),                                             \
		.mwl = DT_INST_PROP_OR(index, mwl, 0),                                             \
		))                                                                                 \
		IF_ENABLED(UTIL_AND(IS_ENABLED(CONFIG_I3C_CONTROLLER),                             \
				    UTIL_NOT(DT_INST_PROP(index, target_mode))), (                 \
		.drv_cfg.dev_list.i3c = i3c_stm32_dev_arr_##index,                                 \
		.drv_cfg.dev_list.num_i3c = ARRAY_SIZE(i3c_stm32_dev_arr_##index),                 \
		.drv_cfg.dev_list.i2c = i3c_i2c_stm32_dev_arr_##index,                             \
		.drv_cfg.dev_list.num_i2c = ARRAY_SIZE(i3c_i2c_stm32_dev_arr_##index),             \
		.drv_cfg.flags = I3C_CONTROLLER_CONFIG_FLAGS_DT_INST(index),                       \
		))                                                                                 \
	};                                                                                         \
                                                                                                   \
	static struct i3c_stm32_data i3c_stm32_data_##index = {                                    \
		.drv_data.ctrl_config.scl.i2c = DT_INST_PROP_OR(index, i2c_scl_hz, 0),             \
		.drv_data.ctrl_config.scl.i3c = DT_INST_PROP_OR(index, i3c_scl_hz, 0),             \
		.drv_data.ctrl_config.scl_od_min.high_ns = DT_INST_PROP(index, od_thigh_min_ns),   \
		.drv_data.ctrl_config.scl_od_min.low_ns = DT_INST_PROP(index, od_tlow_min_ns),     \
		STM32_I3C_DMA_CHANNEL(index, rx, RX, PERIPHERAL, MEMORY)                           \
		STM32_I3C_DMA_CHANNEL(index, tx, TX, MEMORY, PERIPHERAL)                           \
		STM32_I3C_DMA_CHANNEL(index, tc, TC, MEMORY, PERIPHERAL)                           \
		STM32_I3C_DMA_CHANNEL(index, rs, RS, PERIPHERAL, MEMORY)};                         \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(index, i3c_stm32_pm_action);                                      \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, &i3c_stm32_init, PM_DEVICE_DT_INST_GET(index),                \
			      &i3c_stm32_data_##index, &i3c_stm32_cfg_##index, POST_KERNEL,        \
			      CONFIG_I3C_CONTROLLER_INIT_PRIORITY, &i3c_stm32_driver_api);         \
                                                                                                   \
	STM32_I3C_IRQ_HANDLER(index)

DT_INST_FOREACH_STATUS_OKAY(I3C_STM32_INIT)
