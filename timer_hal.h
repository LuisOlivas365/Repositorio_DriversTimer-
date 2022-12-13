// The HAL layer for Timer Group.
// There is no parameter check in the hal layer, so the caller must ensure the correctness of the parameters.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "soc/soc_caps.h"
#include "hal/timer_ll.h"
#include "hal/timer_types.h"
#include "esp_attr.h"
/**
 * Context that should be maintained by both the driver and the HAL
 */
typedef struct {
    timg_dev_t *dev;
    timer_idx_t idx;
} timer_hal_context_t;

/**
 * @brief Init the timer hal. This function should be called first before other hal layer function is called
 *
 * @param hal Context of the HAL layer
 * @param group_num The timer group number
 * @param timer_num The timer number
 *
 * @return None
 */
void thal_init(timer_hal_context_t *hal, timer_group_t group_num, timer_idx_t timer_num);

/**
 * @brief Get interrupt status register address and corresponding control bits mask
 *
 * @param hal Context of the HAL layer
 * @param status_reg[out] interrupt status register address
 * @param mask_bit[out] control bits mask
 */
void timer_hal_get_status_reg_mask_bit(timer_hal_context_t *hal, uint32_t *status_reg, uint32_t *mask_bit);

/**
 * @brief Reset timer peripheral
 *
 * @param hal Context of the HAL layer
 *
 * @return None
 */
void timer_hal_reset_periph(timer_hal_context_t *hal);

/**
 * @brief Set timer clock prescale value
 *
 * @param hal Context of the HAL layer
 * @param divider Prescale value
 *
 * @return None
 */
#define th_set_divider(hal, divider)  timer_ll_set_divider((hal)->dev, (hal)->idx, divider)

/**
 * @brief Get timer clock prescale value
 *
 * @param hal Context of the HAL layer
 * @param divider Pointer to accept the prescale value
 *
 * @return None
 */
#define th_get_divider(hal, divider)  timer_ll_get_divider((hal)->dev, (hal)->idx, divider)

/**
 * @brief Load counter value into time-base counter
 *
 * @param hal Context of the HAL layer
 * @param load_val Counter value
 *
 * @return None
 */
#define th_set_counter_value(hal, load_val)  timer_ll_set_counter_value((hal)->dev, (hal)->idx, load_val)

/**
 * @brief Get counter value from time-base counter
 *
 * @param hal Context of the HAL layer
 * @param timer_val Pointer to accept the counter value
 *
 * @return None
 */
#define th_get_counter_value(hal, timer_val)  timer_ll_get_counter_value((hal)->dev, (hal)->idx, timer_val)

/**
 * @brief Set counter mode, include increment mode and decrement mode.
 *
 * @param hal Context of the HAL layer
 * @param increase_en True to increment mode, fasle to decrement mode
 *
 * @return None
 */
#define th_set_counter_increase(hal, increase_en)  timer_ll_set_counter_increase((hal)->dev, (hal)->idx, increase_en)

/**
 * @brief Get counter mode, include increment mode and decrement mode.
 *
 * @param hal Context of the HAL layer
 * @param counter_dir Pointer to accept the counter mode
 *
 * @return
 *     - true Increment mode
 *     - false Decrement mode
 */
#define th_get_counter_increase(hal)  timer_ll_get_counter_increase((hal)->dev, (hal)->idx)

/**
 * @brief Set counter status, enable or disable counter.
 *
 * @param hal Context of the HAL layer
 * @param counter_en True to enable counter, false to disable counter
 *
 * @return None
 */
#define th_set_counter_enable(hal, counter_en)  timer_ll_set_counter_enable((hal)->dev, (hal)->idx, counter_en)

/**
 * @brief Get counter status.
 *
 * @param hal Context of the HAL layer
 *
 * @return
 *     - true Enable counter
 *     - false Disable conuter
 */
#define th_get_counter_enable(hal)  timer_ll_get_counter_enable((hal)->dev, (hal)->idx)

/**
 * @brief Set auto reload mode.
 *
 * @param hal Context of the HAL layer
 * @param auto_reload_en True to enable auto reload mode, flase to disable auto reload mode
 *
 * @return None
 */
#define th_set_auto_reload(hal, auto_reload_en)  timer_ll_set_auto_reload((hal)->dev, (hal)->idx, auto_reload_en)

/**
 * @brief Get auto reload mode.
 *
 * @param hal Context of the HAL layer
 *
 * @return
 *     - true Enable auto reload mode
 *     - false Disable auto reload mode
 */
#define th_get_auto_reload(hal)  timer_ll_get_auto_reload((hal)->dev, (hal)->idx)

/**
 * @brief Set the counter value to trigger the alarm.
 *
 * @param hal Context of the HAL layer
 * @param alarm_value Counter value to trigger the alarm
 *
 * @return None
 */
#define th_set_alarm_value(hal, alarm_value)  timer_ll_set_alarm_value((hal)->dev, (hal)->idx, alarm_value)

/**
 * @brief Get the counter value to trigger the alarm.
 *
 * @param hal Context of the HAL layer
 * @param alarm_value Pointer to accept the counter value to trigger the alarm
 *
 * @return None
 */
#define th_get_alarm_value(hal, alarm_value)  timer_ll_get_alarm_value((hal)->dev, (hal)->idx, alarm_value)

/**
 * @brief Set the alarm status, enable or disable the alarm.
 *
 * @param hal Context of the HAL layer
 * @param alarm_en True to enable alarm, false to disable alarm
 *
 * @return None
 */
#define th_set_alarm_enable(hal, alarm_en)  timer_ll_set_alarm_enable((hal)->dev, (hal)->idx, alarm_en)

/**
 * @brief Get the alarm status.
 *
 * @param hal Context of the HAL layer
 *
 * @return
 *     - true Enable alarm
 *     - false Disable alarm
 */
#define th_get_alarm_enable(hal)  timer_ll_get_alarm_enable((hal)->dev, (hal)->idx)

/**
 * @brief Set the level interrupt status, enable or disable the level interrupt.
 *
 * @param hal Context of the HAL layer
 * @param level_int_en True to enable level interrupt, false to disable level interrupt
 *
 * @return None
 */
#define th_set_level_int_enable(hal, level_int_en)  timer_ll_set_level_int_enable((hal)->dev, (hal)->idx, level_int_en)

/**
 * @brief Get the level interrupt status.
 *
 * @param hal Context of the HAL layer
 *
 * @return
 *     - true Enable level interrupt
 *     - false Disable level interrupt
 */
#define th_get_level_int_enable(hal)  timer_ll_get_level_int_enable((hal)->dev, (hal)->idx)

/**
 * @brief Set the edge interrupt status, enable or disable the edge interrupt.
 *
 * @param hal Context of the HAL layer
 * @param edge_int_en True to enable edge interrupt, false to disable edge interrupt
 *
 * @return None
 */
#define th_set_edge_int_enable(hal, edge_int_en)  timer_ll_set_edge_int_enable((hal)->dev, (hal)->idx, edge_int_en)

/**
 * @brief Get the edge interrupt status.
 *
 * @param hal Context of the HAL layer
 *
 * @return
 *     - true Enable edge interrupt
 *     - false Disable edge interrupt
 */
#define th_get_edge_int_enable(hal)  timer_ll_get_edge_int_enable((hal)->dev, (hal)->idx)

/**
 * @brief Enable timer interrupt.
 *
 * @param hal Context of the HAL layer
 *
 * @return None
 */
#define th_intr_enable(hal)  timer_ll_intr_enable((hal)->dev, (hal)->idx)

/**
 * @brief Disable timer interrupt.
 *
 * @param hal Context of the HAL layer
 *
 * @return None
 */
#define th_intr_disable(hal)  timer_ll_intr_disable((hal)->dev, (hal)->idx)

/**
 * @brief Clear interrupt status.
 *
 * @param hal Context of the HAL layer
 *
 * @return None
 */
#define th_clear_intr_status(hal)  timer_ll_clear_intr_status((hal)->dev, (hal)->idx)

/**
 * @brief Get interrupt status.
 *
 * @param hal Context of the HAL layer
 * @param intr_status Interrupt status
 *
 * @return None
 */
#define th_get_intr_status(hal, intr_status)  timer_ll_get_intr_status((hal)->dev, intr_status)

/**
 * @brief Get interrupt raw status.
 *
 * @param group_num Timer group number, 0 for TIMERG0 or 1 for TIMERG1
 * @param intr_raw_status Interrupt raw status
 *
 * @return None
 */
#define th_get_intr_raw_status(group_num, intr_raw_status)  timer_ll_get_intr_raw_status(group_num, intr_raw_status)

/**
 * @brief Get interrupt status register address.
 *
 * @param hal Context of the HAL layer
 *
 * @return Interrupt status register address
 */
#define timer_hal_get_intr_status_reg(hal)  timer_ll_get_intr_status_reg((hal)->dev)

#if SOC_TIMER_GROUP_SUPPORT_XTAL
/**
 * @brief Set clock source.
 *
 * @param hal Context of the HAL layer
 * @param use_xtal_en True to use XTAL clock, flase to use APB clock
 *
 * @return None
 */
#define timer_hal_set_use_xtal(hal, use_xtal_en)  timer_ll_set_use_xtal((hal)->dev, (hal)->idx, use_xtal_en)

/**
 * @brief Get clock source.
 *
 * @param hal Context of the HAL layer
 *
 * @return
 *     - true Use XTAL clock
 *     - false Use APB clock
 */
#define timer_hal_get_use_xtal(hal)  timer_ll_get_use_xtal((hal)->dev, (hal)->idx)

void thal_init(timer_hal_context_t *hal, timer_group_t group_num, timer_idx_t timer_num)
{
    hal->dev = TIMER_LL_GET_HW(group_num);
    hal->idx = timer_num;
}

void timer_hal_get_status_reg_mask_bit(timer_hal_context_t *hal, uint32_t *status_reg, uint32_t *mask_bit)
{
    *status_reg = timer_ll_get_intr_status_reg(hal->dev);
    *mask_bit = timer_ll_get_intr_mask_bit(hal->dev, hal->idx);
}

void timer_hal_reset_periph(timer_hal_context_t *hal)
{
    timer_ll_intr_disable(hal->dev, hal->idx);
    timer_ll_set_counter_enable(hal->dev, hal->idx, TIMER_PAUSE);
    timer_ll_set_counter_value(hal->dev, hal->idx, 0ULL);
}

#endif

#ifdef __cplusplus
}
#endif

