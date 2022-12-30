
#include<stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"
#include "driver/periph_ctrl.h"
#include "hal/timer_hal.h"
#include "soc/timer_periph.h"
#include "soc/rtc.h"
#include <stdbool.h>


#define TIMER_GROUP_NUM_ERROR   "TIMER GROUP NUM ERROR"
#define TIMER_NUM_ERROR         "HW TIMER NUM ERROR"
#define TIMER_PARAM_ADDR_ERROR  "HW TIMER PARAM ADDR ERROR"
#define TIMER_NEVER_INIT_ERROR  "HW TIMER NEVER INIT ERROR"
#define TIMER_COUNT_DIR_ERROR   "HW TIMER COUNTER DIR ERROR"
#define TIMER_AUTORELOAD_ERROR  "HW TIMER AUTORELOAD ERROR"
#define TIMER_SCALE_ERROR       "HW TIMER SCALE ERROR"
#define TIMER_ALARM_ERROR       "HW TIMER ALARM ERROR"
#define DIVIDER_RANGE_ERROR     "HW TIMER divider outside of [2, 65536] range error"

#define TIMER_ENTER_CRITICAL(mux)      portENTER_CRITICAL_SAFE(mux);
#define TIMER_EXIT_CRITICAL(mux)       portEXIT_CRITICAL_SAFE(mux);
#define TIMER_DIVIDER         (16)  //  Hardware timer clock divider
#define TIMER_SCALE           (( 80*1000000 ) / TIMER_DIVIDER)  // convert counter value to seconds

static const char *TIMER_TAG = "timer_group";


typedef struct {
    timer_isr_t fn;  /*!< isr function */
    void *args;      /*!< isr function args */
    timer_isr_handle_t timer_isr_handle;  /*!< interrupt handle */
    timer_group_t isr_timer_group;        /*!< timer group of interrupt triggered */
} timer_isr_func_t;

typedef struct {
    timer_hal_context_t hal;
    timer_isr_func_t timer_isr_fun;
} timer_obj_t;

static timer_obj_t *p_timer_obj[TIMER_GROUP_MAX][TIMER_MAX] = {0};
static portMUX_TYPE timer_spinlock[TIMER_GROUP_MAX] = {portMUX_INITIALIZER_UNLOCKED, portMUX_INITIALIZER_UNLOCKED};

esp_err_t tdGet_counter_val(timer_group_t group_num, timer_idx_t timer_num, uint64_t *timer_val)
{
    ESP_RETURN_ON_FALSE(group_num < TIMER_GROUP_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_GROUP_NUM_ERROR);
    ESP_RETURN_ON_FALSE(timer_num < TIMER_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NUM_ERROR);
    ESP_RETURN_ON_FALSE(timer_val != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_PARAM_ADDR_ERROR);
    ESP_RETURN_ON_FALSE(p_timer_obj[group_num][timer_num] != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NEVER_INIT_ERROR);
    TIMER_ENTER_CRITICAL(&timer_spinlock[group_num]);
    th_get_counter_value(&(p_timer_obj[group_num][timer_num]->hal), timer_val);
    TIMER_EXIT_CRITICAL(&timer_spinlock[group_num]);
    return ESP_OK;
}

esp_err_t tdGet_counter_time_sec(timer_group_t group_num, timer_idx_t timer_num, double *time)
{
    ESP_RETURN_ON_FALSE(group_num < TIMER_GROUP_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_GROUP_NUM_ERROR);
    ESP_RETURN_ON_FALSE(timer_num < TIMER_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NUM_ERROR);
    ESP_RETURN_ON_FALSE(time != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_PARAM_ADDR_ERROR);
    ESP_RETURN_ON_FALSE(p_timer_obj[group_num][timer_num] != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NEVER_INIT_ERROR);
    uint64_t timer_val;
    esp_err_t err = tdGet_counter_val(group_num, timer_num, &timer_val);
    if (err == ESP_OK) {
        uint32_t div;
        th_get_divider(&(p_timer_obj[group_num][timer_num]->hal), &div);
        *time = (double)timer_val * div / rtc_clk_apb_freq_get();
#if SOC_TIMER_GROUP_SUPPORT_XTAL
        if (timer_hal_get_use_xtal(&(p_timer_obj[group_num][timer_num]->hal))) {
            *time = (double)timer_val * div / ((int)rtc_clk_xtal_freq_get() * 1000000);
        }
#endif
    }
    return err;
}

esp_err_t tdSet_counter_val(timer_group_t group_num, timer_idx_t timer_num, uint64_t load_val)
{
    ESP_RETURN_ON_FALSE(group_num < TIMER_GROUP_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_GROUP_NUM_ERROR);
    ESP_RETURN_ON_FALSE(timer_num < TIMER_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NUM_ERROR);
    ESP_RETURN_ON_FALSE(p_timer_obj[group_num][timer_num] != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NEVER_INIT_ERROR);
    TIMER_ENTER_CRITICAL(&timer_spinlock[group_num]);
    th_set_counter_value(&(p_timer_obj[group_num][timer_num]->hal), load_val);
    TIMER_EXIT_CRITICAL(&timer_spinlock[group_num]);
    return ESP_OK;
}

esp_err_t td_start(timer_group_t group_num, timer_idx_t timer_num)
{
    ESP_RETURN_ON_FALSE(group_num < TIMER_GROUP_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_GROUP_NUM_ERROR);
    ESP_RETURN_ON_FALSE(timer_num < TIMER_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NUM_ERROR);
    ESP_RETURN_ON_FALSE(p_timer_obj[group_num][timer_num] != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NEVER_INIT_ERROR);
    TIMER_ENTER_CRITICAL(&timer_spinlock[group_num]);
    th_set_counter_enable(&(p_timer_obj[group_num][timer_num]->hal), TIMER_START);
    TIMER_EXIT_CRITICAL(&timer_spinlock[group_num]);
    return ESP_OK;
}

esp_err_t td_pause(timer_group_t group_num, timer_idx_t timer_num)
{
    ESP_RETURN_ON_FALSE(group_num < TIMER_GROUP_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_GROUP_NUM_ERROR);
    ESP_RETURN_ON_FALSE(timer_num < TIMER_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NUM_ERROR);
    ESP_RETURN_ON_FALSE(p_timer_obj[group_num][timer_num] != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NEVER_INIT_ERROR);
    TIMER_ENTER_CRITICAL(&timer_spinlock[group_num]);
    th_set_counter_enable(&(p_timer_obj[group_num][timer_num]->hal), TIMER_PAUSE);
    TIMER_EXIT_CRITICAL(&timer_spinlock[group_num]);
    return ESP_OK;
}

esp_err_t tdSet_counter_mode(timer_group_t group_num, timer_idx_t timer_num, timer_count_dir_t counter_dir)
{
    ESP_RETURN_ON_FALSE(group_num < TIMER_GROUP_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_GROUP_NUM_ERROR);
    ESP_RETURN_ON_FALSE(timer_num < TIMER_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NUM_ERROR);
    ESP_RETURN_ON_FALSE(counter_dir < TIMER_COUNT_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_COUNT_DIR_ERROR);
    ESP_RETURN_ON_FALSE(p_timer_obj[group_num][timer_num] != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NEVER_INIT_ERROR);
    TIMER_ENTER_CRITICAL(&timer_spinlock[group_num]);
    th_set_counter_increase(&(p_timer_obj[group_num][timer_num]->hal), counter_dir);
    TIMER_EXIT_CRITICAL(&timer_spinlock[group_num]);
    return ESP_OK;
}

esp_err_t tdSet_auto_reload(timer_group_t group_num, timer_idx_t timer_num, timer_autoreload_t reload)
{
    ESP_RETURN_ON_FALSE(group_num < TIMER_GROUP_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_GROUP_NUM_ERROR);
    ESP_RETURN_ON_FALSE(timer_num < TIMER_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NUM_ERROR);
    ESP_RETURN_ON_FALSE(reload < TIMER_AUTORELOAD_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_AUTORELOAD_ERROR);
    ESP_RETURN_ON_FALSE(p_timer_obj[group_num][timer_num] != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NEVER_INIT_ERROR);
    TIMER_ENTER_CRITICAL(&timer_spinlock[group_num]);
    th_set_auto_reload(&(p_timer_obj[group_num][timer_num]->hal), reload);
    TIMER_EXIT_CRITICAL(&timer_spinlock[group_num]);
    return ESP_OK;
}

esp_err_t tdSet_divider(timer_group_t group_num, timer_idx_t timer_num, uint32_t divider)
{
    ESP_RETURN_ON_FALSE(group_num < TIMER_GROUP_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_GROUP_NUM_ERROR);
    ESP_RETURN_ON_FALSE(timer_num < TIMER_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NUM_ERROR);
    ESP_RETURN_ON_FALSE(divider > 1 && divider < 65537, ESP_ERR_INVALID_ARG, TIMER_TAG,  DIVIDER_RANGE_ERROR);
    ESP_RETURN_ON_FALSE(p_timer_obj[group_num][timer_num] != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NEVER_INIT_ERROR);
    TIMER_ENTER_CRITICAL(&timer_spinlock[group_num]);
    th_set_divider(&(p_timer_obj[group_num][timer_num]->hal), divider);
    TIMER_EXIT_CRITICAL(&timer_spinlock[group_num]);
    return ESP_OK;
}

esp_err_t tdSet_alarm_value(timer_group_t group_num, timer_idx_t timer_num, uint64_t alarm_value)
{
    ESP_RETURN_ON_FALSE(group_num < TIMER_GROUP_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_GROUP_NUM_ERROR);
    ESP_RETURN_ON_FALSE(timer_num < TIMER_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NUM_ERROR);
    ESP_RETURN_ON_FALSE(p_timer_obj[group_num][timer_num] != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NEVER_INIT_ERROR);
    TIMER_ENTER_CRITICAL(&timer_spinlock[group_num]);
    th_set_alarm_value(&(p_timer_obj[group_num][timer_num]->hal), alarm_value);
    TIMER_EXIT_CRITICAL(&timer_spinlock[group_num]);
    return ESP_OK;
}

esp_err_t tdGet_alarm_value(timer_group_t group_num, timer_idx_t timer_num, uint64_t *alarm_value)
{
    ESP_RETURN_ON_FALSE(group_num < TIMER_GROUP_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_GROUP_NUM_ERROR);
    ESP_RETURN_ON_FALSE(timer_num < TIMER_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NUM_ERROR);
    ESP_RETURN_ON_FALSE(alarm_value != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_PARAM_ADDR_ERROR);
    ESP_RETURN_ON_FALSE(p_timer_obj[group_num][timer_num] != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NEVER_INIT_ERROR);
    TIMER_ENTER_CRITICAL(&timer_spinlock[group_num]);
    th_get_alarm_value(&(p_timer_obj[group_num][timer_num]->hal), alarm_value);
    TIMER_EXIT_CRITICAL(&timer_spinlock[group_num]);
    return ESP_OK;
}

esp_err_t tdSet_alarm(timer_group_t group_num, timer_idx_t timer_num, timer_alarm_t alarm_en)
{
    ESP_RETURN_ON_FALSE(group_num < TIMER_GROUP_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_GROUP_NUM_ERROR);
    ESP_RETURN_ON_FALSE(timer_num < TIMER_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NUM_ERROR);
    ESP_RETURN_ON_FALSE(alarm_en < TIMER_ALARM_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_ALARM_ERROR);
    ESP_RETURN_ON_FALSE(p_timer_obj[group_num][timer_num] != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NEVER_INIT_ERROR);
    TIMER_ENTER_CRITICAL(&timer_spinlock[group_num]);
    th_set_alarm_enable(&(p_timer_obj[group_num][timer_num]->hal), alarm_en);
    TIMER_EXIT_CRITICAL(&timer_spinlock[group_num]);
    return ESP_OK;
}

static void IRAM_ATTR timer_isr_default(void *arg)
{
    bool is_awoken = false;

    timer_obj_t *timer_obj = (timer_obj_t *)arg;
    if (timer_obj == NULL) {
        return;
    }
    if (timer_obj->timer_isr_fun.fn == NULL) {
        return;
    }

    TIMER_ENTER_CRITICAL(&timer_spinlock[timer_obj->timer_isr_fun.isr_timer_group]);
    {
        uint32_t intr_status = 0;
        th_get_intr_status(&(timer_obj->hal), &intr_status);
        if (intr_status & BIT(timer_obj->hal.idx)) {
            // Clear intrrupt status
            th_clear_intr_status(&(timer_obj->hal));
            uint64_t old_alarm_value = 0;
            th_get_alarm_value(&(timer_obj->hal), &old_alarm_value);
            // call user registered callback
            is_awoken = timer_obj->timer_isr_fun.fn(timer_obj->timer_isr_fun.args);
            // reenable alarm if required
            uint64_t new_alarm_value = 0;
            th_get_alarm_value(&(timer_obj->hal), &new_alarm_value);
            bool reenable_alarm = (new_alarm_value != old_alarm_value) || th_get_auto_reload(&timer_obj->hal);
            th_set_alarm_enable(&(timer_obj->hal), reenable_alarm);
        }
    }
    TIMER_EXIT_CRITICAL(&timer_spinlock[timer_obj->timer_isr_fun.isr_timer_group]);

    if (is_awoken) {
        portYIELD_FROM_ISR();
    }
}

esp_err_t td_isr_callback_add(timer_group_t group_num, timer_idx_t timer_num, timer_isr_t isr_handler, void *args, int intr_alloc_flags)
{
    ESP_RETURN_ON_FALSE(group_num < TIMER_GROUP_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_GROUP_NUM_ERROR);
    ESP_RETURN_ON_FALSE(timer_num < TIMER_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NUM_ERROR);
    ESP_RETURN_ON_FALSE(p_timer_obj[group_num][timer_num] != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NEVER_INIT_ERROR);
    esp_err_t ret = ESP_OK;

    tdDisable_intr(group_num, timer_num);
    p_timer_obj[group_num][timer_num]->timer_isr_fun.fn = isr_handler;
    p_timer_obj[group_num][timer_num]->timer_isr_fun.args = args;
    p_timer_obj[group_num][timer_num]->timer_isr_fun.isr_timer_group = group_num;
    ret = td_isr_register(group_num, timer_num, timer_isr_default, (void *)p_timer_obj[group_num][timer_num],
                             intr_alloc_flags, &(p_timer_obj[group_num][timer_num]->timer_isr_fun.timer_isr_handle));
    ESP_RETURN_ON_ERROR(ret, TIMER_TAG, "register interrupt service failed");
    tdEnable_intr(group_num, timer_num);

    return ret;
}

esp_err_t td_isr_callback_remove(timer_group_t group_num, timer_idx_t timer_num)
{
    ESP_RETURN_ON_FALSE(group_num < TIMER_GROUP_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_GROUP_NUM_ERROR);
    ESP_RETURN_ON_FALSE(timer_num < TIMER_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NUM_ERROR);
    ESP_RETURN_ON_FALSE(p_timer_obj[group_num][timer_num] != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NEVER_INIT_ERROR);

    tdDisable_intr(group_num, timer_num);
    p_timer_obj[group_num][timer_num]->timer_isr_fun.fn = NULL;
    p_timer_obj[group_num][timer_num]->timer_isr_fun.args = NULL;
    esp_intr_free(p_timer_obj[group_num][timer_num]->timer_isr_fun.timer_isr_handle);

    return ESP_OK;
}

esp_err_t td_isr_register(timer_group_t group_num, timer_idx_t timer_num,void (*fn)(void *), void *arg, int intr_alloc_flags, timer_isr_handle_t *handle)
{
    ESP_RETURN_ON_FALSE(group_num < TIMER_GROUP_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_GROUP_NUM_ERROR);
    ESP_RETURN_ON_FALSE(timer_num < TIMER_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NUM_ERROR);
    ESP_RETURN_ON_FALSE(fn != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_PARAM_ADDR_ERROR);
    ESP_RETURN_ON_FALSE(p_timer_obj[group_num][timer_num] != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NEVER_INIT_ERROR);

    uint32_t status_reg = 0;
    uint32_t mask = 0;
    timer_hal_get_status_reg_mask_bit(&(p_timer_obj[group_num][timer_num]->hal), &status_reg, &mask);
    return esp_intr_alloc_intrstatus(timer_group_periph_signals.groups[group_num].t0_irq_id + timer_num, intr_alloc_flags, status_reg, mask, fn, arg, handle);
}

esp_err_t td_init(timer_group_t group_num, timer_idx_t timer_num, const timer_config_t *config)
{
    ESP_RETURN_ON_FALSE(group_num < TIMER_GROUP_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_GROUP_NUM_ERROR);
    ESP_RETURN_ON_FALSE(timer_num < TIMER_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NUM_ERROR);
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_PARAM_ADDR_ERROR);
    ESP_RETURN_ON_FALSE(config->divider > 1 && config->divider < 65537, ESP_ERR_INVALID_ARG, TIMER_TAG,  DIVIDER_RANGE_ERROR);

    periph_module_enable(timer_group_periph_signals.groups[group_num].module);

    if (p_timer_obj[group_num][timer_num] == NULL) {
        p_timer_obj[group_num][timer_num] = (timer_obj_t *) heap_caps_calloc(1, sizeof(timer_obj_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (p_timer_obj[group_num][timer_num] == NULL) {
            ESP_LOGE(TIMER_TAG, "TIMER driver malloc error");
            return ESP_FAIL;
        }
    }

    TIMER_ENTER_CRITICAL(&timer_spinlock[group_num]);
    thal_init(&(p_timer_obj[group_num][timer_num]->hal), group_num, timer_num);
    timer_hal_reset_periph(&(p_timer_obj[group_num][timer_num]->hal));
    th_clear_intr_status(&(p_timer_obj[group_num][timer_num]->hal));
    th_set_auto_reload(&(p_timer_obj[group_num][timer_num]->hal), config->auto_reload);
    th_set_divider(&(p_timer_obj[group_num][timer_num]->hal), config->divider);
    th_set_counter_increase(&(p_timer_obj[group_num][timer_num]->hal), config->counter_dir);
    th_set_alarm_enable(&(p_timer_obj[group_num][timer_num]->hal), config->alarm_en);
    th_set_level_int_enable(&(p_timer_obj[group_num][timer_num]->hal), config->intr_type == TIMER_INTR_LEVEL);
    if (config->intr_type != TIMER_INTR_LEVEL) {
        ESP_LOGW(TIMER_TAG, "only support Level Interrupt, switch to Level Interrupt instead");
    }
    th_set_counter_enable(&(p_timer_obj[group_num][timer_num]->hal), config->counter_en);
#if SOC_TIMER_GROUP_SUPPORT_XTAL
    timer_hal_set_use_xtal(&(p_timer_obj[group_num][timer_num]->hal), config->clk_src);
#endif
    TIMER_EXIT_CRITICAL(&timer_spinlock[group_num]);

    return ESP_OK;
}

esp_err_t td_deinit(timer_group_t group_num, timer_idx_t timer_num)
{
    ESP_RETURN_ON_FALSE(group_num < TIMER_GROUP_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_GROUP_NUM_ERROR);
    ESP_RETURN_ON_FALSE(timer_num < TIMER_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NUM_ERROR);
    ESP_RETURN_ON_FALSE(p_timer_obj[group_num][timer_num] != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NEVER_INIT_ERROR);

    TIMER_ENTER_CRITICAL(&timer_spinlock[group_num]);
    th_set_counter_enable(&(p_timer_obj[group_num][timer_num]->hal), TIMER_PAUSE);
    th_intr_disable(&(p_timer_obj[group_num][timer_num]->hal));
    th_clear_intr_status(&(p_timer_obj[group_num][timer_num]->hal));
    TIMER_EXIT_CRITICAL(&timer_spinlock[group_num]);

    heap_caps_free(p_timer_obj[group_num][timer_num]);
    p_timer_obj[group_num][timer_num] = NULL;

    return ESP_OK;
}

esp_err_t tdGet_config(timer_group_t group_num, timer_idx_t timer_num, timer_config_t *config)
{
    ESP_RETURN_ON_FALSE(group_num < TIMER_GROUP_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_GROUP_NUM_ERROR);
    ESP_RETURN_ON_FALSE(timer_num < TIMER_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NUM_ERROR);
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_PARAM_ADDR_ERROR);
    ESP_RETURN_ON_FALSE(p_timer_obj[group_num][timer_num] != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NEVER_INIT_ERROR);

    TIMER_ENTER_CRITICAL(&timer_spinlock[group_num]);
    config->alarm_en = th_get_alarm_enable(&(p_timer_obj[group_num][timer_num]->hal));
    config->auto_reload = th_get_auto_reload(&(p_timer_obj[group_num][timer_num]->hal));
    config->counter_dir = th_get_counter_increase(&(p_timer_obj[group_num][timer_num]->hal));
    config->counter_en = th_get_counter_enable(&(p_timer_obj[group_num][timer_num]->hal));

    uint32_t div;
    th_get_divider(&(p_timer_obj[group_num][timer_num]->hal), &div);
    config->divider = div;

    if (th_get_level_int_enable(&(p_timer_obj[group_num][timer_num]->hal))) {
        config->intr_type = TIMER_INTR_LEVEL;
    } else {
        config->intr_type = TIMER_INTR_MAX;
    }
    TIMER_EXIT_CRITICAL(&timer_spinlock[group_num]);
    return ESP_OK;
}

esp_err_t tdGroup_intr_enable(timer_group_t group_num, timer_intr_t en_mask)
{
    ESP_RETURN_ON_FALSE(group_num < TIMER_GROUP_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_GROUP_NUM_ERROR);
    ESP_RETURN_ON_FALSE(p_timer_obj[group_num] != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NEVER_INIT_ERROR);
    TIMER_ENTER_CRITICAL(&timer_spinlock[group_num]);
    for (int i = 0; i < TIMER_MAX; i++) {
        if (en_mask & BIT(i)) {
            th_intr_enable(&(p_timer_obj[group_num][i]->hal));
        }
    }
    TIMER_EXIT_CRITICAL(&timer_spinlock[group_num]);
    return ESP_OK;
}

esp_err_t tdGroup_intr_disable(timer_group_t group_num, timer_intr_t disable_mask)
{
    ESP_RETURN_ON_FALSE(group_num < TIMER_GROUP_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_GROUP_NUM_ERROR);
    ESP_RETURN_ON_FALSE(p_timer_obj[group_num] != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NEVER_INIT_ERROR);
    TIMER_ENTER_CRITICAL(&timer_spinlock[group_num]);
    for (int i = 0; i < TIMER_MAX; i++) {
        if (disable_mask & BIT(i)) {
            th_intr_disable(&(p_timer_obj[group_num][i]->hal));
        }
    }
    TIMER_EXIT_CRITICAL(&timer_spinlock[group_num]);
    return ESP_OK;
}

esp_err_t tdEnable_intr(timer_group_t group_num, timer_idx_t timer_num)
{
    ESP_RETURN_ON_FALSE(group_num < TIMER_GROUP_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_GROUP_NUM_ERROR);
    ESP_RETURN_ON_FALSE(timer_num < TIMER_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NUM_ERROR);
    ESP_RETURN_ON_FALSE(p_timer_obj[group_num][timer_num] != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NEVER_INIT_ERROR);
    TIMER_ENTER_CRITICAL(&timer_spinlock[group_num]);
    th_intr_enable(&(p_timer_obj[group_num][timer_num]->hal));
    TIMER_EXIT_CRITICAL(&timer_spinlock[group_num]);
    return ESP_OK;
}

esp_err_t tdDisable_intr(timer_group_t group_num, timer_idx_t timer_num)
{
    ESP_RETURN_ON_FALSE(group_num < TIMER_GROUP_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_GROUP_NUM_ERROR);
    ESP_RETURN_ON_FALSE(timer_num < TIMER_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NUM_ERROR);
    ESP_RETURN_ON_FALSE(p_timer_obj[group_num][timer_num] != NULL, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_NEVER_INIT_ERROR);
    TIMER_ENTER_CRITICAL(&timer_spinlock[group_num]);
    th_intr_disable(&(p_timer_obj[group_num][timer_num]->hal));
    TIMER_EXIT_CRITICAL(&timer_spinlock[group_num]);
    return ESP_OK;
}

/* This function is deprecated */
timer_intr_t IRAM_ATTR timer_group_intr_get_in_isr(timer_group_t group_num)
{
    uint32_t intr_raw_status = 0;
    th_get_intr_raw_status(group_num, &intr_raw_status);
    return intr_raw_status;
}

uint32_t IRAM_ATTR timer_group_get_intr_status_in_isr(timer_group_t group_num)
{
    uint32_t intr_status = 0;
    if (p_timer_obj[group_num][TIMER_0] != NULL) {
        th_get_intr_status(&(p_timer_obj[group_num][TIMER_0]->hal), &intr_status);
    }
#if SOC_TIMER_GROUP_TIMERS_PER_GROUP > 1
    else if (p_timer_obj[group_num][TIMER_1] != NULL) {
        th_get_intr_status(&(p_timer_obj[group_num][TIMER_1]->hal), &intr_status);
    }
#endif
    return intr_status;
}

/* This function is deprecated */
void IRAM_ATTR timer_group_intr_clr_in_isr(timer_group_t group_num, timer_idx_t timer_num)
{
    timer_group_clr_intr_status_in_isr(group_num, timer_num);
}

void IRAM_ATTR timer_group_clr_intr_status_in_isr(timer_group_t group_num, timer_idx_t timer_num)
{
    th_clear_intr_status(&(p_timer_obj[group_num][timer_num]->hal));
}

void IRAM_ATTR timer_group_enable_alarm_in_isr(timer_group_t group_num, timer_idx_t timer_num)
{
    th_set_alarm_enable(&(p_timer_obj[group_num][timer_num]->hal), true);
}

uint64_t IRAM_ATTR timer_group_get_counter_value_in_isr(timer_group_t group_num, timer_idx_t timer_num)
{
    uint64_t val;
    th_get_counter_value(&(p_timer_obj[group_num][timer_num]->hal), &val);
    return val;
}

void IRAM_ATTR timer_group_set_alarm_value_in_isr(timer_group_t group_num, timer_idx_t timer_num, uint64_t alarm_val)
{
    th_set_alarm_value(&(p_timer_obj[group_num][timer_num]->hal), alarm_val);
}

void IRAM_ATTR timer_group_set_counter_enable_in_isr(timer_group_t group_num, timer_idx_t timer_num, timer_start_t counter_en)
{
    th_set_counter_enable(&(p_timer_obj[group_num][timer_num]->hal), counter_en);
}

/* This function is deprecated */
void IRAM_ATTR timer_group_clr_intr_sta_in_isr(timer_group_t group_num, timer_intr_t intr_mask)
{
    for (uint32_t timer_idx = 0; timer_idx < TIMER_MAX; timer_idx++) {
        if (intr_mask & BIT(timer_idx)) {
            timer_group_clr_intr_status_in_isr(group_num, timer_idx);
        }
    }
}

bool IRAM_ATTR timer_group_get_auto_reload_in_isr(timer_group_t group_num, timer_idx_t timer_num)
{
    return th_get_auto_reload(&(p_timer_obj[group_num][timer_num]->hal));
}

esp_err_t IRAM_ATTR tdSpinlock_take(timer_group_t group_num)
{
    ESP_RETURN_ON_FALSE(group_num < TIMER_GROUP_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_GROUP_NUM_ERROR);
    TIMER_ENTER_CRITICAL(&timer_spinlock[group_num]);
    return ESP_OK;
}

esp_err_t IRAM_ATTR tdSpinlock_give(timer_group_t group_num)
{
    ESP_RETURN_ON_FALSE(group_num < TIMER_GROUP_MAX, ESP_ERR_INVALID_ARG, TIMER_TAG,  TIMER_GROUP_NUM_ERROR);
    TIMER_EXIT_CRITICAL(&timer_spinlock[group_num]);
    return ESP_OK;
}

typedef struct {
       int timer_group;
       int timer_idx;
       int alarm_interval;
       bool auto_reload;
   } example_timer_info_t;

typedef struct {
       example_timer_info_t info;
       uint64_t timer_counter_value;
} example_timer_event_t;

static xQueueHandle s_timer_queue;

void TimerS(int group, int timer, bool auto_reload, int timer_interval_sec){//Funcion para arrancar el timer
   /* Select and initialize basic parameters of the timer */
       timer_config_t config = {
           .divider = TIMER_DIVIDER,
           .counter_dir = TIMER_COUNT_UP,
           .counter_en = TIMER_PAUSE,
           .alarm_en = TIMER_ALARM_EN,
           .auto_reload = auto_reload,
       }; // default clock source is APB
       td_init(group, timer, &config);

       /* Timer's counter will initially start from value below.
          Also, if auto_reload is set, this value will be automatically reload on alarm */
       tdSet_counter_val(group, timer, 0);

       /* Configure the alarm value and the interrupt on alarm. */
       tdSet_alarm_value(group, timer, timer_interval_sec * TIMER_SCALE);
       tdEnable_intr(group, timer);

       example_timer_info_t *timer_info = calloc(1, sizeof(example_timer_info_t));
       timer_info->timer_group = group;
       timer_info->timer_idx = timer;
       timer_info->auto_reload = auto_reload;
       timer_info->alarm_interval = timer_interval_sec;
       td_isr_callback_add(group, timer, timer_group_isr_callback, timer_info, 0);
       tdSet_auto_reload(group, timer, TIMER_AUTORELOAD_EN);
}

static bool IRAM_ATTR timer_group_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    example_timer_info_t *info = (example_timer_info_t *) args;

    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);

    /* Prepare basic event data that will be then sent back to task */
    example_timer_event_t evt = {
        .info.timer_group = info->timer_group,
        .info.timer_idx = info->timer_idx,
        .info.auto_reload = info->auto_reload,
        .info.alarm_interval = info->alarm_interval,
        .timer_counter_value = timer_counter_value
    };

    if (!info->auto_reload) {
        timer_counter_value += info->alarm_interval * TIMER_SCALE;
        timer_group_set_alarm_value_in_isr(info->timer_group, info->timer_idx, timer_counter_value);
    }

    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(s_timer_queue, &evt, &high_task_awoken);

    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}

void espera (int group, int timer){
    td_start(group, timer);

    int aux=0;
   do{
      s_timer_queue = xQueueCreate(10, sizeof(example_timer_event_t));

      example_timer_event_t evt;
      xQueueReceive(s_timer_queue, &evt, portMAX_DELAY);

      if (evt.info.auto_reload){
         aux=1;
      }

      else{
         aux=0;
      }

   }while(aux==0);
    td_pause(group, timer);
}

 void inline print_timer_counter(uint64_t counter_value)
{
    printf("Counter: 0x%08x%08x\r\n", (uint32_t) (counter_value >> 32),
           (uint32_t) (counter_value));
    printf("Time   : %.8f s\r\n", (double) counter_value / TIMER_SCALE);
}

