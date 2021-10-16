/*
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <random/rand32.h>

#define CONFIG_POSIX_FS

#include <logging/log.h>
LOG_MODULE_REGISTER(esp32_wifi_adapter, CONFIG_LOG_DEFAULT_LEVEL);

#include "esp_system.h"
#include "esp_wifi.h"
#include "stdlib.h"
#include "string.h"
#include "soc/soc_caps.h"
#include "esp_private/wifi_os_adapter.h"
#include "esp_private/wifi.h"
#include "soc/dport_access.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "soc/rtc.h"
#include "soc/syscon_reg.h"
#include "driver/periph_ctrl.h"
#include "esp_phy_init.h"
#include "esp32s2/clk.h"
#include "esp32s2/rom/rtc.h"
#include "os.h"
#include <sys/math_extras.h>

#define portTICK_PERIOD_MS (1000 / 100)

ESP_EVENT_DEFINE_BASE(WIFI_EVENT);

static struct k_thread wifi_task_handle;

struct wifi_spin_lock {
	struct k_spinlock spinlock;
	k_spinlock_key_t key;
};

uint64_t g_wifi_feature_caps;

#ifdef CONFIG_ESP_SPIRAM
K_HEAP_DEFINE(_esp32_wifi_heap, 60 * 1024);
#define _WIFI_HEAP (&_esp32_wifi_heap)

static inline void *w_malloc(size_t size)
{
    void *ptr = k_heap_aligned_alloc(_WIFI_HEAP, sizeof(void *), size, K_NO_WAIT);

    if (ptr == NULL) {
        LOG_ERR("memory allocation failed %zu", size);
    }
    return ptr;
}

static inline void w_free(void *ptr)
{
    if (!ptr)
        return;
    k_heap_free(_WIFI_HEAP, ptr);
}
#else
#define w_malloc k_malloc
#define w_free k_free
#endif

static inline void *w_calloc(size_t nmemb, size_t size)
{
    void *ret;
    size_t bounds;

    if (size_mul_overflow(nmemb, size, &bounds)) {
        return NULL;
    }

    ret = w_malloc(bounds);
    if (ret != NULL) {
        (void)memset(ret, 0, bounds);
    }

    return ret;
}


void *wifi_malloc(size_t size)
{
    return w_malloc(size);
}

void *wifi_realloc(void *ptr, size_t size)
{
	LOG_ERR("%s not yet supported", __func__);
	return NULL;
}

void *wifi_calloc(size_t n, size_t size)
{
	return w_calloc(n, size);
}

static void *wifi_zalloc_wrapper(size_t size)
{
    return w_calloc(1, size);
}

wifi_static_queue_t *wifi_create_queue(int queue_len, int item_size)
{
	wifi_static_queue_t *queue = NULL;

	queue = (wifi_static_queue_t *)k_malloc(sizeof(wifi_static_queue_t));
	if (!queue) {
        LOG_ERR("queue allocation failed %zu", sizeof(wifi_static_queue_t));
		return NULL;
	}

	queue->storage = k_malloc(queue_len * item_size);
	if (queue->storage == NULL) {
        k_free(queue);
		LOG_ERR("msg buffer allocation failed");
		return NULL;
	}

	queue->handle = k_malloc(sizeof(struct k_msgq));
	if (queue->handle == NULL) {
	    k_free(queue->storage);
	    k_free(queue);
		LOG_ERR("queue handle allocation failed");
		return NULL;
	}

	k_msgq_init((struct k_msgq *)queue->handle, queue->storage, item_size, queue_len);

	return queue;
}

void wifi_delete_queue(wifi_static_queue_t *queue)
{
	if (queue) {
        k_free(queue->storage);
        k_free(queue->handle);
        k_free(queue);
	}
}

static void *wifi_create_queue_wrapper(int queue_len, int item_size)
{
	return wifi_create_queue(queue_len, item_size);
}

static void wifi_delete_queue_wrapper(void *queue)
{
	wifi_delete_queue(queue);
}

static bool env_is_chip_wrapper(void)
{
#ifdef CONFIG_IDF_ENV_FPGA
	return false;
#else
	return true;
#endif
}

static void *spin_lock_create_wrapper(void)
{
	struct k_spinlock *wifi_spin_lock = (struct k_spinlock *) k_calloc(1, sizeof(struct k_spinlock));
	if (!wifi_spin_lock) {
	    LOG_ERR("SpinLock alloc");
	    return NULL;
	}

	return (void *)wifi_spin_lock;
}

static uint32_t wifi_int_disable_wrapper(void *wifi_int_mux)
{
	unsigned int *int_mux = (unsigned int *) wifi_int_mux;

	*int_mux = irq_lock();
	return 0;
}

static void wifi_int_restore_wrapper(void *wifi_int_mux, uint32_t tmp)
{
	unsigned int *key = (unsigned int *) wifi_int_mux;

	irq_unlock(*key);
}

static void task_yield_from_isr_wrapper(void)
{
	k_yield();
}

static void *semphr_create_wrapper(uint32_t max, uint32_t init)
{
	struct k_sem *sem = (struct k_sem *) k_malloc(sizeof(struct k_sem));
    if (!sem) {
        LOG_ERR("k_sem alloc");
        return NULL;
    }

	k_sem_init(sem, init, max);
	return (void *) sem;
}

static void *wifi_thread_semphr_get_wrapper(void)
{
	struct k_sem *sem = k_thread_custom_data_get();

	if (sem)
	    return sem;

    sem = (struct k_sem *) k_malloc(sizeof(struct k_sem));
    if (!sem) {
        LOG_ERR("k_sem alloc");
        return NULL;
    }
    k_sem_init(sem, 0, 1);
    k_thread_custom_data_set(sem);
	return sem;
}

static int32_t semphr_take_wrapper(void *semphr, uint32_t block_time_tick)
{
    k_timeout_t tout = K_FOREVER;

	if (block_time_tick != OSI_FUNCS_TIME_BLOCKING)
        tout = K_MSEC(block_time_tick * portTICK_PERIOD_MS);

	return !k_sem_take((struct k_sem *)semphr, tout);
}

static int32_t semphr_give_wrapper(void *semphr)
{
	k_sem_give((struct k_sem *) semphr);
	return 1;
}

static void *mutex_create_wrapper(void)
{
	struct k_mutex *my_mutex = (struct k_mutex *) k_malloc(sizeof(struct k_mutex));
    if (!my_mutex) {
        LOG_ERR("k_mutex alloc");
        return NULL;
    }

	k_mutex_init(my_mutex);
	return (void *)my_mutex;
}

static int32_t mutex_lock_wrapper(void *mutex)
{
	struct k_mutex *my_mutex = (struct k_mutex *) mutex;

	k_mutex_lock(my_mutex, K_FOREVER);
	return 0;
}

static int32_t mutex_unlock_wrapper(void *mutex)
{
	struct k_mutex *my_mutex = (struct k_mutex *) mutex;

	k_mutex_unlock(my_mutex);
	return 0;
}

static void *queue_create_wrapper(uint32_t queue_len, uint32_t item_size)
{
	struct k_queue *queue = (struct k_queue *)k_malloc(sizeof(struct k_queue) + item_size * queue_len);
	void *buf = queue + 1;

	if (queue == NULL) {
		LOG_ERR("queue malloc failed");
		return NULL;
	}

	k_msgq_init((struct k_msgq *)queue, buf, item_size, queue_len);
	return (void *)queue;
}

static int32_t queue_send_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
    k_timeout_t tout = K_FOREVER;
	if (block_time_tick != OSI_FUNCS_TIME_BLOCKING) {
	    int ms = block_time_tick * portTICK_PERIOD_MS;
	    tout = K_MSEC(ms);
	}

    return !k_msgq_put((struct k_msgq *)queue, item, tout);
}

static int32_t queue_send_from_isr_wrapper(void *queue, void *item, void *hptw)
{
	int *hpt = (int *) hptw;

    *hpt = 0;
    return !k_msgq_put((struct k_msgq *)queue, item, K_NO_WAIT);
}

int32_t queue_send_to_back_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
    LOG_ERR("%s", __FUNCTION__);
	return 0;
}

int32_t queue_send_to_front_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
    LOG_ERR("%s", __FUNCTION__);
	return 0;
}

static int32_t queue_recv_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
    k_timeout_t tout = K_FOREVER;
	if (block_time_tick != OSI_FUNCS_TIME_BLOCKING) {
	    int ms = block_time_tick * portTICK_PERIOD_MS;
	    tout = K_MSEC(ms);
	}

    return !k_msgq_get((struct k_msgq *)queue, item, tout);
}

static uint32_t event_group_wait_bits_wrapper(void *event, uint32_t bits_to_wait_for, int clear_on_exit, int wait_for_all_bits, uint32_t block_time_tick)
{
    LOG_ERR("%s", __FUNCTION__);
	return 0;
}

static int32_t task_create_pinned_to_core_wrapper(void *task_func, const char *name, uint32_t stack_depth, void *param, uint32_t prio, void *task_handle, uint32_t core_id)
{
    k_thread_stack_t *stack = k_malloc(stack_depth);
    if (!stack) {
        LOG_ERR("can't alloc stack %d for %s", stack_depth, log_strdup(name));
        return -ENOMEM;
    }
    LOG_ERR("crate %d %s %d (%d, %d)", stack_depth, log_strdup(name), prio, K_LOWEST_THREAD_PRIO, K_HIGHEST_THREAD_PRIO);
	k_tid_t tid = k_thread_create(&wifi_task_handle, stack, stack_depth,
				      (k_thread_entry_t)task_func, param, NULL, NULL,
				      prio, K_INHERIT_PERMS, K_NO_WAIT);

	if (!tid) {
        LOG_ERR("can't start %s", log_strdup(name));
        k_free(stack);
        return -ENOMEM;
	}

	k_thread_name_set(tid, name);

	*(int32_t *)task_handle = (int32_t) tid;
	return 1;
}

static int32_t task_create_wrapper(void *task_func, const char *name, uint32_t stack_depth, void *param, uint32_t prio, void *task_handle)
{
    k_thread_stack_t *stack = k_malloc(stack_depth);
    if (!stack) {
        LOG_ERR("can't alloc stack %d for %s", stack_depth, log_strdup(name));
        return -ENOMEM;
    }
    LOG_ERR("crate2 %d %s", stack_depth, log_strdup(name));
	k_tid_t tid = k_thread_create(&wifi_task_handle, stack, stack_depth,
				      (k_thread_entry_t)task_func, param, NULL, NULL,
				      prio, K_INHERIT_PERMS, K_NO_WAIT);

    if (!tid) {
        LOG_ERR("can't start %s", log_strdup(name));
        k_free(stack);
        return -ENOMEM;
    }

	k_thread_name_set(tid, name);

	*(int32_t *)task_handle = (int32_t) tid;
	return 1;
}

static void task_delete_wrapper(void *task_handle)
{
    k_tid_t tid = task_handle;
    k_thread_abort(tid);
    /* todo: get stck ptr
     void *ptr = stack;
     k_free(stack);
     */
}

static int32_t task_ms_to_tick_wrapper(uint32_t ms)
{
	return (int32_t)(ms / portTICK_PERIOD_MS);
}

static int32_t task_get_max_priority_wrapper(void)
{
//	return (int32_t)(4);
    return K_HIGHEST_APPLICATION_THREAD_PRIO;
}

static int32_t esp_event_post_wrapper(const char* event_base, int32_t event_id, void* event_data, size_t event_data_size, uint32_t ticks_to_wait)
{
	LOG_ERR("%s not yet supported", __func__);
	return 0;
}

static void wifi_apb80m_request_wrapper(void)
{
#ifdef CONFIG_PM_ENABLE
	wifi_apb80m_request();
#endif
}

static void wifi_apb80m_release_wrapper(void)
{
#ifdef CONFIG_PM_ENABLE
	wifi_apb80m_release();
#endif
}

static void timer_arm_wrapper(void *timer, uint32_t tmout, bool repeat)
{
	ets_timer_arm(timer, tmout, repeat);
}

static void timer_disarm_wrapper(void *timer)
{
	ets_timer_disarm(timer);
}

static void timer_done_wrapper(void *ptimer)
{
	ets_timer_done(ptimer);
}

static void timer_setfn_wrapper(void *ptimer, void *pfunction, void *parg)
{
	ets_timer_setfn(ptimer, pfunction, parg);
}

static void timer_arm_us_wrapper(void *ptimer, uint32_t us, bool repeat)
{
	ets_timer_arm_us(ptimer, us, repeat);
}

static int get_time_wrapper(void *t)
{
	return os_get_time(t);
}

static uint32_t esp_clk_slowclk_cal_get_wrapper(void)
{
	/*
	 * The bit width of WiFi light sleep clock calibration is 12 while the one of
	 * system is 19. It should shift 19 - 12 = 7.
	 */
	return (REG_READ(RTC_SLOW_CLK_CAL_REG) >> (RTC_CLK_CAL_FRACT - SOC_WIFI_LIGHT_SLEEP_CLK_WIDTH));
}

static void *malloc_internal_wrapper(size_t size)
{
	void *ptr = w_malloc(size);

	if (ptr == NULL) {
		LOG_ERR("malloc failed");
	}
	return ptr;
}

static void *realloc_internal_wrapper(void *ptr, size_t size)
{
	LOG_ERR("%s not yet supported", __func__);
	return NULL;
}

static void *calloc_internal_wrapper(size_t n, size_t size)
{
	return w_calloc(n, size);
}

static void *zalloc_internal_wrapper(size_t size)
{
	void *ptr = calloc_internal_wrapper(1, size);

	if (ptr) {
		memset(ptr, 0, size);
	}
	return ptr;
}

uint32_t uxQueueMessagesWaiting(void *queue)
{
    LOG_ERR("%s", __FUNCTION__);
	return 0;
}

void *xEventGroupCreate(void)
{
	LOG_ERR("EventGroup not supported!");
	return NULL;
}

void vEventGroupDelete(void *grp)
{
    LOG_ERR("%s", __FUNCTION__);
}

uint32_t xEventGroupSetBits(void *ptr, uint32_t data)
{
	return 0;
}

uint32_t xEventGroupClearBits(void *ptr, uint32_t data)
{
	return 0;
}

void *xTaskGetCurrentTaskHandle(void)
{
	return (void *)k_current_get();
}

void vTaskDelay(uint32_t ticks)
{
	int ms = ticks * portTICK_PERIOD_MS;

	k_sleep(K_MSEC(ms));
}

static void set_intr_wrapper(int32_t cpu_no, uint32_t intr_source, uint32_t intr_num, int32_t intr_prio)
{
	intr_matrix_set(cpu_no, intr_source, intr_num);
}

static void clear_intr_wrapper(uint32_t intr_source, uint32_t intr_num)
{
    LOG_ERR("%s", __FUNCTION__);

}

static void set_isr_wrapper(int32_t n, void *f, void *arg)
{
	irq_disable(n);
	irq_connect_dynamic(n, 1, f, arg, 0);
}

static void intr_on(unsigned int mask)
{
	irq_enable(0);
}

static void intr_off(unsigned int mask)
{
	irq_disable(0);
}

uint32_t esp_get_free_heap_size(void)
{
	/* FIXME: API to get free heap size is not available in Zephyr. */
	/* It is only used by ESP-MESH feature (not supported yet) */
	return 10000;
}

static unsigned long w_random(void)
{
	return sys_rand32_get();
}

static void wifi_clock_enable_wrapper(void)
{
	periph_module_enable(PERIPH_WIFI_MODULE);
}

static void wifi_clock_disable_wrapper(void)
{
	periph_module_disable(PERIPH_WIFI_MODULE);
}

static void wifi_reset_mac_wrapper(void)
{
	DPORT_SET_PERI_REG_MASK(DPORT_CORE_RST_EN_REG, DPORT_MAC_RST);
	DPORT_CLEAR_PERI_REG_MASK(DPORT_CORE_RST_EN_REG, DPORT_MAC_RST);
}

int32_t nvs_set_i8(uint32_t handle, const char *key, int8_t value)
{
	return 0;
}

int32_t nvs_get_i8(uint32_t handle, const char *key, int8_t *out_value)
{
	return 0;
}

int32_t nvs_set_u8(uint32_t handle, const char *key, uint8_t value)
{
	return 0;
}

int32_t nvs_get_u8(uint32_t handle, const char *key, uint8_t *out_value)
{
	return 0;
}

int32_t nvs_set_u16(uint32_t handle, const char *key, uint16_t value)
{
	return 0;
}

int32_t nvs_get_u16(uint32_t handle, const char *key, uint16_t *out_value)
{
	return 0;
}

int32_t nvs_open(const char *name, uint32_t open_mode, uint32_t *out_handle)
{
	return 0;
}

void nvs_close(uint32_t handle)
{
	return;
}

int32_t nvs_commit(uint32_t handle)
{
	return 0;
}

int32_t nvs_set_blob(uint32_t handle, const char *key, const void *value,
		     size_t length)
{
	return 0;
}

int32_t nvs_get_blob(uint32_t handle, const char *key, void *out_value,
		     size_t *length)
{
	return 0;
}

int32_t nvs_erase_key(uint32_t handle, const char *key)
{
	return 0;
}

static int coex_init_wrapper(void)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_init();
#else
	return 0;
#endif
}

static void coex_deinit_wrapper(void)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	coex_deinit();
#endif
}

static int coex_enable_wrapper(void)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_enable();
#else
	return 0;
#endif
}

static void coex_disable_wrapper(void)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	coex_disable();
#endif
}

static uint32_t coex_status_get_wrapper(void)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_status_get();
#else
	return 0;
#endif
}

static void coex_condition_set_wrapper(uint32_t type, bool dissatisfy)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	coex_condition_set(type, dissatisfy);
#endif
}

static int coex_wifi_request_wrapper(uint32_t event, uint32_t latency, uint32_t duration)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_wifi_request(event, latency, duration);
#else
	return 0;
#endif
}

static int coex_wifi_release_wrapper(uint32_t event)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_wifi_release(event);
#else
	return 0;
#endif
}

static int coex_wifi_channel_set_wrapper(uint8_t primary, uint8_t secondary)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_wifi_channel_set(primary, secondary);
#else
	return 0;
#endif
}

static int coex_event_duration_get_wrapper(uint32_t event, uint32_t *duration)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_event_duration_get(event, duration);
#else
	return 0;
#endif
}

static int coex_pti_get_wrapper(uint32_t event, uint8_t *pti)
{
	return 0;
}

static void coex_schm_status_bit_clear_wrapper(uint32_t type, uint32_t status)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	coex_schm_status_bit_clear(type, status);
#endif
}

static void coex_schm_status_bit_set_wrapper(uint32_t type, uint32_t status)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	coex_schm_status_bit_set(type, status);
#endif
}

static int coex_schm_interval_set_wrapper(uint32_t interval)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_schm_interval_set(interval);
#else
	return 0;
#endif
}

static uint32_t coex_schm_interval_get_wrapper(void)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_schm_interval_get();
#else
	return 0;
#endif
}

static uint8_t coex_schm_curr_period_get_wrapper(void)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_schm_curr_period_get();
#else
	return 0;
#endif
}

static void * coex_schm_curr_phase_get_wrapper(void)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_schm_curr_phase_get();
#else
	return NULL;
#endif
}

static int coex_schm_curr_phase_idx_set_wrapper(int idx)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_schm_curr_phase_idx_set(idx);
#else
	return 0;
#endif
}

static int coex_schm_curr_phase_idx_get_wrapper(void)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	return coex_schm_curr_phase_idx_get();
#else
	return 0;
#endif
}

static void esp_empty_wrapper(void)
{

}

bool IRAM_ATTR coex_is_in_isr_wrapper(void)
{
	return k_is_in_isr();
}

wifi_osi_funcs_t g_wifi_osi_funcs = {
	._version = ESP_WIFI_OS_ADAPTER_VERSION,
	._env_is_chip = env_is_chip_wrapper,
	._set_intr = set_intr_wrapper,
	._clear_intr = clear_intr_wrapper,
	._set_isr = set_isr_wrapper,
	._ints_on = intr_on,
	._ints_off = intr_off,
	._is_from_isr = coex_is_in_isr_wrapper,
	._spin_lock_create = spin_lock_create_wrapper,
	._spin_lock_delete = k_free,
	._wifi_int_disable = wifi_int_disable_wrapper,
	._wifi_int_restore = wifi_int_restore_wrapper,
	._task_yield_from_isr = task_yield_from_isr_wrapper,
	._semphr_create = semphr_create_wrapper,
	._semphr_delete = k_free,
	._semphr_take = semphr_take_wrapper,
	._semphr_give = semphr_give_wrapper,
	._wifi_thread_semphr_get = wifi_thread_semphr_get_wrapper,
	._mutex_create = mutex_create_wrapper,
	._recursive_mutex_create = mutex_create_wrapper,
	._mutex_delete = k_free,
	._mutex_lock = mutex_lock_wrapper,
	._mutex_unlock = mutex_unlock_wrapper,
	._queue_create = queue_create_wrapper,
	._queue_delete = k_free,
	._queue_send = queue_send_wrapper,
	._queue_send_from_isr = queue_send_from_isr_wrapper,
	._queue_send_to_back = queue_send_to_back_wrapper,
	._queue_send_to_front = queue_send_to_front_wrapper,
	._queue_recv = queue_recv_wrapper,
	._queue_msg_waiting = uxQueueMessagesWaiting,
	._event_group_create = (void *(*)(void))xEventGroupCreate,
	._event_group_delete = (void (*)(void *))vEventGroupDelete,
	._event_group_set_bits = xEventGroupSetBits,
	._event_group_clear_bits = xEventGroupClearBits,
	._event_group_wait_bits = event_group_wait_bits_wrapper,
	._task_create_pinned_to_core = task_create_pinned_to_core_wrapper,
	._task_create = task_create_wrapper,
	._task_delete = task_delete_wrapper,
	._task_delay = vTaskDelay,
	._task_ms_to_tick = task_ms_to_tick_wrapper,
	._task_get_current_task = (void *(*)(void))xTaskGetCurrentTaskHandle,
	._task_get_max_priority = task_get_max_priority_wrapper,
	._malloc = w_malloc,
	._free = w_free,
	._event_post = esp_event_post_wrapper,
	._get_free_heap_size = esp_get_free_heap_size,
	._rand = sys_rand32_get,
	._dport_access_stall_other_cpu_start_wrap = esp_empty_wrapper,
	._dport_access_stall_other_cpu_end_wrap = esp_empty_wrapper,
	._wifi_apb80m_request = wifi_apb80m_request_wrapper,
	._wifi_apb80m_release = wifi_apb80m_release_wrapper,
	._phy_disable = esp_phy_disable,
	._phy_enable = esp_phy_enable,
	._phy_update_country_info = esp_phy_update_country_info,
	._read_mac = esp_read_mac,
	._timer_arm = timer_arm_wrapper,
	._timer_disarm = timer_disarm_wrapper,
	._timer_done = timer_done_wrapper,
	._timer_setfn = timer_setfn_wrapper,
	._timer_arm_us = timer_arm_us_wrapper,
	._wifi_reset_mac = wifi_reset_mac_wrapper,
	._wifi_clock_enable = wifi_clock_enable_wrapper,
	._wifi_clock_disable = wifi_clock_disable_wrapper,
	._wifi_rtc_enable_iso = esp_empty_wrapper,
	._wifi_rtc_disable_iso = esp_empty_wrapper,
	._esp_timer_get_time = esp_timer_get_time,
	._nvs_set_i8 = nvs_set_i8,
	._nvs_get_i8 = nvs_get_i8,
	._nvs_set_u8 = nvs_set_u8,
	._nvs_get_u8 = nvs_get_u8,
	._nvs_set_u16 = nvs_set_u16,
	._nvs_get_u16 = nvs_get_u16,
	._nvs_open = nvs_open,
	._nvs_close = nvs_close,
	._nvs_commit = nvs_commit,
	._nvs_set_blob = nvs_set_blob,
	._nvs_get_blob = nvs_get_blob,
	._nvs_erase_key = nvs_erase_key,
	._get_random = os_get_random,
	._get_time = get_time_wrapper,
	._random = w_random,
	._slowclk_cal_get = esp_clk_slowclk_cal_get_wrapper,
	._log_write = esp_log_write,
	._log_writev = esp_log_writev,
	._log_timestamp = k_uptime_get_32,
	._malloc_internal =  malloc_internal_wrapper,
	._realloc_internal = realloc_internal_wrapper,
	._calloc_internal = calloc_internal_wrapper,
	._zalloc_internal = zalloc_internal_wrapper,
	._wifi_malloc = wifi_malloc,
	._wifi_realloc = wifi_realloc,
	._wifi_calloc = wifi_calloc,
	._wifi_zalloc = wifi_zalloc_wrapper,
	._wifi_create_queue = wifi_create_queue_wrapper,
	._wifi_delete_queue = wifi_delete_queue_wrapper,
	._coex_init = coex_init_wrapper,
	._coex_deinit = coex_deinit_wrapper,
	._coex_enable = coex_enable_wrapper,
	._coex_disable = coex_disable_wrapper,
	._coex_status_get = coex_status_get_wrapper,
	._coex_condition_set = coex_condition_set_wrapper,
	._coex_wifi_request = coex_wifi_request_wrapper,
	._coex_wifi_release = coex_wifi_release_wrapper,
	._coex_wifi_channel_set = coex_wifi_channel_set_wrapper,
	._coex_event_duration_get = coex_event_duration_get_wrapper,
	._coex_pti_get = coex_pti_get_wrapper,
	._coex_schm_status_bit_clear = coex_schm_status_bit_clear_wrapper,
	._coex_schm_status_bit_set = coex_schm_status_bit_set_wrapper,
	._coex_schm_interval_set = coex_schm_interval_set_wrapper,
	._coex_schm_interval_get = coex_schm_interval_get_wrapper,
	._coex_schm_curr_period_get = coex_schm_curr_period_get_wrapper,
	._coex_schm_curr_phase_get = coex_schm_curr_phase_get_wrapper,
	._coex_schm_curr_phase_idx_set = coex_schm_curr_phase_idx_set_wrapper,
	._coex_schm_curr_phase_idx_get = coex_schm_curr_phase_idx_get_wrapper,
	._magic = ESP_WIFI_OS_ADAPTER_MAGIC,
};
esp_err_t esp_wifi_init(const wifi_init_config_t *config)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
	coex_init();
#endif

	return esp_wifi_init_internal(config);
}
