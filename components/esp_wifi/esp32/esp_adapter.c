// Copyright 2015-2018 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <pthread.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/xtensa_api.h"
#include "freertos/portmacro.h"
#include "freertos/xtensa_api.h"
#include "esp_types.h"
#include "esp_system.h"
#include "esp_task.h"
#include "esp_intr_alloc.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_private/wifi_os_adapter.h"
#include "esp_private/wifi.h"
#include "esp_phy_init.h"
#include "driver/periph_ctrl.h"
#include "nvs.h"
#include "os.h"
#include "esp_smartconfig.h"
#include "esp_coexist_internal.h"
#include "esp_coexist_adapter.h"
#include "esp32/dport_access.h"

#define TAG "esp_adapter"

static void IRAM_ATTR s_esp_dport_access_stall_other_cpu_start(void)
{
    DPORT_STALL_OTHER_CPU_START();
}

static void IRAM_ATTR s_esp_dport_access_stall_other_cpu_end(void)
{
    DPORT_STALL_OTHER_CPU_END();
}

/*
 If CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP is enabled. Prefer to allocate a chunk of memory in SPIRAM firstly.
 If failed, try to allocate it in internal memory then.
 */
IRAM_ATTR void *wifi_malloc( size_t size )
{
#if CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP
    return heap_caps_malloc_prefer(size, 2, MALLOC_CAP_DEFAULT|MALLOC_CAP_SPIRAM, MALLOC_CAP_DEFAULT|MALLOC_CAP_INTERNAL);
#else
    return malloc(size);
#endif
}

/*
 If CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP is enabled. Prefer to allocate a chunk of memory in SPIRAM firstly.
 If failed, try to allocate it in internal memory then.
 */
IRAM_ATTR void *wifi_realloc( void *ptr, size_t size )
{
#if CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP
    return heap_caps_realloc_prefer(ptr, size, 2, MALLOC_CAP_DEFAULT|MALLOC_CAP_SPIRAM, MALLOC_CAP_DEFAULT|MALLOC_CAP_INTERNAL);
#else
    return realloc(ptr, size);
#endif
}

/*
 If CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP is enabled. Prefer to allocate a chunk of memory in SPIRAM firstly.
 If failed, try to allocate it in internal memory then.
 */
IRAM_ATTR void *wifi_calloc( size_t n, size_t size )
{
#if CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP
    return heap_caps_calloc_prefer(n, size, 2, MALLOC_CAP_DEFAULT|MALLOC_CAP_SPIRAM, MALLOC_CAP_DEFAULT|MALLOC_CAP_INTERNAL);
#else
    return calloc(n, size);
#endif
}

static void * IRAM_ATTR wifi_zalloc_wrapper(size_t size)
{
    void *ptr = wifi_calloc(1, size);
    if (ptr) {
        memset(ptr, 0, size);
    }
    return ptr;
}

wifi_static_queue_t* wifi_create_queue( int queue_len, int item_size)
{
    wifi_static_queue_t *queue = NULL;

    queue = (wifi_static_queue_t*)heap_caps_malloc(sizeof(wifi_static_queue_t), MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    if (!queue) {
        return NULL;
    }

#if CONFIG_SPIRAM_USE_MALLOC

    queue->storage = heap_caps_calloc(1, sizeof(StaticQueue_t) + (queue_len*item_size), MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    if (!queue->storage) {
        goto _error;
    }

    queue->handle = xQueueCreateStatic( queue_len, item_size, ((uint8_t*)(queue->storage)) + sizeof(StaticQueue_t), (StaticQueue_t*)(queue->storage));

    if (!queue->handle) {
        goto _error;
    }

    return queue;

_error:
    if (queue) {
        if (queue->storage) {
            free(queue->storage);
        }

        free(queue);
    }

    return NULL;
#else
    queue->handle = xQueueCreate( queue_len, item_size);
    return queue;
#endif
}

void wifi_delete_queue(wifi_static_queue_t *queue)
{
    if (queue) {
        vQueueDelete(queue->handle);

#if CONFIG_SPIRAM_USE_MALLOC
        if (queue->storage) {
            free(queue->storage);
        }
#endif

        free(queue);
    }
}

static void * wifi_create_queue_wrapper(int queue_len, int item_size)
{
    return wifi_create_queue(queue_len, item_size);
}

static void wifi_delete_queue_wrapper(void *queue)
{
    wifi_delete_queue(queue);
}

static void set_isr_wrapper(int32_t n, void *f, void *arg)
{
    xt_set_interrupt_handler(n, (xt_handler)f, arg);
}

static void * spin_lock_create_wrapper(void)
{
    portMUX_TYPE tmp = portMUX_INITIALIZER_UNLOCKED;
    void *mux = malloc(sizeof(portMUX_TYPE));

    if (mux) {
        memcpy(mux,&tmp,sizeof(portMUX_TYPE));
        return mux;
    }
    return NULL;
}

static uint32_t IRAM_ATTR wifi_int_disable_wrapper(void *wifi_int_mux)
{
    if (xPortInIsrContext()) {
        portENTER_CRITICAL_ISR(wifi_int_mux);
    } else {
        portENTER_CRITICAL(wifi_int_mux);
    }

    return 0;
}

static void IRAM_ATTR wifi_int_restore_wrapper(void *wifi_int_mux, uint32_t tmp)
{
    if (xPortInIsrContext()) {
        portEXIT_CRITICAL_ISR(wifi_int_mux);
    } else {
        portEXIT_CRITICAL(wifi_int_mux);
    }
}

static void IRAM_ATTR task_yield_from_isr_wrapper(void)
{
    portYIELD_FROM_ISR();
}

static void * semphr_create_wrapper(uint32_t max, uint32_t init)
{
    return (void *)xSemaphoreCreateCounting(max, init);
}

static void semphr_delete_wrapper(void *semphr)
{
    vSemaphoreDelete(semphr);
}

static void wifi_thread_semphr_free(void* data)
{
    xSemaphoreHandle *sem = (xSemaphoreHandle*)(data);

    if (sem) {
        vSemaphoreDelete(sem);
    }
}

static void * wifi_thread_semphr_get_wrapper(void)
{
    static bool s_wifi_thread_sem_key_init = false;
    static pthread_key_t s_wifi_thread_sem_key;
    xSemaphoreHandle sem = NULL;

    if (s_wifi_thread_sem_key_init == false) {
        if (0 != pthread_key_create(&s_wifi_thread_sem_key, wifi_thread_semphr_free)) {
            return NULL;
        }
        s_wifi_thread_sem_key_init = true;
    }

    sem = pthread_getspecific(s_wifi_thread_sem_key);
    if (!sem) {
        sem = xSemaphoreCreateCounting(1, 0);
        if (sem) {
            pthread_setspecific(s_wifi_thread_sem_key, sem);
            ESP_LOGV(TAG, "thread sem create: sem=%p", sem);
        }
    }

    ESP_LOGV(TAG, "thread sem get: sem=%p", sem);
    return (void*)sem;
}

static int32_t IRAM_ATTR semphr_take_from_isr_wrapper(void *semphr, void *hptw)
{
    return (int32_t)xSemaphoreTakeFromISR(semphr, hptw);
}

static int32_t IRAM_ATTR semphr_give_from_isr_wrapper(void *semphr, void *hptw)
{
    return (int32_t)xSemaphoreGiveFromISR(semphr, hptw);
}

static int32_t semphr_take_wrapper(void *semphr, uint32_t block_time_tick)
{
    if (block_time_tick == OSI_FUNCS_TIME_BLOCKING) {
        return (int32_t)xSemaphoreTake(semphr, portMAX_DELAY);
    } else {
        return (int32_t)xSemaphoreTake(semphr, block_time_tick);
    }
}

static int32_t semphr_give_wrapper(void *semphr)
{
    return (int32_t)xSemaphoreGive(semphr);
}

static void * recursive_mutex_create_wrapper(void)
{
    return (void *)xSemaphoreCreateRecursiveMutex();
}

static void * mutex_create_wrapper(void)
{
    return (void *)xSemaphoreCreateMutex();
}

static void mutex_delete_wrapper(void *mutex)
{
    vSemaphoreDelete(mutex);
}

static int32_t IRAM_ATTR mutex_lock_wrapper(void *mutex)
{
    return (int32_t)xSemaphoreTakeRecursive(mutex, portMAX_DELAY);
}

static int32_t IRAM_ATTR mutex_unlock_wrapper(void *mutex)
{
    return (int32_t)xSemaphoreGiveRecursive(mutex);
}

static void * queue_create_wrapper(uint32_t queue_len, uint32_t item_size)
{
    return (void *)xQueueCreate(queue_len, item_size);
}

static int32_t queue_send_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
    if (block_time_tick == OSI_FUNCS_TIME_BLOCKING) {
        return (int32_t)xQueueSend(queue, item, portMAX_DELAY);
    } else {
        return (int32_t)xQueueSend(queue, item, block_time_tick);
    }
}

static int32_t IRAM_ATTR queue_send_from_isr_wrapper(void *queue, void *item, void *hptw)
{
    return (int32_t)xQueueSendFromISR(queue, item, hptw);
}

static int32_t queue_send_to_back_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
    return (int32_t)xQueueGenericSend(queue, item, block_time_tick, queueSEND_TO_BACK);
}

static int32_t queue_send_to_front_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
    return (int32_t)xQueueGenericSend(queue, item, block_time_tick, queueSEND_TO_FRONT);
}

static int32_t queue_recv_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
    if (block_time_tick == OSI_FUNCS_TIME_BLOCKING) {
        return (int32_t)xQueueReceive(queue, item, portMAX_DELAY);
    } else {
        return (int32_t)xQueueReceive(queue, item, block_time_tick);
    }
}

static uint32_t event_group_wait_bits_wrapper(void *event, uint32_t bits_to_wait_for, int clear_on_exit, int wait_for_all_bits, uint32_t block_time_tick)
{
    if (block_time_tick == OSI_FUNCS_TIME_BLOCKING) {
        return (uint32_t)xEventGroupWaitBits(event, bits_to_wait_for, clear_on_exit, wait_for_all_bits, portMAX_DELAY);
    } else {
        return (uint32_t)xEventGroupWaitBits(event, bits_to_wait_for, clear_on_exit, wait_for_all_bits, block_time_tick);
    }
}

static int32_t task_create_pinned_to_core_wrapper(void *task_func, const char *name, uint32_t stack_depth, void *param, uint32_t prio, void *task_handle, uint32_t core_id)
{
    return (uint32_t)xTaskCreatePinnedToCore(task_func, name, stack_depth, param, prio, task_handle, (core_id < portNUM_PROCESSORS ? core_id : tskNO_AFFINITY));
}

static int32_t task_create_wrapper(void *task_func, const char *name, uint32_t stack_depth, void *param, uint32_t prio, void *task_handle)
{
    return (uint32_t)xTaskCreate(task_func, name, stack_depth, param, prio, task_handle);
}

static int32_t IRAM_ATTR task_ms_to_tick_wrapper(uint32_t ms)
{
    return (int32_t)(ms / portTICK_PERIOD_MS);
}

static int32_t task_get_max_priority_wrapper(void)
{
    return (int32_t)(configMAX_PRIORITIES);
}

static int32_t esp_event_post_wrapper(const char* event_base, int32_t event_id, void* event_data, size_t event_data_size, uint32_t ticks_to_wait)
{
    if (ticks_to_wait == OSI_FUNCS_TIME_BLOCKING) {
        return (int32_t)esp_event_post(event_base, event_id, event_data, event_data_size, portMAX_DELAY);
    } else {
        return (int32_t)esp_event_post(event_base, event_id, event_data, event_data_size, ticks_to_wait);
    }
}
#if 0 /* use of FreeRTOS Timers */

typedef struct _esp_adapter_timer_st {
    void (*pCallback)(void *);
    void* args;
    TimerHandle_t OsTimerHandle;
    uint32_t  busy     :1;
    uint32_t  periodic :1;
    uint32_t  running  :1;
    volatile uint32_t  state    :3;
    uint32_t  period;
} ESP_AdapterTimer_st;

enum {
    _state_running,
    _state_stopped,
    _state_exec_cb,
};

#define __ESP_ADAPTER_MAX_TIMERS        20
static ESP_AdapterTimer_st __esp_adapter_timers_arr[__ESP_ADAPTER_MAX_TIMERS] = {0};
#define __is_valid_timer_info(ptr)      \
    ((uint32_t)(ptr) >= (uint32_t)__esp_adapter_timers_arr    \
        && (uint32_t)(ptr) < (uint32_t)(__esp_adapter_timers_arr + __ESP_ADAPTER_MAX_TIMERS))

typedef enum _esp_adapter_timers_errcode_et {
    ESP_ADAPTER_TIMERS_ERRCODE__NO_AVAIL_STATIC_LOC             = (1 << 0),
    ESP_ADAPTER_TIMERS_ERRCODE__NO_AVAIL_DYNAMIC_LOC            = (1 << 1),
    ESP_ADAPTER_TIMERS_ERRCODE__OS_TIMER_CREATION_FAILED        = (1 << 2),
    ESP_ADAPTER_TIMERS_ERRCODE__OS_TIMER_DELETE_FAILED          = (1 << 3),
    ESP_ADAPTER_TIMERS_ERRCODE__OS_TIMER_START_FAILED           = (1 << 4),
    ESP_ADAPTER_TIMERS_ERRCODE__OS_TIMER_STOP_FAILED            = (1 << 5),
    ESP_ADAPTER_TIMERS_ERRCODE__OS_TIMER_STOP_IN_CB_FAILED      = (1 << 6),
    ESP_ADAPTER_TIMERS_ERRCODE__OS_TIMER_CHANGE_PERIOD_FAILED   = (1 << 7),
    ESP_ADAPTER_TIMERS_ERRCODE__CALLBACK_NOT_VALID_TIMER_INFO   = (1 << 8),
    ESP_ADAPTER_TIMERS_ERRCODE__CALLBACK_OF_FREE_TIMER_INFO     = (1 << 9),
    ESP_ADAPTER_TIMERS_ERRCODE__TRY_INIT_AN_INITIALISED_TIMER   = (1 << 10),
    ESP_ADAPTER_TIMERS_ERRCODE__START_NON_INITIALISED_TIMER     = (1 << 11),
    ESP_ADAPTER_TIMERS_ERRCODE__STOP_NON_INITIALISED_TIMER      = (1 << 12),
    ESP_ADAPTER_TIMERS_ERRCODE__DELETE_NON_INITIALISED_TIMER    = (1 << 13),
} ESP_AdapterTimersErrCode_et;

static uint32_t __esp_adapter_timers_err_flags = 0;
#define __esp_adapter_timers_raise_flag(flag) __esp_adapter_timers_err_flags |= flag;

#define __store_timer_info(ptimer, pTimerInfo)  ((ETSTimer*)ptimer)->timer_arg = pTimerInfo
#define __load_timer_info(ptimer, pTimerInfo)   pTimerInfo = ((ETSTimer*)ptimer)->timer_arg
#define __is_initialized(ptimer) \
    (__is_valid_timer_info(((ETSTimer*)ptimer)->timer_arg) && \
    ((ESP_AdapterTimer_st*)(((ETSTimer*)ptimer)->timer_arg))->busy)

static void __esp_adapter_timers_callbask( TimerHandle_t pxTimer )
{
    /* callback function from FreeRTOS */

    ESP_AdapterTimer_st* pTimerInfo = pvTimerGetTimerID(pxTimer);

    if( ! __is_valid_timer_info(pTimerInfo)) {
        __esp_adapter_timers_raise_flag(ESP_ADAPTER_TIMERS_ERRCODE__CALLBACK_NOT_VALID_TIMER_INFO);
        return;
    }

    pTimerInfo->state = _state_exec_cb;

    if( ! pTimerInfo->busy ){
        __esp_adapter_timers_raise_flag(ESP_ADAPTER_TIMERS_ERRCODE__CALLBACK_OF_FREE_TIMER_INFO);
        return;
    }

    if( pTimerInfo->periodic == false ) {
        if( pdPASS != xTimerStop(pTimerInfo->OsTimerHandle, 0)) {
            __esp_adapter_timers_raise_flag(ESP_ADAPTER_TIMERS_ERRCODE__OS_TIMER_STOP_IN_CB_FAILED);
            return;
        }
        pTimerInfo->running = 0;
    }

    pTimerInfo->pCallback (pTimerInfo->args);

    pTimerInfo->state = pTimerInfo->periodic ? _state_running : _state_stopped;
}
static void timer_setfn_wrapper(void *ptimer, void *pfunction, void *parg)
{
    /* create the timer */

    if( __is_initialized(ptimer) ) {
        __esp_adapter_timers_raise_flag(ESP_ADAPTER_TIMERS_ERRCODE__TRY_INIT_AN_INITIALISED_TIMER);
        return;
    }

    int i = 0;
    ESP_AdapterTimer_st* pTimerInfo = __esp_adapter_timers_arr;
    while(i < __ESP_ADAPTER_MAX_TIMERS && pTimerInfo->busy)
    {
        ++i;
        ++pTimerInfo;
    }
    if(i >= __ESP_ADAPTER_MAX_TIMERS)
    {
        __esp_adapter_timers_raise_flag(ESP_ADAPTER_TIMERS_ERRCODE__NO_AVAIL_STATIC_LOC);
        return;
    }
    pTimerInfo->busy = true;
    pTimerInfo->pCallback = pfunction;
    pTimerInfo->args = parg;
    pTimerInfo->periodic = true;
    pTimerInfo->state = _state_stopped;

    pTimerInfo->OsTimerHandle = xTimerCreate( "WiFiDrvTmr",       // Just a text name, not used by the kernel.
        10000,   // The timer period in ticks.
        pdTRUE,        // The timers will auto-reload themselves when they expire.
        pTimerInfo,  // Assign each timer a unique id equal to its array index.
        __esp_adapter_timers_callbask // Each timer calls the same callback when it expires.
    );

    if(pTimerInfo->OsTimerHandle == NULL)
    {
        __esp_adapter_timers_raise_flag(ESP_ADAPTER_TIMERS_ERRCODE__OS_TIMER_CREATION_FAILED);
        return;
    }

    __store_timer_info(ptimer, pTimerInfo);
}
static void timer_done_wrapper(void *ptimer)
{
    /* delete the timer */

    if( ! __is_initialized(ptimer) ) {
        __esp_adapter_timers_raise_flag(ESP_ADAPTER_TIMERS_ERRCODE__DELETE_NON_INITIALISED_TIMER);
        return;
    }

    ESP_AdapterTimer_st* pTimerInfo;
    __load_timer_info(ptimer, pTimerInfo);

    if( pdPASS != xTimerDelete(pTimerInfo->OsTimerHandle, 0) ) {
        __esp_adapter_timers_raise_flag(ESP_ADAPTER_TIMERS_ERRCODE__OS_TIMER_DELETE_FAILED);
        return;
    }

    memset(pTimerInfo, 0, sizeof(*pTimerInfo));
    __store_timer_info(ptimer, NULL);
}
static void timer_arm_us_wrapper(void *timer, uint32_t us, bool repeat)
{
    /* start the timer */

    if( !__is_initialized(timer) ) {
        __esp_adapter_timers_raise_flag(ESP_ADAPTER_TIMERS_ERRCODE__START_NON_INITIALISED_TIMER);
        return;
    }

    ESP_AdapterTimer_st* pTimerInfo;
    __load_timer_info(timer, pTimerInfo);
    pTimerInfo->periodic = repeat;
    pTimerInfo->period = us / 1000;
    pTimerInfo->running = 1;
    pTimerInfo->state = _state_running;

    if( pdPASS != xTimerChangePeriod(pTimerInfo->OsTimerHandle, us / 1000 / portTICK_PERIOD_MS, 0) ) {
        __esp_adapter_timers_raise_flag(ESP_ADAPTER_TIMERS_ERRCODE__OS_TIMER_CHANGE_PERIOD_FAILED);
        return;
    }

    if( pdPASS != xTimerStart(pTimerInfo->OsTimerHandle, 0) ) {
        __esp_adapter_timers_raise_flag(ESP_ADAPTER_TIMERS_ERRCODE__OS_TIMER_CHANGE_PERIOD_FAILED);
        return;
    }
}
static void  timer_arm_wrapper(void *timer, uint32_t tmout, bool repeat)
{
    /* set timer period in us */

    timer_arm_us_wrapper(timer, tmout *1000, repeat);
}
static void  timer_disarm_wrapper(void *timer)
{
    /* stop the timer */

    if( !__is_initialized(timer) ) {
        __esp_adapter_timers_raise_flag(ESP_ADAPTER_TIMERS_ERRCODE__STOP_NON_INITIALISED_TIMER);
        return;
    }

    ESP_AdapterTimer_st* pTimerInfo;
    __load_timer_info(timer, pTimerInfo);

    if( pdPASS != xTimerStop(pTimerInfo->OsTimerHandle, 0) ) {
        __esp_adapter_timers_raise_flag(ESP_ADAPTER_TIMERS_ERRCODE__OS_TIMER_STOP_FAILED);
        return;
    }

    pTimerInfo->state = _state_stopped;
    pTimerInfo->running = 0;
}

void __esp_adapter_timers_test(void*param,void(*print)(void*,const char* fmt, ...))
{
    int busy_timers_count = 0;
    int i = 0;
    ESP_AdapterTimer_st* pTimerInfo = __esp_adapter_timers_arr;

    print(param, "-- ESP Adapter Timers Logs\n");
    for(i=0; i < __ESP_ADAPTER_MAX_TIMERS; ++i, ++pTimerInfo) {
        if(pTimerInfo->busy) {
            ++ busy_timers_count;
            print(param, "   [%2d] %5d ms ~~ %s ~~ %s\n", i, pTimerInfo->period,
                pTimerInfo->state == _state_stopped ? "STOPPED"
                    : pTimerInfo->state == _state_exec_cb ? "EXEC_CB" : "RUNNING",
                pTimerInfo->periodic ? "Periodic":"Once");
        }
    }

    print(param, "   == ERROR FLAGS :: %08X\n", __esp_adapter_timers_err_flags);
}

#else /* use of the ETS Timers */
void __esp_adapter_timers_test(void*param,void(*print)(void*,const char* fmt, ...))
{
}
static void IRAM_ATTR timer_arm_wrapper(void *timer, uint32_t tmout, bool repeat)
{
    ets_timer_arm(timer, tmout, repeat);
}

static void IRAM_ATTR timer_disarm_wrapper(void *timer)
{
    ets_timer_disarm(timer);
}
uint8_t __esp_adapter_done_timers_arr[30] = {0};
int     __esp_adapter_done_timers_counter = 0;
static void timer_done_wrapper(void *ptimer)
{
    ets_timer_done(ptimer);
}

static void* __adapter_cbs[30]={ 0 };
int __esp_adapter_timers_counter = 0;
extern char __which_callback;
#define __adabter_fun_cb(id) \
    static void __adapter_fun_cb_##id(void *timer_arg){ \
    __which_callback = 'A' + id;                        \
    ((ETSTimerFunc*)__adapter_cbs[id])(timer_arg);      \
    }
__adabter_fun_cb(0)
__adabter_fun_cb(1)
__adabter_fun_cb(2)
__adabter_fun_cb(3)
__adabter_fun_cb(4)
__adabter_fun_cb(5)
__adabter_fun_cb(6)
__adabter_fun_cb(7)
__adabter_fun_cb(8)
__adabter_fun_cb(9)
__adabter_fun_cb(10)
__adabter_fun_cb(11)
__adabter_fun_cb(12)
__adabter_fun_cb(13)
__adabter_fun_cb(14)

__adabter_fun_cb(15)

__adabter_fun_cb(16)
__adabter_fun_cb(17)
__adabter_fun_cb(18)
__adabter_fun_cb(19)
__adabter_fun_cb(20)
__adabter_fun_cb(21)
__adabter_fun_cb(22)
__adabter_fun_cb(23)
__adabter_fun_cb(24)
__adabter_fun_cb(25)
__adabter_fun_cb(26)
__adabter_fun_cb(27)
__adabter_fun_cb(28)
__adabter_fun_cb(29)
static void* __adapter_pfunction[] = {
    (void*)__adapter_fun_cb_0,
    (void*)__adapter_fun_cb_1,
    (void*)__adapter_fun_cb_2,
    (void*)__adapter_fun_cb_3,
    (void*)__adapter_fun_cb_4,
    (void*)__adapter_fun_cb_5,
    (void*)__adapter_fun_cb_6,
    (void*)__adapter_fun_cb_7,
    (void*)__adapter_fun_cb_8,
    (void*)__adapter_fun_cb_9,
    (void*)__adapter_fun_cb_10,
    (void*)__adapter_fun_cb_11,
    (void*)__adapter_fun_cb_12,
    (void*)__adapter_fun_cb_13,
    (void*)__adapter_fun_cb_14,
    (void*)__adapter_fun_cb_15,
    (void*)__adapter_fun_cb_16,
    (void*)__adapter_fun_cb_17,
    (void*)__adapter_fun_cb_18,
    (void*)__adapter_fun_cb_19,
    (void*)__adapter_fun_cb_20,
    (void*)__adapter_fun_cb_21,
    (void*)__adapter_fun_cb_22,
    (void*)__adapter_fun_cb_23,
    (void*)__adapter_fun_cb_24,
    (void*)__adapter_fun_cb_25,
    (void*)__adapter_fun_cb_26,
    (void*)__adapter_fun_cb_27,
    (void*)__adapter_fun_cb_28,
    (void*)__adapter_fun_cb_29,
};

static void timer_setfn_wrapper(void *ptimer, void *pfunction, void *parg)
{
    ets_timer_setfn(ptimer, pfunction, parg);
    //__adapter_cbs[__esp_adapter_timers_counter] = pfunction;
    //ets_timer_setfn(ptimer, __adapter_pfunction[__esp_adapter_timers_counter++], parg);
}

static void IRAM_ATTR timer_arm_us_wrapper(void *ptimer, uint32_t us, bool repeat)
{
    ets_timer_arm_us(ptimer, us, repeat);
}
#endif
static int get_time_wrapper(void *t)
{
    return os_get_time(t);
}

static void * IRAM_ATTR malloc_internal_wrapper(size_t size)
{
    return heap_caps_malloc(size, MALLOC_CAP_8BIT|MALLOC_CAP_DMA|MALLOC_CAP_INTERNAL);
}

static void * IRAM_ATTR realloc_internal_wrapper(void *ptr, size_t size)
{
    return heap_caps_realloc(ptr, size, MALLOC_CAP_8BIT|MALLOC_CAP_DMA|MALLOC_CAP_INTERNAL);
}

static void * IRAM_ATTR calloc_internal_wrapper(size_t n, size_t size)
{
    return heap_caps_calloc(n, size, MALLOC_CAP_8BIT|MALLOC_CAP_DMA|MALLOC_CAP_INTERNAL);
}

static void * IRAM_ATTR zalloc_internal_wrapper(size_t size)
{
    void *ptr = heap_caps_calloc(1, size, MALLOC_CAP_8BIT|MALLOC_CAP_DMA|MALLOC_CAP_INTERNAL);
    if (ptr) {
        memset(ptr, 0, size);
    }
    return ptr;
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
#if CONFIG_SW_COEXIST_ENABLE
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

int IRAM_ATTR coex_bt_request_wrapper(uint32_t event, uint32_t latency, uint32_t duration)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
    return coex_bt_request(event, latency, duration);
#else
    return 0;
#endif
}

int IRAM_ATTR coex_bt_release_wrapper(uint32_t event)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
    return coex_bt_release(event);
#else
    return 0;
#endif
}

int coex_register_bt_cb_wrapper(coex_func_cb_t cb)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
    return coex_register_bt_cb(cb);
#else
    return 0;
#endif
}

uint32_t IRAM_ATTR coex_bb_reset_lock_wrapper(void)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
    return coex_bb_reset_lock();
#else
    return 0;
#endif
}

void IRAM_ATTR coex_bb_reset_unlock_wrapper(uint32_t restore)
{
#if CONFIG_ESP32_WIFI_SW_COEXIST_ENABLE
    coex_bb_reset_unlock(restore);
#endif
}

int32_t IRAM_ATTR coex_is_in_isr_wrapper(void)
{
    return !xPortCanYield();
}

wifi_osi_funcs_t g_wifi_osi_funcs = {
    ._version = ESP_WIFI_OS_ADAPTER_VERSION,
    ._set_isr = set_isr_wrapper,
    ._ints_on = xt_ints_on,
    ._ints_off = xt_ints_off,
    ._spin_lock_create = spin_lock_create_wrapper,
    ._spin_lock_delete = free,
    ._wifi_int_disable = wifi_int_disable_wrapper,
    ._wifi_int_restore = wifi_int_restore_wrapper,
    ._task_yield_from_isr = task_yield_from_isr_wrapper,
    ._semphr_create = semphr_create_wrapper,
    ._semphr_delete = semphr_delete_wrapper,
    ._semphr_take = semphr_take_wrapper,
    ._semphr_give = semphr_give_wrapper,
    ._wifi_thread_semphr_get = wifi_thread_semphr_get_wrapper,
    ._mutex_create = mutex_create_wrapper,
    ._recursive_mutex_create = recursive_mutex_create_wrapper,
    ._mutex_delete = mutex_delete_wrapper,
    ._mutex_lock = mutex_lock_wrapper,
    ._mutex_unlock = mutex_unlock_wrapper,
    ._queue_create = queue_create_wrapper,
    ._queue_delete = (void(*)(void *))vQueueDelete,
    ._queue_send = queue_send_wrapper,
    ._queue_send_from_isr = queue_send_from_isr_wrapper,
    ._queue_send_to_back = queue_send_to_back_wrapper,
    ._queue_send_to_front = queue_send_to_front_wrapper,
    ._queue_recv = queue_recv_wrapper,
    ._queue_msg_waiting = (uint32_t(*)(void *))uxQueueMessagesWaiting,
    ._event_group_create = (void *(*)(void))xEventGroupCreate,
    ._event_group_delete = (void(*)(void *))vEventGroupDelete,
    ._event_group_set_bits = (uint32_t(*)(void *,uint32_t))xEventGroupSetBits,
    ._event_group_clear_bits = (uint32_t(*)(void *,uint32_t))xEventGroupClearBits,
    ._event_group_wait_bits = event_group_wait_bits_wrapper,
    ._task_create_pinned_to_core = task_create_pinned_to_core_wrapper,
    ._task_create = task_create_wrapper,
    ._task_delete = (void(*)(void *))vTaskDelete,
    ._task_delay = vTaskDelay,
    ._task_ms_to_tick = task_ms_to_tick_wrapper,
    ._task_get_current_task = (void *(*)(void))xTaskGetCurrentTaskHandle,
    ._task_get_max_priority = task_get_max_priority_wrapper,
    ._malloc = malloc,
    ._free = free,
    ._event_post = esp_event_post_wrapper,
    ._get_free_heap_size = esp_get_free_heap_size,
    ._rand = esp_random,
    ._dport_access_stall_other_cpu_start_wrap = s_esp_dport_access_stall_other_cpu_start,
    ._dport_access_stall_other_cpu_end_wrap = s_esp_dport_access_stall_other_cpu_end,
    ._phy_rf_deinit = esp_phy_rf_deinit,
    ._phy_load_cal_and_init = esp_phy_load_cal_and_init,
    ._phy_common_clock_enable = esp_phy_common_clock_enable,
    ._phy_common_clock_disable = esp_phy_common_clock_disable,
    ._read_mac = esp_read_mac,
    ._timer_arm = timer_arm_wrapper,
    ._timer_disarm = timer_disarm_wrapper,
    ._timer_done = timer_done_wrapper,
    ._timer_setfn = timer_setfn_wrapper,
    ._timer_arm_us = timer_arm_us_wrapper,
    ._periph_module_enable = periph_module_enable,
    ._periph_module_disable = periph_module_disable,
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
    ._random = os_random,
    ._log_write = esp_log_write,
    ._log_writev = esp_log_writev,
    ._log_timestamp = esp_log_timestamp,
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
    ._modem_sleep_enter = esp_modem_sleep_enter,
    ._modem_sleep_exit = esp_modem_sleep_exit,
    ._modem_sleep_register = esp_modem_sleep_register,
    ._modem_sleep_deregister = esp_modem_sleep_deregister,
    ._coex_status_get = coex_status_get_wrapper,
    ._coex_condition_set = coex_condition_set_wrapper,
    ._coex_wifi_request = coex_wifi_request_wrapper,
    ._coex_wifi_release = coex_wifi_release_wrapper,
    ._magic = ESP_WIFI_OS_ADAPTER_MAGIC,
};

coex_adapter_funcs_t g_coex_adapter_funcs = {
    ._version = COEX_ADAPTER_VERSION,
    ._spin_lock_create = spin_lock_create_wrapper,
    ._spin_lock_delete = free,
    ._int_disable = wifi_int_disable_wrapper,
    ._int_enable = wifi_int_restore_wrapper,
    ._task_yield_from_isr = task_yield_from_isr_wrapper,
    ._semphr_create = semphr_create_wrapper,
    ._semphr_delete = semphr_delete_wrapper,
    ._semphr_take_from_isr = semphr_take_from_isr_wrapper,
    ._semphr_give_from_isr = semphr_give_from_isr_wrapper,
    ._semphr_take = semphr_take_wrapper,
    ._semphr_give = semphr_give_wrapper,
    ._is_in_isr = coex_is_in_isr_wrapper,
    ._malloc_internal =  malloc_internal_wrapper,
    ._free = free,
    ._timer_disarm = timer_disarm_wrapper,
    ._timer_done = timer_done_wrapper,
    ._timer_setfn = timer_setfn_wrapper,
    ._timer_arm_us = timer_arm_us_wrapper,
    ._esp_timer_get_time = esp_timer_get_time,
    ._magic = COEX_ADAPTER_MAGIC,
};
