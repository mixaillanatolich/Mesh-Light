/* Copyright (c) 2010 - 2020, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <string.h>

/* HAL */
#include "boards.h"
#include "app_timer.h"

#include "simple_hal.h"

/* Core */
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"
#include "proxy.h"
#include "flash_manager.h"
#include "mesh_config.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Models */
#include "generic_onoff_server.h"
#include "generic_level_server.h"
#include "access.h"
/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "nrf_mesh_config_app.h"
#include "nrf.h"
#include "example_common.h"
#include "pwm_utils.h"
#include "nrf_mesh_config_examples.h"
#include "app_onoff.h"
#include "app_level.h"
#include "ble_softdevice_support.h"
#include "config_server.h"
#include "app_config_models.h"
#include "app_main_config.h"
#include "flash_helper.h"
#include "app_scheduler.h"

#include "nrf_mesh_dfu.h"

#include "base_communication_common.h"
#include "base_communication_client.h"
#include "base_communication_server.h"

#include "serial_commands.h"
/*****************************************************************************
 * Definitions
 *****************************************************************************/


/*****************************************************************************
 * Variables
 *****************************************************************************/

static const uint8_t    app_version = 1;
static const uint16_t   app_build   = 5;

static bool m_device_provisioned;
static dsm_local_unicast_address_t node_address;

char *device_name_with_addr;

static led_config_t led1_config;

static timestamp_t commissioning_start_time = 0;

static uint8_t configure_finished_successfull;
static uint8_t random_start_publish_delay = 0;

static bool need_status_publish;
static bool need_level_publish;

static bool health_publish_started = false;

static base_communication_server_t m_base_communication_command_server;
static base_communication_client_t m_base_communication_command_client;
 /*****************************************************************************
 * END variables
 *****************************************************************************/

/*****************************************************************************
 * DFU Support
 *****************************************************************************/

static bool fw_updated_event_is_for_me(const nrf_mesh_evt_dfu_t * p_evt)
{
    switch (p_evt->fw_outdated.transfer.dfu_type)
    {
        case NRF_MESH_DFU_TYPE_APPLICATION:
            return (p_evt->fw_outdated.current.application.app_id == p_evt->fw_outdated.transfer.id.application.app_id &&
                    p_evt->fw_outdated.current.application.company_id == p_evt->fw_outdated.transfer.id.application.company_id &&
                    p_evt->fw_outdated.current.application.app_version < p_evt->fw_outdated.transfer.id.application.app_version);

        case NRF_MESH_DFU_TYPE_BOOTLOADER:
            return (p_evt->fw_outdated.current.bootloader.bl_id == p_evt->fw_outdated.transfer.id.bootloader.bl_id &&
                    p_evt->fw_outdated.current.bootloader.bl_version < p_evt->fw_outdated.transfer.id.bootloader.bl_version);

        case NRF_MESH_DFU_TYPE_SOFTDEVICE:
            return false;

        default:
            return false;
    }
}

static const uint32_t * optimal_bank_address(void)
{
    /* The incoming transfer has to fit on both sides of the bank address: First it needs to fit
     * above the bank address when we receive it, then it needs to fit below the bank address when
     * we install it. We want to put the bank address in the middle of the available application
     * code area, to maximize the potential transfer size we can accept. */
    const uint32_t * p_start;
    uint32_t dummy;
    ERROR_CHECK(mesh_stack_persistence_flash_usage(&p_start, &dummy));

    __LOG_RELEASE(LOG_SRC_APP, LOG_LEVEL_INFO, "p_start %X\n", (intptr_t)p_start);
    __LOG_RELEASE(LOG_SRC_APP, LOG_LEVEL_INFO, "dummy %X\n", dummy);
    __LOG_RELEASE(LOG_SRC_APP, LOG_LEVEL_INFO, "CODE_START %X\n", CODE_START);
    __LOG_RELEASE(LOG_SRC_APP, LOG_LEVEL_INFO, "CODE_END %X\n", CODE_END);
    __LOG_RELEASE(LOG_SRC_APP, LOG_LEVEL_INFO, "CODE_SIZE %X\n", CODE_SIZE);

    uint32_t middle_of_app_area = (CODE_START + (intptr_t) p_start) / 2;
    __LOG_RELEASE(LOG_SRC_APP, LOG_LEVEL_INFO, "middle_of_app_area %X\n", middle_of_app_area);

    /* The bank can't start in the middle of the application code, and should be page aligned: */
    const uint32_t *bank_address = (const uint32_t *) ALIGN_VAL(MAX(middle_of_app_area, CODE_END), PAGE_SIZE);
    __LOG_RELEASE(LOG_SRC_APP, LOG_LEVEL_INFO, "bank_address %X\n", bank_address);
    
    return bank_address;
}
/*
         15747>, main.c,  180, p_start 71000
<t:      15749>, main.c,  181, dummy 6000
<t:      15751>, main.c,  182, CODE_START 26000
<t:      15753>, main.c,  183, CODE_END 4ECAC
<t:      15755>, main.c,  184, CODE_SIZE 28CAC
<t:      15757>, main.c,  187, middle_of_app_area 4B800
<t:      15760>, main.c,  191, bank_address 4F000

*/



static void factory_reset(void);

/*****************************************************************************
 * Forward declaration of static functions
 *****************************************************************************/
static void app_onoff_server_set_cb(const app_onoff_server_t * p_server, bool onoff);
static void app_onoff_server_get_cb(const app_onoff_server_t * p_server, bool * p_present_onoff);
static void app_onoff_server_transition_cb(const app_onoff_server_t * p_server,
                                                uint32_t transition_time_ms, bool target_onoff);


static void app_level_server_set_cb(const app_level_server_t * p_server, uint16_t present_level);
static void app_level_server_get_cb(const app_level_server_t * p_server, uint16_t * p_present_level);
static void app_level_server_transition_cb(const app_level_server_t * p_server,
                                                uint32_t transition_time_ms, uint16_t target_level,
                                                app_transition_type_t transition_type);


/*****************************************************************************
 * Static variables
 *****************************************************************************/


/* PWM hardware instance and associated variables */
/* Note: PWM cycle period determines the the max value that can be used to represent 100%
 * duty cycles, therefore present_level value scaling is required to get pwm tick value
 * between 0 and pwm_utils_contex_t:pwm_ticks_max.
 */
static APP_PWM_INSTANCE(PWM0, 1);
static app_pwm_config_t m_pwm0_config = APP_PWM_DEFAULT_CONFIG_1CH_LOW(200, BSP_LED_0);
static pwm_utils_contex_t m_pwm = {
                                    .p_pwm = &PWM0,
                                    .p_pwm_config = &m_pwm0_config,
                                    .channel = 0
                                  };


/****************************** Storage *************************************/

/* Try to store app data. If busy, ask user to manually initiate operation. */
static void save_led_config() {

    if (!m_device_provisioned) { return; }

    uint32_t status = app_flash_data_store(LED1_CONFIG_ENTRY_HANDLE, &led1_config, sizeof(led1_config));
    if (status == NRF_SUCCESS) {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Storing: Led config\n");
    } else if (status == NRF_ERROR_NOT_SUPPORTED) {
       __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Cannot store Led config: Persistent storage not enabled\n");
    } else {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Flash busy. Cannot store Led config. To try again. \n");
    }

}

static void load_led_config() {

    if (app_flash_data_load(LED1_CONFIG_ENTRY_HANDLE, &led1_config, sizeof(led1_config)) == NRF_SUCCESS) {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Led config loaded. level: %x - %d\n", led1_config.level, led1_config.level);
        
    } else {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Failed to load led config\n");

        if (m_device_provisioned) {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Provisioning not finished successfully! hard reset!\n");
            factory_reset();
            return;
        }

        led1_config.on = true;
        led1_config.level = LED_LEVEL_DEFAULT;
        led1_config.min_level = 0;
        led1_config.max_level = UINT16_MAX;

        save_led_config();
    }
}

/****************************************************************************/

/************************** LED Control *************************************/

static void apply_led_config() {
    if (led1_config.on) {
        pwm_utils_enable(&m_pwm);
        pwm_utils_level_set(&m_pwm, led1_config.level);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Get: Level: %d\n", pwm_utils_level_get(&m_pwm));
    } else {
        pwm_utils_disable(&m_pwm);
    }

    serial_send_on_off_command(led1_config.on);
}

void led1_level_set(uint16_t level) {
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "SET: Level: %d\n", level);
    led1_config.level = level;
    apply_led_config();
    save_led_config();
}

void led1_state_on_set(bool is_on) {
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "SET: State: %d\n", is_on);
    led1_config.on = is_on;
    apply_led_config();
    save_led_config();
}


/****************************************************************************/


/* Generic OnOff server structure definition and initialization */
APP_ONOFF_SERVER_DEF(m_onoff_server_0,
                     APP_FORCE_SEGMENTATION,
                     APP_MIC_SIZE,
                     app_onoff_server_set_cb,
                     app_onoff_server_get_cb,
                     app_onoff_server_transition_cb)

/* Callback for updating the hardware state */
static void app_onoff_server_set_cb(const app_onoff_server_t * p_server, bool onoff)
{
    /* Resolve the server instance here if required, this example uses only 1 instance. */

    led1_state_on_set(onoff);
}

/* Callback for reading the hardware state */
static void app_onoff_server_get_cb(const app_onoff_server_t * p_server, bool * p_present_onoff)
{
    /* Resolve the server instance here if required, this example uses only 1 instance. */


    *p_present_onoff = led1_config.on;
}

/* Callback for updating the hardware state */
static void app_onoff_server_transition_cb(const app_onoff_server_t * p_server,
                                                uint32_t transition_time_ms, bool target_onoff)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transition time: %d, Target OnOff: %d\n",
                                       transition_time_ms, target_onoff);
}

/* Application level generic level server structure definition and initialization */
APP_LEVEL_SERVER_DEF(m_level_server_0,
                     APP_FORCE_SEGMENTATION,
                     APP_MIC_SIZE,
                     NULL,
                     app_level_server_set_cb,
                     app_level_server_get_cb,
                     app_level_server_transition_cb);


/* Callback for updating the hardware state */
static void app_level_server_set_cb(const app_level_server_t * p_server, uint16_t present_level)
{
    led1_level_set(present_level);
}

/* Callback for reading the hardware state */
static void app_level_server_get_cb(const app_level_server_t * p_server, uint16_t * p_present_level)
{
    *p_present_level = led1_config.level;
}

/* Callback for updateing according to transition time. */
static void app_level_server_transition_cb(const app_level_server_t * p_server,
                                                uint32_t transition_time_ms, uint16_t target_level,
                                                app_transition_type_t transition_type)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transition time: %d, Target level: %d, Transition type: %d\n",
                                       transition_time_ms, target_level, transition_type);
}

/*************************************************************************************************/
void serial_command_received(serial_command_t command){
    uint8_t command_code = command.command_id;

    switch (command_code) {
        case SERIAL_COMMAND_ACK:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "SERIAL: ACK command received\n");
            break;
        case SERIAL_COMMAND_FACTORY_RESET:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "SERIAL: Factory Reset command received\n");
            // Send ACK
            serial_send_ack_command(0);

            factory_reset();
            break;
        default:
            // UNKNOWN COMMAND
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "SERIAL: Unknown command received\n");
    }
}

void serial_status_received(bool connected) {
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "serial_status_received %d \n", connected);

}


/************************** base communication command handle *************************************/

uint16_t convertFrom8To16(uint8_t dataFirst, uint8_t dataSecond) {
    uint16_t dataBoth = 0x0000;

    dataBoth = dataFirst;
    dataBoth = dataBoth << 8;
    dataBoth |= dataSecond;
    return dataBoth;
}



static uint8_t base_communication_command_cb(const base_communication_server_t * p_server, 
                                              const base_communication_msg_t *command, 
                                              base_communication_msg_response_t * p_response) {

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "base_ommunication_command_id %02x\n",command->opcode);
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "tid %d \n",command->tid);
   // __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "length %d \n",command->length);
 //   __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Unencrypted data", command->command_buffer, command->length);

    if (command->opcode == BASE_COMMUNICATION_SUB_COMMAND_COMMISSIONING_FINISHED) {

        fwid_t* p_fwid = nrf_mesh_dfu_current_fwid_get();

        static uint8_t msg[18];
        msg[0] = configure_finished_successfull;
        msg[1] = app_version;
        msg[2] = (app_build & 0xff);
        msg[3] = (app_build >> 8);

        msg[4] = (p_fwid->sd & 0xff);
        msg[5] = (p_fwid->sd >> 8);
        msg[6] = p_fwid->bootloader.bl_id;
        msg[7] = p_fwid->bootloader.bl_version;
        
        msg[8] = (uint8_t)p_fwid->app.company_id;
        msg[9] = (uint8_t)(p_fwid->app.company_id>>8);
        msg[10] = (uint8_t)(p_fwid->app.company_id>>16);
        msg[11] = (uint8_t)(p_fwid->app.company_id>>24);

        msg[12] = (p_fwid->app.app_id & 0xff);
        msg[13] = (p_fwid->app.app_id >> 8);

        msg[14] = (uint8_t)p_fwid->app.app_version;
        msg[15] = (uint8_t)(p_fwid->app.app_version>>8);
        msg[16] = (uint8_t)(p_fwid->app.app_version>>16);
        msg[17] = (uint8_t)(p_fwid->app.app_version>>24);
        
        p_response->response = &msg[0];
        p_response->length = 18;
        
        if (!configure_finished_successfull) {
            m_device_provisioned = true;
            commissioning_start_time = 0;
            led1_config.on = true;
            led1_config.level = LED_LEVEL_DEFAULT;
            led1_config.min_level = 0;
            led1_config.max_level = UINT16_MAX;

            save_led_config();
        }

        return BASE_COMMUNICATION_STATUS_MESSAGE;
    } else if (command->opcode == BASE_COMMUNICATION_SUB_COMMAND_IP_ADDR_GET) {

        ble_gap_addr_t ble_addr;
        sd_ble_gap_addr_get(&ble_addr);

        static uint8_t msg[6];
        msg[0] = ble_addr.addr[0];
        msg[1] = ble_addr.addr[1];
        msg[2] = ble_addr.addr[2];
        msg[3] = ble_addr.addr[3];
        msg[4] = ble_addr.addr[4];
        msg[5] = ble_addr.addr[5];

        p_response->response = &msg[0];
        p_response->length = 6;

        return BASE_COMMUNICATION_STATUS_MESSAGE;

    } else if (command->opcode == BASE_COMMUNICATION_SUB_COMMAND_FW_VERSION_GET) {

        static uint8_t msg[3];
        msg[0] = app_version;
        msg[1] = (app_build & 0xff);
        msg[2] = (app_build >> 8);

        p_response->response = &msg[0];
        p_response->length = 3;

        return BASE_COMMUNICATION_STATUS_MESSAGE;

    } else if (command->opcode == BASE_COMMUNICATION_SUB_COMMAND_FW_DFU) {
        uint32_t res = nrf_mesh_dfu_rx(command->command, command->length, NULL);
        uint8_t *res_p = (uint8_t *)&res;
        static uint8_t msg[4];
        msg[0] = res_p[0];
        msg[1] = res_p[1];
        msg[2] = res_p[2];
        msg[3] = res_p[3];

        p_response->response = &msg[0];
        p_response->length = 4;
        if (res != 0) {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Msg resp %d - %x\n",res, res);
        }
        return BASE_COMMUNICATION_STATUS_MESSAGE;

    } 

    return BASE_COMMUNICATION_STATUS_ERROR_NO_HANDLER;
}

static void base_communication_command_timeout_cb(access_model_handle_t handle, void * p_client)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Acknowledged send timedout\n");
}

static void base_communication_command_status_cb(const base_communication_client_t * p_client, base_communication_status_t status, uint16_t src)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "src %02x \n", src);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "status %02x \n", status);
}

/****************************************************************************/

static void app_model_init(void)
{
    /* Instantiate onoff server on element index APP_ONOFF_ELEMENT_INDEX */
    ERROR_CHECK(app_onoff_init(&m_onoff_server_0, APP_ONOFF_ELEMENT_INDEX));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App OnOff Model Handle: %d\n", m_onoff_server_0.server.model_handle);

    /* Instantiate level server on element index 0 */
    ERROR_CHECK(app_level_init(&m_level_server_0, APP_LEVEL_ELEMENT_INDEX));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App Level Model Handle: %d\n", m_level_server_0.server.model_handle);

    m_base_communication_command_server.command_cb = base_communication_command_cb;
    ERROR_CHECK(base_communication_server_init(&m_base_communication_command_server, 0));
    ERROR_CHECK(access_model_subscription_list_alloc(m_base_communication_command_server.model_handle));

    m_base_communication_command_client.status_cb = base_communication_command_status_cb;
    m_base_communication_command_client.timeout_cb = base_communication_command_timeout_cb;
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing client %d\n", base_generic_client_init(&m_base_generic_command_client, 0));
    ERROR_CHECK(base_communication_client_init(&m_base_communication_command_client, 0));
    ERROR_CHECK(access_model_subscription_list_alloc(m_base_communication_command_client.model_handle));
}

/*************************************************************************************************/

static void node_reset(void) {
    __LOG_RELEASE(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    /* This function may return if there are ongoing flash operations. */
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    mesh_stack_device_reset();
}

static void factory_reset(void) {

//    if (mesh_stack_is_device_provisioned()) {
#if MESH_FEATURE_GATT_PROXY_ENABLED
        (void) proxy_stop();
#endif
        app_flash_clear(&led1_config, sizeof(led1_config));
        mesh_stack_config_clear();
        node_reset();
//    } else {
//        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "The device is unprovisioned. Resetting has no effect.\n");
//    }

}

static void config_server_evt_cb(const config_server_evt_t * p_evt) {
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET) {
        /* Trigger clearing of application data and schedule node reset. */
        factory_reset();
    } else if (p_evt->type == CONFIG_SERVER_EVT_APPKEY_ADD) {
        uint32_t status = app_config_bind_app_keys(node_address);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "app_config_bind_app_keys: %x\n",status);
        if (status != NRF_SUCCESS) {
            configure_finished_successfull = 0x01;
            return;
        }
         status = app_config_health_model_publication(node_address);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "app_config_health_model_publication: %x\n",status);
        if (status != NRF_SUCCESS) {
            configure_finished_successfull = 0x02;
            return;
        }
        status = app_config_led_status_model_publication(node_address);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "app_config_led_status_model_publication: %x\n",status);
        if (status != NRF_SUCCESS) {
            configure_finished_successfull = 0x03;
            return;
        }
        status = app_config_led_level_model_publication(node_address);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "app_config_led_level_model_publication: %x\n",status);
        if (status != NRF_SUCCESS) {
            configure_finished_successfull = 0x04;
            return;
        }
        status = app_config_subscription_to_communication_group(node_address);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "app_config_subscription_to_communication_group: %x\n",status);
        if (status != NRF_SUCCESS) {
            configure_finished_successfull = 0x05;
            return;
        }        
        status = app_config_communication_client_model_publication(node_address);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "app_config_communication_client_model_publication: %x\n",status);
        if (status != NRF_SUCCESS) {
            configure_finished_successfull = 0x06;
            return;
        }

    }
}

#if NRF_MESH_LOG_ENABLE
static const char m_usage_string[] = "run!\n";
#endif

static void button_event_handler(uint32_t button_number)
{
    /* Increase button number because the buttons on the board is marked with 1 to 4 */
    button_number++;
    uint16_t new_level = 0;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);
    switch (button_number) {
        case 1:{
            new_level = (led1_config.level < APP_LEVEL_STEP_SIZE) ? 0 : led1_config.level - APP_LEVEL_STEP_SIZE;
            break;
        }

        case 2: {
            new_level = ((UINT16_MAX - led1_config.level) < APP_LEVEL_STEP_SIZE) ? UINT16_MAX : led1_config.level + APP_LEVEL_STEP_SIZE;
            break;
        }

        case 3: {
            led1_state_on_set(!led1_config.on);
            app_onoff_status_publish(&m_onoff_server_0);
            break;
        }

        /* Initiate node reset */
        case 4: {
            /* Clear all the states to reset the node. */
            factory_reset();
            break;
        }

        default:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
            break;
    }

    if (button_number == 1 || button_number == 2) {
        led1_level_set(new_level);
        app_level_current_value_publish(&m_level_server_0);
    }
}

static void app_rtt_input_handler(int key)
{
    if (key >= '1' && key <= '4')
    {
        uint32_t button_number = key - '1';
        button_event_handler(button_number);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
    }
}

static void device_identification_start_cb(uint8_t attention_duration_s)
{

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "device_identification_start_cb\n");

    hal_led_mask_set(LEDS_MASK, false);
    hal_led_blink_ms(BSP_LED_2_MASK  | BSP_LED_3_MASK,
                     LED_BLINK_ATTENTION_INTERVAL_MS,
                     LED_BLINK_ATTENTION_COUNT(attention_duration_s));
}

static void device_identification_stop_cb()
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "device_identification_stop_cb\n");
}

static void provisioning_aborted_cb(void)
{

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "provisioning_aborted_cb\n");

    hal_led_blink_stop();
}

static void unicast_address_print(void)
{
    __LOG_RELEASE(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);
}

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

#if MESH_FEATURE_GATT_ENABLED
    /* Restores the application parameters after switching from the Provisioning
     * service to the Proxy  */
    gap_params_init();
    conn_params_init();
#endif

    commissioning_start_time = timer_now();

    dsm_local_unicast_addresses_get(&node_address);

    unicast_address_print();

    hal_led_blink_stop();
    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    app_model_init();
}

static void random_delay() {
    ble_gap_addr_t ble_addr;
    sd_ble_gap_addr_get(&ble_addr);
    uint8_t sum = 0;
    sum += ble_addr.addr[0];
    sum += ble_addr.addr[1];
    sum += ble_addr.addr[2];
    sum += ble_addr.addr[3];
    sum += ble_addr.addr[4];
    sum += ble_addr.addr[5];
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "random delay %d - %d\n", sum, sum/14);
    random_start_publish_delay = sum/14;
}

/********** WDT Timer ***********/
void wdt_timer_init(void) {
    //Configure Watchdog. a) Pause watchdog while the CPU is halted by the debugger.  b) Keep the watchdog running while the CPU is sleeping.
    NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | ( WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);  
    NRF_WDT->CRV = 3*32768;             //ca 3 sek. timout
    NRF_WDT->RREN |= WDT_RREN_RR0_Msk;  //Enable reload register 0
    NRF_WDT->TASKS_START = 1;           //Start the Watchdog timer
}

static void reset_wdt_timer(void) {
    NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}


/********** checks ***********/


static void check_commissioning_state() {

    if (commissioning_start_time == 0) return;

    if (timer_diff(timer_now(), commissioning_start_time) > SEC_TO_US(30)) {
        commissioning_start_time = 0;
        factory_reset();
    }
} 

static void check_health_server_start() {
    if (!health_publish_started && m_device_provisioned && timer_now() > SEC_TO_US(random_start_publish_delay) ) {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "health_publish_started\n");
        health_publish_started = true;
        app_config_health_model_publication(node_address);
    }

    if (m_device_provisioned && need_send_welcome_event()) {
        need_status_publish = true;
        schedule_event(4);
    }
}

static void check_send_event() {
    if (need_status_publish) {
        need_status_publish = false;
        uint32_t res = app_onoff_status_publish(&m_onoff_server_0);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "simple_on_off_server_status_publish res: %x\n",res);
    } else if (need_level_publish) {
        need_level_publish = false;
        uint32_t res = app_level_current_value_publish(&m_level_server_0);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "simple_level_server_status_publish res: %x\n",res);
    }
}

/********** App scheduler ***********/

/* Select max event size. This must be large enough for app timer events as well.
 * uint8_t => button number variable
 * APP_TIMER_SCHED_EVENT_DATA_SIZE => App timer event
 */
#define APP_SCHED_EVENT_SIZE        MAX(sizeof(uint8_t), APP_TIMER_SCHED_EVENT_DATA_SIZE)
#define APP_SCHED_QUEUE_SIZE        (128)


static void scheduled_event_handler(void * p_event_data, uint16_t event_size) {
    uint8_t * event = (uint8_t *) p_event_data;
    uint32_t event_id = *event;
    if (event_id >= 0 && event_id <= 3) {
        button_event_handler(event_id);
    } else if (event_id == 4) {
        check_send_event();
    }
}

static void button_event_handler_irq(uint32_t button_number) {
    schedule_event(button_number);
}

void schedule_event(uint32_t event_id) {
    static uint8_t event;
    event = event_id;
    app_sched_event_put((void *)&event, sizeof(event), scheduled_event_handler);
}

/********** init and setup ***********/

static void app_start(void) {

    load_led_config();
    apply_led_config();
    
    __LOG_RELEASE(LOG_SRC_APP, LOG_LEVEL_INFO, "Starting application \n");

    /* Install rx callback to intercept incoming ADV packets so that they can be passed to the
    EnOcean packet processor */
    //nrf_mesh_rx_cb_set(rx_callback);
}

static void app_mesh_core_event_cb (const nrf_mesh_evt_t * p_evt);

static nrf_mesh_evt_handler_t m_mesh_core_event_handler = { .evt_cb = app_mesh_core_event_cb };

static void app_mesh_core_event_cb(const nrf_mesh_evt_t * p_evt)
{
    /* USER_NOTE: User can insert mesh core event proceesing here */
    switch (p_evt->type)
    {
        /* Start user application specific functions only when flash is stable */
        case NRF_MESH_EVT_FLASH_STABLE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "Mesh evt: FLASH_STABLE \n");
            {
                static bool s_app_started;
                if (!s_app_started)
                {
                    /* Flash operation initiated during initialization has been completed */
                    app_start();
                    s_app_started = true;
                }
            }
            break;
        case NRF_MESH_EVT_TX_COMPLETE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Mesh evt: NRF_MESH_EVT_TX_COMPLETE %x - %x\n",p_evt->params.tx_complete.token,p_evt->params.tx_complete.timestamp);
            {
                
            }
            break;
        case NRF_MESH_EVT_DFU_FIRMWARE_OUTDATED:
        case NRF_MESH_EVT_DFU_FIRMWARE_OUTDATED_NO_AUTH:

             __LOG_RELEASE(LOG_SRC_APP, LOG_LEVEL_DBG1, "Mesh evt: NRF_MESH_EVT_DFU_FIRMWARE_OUTDATED \n");

            if (fw_updated_event_is_for_me(&p_evt->params.dfu))
            {
                const uint32_t * bank_address = optimal_bank_address();
                __LOG_RELEASE(LOG_SRC_APP, LOG_LEVEL_INFO, "Requesting DFU transfer with bank at 0x%p\n", bank_address);

                ERROR_CHECK(nrf_mesh_dfu_request(p_evt->params.dfu.fw_outdated.transfer.dfu_type,
                                                 &p_evt->params.dfu.fw_outdated.transfer.id,
                                                 bank_address));
                hal_led_mask_set(LEDS_MASK, false); /* Turn off all LEDs */
            }
            else
            {
                /**
                 * While preparing for the start of the DFU process, the DFU module
                 * will notify about any other ongoing DFU transfers by sending
                 * @ref NRF_MESH_EVT_DFU_FIRMWARE_OUTDATED_NO_AUTH or
                 * @ref NRF_MESH_EVT_DFU_FIRMWARE_OUTDATED.
                 *
                 * Check the current DFU state to avoid reverting the target state
                 * to the relay state.
                 */
                nrf_mesh_dfu_transfer_state_t state;
                uint32_t error_code = nrf_mesh_dfu_state_get(&state);
                if (error_code == NRF_SUCCESS && (state.state == NRF_MESH_DFU_STATE_INITIALIZED ||
                                                  state.state == NRF_MESH_DFU_STATE_FIND_FWID ||
                                                  state.state == NRF_MESH_DFU_STATE_RELAY_CANDIDATE ||
                                                  state.state == NRF_MESH_DFU_STATE_RELAY))
                {
                    ERROR_CHECK(nrf_mesh_dfu_relay(p_evt->params.dfu.fw_outdated.transfer.dfu_type,
                                                    &p_evt->params.dfu.fw_outdated.transfer.id));
                }
            }
            break;

        case NRF_MESH_EVT_DFU_START:
            __LOG_RELEASE(LOG_SRC_APP, LOG_LEVEL_INFO, "Mesh evt: NRF_MESH_EVT_DFU_START \n");
            hal_led_mask_set(LEDS_MASK_DFU_RUNNING, true);
            break;

        case NRF_MESH_EVT_DFU_END:
            __LOG_RELEASE(LOG_SRC_APP, LOG_LEVEL_INFO, "Mesh evt: NRF_MESH_EVT_DFU_END \n");
            hal_led_mask_set(LEDS_MASK, false); /* Turn off all LEDs */
            hal_led_mask_set(LEDS_MASK_DFU_ENDED, true);
            break;

        case NRF_MESH_EVT_DFU_BANK_AVAILABLE:
            __LOG_RELEASE(LOG_SRC_APP, LOG_LEVEL_INFO, "Mesh evt: NRF_MESH_EVT_DFU_BANK_AVAILABLE \n");
            hal_led_mask_set(LEDS_MASK, false); /* Turn off all LEDs */
            ERROR_CHECK(nrf_mesh_dfu_bank_flash(p_evt->params.dfu.bank.transfer.dfu_type));
            break;
        default:
            break;
    }
}

static void mesh_init(void)
{
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_THREAD,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = NULL,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };

    uint32_t status = mesh_stack_init(&init_params, &m_device_provisioned);
    switch (status)
    {
        case NRF_ERROR_INVALID_DATA:
            __LOG_RELEASE(LOG_SRC_APP, LOG_LEVEL_INFO, "Data in the persistent memory was corrupted. Device starts as unprovisioned.\n");
            __LOG_RELEASE(LOG_SRC_APP, LOG_LEVEL_INFO, "Reset device before start provisioning.\n");
            factory_reset();
            break;
        case NRF_SUCCESS:
            break;
        default:
            ERROR_CHECK(status);
    }

    /* Register event handler to receive NRF_MESH_EVT_FLASH_STABLE. Application functionality will
    be started after this event */
    nrf_mesh_evt_handler_add(&m_mesh_core_event_handler);
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS | LOG_SRC_BEARER | LOG_SRC_DFU, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG_RELEASE(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light -----\n");
    __LOG_RELEASE(LOG_SRC_APP, LOG_LEVEL_INFO, "App version: %x (build: %x)\n", app_version, app_build);

    pwm_utils_init(&m_pwm);
    reset_wdt_timer();

    ERROR_CHECK(app_timer_init());
    hal_leds_init();
    reset_wdt_timer();

    ble_stack_init();
    reset_wdt_timer();

#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_event_handler_irq));
#endif

    reset_wdt_timer();
#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif
    reset_wdt_timer();

    mesh_init();

    app_flash_init();

    serial_commands_init(serial_command_received, serial_status_received);
}

static void start(void)
{
    rtt_input_enable(app_rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);

    if (!m_device_provisioned) {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_sd_ble_opt_set_cb = NULL,
            .prov_complete_cb = provisioning_complete_cb,
            .prov_device_identification_start_cb = device_identification_start_cb,
            .prov_device_identification_stop_cb = device_identification_stop_cb,
            .prov_abort_cb = provisioning_aborted_cb,
            .p_device_uri = EX_URI_PET_PROJECT
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    } else {

        dsm_local_unicast_addresses_get(&node_address);

        unicast_address_print();

        random_delay();
    }

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    ERROR_CHECK(mesh_stack_start());

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);

}

int main(void) {

    APP_SCHED_INIT(APP_SCHED_EVENT_SIZE, APP_SCHED_QUEUE_SIZE);

    wdt_timer_init();

    initialize();
    reset_wdt_timer();
    start();
    reset_wdt_timer();

    //timestamp_t event_time = timer_now();
    //bool once_event = false;

    optimal_bank_address();
    
    for (;;) {

        app_sched_execute();

        bool done = nrf_mesh_process();
        if (done) {
            (void)sd_app_evt_wait();
        }

        serial_commands_process();

        reset_wdt_timer();

        check_commissioning_state();

        check_health_server_start();
    }
}

//TODO
/*

need fix crash

enocean switch
enocean sensor

storage enocean devices

uart communication (refactor)

on/off enocean Feature


open -n -a /Applications/SEGGER\ Embedded\ Studio\ for\ ARM\ 4.50/SEGGER\ Embedded\ Studio\ for\ ARM\ 4.50.app
*/