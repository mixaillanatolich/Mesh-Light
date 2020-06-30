
#ifndef APP_MAIN_CONFIG_H__
#define APP_MAIN_CONFIG_H__

#include <stdbool.h>
#include <stdint.h>
#include <string.h>


/*****************************************************************************
 * Definitions
 *****************************************************************************/

#define APP_ONOFF_ELEMENT_INDEX     (0)
#define APP_LEVEL_ELEMENT_INDEX     (0)

#define APP_LEVEL_STEP_SIZE     (1024L)

/* Controls if the model instance should force all mesh messages to be segmented messages. */
#define APP_FORCE_SEGMENTATION  (false)
/* Controls the MIC size used by the model instance for sending the mesh messages. */
#define APP_MIC_SIZE            (NRF_MESH_TRANSMIC_SIZE_SMALL)

#define LED_LEVEL_DEFAULT INT16_MAX
#define LED_MIN_LEVEL_DEFAULT 0
#define LED_MAX_LEVEL_DEFAULT UINT16_MAX

#define LEDS_MASK_DFU_RUNNING   (BSP_LED_2_MASK)
#define LEDS_MASK_DFU_ENDED     (BSP_LED_1_MASK)

#define NO_COMMISSIONING_TIMEOUT 1

/******** Storage definitions *********/
#define APP_FLASH_PAGE_COUNT           (1)
#define LED1_CONFIG_ENTRY_HANDLE          (0x00001)
#define ENOCEAN_DATA_PTM215B_ENTRY_HANDLE (0x00005)
#define ENOCEAN_DATA_EMDCB_ENTRY_HANDLE   (0x00006)


/******** End storage definitions *********/



/**************************** Structs ****************************************/
typedef struct {
    bool        on;
    uint16_t    level;
    uint16_t    min_level;
    uint16_t    max_level;
} led_config_t;
/*****************************************************************************/

#endif /* APP_MAIN_CONFIG_H__ */
