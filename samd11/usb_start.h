/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file or main.c
 * to avoid loosing it when reconfiguring.
 */
#ifndef USB_DEVICE_MAIN_H
#define USB_DEVICE_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "cdcdf_acm.h"
#include "cdcdf_acm_desc.h"

void cdcd_acm_example(void);
void cdc_device_acm_init(void);

#include "hiddf_mouse.h"
#include "hiddf_mouse_desc.h"

void hid_mouse_example(void);

#include "hiddf_keyboard.h"
#include "hiddf_keyboard_desc.h"

void hid_keyboard_example(void);

#include "hiddf_generic.h"
#include "hiddf_generic_desc.h"

void hid_generic_example(void);

#include "mscdf.h"
#include "mscdf_desc.h"

void usbd_msc_init(void);
void usbd_msc_example(uint8_t *ramd_buf, uint8_t *usbdisk_buf);

/**
 * \brief Initialize MSC for LUN accessing that is called before usbdc_start()
 * \param[in] ramd_buf Pointer to memory for RAM Disk (if RAM Disk demo enabled).
 *                     The size is defined by configuration of \c CONF_USB_MSC_LUN0_CAPACITY.
 * \param[in] usbdisk_buf Pointer to memory/cache for USB and Disk to exchange data (if a non-RAM Disk demo enabled).
 *                        The size is defined by configuration of \c CONF_USB_MSC_LUN_BUF_SECTORS.
 */
void mscdf_demo_init(uint8_t *ramd_buf, uint8_t *usbdisk_buf);
/**
 * \brief Non-blocking task that is invoked in main loop, to process USB and disk tasks
 */
void mscdf_demo_task(void);

#include "vendordf.h"
#include "vendordf_desc.h"

void vendor_example(void);

#include "cdcdf_acm.h"
#include "hiddf_mouse.h"
#include "hiddf_keyboard.h"
#include "hiddf_generic.h"
#include "mscdf.h"
#include "composite_desc.h"

/**
 * \brief Initialize device and attach functions
 */
void composite_device_init(void);
/**
 * \brief Start the device
 */
void composite_device_start(void);
/**
 * Example of using Composite Function.
 * \note
 * In this example, we will use a PC as a USB host:
 * - Connect the DEBUG USB on XPLAINED board to PC for program download.
 * - Connect the TARGET USB on XPLAINED board to PC for running program.
 */
void composite_device_example(void);

/**
 * \berif Initialize USB
 */
void usb_init(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // USB_DEVICE_MAIN_H
