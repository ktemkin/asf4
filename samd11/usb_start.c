/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file or main.c
 * to avoid loosing it when reconfiguring.
 */
#include "atmel_start.h"
#include "usb_start.h"

#if CONF_USBD_HS_SP
static uint8_t single_desc_bytes[] = {
    /* Device descriptors and Configuration descriptors list. */
    CDCD_ACM_HS_DESCES_LS_FS};
static uint8_t single_desc_bytes_hs[] = {
    /* Device descriptors and Configuration descriptors list. */
    CDCD_ACM_HS_DESCES_HS};
#define CDCD_ECHO_BUF_SIZ CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ_HS
#else
static uint8_t single_desc_bytes[] = {
    /* Device descriptors and Configuration descriptors list. */
    CDCD_ACM_DESCES_LS_FS};
#define CDCD_ECHO_BUF_SIZ CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ
#endif

static struct usbd_descriptors single_desc[]
    = {{single_desc_bytes, single_desc_bytes + sizeof(single_desc_bytes)}
#if CONF_USBD_HS_SP
       ,
       {single_desc_bytes_hs, single_desc_bytes_hs + sizeof(single_desc_bytes_hs)}
#endif
};

/** Buffers to receive and echo the communication bytes. */
static uint32_t usbd_cdc_buffer[CDCD_ECHO_BUF_SIZ / 4];

/** Ctrl endpoint buffer */
static uint8_t ctrl_buffer[64];

/**
 * \brief Callback invoked when bulk OUT data received
 */
static bool usb_device_cb_bulk_out(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	cdcdf_acm_write((uint8_t *)usbd_cdc_buffer, count);

	/* No error. */
	return false;
}

/**
 * \brief Callback invoked when bulk IN data received
 */
static bool usb_device_cb_bulk_in(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	/* Echo data. */
	cdcdf_acm_read((uint8_t *)usbd_cdc_buffer, sizeof(usbd_cdc_buffer));

	/* No error. */
	return false;
}

/**
 * \brief Callback invoked when Line State Change
 */
static bool usb_device_cb_state_c(usb_cdc_control_signal_t state)
{
	if (state.rs232.DTR) {
		/* Callbacks must be registered after endpoint allocation */
		cdcdf_acm_register_callback(CDCDF_ACM_CB_READ, (FUNC_PTR)usb_device_cb_bulk_out);
		cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE, (FUNC_PTR)usb_device_cb_bulk_in);
		/* Start Rx */
		cdcdf_acm_read((uint8_t *)usbd_cdc_buffer, sizeof(usbd_cdc_buffer));
	}

	/* No error. */
	return false;
}

/**
 * \brief CDC ACM Init
 */
void cdc_device_acm_init(void)
{
	/* usb stack init */
	usbdc_init(ctrl_buffer);

	/* usbdc_register_funcion inside */
	cdcdf_acm_init();

	usbdc_start(single_desc);
	usbdc_attach();
}

/**
 * Example of using CDC ACM Function.
 * \note
 * In this example, we will use a PC as a USB host:
 * - Connect the DEBUG USB on XPLAINED board to PC for program download.
 * - Connect the TARGET USB on XPLAINED board to PC for running program.
 * The application will behave as a virtual COM.
 * - Open a HyperTerminal or other COM tools in PC side.
 * - Send out a character or string and it will echo the content received.
 */
void cdcd_acm_example(void)
{
	while (!cdcdf_acm_is_enabled()) {
		// wait cdc acm to be installed
	};

	cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)usb_device_cb_state_c);

	while (1) {
	}
}

static uint8_t single_desc_bytes[] = {
    /* Device descriptors and Configuration descriptors list. */
    HID_MOUSE_DESCES_LS_FS};

static struct usbd_descriptors single_desc[] = {{single_desc_bytes, single_desc_bytes + sizeof(single_desc_bytes)}
#if CONF_USBD_HS_SP
                                                ,
                                                {NULL, NULL}
#endif
};

/** Ctrl endpoint buffer */
static uint8_t ctrl_buffer[64];

/**
 * Example of using HID Mouse Function.
 * \note
 * In this example, we will use a PC as a USB host:
 * - Connect the DEBUG USB on XPLAINED board to PC for program download.
 * - Connect the TARGET USB on XPLAINED board to PC for running program.
 * The application will behave as a mouse which can be recongnized by PC.
 */
void hid_mouse_example(void)
{
	/* usb stack init */
	usbdc_init(ctrl_buffer);

	/* usbdc_register_funcion inside */
	hiddf_mouse_init();

	usbdc_start(single_desc);
	usbdc_attach();

	while (!hiddf_mouse_is_enabled()) {
		// wait hid mouse to be installed
	};
}

static uint8_t single_desc_bytes[] = {
    /* Device descriptors and Configuration descriptors list. */
    HID_KEYBOARD_DESCES_LS_FS};

static struct usbd_descriptors single_desc[] = {{single_desc_bytes, single_desc_bytes + sizeof(single_desc_bytes)}
#if CONF_USBD_HS_SP
                                                ,
                                                {NULL, NULL}
#endif
};

/** Ctrl endpoint buffer */
static uint8_t ctrl_buffer[64];

/**
 * Example of using HID Keyboard Function.
 * \note
 * In this example, we will use a PC as a USB host:
 * - Connect the DEBUG USB on XPLAINED board to PC for program download.
 * - Connect the TARGET USB on XPLAINED board to PC for running program.
 * The application will behave as a keyboard which can be recongnized by PC.
 */
void hid_keyboard_example(void)
{
	/* usb stack init */
	usbdc_init(ctrl_buffer);

	/* usbdc_register_funcion inside */
	hiddf_keyboard_init();

	usbdc_start(single_desc);
	usbdc_attach();

	while (!hiddf_keyboard_is_enabled()) {
		// wait hid keyboard to be installed
	};
}

const static uint8_t customer_hid_report[] = {CONF_USB_HID_GENERIC_REPORT};

static uint8_t single_desc_bytes[] = {
    /* Device descriptors and Configuration descriptors list. */
    HID_GENERIC_DESCES_LS_FS};

static struct usbd_descriptors single_desc[] = {{single_desc_bytes, single_desc_bytes + sizeof(single_desc_bytes)}
#if CONF_USBD_HS_SP
                                                ,
                                                {NULL, NULL}
#endif
};

/** Ctrl endpoint buffer */
static uint8_t ctrl_buffer[64];

/**
 * Example of using HID Generic Function.
 * \note
 * In this example, we will use a PC as a USB host:
 * - Connect the DEBUG USB on XPLAINED board to PC for program download.
 * - Connect the TARGET USB on XPLAINED board to PC for running program.
 * The application will behave as a generic which can be recongnized by PC.
 */
void hid_generic_example(void)
{
	/* usb stack init */
	usbdc_init(ctrl_buffer);

	/* usbdc_register_funcion inside */
	hiddf_generic_init(customer_hid_report, CONF_USB_HID_GENERIC_REPORT_LEN);

	usbdc_start(single_desc);
	usbdc_attach();

	while (!hiddf_generic_is_enabled()) {
		// wait hid generic to be installed
	};
}

	/* Max LUN number */
#define CONF_USB_MSC_MAX_LUN 0

#if CONF_USB_MSC_LUN_DEMO

/* MSC LUN number */
#define LUN_RAM 0

/* MSC LUN definitions */
#define DISK_INFORMATION(n)                                                                                            \
	{                                                                                                                  \
		CONF_USB_MSC_LUN##n##_TYPE, (CONF_USB_MSC_LUN##n##_RMB << 7),                                                  \
		    ((CONF_USB_MSC_LUN##n##_ISO << 6) + (CONF_USB_MSC_LUN##n##_ECMA << 3) + CONF_USB_MSC_LUN##n##_ANSI),       \
		    CONF_USB_MSC_LUN##n##_REPO, 31, 0x00, 0x00, 0x00, CONF_USB_MSC_LUN##n##_FACTORY,                           \
		    CONF_USB_MSC_LUN##n##_PRODUCT, CONF_USB_MSC_LUN##n##_PRODUCT_VERSION                                       \
	}

#define DISK_CAPACITY(n)                                                                                               \
	{                                                                                                                  \
		(uint8_t)(CONF_USB_MSC_LUN##n##_LAST_BLOCK_ADDR >> 24),                                                        \
		    (uint8_t)(CONF_USB_MSC_LUN##n##_LAST_BLOCK_ADDR >> 16),                                                    \
		    (uint8_t)(CONF_USB_MSC_LUN##n##_LAST_BLOCK_ADDR >> 8),                                                     \
		    (uint8_t)(CONF_USB_MSC_LUN##n##_LAST_BLOCK_ADDR >> 0),                                                     \
		    (uint8_t)((uint32_t)(CONF_USB_MSC_LUN##n##_BLOCK_SIZE) >> 24),                                             \
		    (uint8_t)((uint32_t)(CONF_USB_MSC_LUN##n##_BLOCK_SIZE) >> 16),                                             \
		    (uint8_t)((uint32_t)(CONF_USB_MSC_LUN##n##_BLOCK_SIZE) >> 8),                                              \
		    (uint8_t)((uint32_t)(CONF_USB_MSC_LUN##n##_BLOCK_SIZE) >> 0)                                               \
	}

/* MSC LUN settings */

static uint8_t *ram_disk_buf;

static uint8_t       msc_lun;
static bool          xfer_dir;
static volatile bool usb_busy;
static uint32_t      disk_addr;
static uint32_t      usb_remain = 0;

/* Inquiry Information */
static uint8_t inquiry_info[CONF_USB_MSC_MAX_LUN + 1][36] = {DISK_INFORMATION(0)};

/* Capacities of Disk */
static uint8_t format_capa[CONF_USB_MSC_MAX_LUN + 1][8] = {DISK_CAPACITY(0)};

#endif

#if CONF_USBD_HS_SP
static uint8_t single_desc_bytes[] = {
    /* Device descriptors and Configuration descriptors list. */
    MSC_HS_DESCES_LS_FS};
static uint8_t single_desc_bytes_hs[] = {
    /* Device descriptors and Configuration descriptors list. */
    MSC_HS_DESCES_HS};
#else
static uint8_t single_desc_bytes[] = {
    /* Device descriptors and Configuration descriptors list. */
    MSC_DESCES_LS_FS};
#endif

static struct usbd_descriptors single_desc[]
    = {{single_desc_bytes, single_desc_bytes + sizeof(single_desc_bytes)}
#if CONF_USBD_HS_SP
       ,
       {single_desc_bytes_hs, single_desc_bytes_hs + sizeof(single_desc_bytes_hs)}
#endif
};

/* Ctrl endpoint buffer */
static uint8_t ctrl_buffer[64];

#if CONF_USB_MSC_LUN_DEMO

/**
 * \brief Eject Disk
 * \param[in] lun logic unit number
 * \return Operation status.
 */
static int32_t msc_disk_eject(uint8_t lun)
{
	if (lun > CONF_USB_MSC_MAX_LUN) {
		return ERR_NOT_FOUND;
	}
	return ERR_NONE;
}

/**
 * \brief Inquiry whether Disk is ready
 * \param[in] lun logic unit number
 * \return Operation status.
 */
static int32_t msc_disk_is_ready(uint8_t lun)
{
	if (lun > CONF_USB_MSC_MAX_LUN) {
		return ERR_NOT_FOUND;
	}
	return ERR_NONE;
}

/**
 * \brief Callback invoked when a new read blocks command received
 * \param[in] lun logic unit number
 * \param[in] addr start address of disk to be read
 * \param[in] nblocks block amount to be read
 * \return Operation status.
 */
static int32_t msc_new_read(uint8_t lun, uint32_t addr, uint32_t nblocks)
{
	int32_t  rc  = msc_disk_is_ready(lun);
	uint32_t lba = 0;
	if (rc != ERR_NONE) {
		return rc;
	}
	lba = (format_capa[lun][0] << 24) + (format_capa[lun][1] << 16) + (format_capa[lun][2] << 8) + format_capa[lun][3];
	if (addr > lba || addr + nblocks > lba + 1) {
		return ERR_BAD_ADDRESS;
	}
	msc_lun  = lun;
	xfer_dir = true;

	disk_addr  = addr;
	usb_remain = nblocks;

	return ERR_NONE;
}

/**
 * \brief Callback invoked when a new write blocks command received
 * \param[in] lun logic unit number
 * \param[in] addr start address of disk to be written
 * \param[in] nblocks block amount to be written
 * \return Operation status.
 */
static int32_t msc_new_write(uint8_t lun, uint32_t addr, uint32_t nblocks)
{
	int32_t  rc  = msc_disk_is_ready(lun);
	uint32_t lba = 0;
	if (rc != ERR_NONE) {
		return rc;
	}
	lba = (format_capa[lun][0] << 24) + (format_capa[lun][1] << 16) + (format_capa[lun][2] << 8) + format_capa[lun][3];
	if (addr > lba || addr + nblocks > lba + 1) {
		return ERR_BAD_ADDRESS;
	}
	msc_lun  = lun;
	xfer_dir = false;

	disk_addr  = addr;
	usb_remain = nblocks;

	return ERR_NONE;
}

/**
 * \brief Callback invoked when a blocks transfer is done
 * \param[in] lun logic unit number
 * \return Operation status.
 */
static int32_t msc_xfer_done(uint8_t lun)
{
	if (lun > CONF_USB_MSC_MAX_LUN) {
		return ERR_DENIED;
	}
	usb_busy = false;
	if (LUN_RAM == msc_lun) {
		usb_remain = 0;
		if (!xfer_dir) {
			/* Terminate write */
			mscdf_xfer_blocks(false, NULL, 0);
		}
		return ERR_NONE;
	}
	return ERR_NONE;
} /**
   * \brief Disk loop
   */
static void msc_disk_task(void)
{
	/* RAM Disk transfer directly between RAM and USB */
}

/**
 * \brief USB loop
 */
static void msc_usb_task(void)
{
	if (false == usb_busy && 0 != usb_remain) {
		if (0 == xfer_dir) {
			/* RAM Disk directly exchange data with USB */
			if (msc_lun == LUN_RAM) {
				usb_busy = true;
				mscdf_xfer_blocks(false, &ram_disk_buf[disk_addr << 9], usb_remain);
				return;
			}
		} else {
			/* RAM Disk directly exchange data with USB */
			if (msc_lun == LUN_RAM) {
				usb_busy = true;
				mscdf_xfer_blocks(true, &ram_disk_buf[disk_addr << 9], usb_remain);
				return;
			}
		}
	}
}

/**
 * \brief Callback invoked when inquiry data command received
 * \param[in] lun logic unit number
 * \return Operation status.
 */
static uint8_t *msc_inquiry_info(uint8_t lun)
{
	if (lun > CONF_USB_MSC_MAX_LUN) {
		return NULL;
	} else {
		usb_busy   = false;
		usb_remain = 0;
		return &inquiry_info[lun][0];
	}
}

/**
 * \brief Callback invoked when read format capacities command received
 * \param[in] lun logic unit number
 * \return Operation status.
 */
static uint8_t *msc_get_capacity(uint8_t lun)
{
	if (lun > CONF_USB_MSC_MAX_LUN) {
		return NULL;
	} else {
		return &format_capa[lun][0];
	}
}

void mscdf_demo_init(uint8_t *ramd_buf, uint8_t *usbdisk_buf)
{
	ram_disk_buf = ramd_buf;
	mscdf_register_callback(MSCDF_CB_INQUIRY_DISK, (FUNC_PTR)msc_inquiry_info);
	mscdf_register_callback(MSCDF_CB_GET_DISK_CAPACITY, (FUNC_PTR)msc_get_capacity);
	mscdf_register_callback(MSCDF_CB_START_READ_DISK, (FUNC_PTR)msc_new_read);
	mscdf_register_callback(MSCDF_CB_START_WRITE_DISK, (FUNC_PTR)msc_new_write);
	mscdf_register_callback(MSCDF_CB_EJECT_DISK, (FUNC_PTR)msc_disk_eject);
	mscdf_register_callback(MSCDF_CB_TEST_DISK_READY, (FUNC_PTR)msc_disk_is_ready);
	mscdf_register_callback(MSCDF_CB_XFER_BLOCKS_DONE, (FUNC_PTR)msc_xfer_done);
}

void mscdf_demo_task(void)
{
	msc_disk_task();
	msc_usb_task();
}

#endif

/**
 * \brief USB MSC Init
 */
void usbd_msc_init(void)
{
	/* usb stack init */
	usbdc_init(ctrl_buffer);

	/* usbdc_register_funcion inside */
	mscdf_init(CONF_USB_MSC_MAX_LUN);
}

/**
 * \brief Example of using MSC Function.
 * \param[in] ramd_buf Pointer to the memory block for RAM Disk. Size of the
 *            disk is defined by \ref CONF_USB_MSC_LUN0_CAPACITY in KB.
 * \param[in] usbdisk_buf Pointer to the buffer for disk read/write,
 *            e.g., during reading from SD/MMC, data is read to the buffer and
 *            then sent to host from the buffer. Size of the buffer is defined
 *            by \ref CONF_USB_MSC_LUN_BUF_SECTORS in number of sectors.
 * \note
 * In this example, we will use a PC as a USB host:
 * - Connect the DEBUG USB on XPLAINED board to PC for program download.
 * - Connect the TARGET USB on XPLAINED board to PC for running program.
 * The application will behave as a massive storage device which can be
 * recognized by PC. Only one logic unit is supported in this example.
 */
void usbd_msc_example(uint8_t *ramd_buf, uint8_t *usbdisk_buf)
{

#if CONF_USB_MSC_LUN_DEMO
	mscdf_demo_init(ramd_buf, usbdisk_buf);
#endif

	usbdc_start(single_desc);
	usbdc_attach();

	while (!mscdf_is_enabled()) {
		/* wait massive storage to be installed */
	};

	while (1) {

#if CONF_USB_MSC_LUN_DEMO
		mscdf_demo_task();
#endif
	}
}

static uint8_t single_desc_bytes[] = {
    /* Device descriptors and Configuration descriptors list. */
    VENDOR_DESCES_LS_FS};

static struct usbd_descriptors single_desc[] = {{single_desc_bytes, single_desc_bytes + sizeof(single_desc_bytes)}
#if CONF_USBD_HS_SP
                                                ,
                                                {NULL, NULL}
#endif
};

/** Ctrl endpoint buffer */
static uint8_t ctrl_buffer[64];

/** Loop back buffer for data validation */
COMPILER_ALIGNED(4)
static uint8_t main_buf_loopback[1024];

/**
 * \brief Ctrl endpointer out transfer callback function
 * \param[in] count data amount to transfer
 */
static void usb_device_cb_ctrl_out(uint16_t count)
{
	uint16_t recv_cnt = 1024;

	if (recv_cnt > count) {
		recv_cnt = count;
	}
	vendordf_read(USB_EP_TYPE_CONTROL, main_buf_loopback, recv_cnt);
	vendordf_read(USB_EP_TYPE_INTERRUPT, main_buf_loopback, 1024);
	vendordf_read(USB_EP_TYPE_BULK, main_buf_loopback, 1024);
	vendordf_read(USB_EP_TYPE_ISOCHRONOUS, main_buf_loopback, 1024);
}

/**
 * \brief Ctrl endpointer in transfer callback function
 * \param[in] count data amount to transfer
 */
static void usb_device_cb_ctrl_in(uint16_t count)
{
	uint16_t recv_cnt = 1024;

	if (recv_cnt > count) {
		recv_cnt = count;
	}
	vendordf_write(USB_EP_TYPE_CONTROL, main_buf_loopback, recv_cnt);
}

/**
 * \brief Interrupt endpointer out transfer callback function
 * \param[in] ep Endpointer number
 * \param[in] rc Transfer status codes
 * \param[in] count Data amount to transfer
 * \return Operation status.
 */
static bool usb_device_cb_int_out(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	vendordf_write(USB_EP_TYPE_INTERRUPT, main_buf_loopback, count);
	return false;
}

/**
 * \brief Interrupt endpointer in transfer callback function
 * \param[in] ep Endpointer number
 * \param[in] rc Transfer status codes
 * \param[in] count Data amount to transfer
 * \return Operation status.
 */
static bool usb_device_cb_int_in(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	vendordf_read(USB_EP_TYPE_INTERRUPT, main_buf_loopback, count);
	return false;
}

/**
 * \brief Bulk endpointer out transfer callback function
 * \param[in] ep Endpointer number
 * \param[in] rc Transfer status codes
 * \param[in] count Data amount to transfer
 * \return Operation status.
 */
static bool usb_device_cb_bulk_out(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	vendordf_write(USB_EP_TYPE_BULK, main_buf_loopback, count);

	return false;
}

/**
 * \brief Bulk endpointer in transfer callback function
 * \param[in] ep Endpointer number
 * \param[in] rc Transfer status codes
 * \param[in] count Data amount to transfer
 * \return Operation status.
 */
static bool usb_device_cb_bulk_in(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	vendordf_read(USB_EP_TYPE_BULK, main_buf_loopback, count);

	return false;
}

/**
 * \brief Isochronous endpointer out transfer callback function
 * \param[in] ep Endpointer number
 * \param[in] rc Transfer status codes
 * \param[in] count Data amount to transfer
 * \return Operation status.
 */
static bool usb_device_cb_iso_out(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	vendordf_write(USB_EP_TYPE_ISOCHRONOUS, main_buf_loopback, count);

	return false;
}

/**
 * \brief Isochronous endpointer in transfer callback function
 * \param[in] ep Endpointer number
 * \param[in] rc Transfer status codes
 * \param[in] count Data amount to transfer
 * \return Operation status.
 */
static bool usb_device_cb_iso_in(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	vendordf_read(USB_EP_TYPE_ISOCHRONOUS, main_buf_loopback, count);

	return false;
}

/**
 * Example of Vendor Function.
 * \note
 * In this example, we will use a PC as a USB host:
 * - Connect the DEBUG USB on XPLAINED board to PC for program download.
 * - Connect the TARGET USB on XPLAINED board to PC for running program.
 * The example uses a vendor class which implements a loopback on the following
 * endpoints types: control, interrupt, bulk and isochronous.
 * The host application developed on libusb library is provided with application
 * note AVR4901. Except for control endpoint, other type endpoints are optional.
 * Application always run from control endpoint firstly.
 */
void vendor_example(void)
{
	/* usb stack init */
	usbdc_init(ctrl_buffer);

	/* usbdc_register_funcion inside */
	vendordf_init();
	vendordf_register_callback(VENDORDF_CB_CTRL_READ_REQ, (FUNC_PTR)usb_device_cb_ctrl_out);
	vendordf_register_callback(VENDORDF_CB_CTRL_WRITE_REQ, (FUNC_PTR)usb_device_cb_ctrl_in);
	vendordf_register_callback(VENDORDF_CB_INT_READ, (FUNC_PTR)usb_device_cb_int_out);
	vendordf_register_callback(VENDORDF_CB_INT_WRITE, (FUNC_PTR)usb_device_cb_int_in);
	vendordf_register_callback(VENDORDF_CB_BULK_READ, (FUNC_PTR)usb_device_cb_bulk_out);
	vendordf_register_callback(VENDORDF_CB_BULK_WRITE, (FUNC_PTR)usb_device_cb_bulk_in);
	vendordf_register_callback(VENDORDF_CB_ISO_READ, (FUNC_PTR)usb_device_cb_iso_out);
	vendordf_register_callback(VENDORDF_CB_ISO_WRITE, (FUNC_PTR)usb_device_cb_iso_in);

	usbdc_start(single_desc);
	usbdc_attach();

	while (!vendordf_is_enabled()) {
		// wait vendor application to be installed
	};
}

	/* Max LUN number */
#define CONF_USB_MSC_MAX_LUN 0

#if CONF_USBD_HS_SP
static uint8_t multi_desc_bytes[] = {
    /* Device descriptors and Configuration descriptors list. */
    COMPOSITE_HS_DESCES_LS_FS};
static uint8_t multi_desc_bytes_hs[] = {
    /* Device descriptors and Configuration descriptors list. */
    COMPOSITE_HS_DESCES_HS};
#else
static uint8_t multi_desc_bytes[] = {
    /* Device descriptors and Configuration descriptors list. */
    COMPOSITE_DESCES_LS_FS};
#endif

static struct usbd_descriptors multi_desc[] = {{multi_desc_bytes, multi_desc_bytes + sizeof(multi_desc_bytes)}
#if CONF_USBD_HS_SP
                                               ,
                                               {multi_desc_bytes_hs, multi_desc_bytes_hs + sizeof(multi_desc_bytes_hs)}
#endif
};

/** Ctrl endpoint buffer */
static uint8_t ctrl_buffer[64];

void composite_device_init(void)
{
	/* usb stack init */
	usbdc_init(ctrl_buffer);

	/* usbdc_register_funcion inside */
#if CONF_USB_COMPOSITE_CDC_ACM_EN
	cdcdf_acm_init();
#endif
#if CONF_USB_COMPOSITE_HID_MOUSE_EN
	hiddf_mouse_init();
#endif
#if CONF_USB_COMPOSITE_HID_KEYBOARD_EN
	hiddf_keyboard_init();
#endif
#if CONF_USB_COMPOSITE_MSC_EN
	mscdf_init(CONF_USB_MSC_MAX_LUN);
#endif
}

void composite_device_start(void)
{
	usbdc_start(multi_desc);
	usbdc_attach();
}

void composite_device_example(void)
{

	/* Initialize */
	/* It's done with system init ... */

	/* Before start do function related initializations */
	/* Add your code here ... */

	/* Start device */
	composite_device_start();

	/* Main loop */
	while (1) {
		if (cdcdf_acm_is_enabled()) {
			/* CDC ACM process*/
		}
		if (hiddf_mouse_is_enabled()) {
			/* HID Mouse process */
		}
		if (hiddf_keyboard_is_enabled()) {
			/* HID Keyboard process */
		};
		if (mscdf_is_enabled()) {
			/* MSC process */
		}
	}
}

void usb_init(void)
{

	cdc_device_acm_init();

	usbd_msc_init();

	composite_device_init();
}
