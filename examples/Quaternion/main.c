/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <drivers/i2c.h>
#include <logging/log.h>

#include "bhy2.h"
#include "bhy2_parse.h"
#include "common.h"

#include "Bosch_APP30_SHUTTLE_BHI260_aux_BMM150.fw.h"



/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   10

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)


/* BHI Defines */

#define WORK_BUFFER_SIZE   2048
#if defined (PC)
#define MAX_READ_WRITE_LEN 44
#else
#define MAX_READ_WRITE_LEN 256
#endif

#define QUAT_SENSOR_ID     BHY2_SENSOR_ID_RV_WU


static void parse_quaternion(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
static void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
static void print_api_error(int8_t rslt, struct bhy2_dev *dev);

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
	int ret;
	int a = 0;
	int8_t rslt=1;

	if (!device_is_ready(led.port)) {
		return 0; 
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return ret;
	}

	uint8_t product_id = 0;
	uint16_t version = 0;
	static uint8_t work_buffer[WORK_BUFFER_SIZE];
	uint8_t hint, hintr_ctrl, hif_ctrl, boot_status;
	static struct bhy2_dev bhy2;
	
	rslt = bhy2_init(BHY2_I2C_INTERFACE, bhy2_i2c_read, bhy2_i2c_write, bhy2_delay_us, MAX_READ_WRITE_LEN, NULL, &bhy2);
	print_api_error(rslt, &bhy2);
	
	rslt = bhy2_soft_reset(&bhy2);
	print_api_error(rslt, &bhy2);

	rslt = bhy2_get_product_id(&product_id, &bhy2);
	print_api_error(rslt, &bhy2);

	//Check for a valid product ID
	if (product_id != BHY2_PRODUCT_ID)
	{
		printk("Product ID read %X. Expected %X\r\n", product_id, BHY2_PRODUCT_ID);
	}
	else
	{
		printk("BHI260/BHA260 found. Product ID read %X\r\n", product_id);
	}
    
	// Check the interrupt pin and FIFO configurations. Disable status and debug
	hintr_ctrl = BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG;

	rslt = bhy2_set_host_interrupt_ctrl(hintr_ctrl, &bhy2);
	print_api_error(rslt, &bhy2);
	rslt = bhy2_get_host_interrupt_ctrl(&hintr_ctrl, &bhy2);
	print_api_error(rslt, &bhy2);

	printk("Host interrupt control\r\n");
	printk("    Wake up FIFO %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_FIFO_W) ? "disabled" : "enabled");
	printk("    Non wake up FIFO %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_FIFO_NW) ? "disabled" : "enabled");
	printk("    Status FIFO %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_STATUS_FIFO) ? "disabled" : "enabled");
	printk("    Debugging %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_DEBUG) ? "disabled" : "enabled");
	printk("    Fault %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_FAULT) ? "disabled" : "enabled");
	printk("    Interrupt is %s.\r\n", (hintr_ctrl & BHY2_ICTL_ACTIVE_LOW) ? "active low" : "active high");
	printk("    Interrupt is %s triggered.\r\n", (hintr_ctrl & BHY2_ICTL_EDGE) ? "pulse" : "level");
	printk("    Interrupt pin drive is %s.\r\n", (hintr_ctrl & BHY2_ICTL_OPEN_DRAIN) ? "open drain" : "push-pull");
    
	//Configure the host interface 
	hif_ctrl = 0;
	rslt = bhy2_set_host_intf_ctrl(hif_ctrl, &bhy2);
	print_api_error(rslt, &bhy2);
    
	// Check if the sensor is ready to load firmware
	rslt = bhy2_get_boot_status(&boot_status, &bhy2);
	print_api_error(rslt, &bhy2);
    
	if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
	{
		uint8_t sensor_error;
		int8_t temp_rslt;
		printk("Loading firmware into RAM.\r\n");

		rslt = bhy2_upload_firmware_to_ram(bhy2_firmware_image, sizeof(bhy2_firmware_image), &bhy2);
		temp_rslt = bhy2_get_error_value(&sensor_error, &bhy2);
		if (sensor_error)
		{
			printk("%s\r\n", get_sensor_error_text(sensor_error));
		}
		print_api_error(rslt, &bhy2);
		print_api_error(temp_rslt, &bhy2);

		printk("Booting from RAM.\r\n");
        
		rslt = bhy2_boot_from_ram(&bhy2);
        temp_rslt = bhy2_get_error_value(&sensor_error, &bhy2);
		if (sensor_error)
		{
			printk("%s\r\n", get_sensor_error_text(sensor_error));
		}
		print_api_error(rslt, &bhy2);
		print_api_error(temp_rslt, &bhy2);

		rslt = bhy2_get_kernel_version(&version, &bhy2);
		print_api_error(rslt, &bhy2);
		if ((rslt == BHY2_OK) && (version != 0))
		{
			printk("Boot successful. Kernel version %u.\r\n", version);
		}
        
		rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, NULL, &bhy2);
		print_api_error(rslt, &bhy2);
		rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, parse_meta_event, NULL, &bhy2);
		print_api_error(rslt, &bhy2);
		rslt = bhy2_register_fifo_parse_callback(QUAT_SENSOR_ID, parse_quaternion, NULL, &bhy2);
		print_api_error(rslt, &bhy2);

		rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy2);
		print_api_error(rslt, &bhy2);
	}
	else
	{
		printk("Host interface not ready. Exiting\r\n");

		return 0;
	}
    
	// Update the callback table to enable parsing of sensor data
	rslt = bhy2_update_virtual_sensor_list(&bhy2);
	print_api_error(rslt, &bhy2);

	float sample_rate = 100.0; // Read out data measured at 100Hz
	uint32_t report_latency_ms = 0; // Report immediately
	rslt = bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, sample_rate, report_latency_ms, &bhy2);
	print_api_error(rslt, &bhy2);
	printk("Enable %s at %.2fHz.\r\n", get_sensor_name(QUAT_SENSOR_ID), sample_rate);

    while (1)
    {
		a++;
		if(a==10000) a = 0;
		ret = gpio_pin_toggle_dt(&led);
		// if (a<10) printk("Printing test %d\n", a);
		if (ret < 0) {
			return ret;
		}
        // Data from the FIFO is read and the relevant callbacks if registered are called
        if(bhy2_get_interrupt_status(&hint, &bhy2))
        {
            rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy2);
            print_api_error(rslt, &bhy2);		
        }
		k_msleep(SLEEP_TIME_MS);
    }
    return rslt;

}

static void parse_quaternion(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    struct bhy2_data_quaternion data;
    uint32_t s, ns;
    if (callback_info->data_size != 11) // Check for a valid payload size. Includes sensor ID 
    {
        return;
    }

    bhy2_parse_quaternion(callback_info->data_ptr, &data);

    uint64_t timestamp = *callback_info->time_stamp; //  Store the last timestamp 

    timestamp = timestamp * 15625; // Timestamp is now in nanoseconds
    s = (uint32_t)(timestamp / UINT64_C(1000000000));
    ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

    /*printk("SID: %u; T: %u.%09u; x: %f, y: %f, z: %f, w: %f; acc: %.2f\r\n",
           callback_info->sensor_id,
           s,
           ns,
           data.x / 16384.0f,
           data.y / 16384.0f,
           data.z / 16384.0f,
           data.w / 16384.0f,
           ((data.accuracy * 180.0f) / 16384.0f) / 3.141592653589793f);*/
    printk("%f %f %f %f\n",
        data.w / 16384.0f,
        data.x / 16384.0f,
        data.y / 16384.0f,
        data.z / 16384.0f);
}

static void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];
    char *event_text;

    if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT)
    {
        event_text = "[META EVENT]";
    }
    else if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT_WU)
    {
        event_text = "[META EVENT WAKE UP]";
    }
    else
    {
        return;
    }

    switch (meta_event_type)
    {
        case BHY2_META_EVENT_FLUSH_COMPLETE:
            printk("%s Flush complete for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
            printk("%s Sample rate changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_POWER_MODE_CHANGED:
            printk("%s Power mode changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_ALGORITHM_EVENTS:
            printk("%s Algorithm event\r\n", event_text);
            break;
        case BHY2_META_EVENT_SENSOR_STATUS:
            printk("%s Accuracy for sensor id %u changed to %u\r\n", event_text, byte1, byte2);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_MAIN:
            printk("%s BSX event (do steps main)\r\n", event_text);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_CALIB:
            printk("%s BSX event (do steps calib)\r\n", event_text);
            break;
        case BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
            printk("%s BSX event (get output signal)\r\n", event_text);
            break;
        case BHY2_META_EVENT_SENSOR_ERROR:
            printk("%s Sensor id %u reported error 0x%02X\r\n", event_text, byte1, byte2);
            break;
        case BHY2_META_EVENT_FIFO_OVERFLOW:
            printk("%s FIFO overflow\r\n", event_text);
            break;
        case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED:
            printk("%s Dynamic range changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_FIFO_WATERMARK:
            printk("%s FIFO watermark reached\r\n", event_text);
            break;
        case BHY2_META_EVENT_INITIALIZED:
            printk("%s Firmware initialized. Firmware version %u\r\n", event_text, ((uint16_t)byte2 << 8) | byte1);
            break;
        case BHY2_META_TRANSFER_CAUSE:
            printk("%s Transfer cause for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_SENSOR_FRAMEWORK:
            printk("%s Sensor framework event for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_RESET:
            printk("%s Reset event\r\n", event_text);
            break;
        case BHY2_META_EVENT_SPACER:
            break;
        default:
            printk("%s Unknown meta event with id: %u\r\n", event_text, meta_event_type);
            break;
    }
}

static void print_api_error(int8_t rslt, struct bhy2_dev *dev)
{
    if (rslt != BHY2_OK)
    {
        printk("%s\r\n", get_api_error(rslt));
        if ((rslt == BHY2_E_IO) && (dev != NULL))
        {
            printk("Error %d\r\n",dev->hif.intf_rslt );//, get_coines_error(dev->hif.intf_rslt));
            dev->hif.intf_rslt = BHY2_INTF_RET_SUCCESS;
        }
        
    }
}


