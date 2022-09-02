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
#include "parse.h"
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
    uint8_t cbref;
	
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
        
		rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, (void*)&cbref, &bhy2);
		print_api_error(rslt, &bhy2);
		rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, parse_meta_event, (void*)&cbref, &bhy2);
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
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return ret;
		}
        
        rslt = bhy2_get_interrupt_status(&hint, &bhy2);
        print_api_error(rslt, &bhy2);

        if(hint)
        {
            // Data from the FIFO is read and the relevant callbacks if registered are called
            rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy2);
            print_api_error(rslt, &bhy2);		
        }
    }
    return rslt;

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


