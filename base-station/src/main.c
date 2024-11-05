#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#define STACK_SIZE 500

#define SLEEP_TIME 1000

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart), "Console device is not ACM CDC UART device");

void main_task()
{
    /* Configure to set Console output to USB Serial */
    const struct device *usb_device = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    uint32_t dtr = 0;

    /* Check if USB can be initialised, bails out if fail is returned */
    if (usb_enable(NULL) != 0)
    {
        return;
    }

    /* Wait for a console connection, if the DTR flag was set to activate USB.
     * If you wish to start generating serial data immediately, you can simply
     * remove the while loop, to not wait until the control line is set.
     */
    while (!dtr)
    {
        uart_line_ctrl_get(usb_device, UART_LINE_CTRL_DTR, &dtr);
        k_sleep(K_MSEC(100));
    }
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(mysensor));
    struct sensor_value temperature, press, light, uv, adc, conf;

    printk("*******************************************\n");
    while (!device_is_ready(dev))
    {
        printk("Sensor: device not ready.\n");
        k_msleep(SLEEP_TIME);
    }

    printk("Starting base station...\n");
    printk("Configuring sensor node(s)...\n");
    printk("Setting sampling frequency...\n");
    // sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_frequency);

    printk("*******************************************\n\n");
    k_msleep(3000);
    int i = 0;
    for (;;)
    {
        printk("Itr: %d\n", i);
        // Update the sampling rate every 25 interations
        if ((i % 25) == 0 && (i > 24))
            if (sensor_attr_set(dev, SENSOR_CHAN_VOLTAGE, SENSOR_ATTR_SAMPLING_FREQUENCY, &conf) < 0)
                printk("Error in setting sample rate");

        if (sensor_sample_fetch(dev) < 0)
        {
            printk("ERROR: Could not fetch samples from sensor node.\n");
        }

        if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature) < 0)
        {
            printk("ERROR: Could not fetch 'TEMP' from sensor node.\n");
        }

        if (sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press) < 0)
        {
            printk("ERROR: Could not fetch 'press' from sensor node.\n");
        }

        if (sensor_channel_get(dev, SENSOR_CHAN_LIGHT, &light) < 0)
        {
            printk("ERROR: Could not fetch 'light' from sensor node.\n");
        }

        if (sensor_channel_get(dev, SENSOR_CHAN_IR, &uv) < 0)
        {
            printk("ERROR: Could not fetch 'UV' from sensor node.\n");
        }

        if (sensor_channel_get(dev, SENSOR_CHAN_VOLTAGE, &adc) < 0)
        {
            printk("ERROR: Could not fetch 'adc' from sensor node.\n");
        }

        printk("============ SENSOR READINGS ==============\n");
        printk("Light lux: %d.%02d\n", light.val1, light.val2);
        printk("Pressure (hPa): %d.%02d\n", press.val1, press.val2);
        printk("Temperature (C): %d.%02d\n", temperature.val1, temperature.val2);
        printk("UV : %d\n", uv.val1);
        printk("Moisture level mV: %d\n", adc.val1);
        printk("===========================================\n\n");
        i++;
        k_msleep(SLEEP_TIME);
    }
}

K_THREAD_DEFINE(main_thread, STACK_SIZE, main_task, NULL, NULL, NULL, 1, 0, 0);
