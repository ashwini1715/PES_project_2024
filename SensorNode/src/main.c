#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <string.h>
#include <stdio.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys_clock.h>
#include <zephyr/drivers/adc.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

#define UART_DEV_NAME "uart1"
#define UART_BAUD_RATE 115200
#define LED0_NODE DT_ALIAS(led0)
#define SENSOR_ADDRESS 0x53
#define I2C_PORT "i2c@40044000"
uint32_t final_light;
uint32_t UV_final;
int samplerate[] = {25, 125, 500};
int adcrate;

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
    !DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
    ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

int err;
uint16_t buf;

struct adc_sequence sequence = {
    .buffer = &buf,
    /* buffer size in bytes, not number of samples */
    .buffer_size = sizeof(buf),
};

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
    DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
                         DT_SPEC_AND_COMMA)};

const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
K_SEM_DEFINE(adc_sem, 0, 1);

/* Check overlay exists for CDC UART console */
BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart), "Console device is not ACM CDC UART device");
// DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart);
static const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));
/*DATA STRUCTS */

/*
Storage for the data to be sent:
String 0: temp
String 1: press
String 2: UV
String 3: Light
String 4: ADC
*/
char chardata[5][20];
// BME DATA
struct sensor_value temp, press, humidity;

/*HELPER FUNCTIONS*/

void int_to_string(int num, char *str)
{
    sprintf(str, "%d", num);
}
void float_to_string(float num, char *str)
{
    sprintf(str, "%.2f", num); // round to 2 decimal places
}

void concatenate_strings(char *str1, char *str2, char *result) // USED FOR BME
{
    strcpy(result, str1);
    strcat(result, ".");
    strcat(result, str2);
}

/*UART FUNCTIONS*/
// DONT CHANGE THIS

int uart_init(void)
{

    if (uart_dev == NULL)
    {
        printk("Device NULL");
        return 0;
    }

    if (!device_is_ready(uart_dev))
    {
        printk("UART device not ready\n");
        return 0;
    }

    return 1;
}
/*

This will send the chardata array

*/

void sender()
{
    int index = 1;
    // while (1)
    // // Increase to send more lines
    // {
    // Increase to send more lines
    int num_of_lines = 5;
    if (!device_is_ready(uart_dev))
    {
        printk("UART device not ready\n");
        return;
    }
    printk("inside sender");
    char ch;
    if (uart_poll_in(uart_dev, &ch) == 0)
    {
        if (ch == '1') // data sender
        {
            for (int i = 0; i < num_of_lines; i++)
            {
                printk("Sending :");
                for (int j = 0; j < 8; j++)
                {
                    if (chardata[i][j] == '␀')
                        break;
                    uart_poll_out(uart_dev, chardata[i][j]);
                    printk(" %c", chardata[i][j]);
                }
                printk("\n");
                uart_poll_out(uart_dev, '\n');
                k_sleep(K_MSEC(100)); // Sending every second (CAN BE CHANGED, CHANGE ALSO RECV DELAY)
            }
        }
        else if (ch == '2')
        {

            adcrate = samplerate[index];
            index = index == 2 ? 0 : index++;
            char buffer[6];
            int_to_string(adcrate, buffer);
            printk("Sending :");
            for (int j = 0; j < 6; j++)
            {
                if (buffer[j] == '␀')
                    break;
                uart_poll_out(uart_dev, buffer[j]);
                printk(" %c", buffer[j]);
            }
            printk("\n");
            uart_poll_out(uart_dev, '\n');
            k_sleep(K_MSEC(100));
        }
        // else
        //     continue;
    }
    // }
}

/* SENSORS FUNCTIONS*/
const struct device *i2c_dev;
struct device *const dev = DEVICE_DT_GET_ONE(bosch_bme680);
void adc_sem_handler()
{
    while (1)
    {
        k_sem_take(&adc_sem, K_FOREVER);
        printk("Inside Semaphore...!!!\n");
        gpio_pin_toggle_dt(&led0);
        k_msleep(1000);
    }
}

void ADCinit_sensor()
{

    /* Configure channels individually prior to sampling. */
    for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++)
    {
        if (!adc_is_ready_dt(&adc_channels[i]))
        {
            printk("ADC controller device %s not ready\n", adc_channels[i].dev->name);
        }

        err = adc_channel_setup_dt(&adc_channels[i]);
        if (err < 0)
        {
            printk("Could not setup channel #%d (%d)\n", i, err);
        }
    }
}

void UV_init(void)
{
    i2c_dev = device_get_binding(I2C_PORT);
    // const struct device *const i2c_dev = device_get_binding(I2C_PORT);
    if (i2c_dev == NULL || !device_is_ready(i2c_dev))
    {
        printk("UV NOT READY");
    }
}

void UVsensor_config()
{
    i2c_reg_write_byte(i2c_dev, SENSOR_ADDRESS, 0x00, 0x02);
    k_sleep(K_MSEC(1000));
    uint8_t status, gain, res;
    i2c_reg_read_byte(i2c_dev, SENSOR_ADDRESS, 0x07, &status);
    // printk("The status value is %u\n",status);
    i2c_reg_read_byte(i2c_dev, SENSOR_ADDRESS, 0x05, &gain);
    // printk("The gain value is %u\n",gain);
    i2c_reg_read_byte(i2c_dev, SENSOR_ADDRESS, 0x04, &res);
    // printk("The resoltuion value is %u\n",res);
}
void read_ALS()
{
    i2c_reg_write_byte(i2c_dev, SENSOR_ADDRESS, 0x00, 0x02);
    uint8_t msb, mid, lsb;
    uint32_t als_data;
    char t1[8];
    i2c_reg_read_byte(i2c_dev, SENSOR_ADDRESS, 0x0F, &msb);
    i2c_reg_read_byte(i2c_dev, SENSOR_ADDRESS, 0x0E, &mid);
    i2c_reg_read_byte(i2c_dev, SENSOR_ADDRESS, 0x0D, &lsb);

    als_data = msb << 16 | mid << 8 | lsb;
    double als_data_new = msb * 65536 + mid * 256 + lsb;
    printk("Als new %f\n", als_data_new);
    final_light = (0.6 * als_data_new) / (3 * 1);
    printk("The ALS value is %u\n", final_light);
    int_to_string(final_light, chardata[3]);
    // concatenate_strings(t1, chardata[0]);
}
void read_UV()
{
    i2c_reg_write_byte(i2c_dev, SENSOR_ADDRESS, 0x00, 0x0A);
    char t1[8];
    uint8_t x, y, z;
    uint8_t a, b, c;

    i2c_reg_read_byte(i2c_dev, SENSOR_ADDRESS, 0x12, &x);
    // printk("The resoltuion value is %u\n",x);
    i2c_reg_read_byte(i2c_dev, SENSOR_ADDRESS, 0x11, &y);
    // printk("The resoltuion value is %u\n",y);
    i2c_reg_read_byte(i2c_dev, SENSOR_ADDRESS, 0x10, &z);
    // printk("The resoltuion value is %u\n",z);
    int32_t data;

    data = x << 16 | y << 8 | z;
    data = data / (3 * 1);
    printk("The final UV Index is %u\n", data);
    int_to_string(data, chardata[2]);
}

void bme680_init(void)
{

    // const struct device *const dev = DEVICE_DT_GET_ONE(bosch_bme680);
    if (dev == NULL)
    {

        /* No such node, or the node does not have status "okay". */

        printk("\nError: no device found.\n");

        return NULL;
    }

    if (!device_is_ready(dev))
    {

        printk("\nError: Device \"%s\" is not ready; "

               "check the driver initialization logs for errors.\n",

               dev->name);

        return NULL;
    }
    return dev;
}

void bme680_fetchSample(void)
{
    sensor_sample_fetch(dev);
    printk("258");
}
void bme680_gettemp()
{
    sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
    char t1[8], t2[8];
    int_to_string(temp.val1, t1);
    int_to_string(temp.val2, t2);
    concatenate_strings(t1, t2, chardata[0]);
    printk("T1: %s \t T2: %s\t Con: %s\n", t1, t2, chardata[0]);
}
void bme680_getpress()
{
    sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);
    char t1[8], t2[8];
    int_to_string(press.val1, t1);
    int_to_string(press.val2, t2);
    concatenate_strings(t1, t2, chardata[1]);
    printk("T1: %s \t T2: %s\t Con: %s\n", t1, t2, chardata[1]);
}
void moisturefetch_ADC()
{
    adcrate = samplerate[0];
    while (1)
    {
        for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++)
        {
            int32_t val_mv;
            (void)adc_sequence_init_dt(&adc_channels[i], &sequence);

            err = adc_read_dt(&adc_channels[i], &sequence);
            if (err < 0)
            {
                printk("Could not read (%d)\n", err);
                continue;
            }

            /*
             * If using differential mode, the 16 bit value
             * in the ADC sample buffer should be a signed 2's
             * complement value.
             */
            if (adc_channels[i].channel_cfg.differential)
            {
                val_mv = (int32_t)((int16_t)buf);
            }
            else
            {
                val_mv = (int32_t)buf;
            }
            // printk("%"PRId32, val_mv);
            err = adc_raw_to_millivolts_dt(&adc_channels[i],
                                           &val_mv);
            /* conversion to mV may not be supported, skip if not */
            if (err < 0)
            {
                printk(" (value in mV not available)\n");
            }

            else
            {
                printk(" SOil Moisture Value: %" PRId32 " mV\n", val_mv);
            }

            int_to_string(val_mv, chardata[4]);

            if (val_mv > 2600)
            {
                printk("Dry soil!!!!\n");
                k_sem_give(&adc_sem);
                continue;
            }

            else

                gpio_pin_set_dt(&led0, 0);
        }
        k_sleep(K_MSEC(adcrate));
    }
}
int main(void)
{
    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
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

    if (!device_is_ready(uart_dev))
    {
        printk("UART device not ready\n");
        return;
    }
    // printk("TEST111");
    /*SENSOR INIT*/

    bme680_init();
    // printk("TEST222");
    UV_init();
    // printk("TEST333");
    ADCinit_sensor();
    // printk("TEST444");
    while (1)
    {
        bme680_fetchSample();
        // printk("bme sample");
        bme680_gettemp();
        // printk("temp");
        bme680_getpress();
        // printk("press");
        UVsensor_config();
        // printk("UV config");
        read_ALS();
        // printk("ALS");
        read_UV();
        // moisturefetch_ADC();
        //  printk(" UV");
        //  moisturefetch_ADC();
        //  printk("ADC");
        sender();
        k_sleep(K_MSEC(500));
    }
}

K_THREAD_DEFINE(adc_interrupt, 5000, adc_sem_handler, NULL, NULL, NULL, 5, 0, 0);
// K_THREAD_DEFINE(senderrecv, 500, sender, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(adc_sampler, 5000, moisturefetch_ADC, NULL, NULL, NULL, 5, 0, 0);