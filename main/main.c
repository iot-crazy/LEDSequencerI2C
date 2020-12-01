/* LEDSequencer

    Play sequences to a bank of 16 LEDs using an MCP23018 PIO controller
*/
#include <stdio.h>
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/i2c.h"
#include "patterns.h"



/* Settings for the i2c library */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */
#define GPIO_PULLUP_ENABLE true
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_NUM I2C_NUM_0

#define MCP_ADDRESS 0x20 /* The address of the MCp23018 device, refer to data sheet for how to set this */

/* Register addresses in the MCP23018 device */
#define IODIR 0x00
#define GPIOA 0x12
#define GPIOB 0x13
#define INTCONA 0x08
#define INTCONB 0x09
#define GPINTENA 0x04
#define GPINTENB 0x05
#define IOCONA 0x0A
#define IOCONB 0x0B
#define GPPUA 0x0C
#define GPPUB 0x0D

typedef enum
{
    Forward = 0,
    Backward = 1,
    ForwardThenBackward = 2,
    BackwardThenForward = 3
} playDirection;

struct displayData
{
    int a;
    int b;
};
struct displayData d;

SemaphoreHandle_t xMutex;

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void writeI2c(uint8_t regaddr, int data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MCP_ADDRESS << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
}

/* Write the frame to the LED array */
void writeDisplay(void *pvParameters)
{
    bool bankSelect = true; // start with register bank A
    for (;;)
    {
        //get the display data
        xSemaphoreTake(xMutex, 1 / portTICK_PERIOD_MS);
        int a = d.a;
        int b = d.b;
        xSemaphoreGive(xMutex);
        int targetRegister;
        int sourceData;

        // toggle between the registers so only one LED is on at a time, the unused register can all be turned off
        if (bankSelect)
        {
            targetRegister = GPIOA;
            sourceData = a;
            writeI2c(GPIOB, 0xFF);
            bankSelect = false;
        }
        else
        {
            targetRegister = GPIOB;
            sourceData = b;
            writeI2c(GPIOA, 0xFF);
            bankSelect = true;
        }

        for (int i = 1; i <= 128; i = i * 2)
        {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (MCP_ADDRESS << 1) | WRITE_BIT, ACK_CHECK_EN);
            i2c_master_write_byte(cmd, targetRegister | WRITE_BIT, ACK_CHECK_EN);
            i2c_master_write_byte(cmd, ~(sourceData & i) | WRITE_BIT, ACK_CHECK_EN); // the register is in open drain mode, so we must invert the bits to zero means 'LED ON'
            i2c_master_stop(cmd);
            i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
            i2c_cmd_link_delete(cmd);
            vTaskDelay((1 / portTICK_PERIOD_MS));
        }
    }
}

/* Write an image to the buffer for display */
void writeFrame(int content)
{
    xSemaphoreTake(xMutex, 1 / portTICK_PERIOD_MS);
    d.a = content & 0xff;
    d.b = content >> 8;
    xSemaphoreGive(xMutex);
}

void playSequencePhase(int sequence[], int startIndex, int endIndex, int delayMillis, bool direction)
{
    int pos = startIndex;
    do
    {
        writeFrame(sequence[pos]);
        vTaskDelay(delayMillis / portTICK_PERIOD_MS);
        direction ? pos++ : pos--;
    } while (pos != endIndex);
}

/* Play a sequence from an array */
void playSequence(int sequence[], int sequenceLength, int runTimes, int delayMillis, playDirection direction)
{
    for (int t = 0; t < runTimes; t++)
    {
        switch (direction)
        {
        case Forward:
            playSequencePhase(sequence, 0, sequenceLength, delayMillis, true);
            break;

        case ForwardThenBackward:
            playSequencePhase(sequence, 0, sequenceLength, delayMillis, true);
            playSequencePhase(sequence, sequenceLength - 2, 1, delayMillis, false);
            break;

        case BackwardThenForward:
            playSequencePhase(sequence, sequenceLength - 1, 1, delayMillis, false);
            playSequencePhase(sequence, 0, sequenceLength, delayMillis, true);
            break;

        case Backward:
            playSequencePhase(sequence, sequenceLength, 0, delayMillis, false);
            break;
        }
    }
}

/*    */
void playMovie(void *pvParameters)
{
    for (;;)
    {
        playSequence(inout, (sizeof(inout) / sizeof(inout[0])), 5, 100, ForwardThenBackward);
        playSequence(sideside, (sizeof(sideside) / sizeof(sideside[0])), 5, 250, Forward);
        playSequence(halfsideside, (sizeof(halfsideside) / sizeof(halfsideside[0])), 5, 250, Forward);
        /* for (int t = 0; t < 5; t++)
        {
            int x = 1;
            while (x <= 32768)
            {
                xSemaphoreTake(xMutex, 1 / portTICK_PERIOD_MS);
                d.a = x & 0xff;
                d.b = x >> 8;
                xSemaphoreGive(xMutex);
                vTaskDelay((50 / portTICK_PERIOD_MS));
                x = x * 2;
            }
        }*/
    }
}

void configMCP()
{
    ESP_ERROR_CHECK(i2c_master_init());
    writeI2c(GPPUA, 0xFF);
    writeI2c(GPPUB, 0xFF);
    writeI2c(IODIR, 0x0000);
    writeI2c(IOCONA, 0b00000110);
    writeI2c(IOCONB, 0b00000110);
}

void app_main(void)
{
    printf("Starting \n");
    xMutex = xSemaphoreCreateMutex();

    configMCP();
    writeI2c(GPIOA, 0x00);
    writeI2c(GPIOB, 0x00);
    xTaskCreatePinnedToCore(playMovie, "Play the movie", 1000, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(writeDisplay, "Write data to the display", 1000, NULL, 10, NULL, 1);
}
