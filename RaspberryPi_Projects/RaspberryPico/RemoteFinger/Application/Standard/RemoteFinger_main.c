/********************************* SYSTEM *************************************/
/* FreeRTOS V202107.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 *******************************************************************************/

/********************************* HARDWARE ************************************/
/* MPU6050 - world’s first integrated 6-axis MotionTracking device that combines
 * a 3-axis gyroscope, 3-axis accelerometer, and a Digital Motion Processor™ (DMP) 
 * Datasheet: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 * Register map: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 *******************************************************************************/

/* Standard includes. */
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* SDK includes */
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

/* Priorities for the tasks */
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

/* The rate at which data is sent to the queue. The rate is once every mainQUEUE_SEND_FREQUENCY_MS (once every 1000ms by default) */
#define mainQUEUE_SEND_FREQUENCY_MS			( 1000 / portTICK_PERIOD_MS )

/* The number of items the queue can hold */
#define mainQUEUE_LENGTH					( 1 )

/* By default the MPU6050 devices are on bus address 0x68 */ 
#define MPU6050_I2C_ADDRESS   				 0x68
#define I2C_BAUD_RATE_400KHz				 ((uint32_t)4E5)
#define MPU6050_REGISTER_I2C_READ_FAIL		 ((uint8_t)0xFF)

/*-----------------------------------------------------------*/

/* Main function called by main() from main.c (lol). This one does some setup and starts the scheduler */
void RemoteFinger_main( void );

/*-----------------------------------------------------------*/

/* Tasks declarations */
static void prvQueueReceiveTask( void *pvParameters );
static void prvQueueSendTask( void *pvParameters );

/*-----------------------------------------------------------*/

/* The queue instance */
static QueueHandle_t xQueue = NULL;

/*-----------------------------------------------------------*/

#ifdef i2c_default

static uint8_t read_mpu6050_register(uint8_t registerAddress)
{
	uint8_t reg_value;
    uint8_t reg_address = registerAddress;
	uint32_t errorCount = 0;

    while((!i2c_write_blocking(i2c_default, MPU6050_I2C_ADDRESS, &reg_address, 1, true)) && (errorCount < 6));
	{
		errorCount++;
		printf("I2C write transaction failed. Retrying... "); /* Data not acknowledged by slave (or some other error) */
	}

    while((!i2c_read_blocking(i2c_default, MPU6050_I2C_ADDRESS, &reg_value, 1, false)) && (errorCount < 6));
	{
		errorCount++;
		printf("I2C read transaction failed. Retrying... "); 
	}

	if(errorCount >= 6)
	{
		return MPU6050_REGISTER_I2C_READ_FAIL;
	}
	else
	{
		return reg_value;
	}
}

static void mpu6050_reset() 
{
	size_t length;

	/* Register: PWR_MGMT_1 (0x6B), Value: 0x80, Action: Reset all internal registers (of MPU6050) to default values */
    uint8_t outputData_Reset[] = {0x6B, 0x80};
	length = sizeof(outputData_Reset);
	i2c_write_blocking(i2c_default, MPU6050_I2C_ADDRESS, outputData_Reset, length, false);
	sleep_us(1); /* Give the slave device some time to perform the reset */

	/* Register: PWR_MGMT_1 (0x6B), Value: 0x00, Action: Take MPU6050 out of sleep mode (which is the default state after reset) */
	uint8_t outputData_WakeUp[] = {0x6B, 0x00};
	length = sizeof(outputData_WakeUp);
	i2c_write_blocking(i2c_default, MPU6050_I2C_ADDRESS, outputData_WakeUp, length, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) 
{
    /* For this particular device, we send the device the register we want to read
     * first, then subsequently read from the device. The register is auto incrementing
     * so we don't need to keep sending the register we want, just the first.
	 */

    uint8_t buffer[6];

    /* Start reading acceleration registers from register 0x3B for 6 bytes */
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDRESS, &val, 1, true); 
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    /* Now gyro data from reg 0x43 for 6 bytes. The register is auto incrementing on each read */
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDRESS, buffer, 6, false);  

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    /* Now temperature from reg 0x41 for 2 bytes. The register is auto incrementing on each read */
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU6050_I2C_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU6050_I2C_ADDRESS, buffer, 2, false);

    *temp = buffer[0] << 8 | buffer[1];
}
#endif

void RemoteFinger_main( void )
{
#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
    #warning i2c/mpu6050_i2c code requires a board with I2C pins
    printf("Default I2C pins were not defined");
#else
    printf("Hello, MPU6050! Setting up I2C config... \n");
    /* Set I2C for MPU6050 communication on the default I2C0 SDA and SCL pins (4, 5 on a Pico) */
    i2c_init(i2c_default, I2C_BAUD_RATE_400KHz);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    mpu6050_reset();
	printf("MPU6050 config completed \n");
#endif

	printf("Setting up the RTOS configuration... \n");
    /* Create the queue. */
	xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint32_t ) );

	if( xQueue != NULL )
	{
		/* Create the tasks */
		xTaskCreate( prvQueueReceiveTask,				/* The function that implements the task. */
					"Rx", 								/* The text name assigned to the task - for debug only as it is not used by the kernel. */
					configMINIMAL_STACK_SIZE, 			/* The size of the stack to allocate to the task. */
					NULL, 								/* The parameter passed to the task - not used in this case. */
					mainQUEUE_RECEIVE_TASK_PRIORITY, 	/* The priority assigned to the task. */
					NULL );								/* The task handle is not required, so NULL is passed. */

		xTaskCreate( prvQueueSendTask, "TX", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL );

		/* Start the tasks and timer running. */
		printf("RTOS configuration finished, starting the scheduler... \n");
		vTaskStartScheduler();
	}

	/* If all is well, the scheduler will now be running, and the following
	line will never be reached.  If the following line does execute, then
	there was insufficient FreeRTOS heap memory available for the Idle and/or
	timer tasks to be created.  See the memory management section on the
	FreeRTOS web site for more details on the FreeRTOS heap
	http://www.freertos.org/a00111.html. */
	for( ;; );
}

static void prvQueueSendTask( void *pvParameters )
{
	TickType_t xNextWakeTime;
	const unsigned long ulValueToSend = 100UL;

	/* Remove compiler warning about unused parameter. */
	( void ) pvParameters;

	/* Initialise xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();

	for( ;; )
	{
		/* Place this task in the blocked state until it is time to run again. */
		int32_t SendToQueue_freq_ms = mainQUEUE_SEND_FREQUENCY_MS/10;
		vTaskDelayUntil( &xNextWakeTime, SendToQueue_freq_ms );
		printf("SendToQueue_freq_ms = %u ms\n", SendToQueue_freq_ms);
		
		/* Send to the queue - causing the queue receive task to unblock and
		toggle the LED.  0 is used as the block time so the sending operation
		will not block - it shouldn't need to block as the queue should always
		be empty at this point in the code. */
		xQueueSend( xQueue, &ulValueToSend, 0U );
	}
}

static void prvQueueReceiveTask( void *pvParameters )
{
unsigned long ulReceivedValue;
const unsigned long ulExpectedValue = 100UL;

	/* Remove compiler warning about unused parameter. */
	( void ) pvParameters;

	for( ;; )
	{
		/* Wait until something arrives in the queue - this task will block
		indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
		FreeRTOSConfig.h. */
		xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY );

		/*  To get here something must have been received from the queue, but
		is it the expected value?  If it is, perform task activities */
		if( ulReceivedValue == ulExpectedValue )
		{
			gpio_xor_mask( 1u << PICO_DEFAULT_LED_PIN );

#ifdef i2c_default
			int16_t acceleration[3], gyro[3], temp;

			mpu6050_read_raw(acceleration, gyro, &temp);

			/* Read raw values */
			printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
			printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
			/* Read chip temp */
			printf("Temp. = %f\n", (temp / 340.0) + 36.53);		
#endif
			/* Clear the variable so the next time this task runs a correct value needs to be supplied again */
			ulReceivedValue = 0U;
		}
	}
}







