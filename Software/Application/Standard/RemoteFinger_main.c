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
/********************************* TASKS SCHEDULE *******************************/
/* BluetoothComms_Task - runs every 100ms, priority MIN+1, conditional execution based on receiving Queue message
 * AcquireSensorData_Task - runs every 100ms, priority MAX+2
/********************************* HARDWARE ************************************/
/* Raspberry Pico W - RP2040 raspberry microcontroller board
 * MPU6050 - world’s first integrated 6-axis MotionTracking device that combines
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
#include "pico/cyw43_arch.h"

/* Error handling macros */
#define STATUS_SUCCESS						 0
#define MPU6050_REGISTER_I2C_READ_FAIL		 ((uint8_t)0xFF)
#define MPU6050_SENSOR_DATA_READ_FAIL		 ((uint32_t)0xDEADBEEF)

/* Priorities for the tasks */
#define	AcquireSensorData_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define BluetoothComms_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

/* Task periods (ms) */
#define AcquireSensorData_TASK_PERIOD		( 100 )
#define BluetoothComms_TASK_PERIOD			( 100 )

/* The number of items the queue can hold */
#define mainQUEUE_LENGTH					( 1 )

/* By default the MPU6050 devices are on bus address 0x68 */ 
#define MPU6050_I2C_ADDRESS   				 0x68
#define I2C_BAUD_RATE_400KHz				 ((uint32_t)4E5)

/*-----------------------------------------------------------*/

/* Main function called by main() from main.c (lol). This one does some setup and starts the scheduler */
void RemoteFinger_main( void );

/*-----------------------------------------------------------*/

/* Tasks declarations */
static void BluetoothComms_Task();
static void AcquireSensorData_Task();

/*-----------------------------------------------------------*/

/* The queue instance */
static QueueHandle_t xQueue = NULL;

/*-----------------------------------------------------------*/

typedef struct 
{
	float X;
	float Y;
	float Z;
}AxisType;

/*-----------------------------------------------------------*/

#ifdef i2c_default

static uint8_t read_mpu6050_register(uint8_t registerAddress)
{
	uint8_t reg_value;
    const uint8_t reg_address = registerAddress;
	const uint32_t maxRetries = 5;
	const uint32_t retryDelayUs = 5;
	const size_t dataToSend_length = sizeof(reg_address);
	const size_t dataToRead_length = sizeof(reg_value);
	uint32_t errorCount = 0;

	/* Send a request to read a register of the slave device */
	/* This is done with a write transaction providing the address of the register you want to read */
    while(!i2c_write_blocking(i2c_default, MPU6050_I2C_ADDRESS, &reg_address, dataToSend_length, true))
	{
		printf("I2C write transaction failed. Retrying... "); /* Data not acknowledged by slave (or some other error) */

		sleep_us(retryDelayUs);
		errorCount++;
		if(errorCount > maxRetries)
		{
			return MPU6050_REGISTER_I2C_READ_FAIL;
		}
	}
	errorCount = 0;

	/* Send a read command specifying the number of bytes (1) you want to read (of the register address you specified in prev command) */
    while(!i2c_read_blocking(i2c_default, MPU6050_I2C_ADDRESS, &reg_value, dataToRead_length, false))
	{
		printf("I2C read transaction failed. Retrying... "); 

		sleep_us(retryDelayUs);
		errorCount++;
		if(errorCount > maxRetries)
		{
			return MPU6050_REGISTER_I2C_READ_FAIL;
		}
	}

	return reg_value;
}

static uint8_t mpu6050_reset() 
{
	size_t length;
	uint32_t errorCount = 0;
	const uint32_t maxRetries = 5;
	const uint32_t retryDelayUs = 5;

	/* Register: PWR_MGMT_1 (0x6B), Value: 0x80, Action: Reset all internal registers (of MPU6050) to default values */
    uint8_t outputData_Reset[] = {0x6B, 0x80};
	length = sizeof(outputData_Reset);
	while(!i2c_write_blocking(i2c_default, MPU6050_I2C_ADDRESS, outputData_Reset, length, false))
	{
		printf("I2C write transaction failed. Retrying... "); /* Data not acknowledged by slave (or some other error) */

		sleep_us(retryDelayUs);
		errorCount++;
		if(errorCount > maxRetries)
		{
			return MPU6050_REGISTER_I2C_READ_FAIL;
		}
	}
	errorCount = 0;

	sleep_ms(1); /* Give the slave device some time to perform the reset */

	/* Register: PWR_MGMT_1 (0x6B), Value: 0x00, Action: Take MPU6050 out of sleep mode (which is the default state after reset) */
	uint8_t outputData_WakeUp[] = {0x6B, 0x00};
	length = sizeof(outputData_WakeUp);
	while(!i2c_write_blocking(i2c_default, MPU6050_I2C_ADDRESS, outputData_WakeUp, length, false))
	{
		printf("I2C write transaction failed. Retrying... ");

		sleep_us(retryDelayUs);
		errorCount++;
		if(errorCount > maxRetries)
		{
			return MPU6050_REGISTER_I2C_READ_FAIL;
		}
	}

	return STATUS_SUCCESS;
}

/* -------------------------------------------------------------------------------------*/
/* Function for reading sensor data (Accelerometer, Gyroscope, SensorTemp) from MPU6050 */
/* -------------------------------------------------------------------------------------*/
/** Gyroscope measurements are written to the registers read in this function at the Sample Rate (8kHz default)
 * as defined in register SMPRT_DIV (0x19) 
 * By default Full Scale Range is +/- 250 deg/s and LSB Sensitivity is 131 LSB/(deg/s) so per 1 digital value (1 LSB)
 * we have 0.00763 deg/s
 * Each 16-bit gyroscope measurement has a full scale defined in FS_SEL field of GYRO_CONFIG register (0x1B)  */
/* -----------------------------------------------------------------------------------------------------------*/
/** Accelerometer measurements are written to the registers read in this function at the Sample Rate as defined in 
 * register SMPRT_DIV (0x19) (8kHz default but accelerometer output is limited to 1kHz so it sends the same samples 
 * more than once if Sample Rate is higher)
 * By default Full Scale Range is +/- 2g and LSB Sensitivity is 16384 LSB/g so per 1 digital value (1 LSB) we have 0.000061g
 * Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS field of ACCEL_CONFIG register (0x1C) */
/* -----------------------------------------------------------------------------------------------------------------*/
/** Temp measurements are written to the registers at the Sample Rate (8kHz default) as defined in register SMPRT_DIV (0x19)
 * Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53 */

static uint32_t mpu6050_read_sensor_data(AxisType *AccelerometerInstance, AxisType *GyroscopeInstance, float *Temperature) 
{
    uint8_t readBuffer[6];
	uint8_t regAddress;
	size_t lengthToSend;
	size_t lengthToRead;
	uint32_t errorCount = 0;
	const uint32_t maxRetries = 5;
	const uint32_t retryDelayUs = 5;

	/* Registers: ACCEL_XOUT_H (0x3B), ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L (0x40) */
	/* Values: ACCEL_XOUT[15:8], ACCEL_XOUT[7:0], ACCEL_YOUT[15:8], ACCEL_YOUT[7:0], ACCEL_ZOUT[15:8], ACCEL_ZOUT[7:0] */ 
	/* Action: Read Accelerometer Measurements */
    regAddress = 0x3B;
	lengthToSend = sizeof(regAddress);
	lengthToRead = sizeof(readBuffer);
	/* Request read at address 0x3B by sending a write transaction with the address */
	while(!i2c_write_blocking(i2c_default, MPU6050_I2C_ADDRESS, &regAddress, lengthToSend, true))
	{
		printf("I2C write transaction failed. Retrying... ");

		sleep_us(retryDelayUs);
		errorCount++;
		if(errorCount > maxRetries)
		{
			return MPU6050_SENSOR_DATA_READ_FAIL;
		}
	}
	errorCount = 0;
	/* Send a read transaction with the number of bytes (6) you want to read, to which the slave device will respond with the requested data */ 
	while(!i2c_read_blocking(i2c_default, MPU6050_I2C_ADDRESS, readBuffer, lengthToRead, false))
	{
		printf("I2C read transaction failed. Retrying... ");

		sleep_us(retryDelayUs);
		errorCount++;
		if(errorCount > maxRetries)
		{
			return MPU6050_SENSOR_DATA_READ_FAIL;
		}
	}
	errorCount = 0;

	/* Combine the High and Low words (8bit values) of each accelerometer axis into int16 values */
	/* Also divide by LSB sensitivity (16384 LSB/g by default) to get the value in 'g' unit */
    AccelerometerInstance->X = ((int16_t)((readBuffer[0] << 8) | (readBuffer[1])))/16384.0F;
	AccelerometerInstance->Y = ((int16_t)((readBuffer[2] << 8) | (readBuffer[3])))/16384.0F;
	AccelerometerInstance->Z = ((int16_t)((readBuffer[4] << 8) | (readBuffer[5])))/16384.0F;

	/* Registers: GYRO_XOUT_H (0x43), GYRO_XOUT_L, GYRO_YOUT_H, GYRO_YOUT_L, GYRO_ZOUT_H, and GYRO_ZOUT_L (0x48) */
	/* Values: GYRO_XOUT[15:8], GYRO_XOUT[7:0], GYRO_YOUT[15:8], GYRO_YOUT[7:0], GYRO_ZOUT[15:8], GYRO_ZOUT[7:0] */ 
	/* Action: Read Gyroscope Measurements */
    regAddress = 0x43;
	/* Request read at address 0x3B by sending a write transaction with the address */
	while(!i2c_write_blocking(i2c_default, MPU6050_I2C_ADDRESS, &regAddress, lengthToSend, true))
	{
		printf("I2C write transaction failed. Retrying... ");

		sleep_us(retryDelayUs);
		errorCount++;
		if(errorCount > maxRetries)
		{
			return MPU6050_SENSOR_DATA_READ_FAIL;
		}
	}
	errorCount = 0;
	/* Send a read transaction with the number of bytes (6) you want to read, to which the slave device will respond with the requested data */
	while(!i2c_read_blocking(i2c_default, MPU6050_I2C_ADDRESS, readBuffer, lengthToRead, false))
	{
		printf("I2C read transaction failed. Retrying... ");

		sleep_us(retryDelayUs);
		errorCount++;
		if(errorCount > maxRetries)
		{
			return MPU6050_SENSOR_DATA_READ_FAIL;
		}
	}
	errorCount = 0;

	/* Combine the High and Low words (8bit values) of each gyroscope axis into int16 values */
	/* Also divide by LSB sensitivity (131 LSB/(deg/s) by default) to get the value in 'deg/s' unit */
    GyroscopeInstance->X = ((int16_t)((readBuffer[0] << 8) | (readBuffer[1])))/131.0F;
	GyroscopeInstance->Y = ((int16_t)((readBuffer[2] << 8) | (readBuffer[3])))/131.0F;
	GyroscopeInstance->Z = ((int16_t)((readBuffer[4] << 8) | (readBuffer[5])))/131.0F;

	/* Registers: TEMP_OUT_H (0x41) and TEMP_OUT_L (0x42) */
	/* Values: TEMP_OUT[15:8], TEMP_OUT[7:0] */ 
	/* Action: Read Temperature Measurements */
    regAddress = 0x41;
	lengthToSend = sizeof(regAddress);
	lengthToRead = 2U;
	/* Request read at address 0x3B by sending a write transaction with the address */
	while(!i2c_write_blocking(i2c_default, MPU6050_I2C_ADDRESS, &regAddress, lengthToSend, true))
	{
		printf("I2C write transaction failed. Retrying... ");

		sleep_us(retryDelayUs);
		errorCount++;
		if(errorCount > maxRetries)
		{
			return MPU6050_SENSOR_DATA_READ_FAIL;
		}
	}
	errorCount = 0;
	
	/* Send a read transaction with the number of bytes (6) you want to read, to which the slave device will respond with the requested data */
	while(!i2c_read_blocking(i2c_default, MPU6050_I2C_ADDRESS, readBuffer, lengthToRead, false))
	{
		printf("I2C write transaction failed. Retrying... ");

		sleep_us(retryDelayUs);
		errorCount++;
		if(errorCount > maxRetries)
		{
			return MPU6050_SENSOR_DATA_READ_FAIL;
		}
	}

	/* Combine the High and Low words (8bit values) of the temperature into a int16 value */
	/* Also convert the raw value to actual Celsius degress with: (Temp / 340.0) + 36.53)*/
    *Temperature = ((int16_t)((readBuffer[0] << 8) | (readBuffer[1]))/340.0F) + 36.53F;

	return STATUS_SUCCESS;
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

	/* TO DO - Add error logging to NVM, reset reactions, system status indicators such as LEDs or 7seg or LCD  */
    (void)mpu6050_reset();
	printf("MPU6050 config completed \n");
#endif

	printf("Setting up the RTOS configuration... \n");
    /* Create the queue. */
	xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint32_t ) );

	if( xQueue != NULL )
	{
		/* Create the tasks */
		xTaskCreate( BluetoothComms_Task,				/* The function that implements the task. */
					"BluetoothComms_Task", 								/* The text name assigned to the task - for debug only as it is not used by the kernel. */
					configMINIMAL_STACK_SIZE, 			/* The size of the stack to allocate to the task. */
					NULL, 								/* The parameter passed to the task - not used in this case. */
					BluetoothComms_TASK_PRIORITY, 	/* The priority assigned to the task. */
					NULL );								/* The task handle is not required, so NULL is passed. */

		xTaskCreate( AcquireSensorData_Task, "AcquireSensorData_Task", configMINIMAL_STACK_SIZE, NULL, AcquireSensorData_TASK_PRIORITY, NULL );

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

static void AcquireSensorData_Task()
{
	TickType_t xTaskStartTime;
	const TickType_t xTaskPeriod = pdMS_TO_TICKS(AcquireSensorData_TASK_PERIOD);

	xTaskStartTime = xTaskGetTickCount();

	for( ;; )
	{
		printf("ENTERING SENSOR TASK \n");
		printf("Task Start Time: %x \n", xTaskStartTime);

		/* Toggle the onboard LED to signal data reading */
		cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

#ifdef i2c_default
		static AxisType AccelerometerInstance, GyroscopeInstance; 
		static float Temperature;

		/* TO DO - Add error logging to NVM, reset reactions, system status indicators such as LEDs or 7seg or LCD  */			
		(void)mpu6050_read_sensor_data(&AccelerometerInstance, &GyroscopeInstance, &Temperature);

		/* Print MPU6050 data */
		printf("Accelerometer[g]: X = %f, Y = %f, Z = %f\n", AccelerometerInstance.X, AccelerometerInstance.Y, AccelerometerInstance.Z);
		printf("Gyroscope[deg/s]: X = %f, Y = %f, Z = %f\n", GyroscopeInstance.X, GyroscopeInstance.Y, GyroscopeInstance.Z);
		printf("Sensor Temperature[degC]: %f \n", Temperature);

		/* Send sensor data to the queue */
		xQueueSend(xQueue, &AccelerometerInstance, portMAX_DELAY);
#endif
		/* Wait task period before reading and sending new data */
		vTaskDelayUntil(&xTaskStartTime, xTaskPeriod);
	}

}

static void BluetoothComms_Task()
{
	AxisType AccelerometerInstance;
	TickType_t xTaskStartTime;
	const TickType_t xTaskPeriod = pdMS_TO_TICKS(BluetoothComms_TASK_PERIOD);

	xTaskStartTime = xTaskGetTickCount();

	for( ;; )
	{
		/* xQueueReceive will block this task (indefinitely, due to portMAX_DELAY) 
		 * and not execute any further instructions until a message is received on the queue */
		xQueueReceive(xQueue, &AccelerometerInstance, portMAX_DELAY );

		/* Execute this code only if sensor data is valid,
		 * clean 0.0 would only happen if sensor is working incorrectly, I think */
		if( AccelerometerInstance.X != 0.0F )
		{

			printf("ENTERING BLUETOOTH TASK \n");
			printf("Task Start Time: %x \n", xTaskStartTime);
			printf(" -data sending will be done here- \n");

			printf("Example of sensor data received from the queue: X = %f \n", AccelerometerInstance.X);
		}
		else
		{
			printf("\n\n WARNING - Invalid sensor data \n\n");
		}
		/* Wait task period before acquiring new data */
		vTaskDelayUntil(&xTaskStartTime, xTaskPeriod);
	}
}






