/*
 * main.cpp
 *
 *  Created on: Jan 21, 2020
 *      Author: Jonas Juffinger
 */

/*
------------------------------- README -------------------------------

# Status LED
The green LED blinks while starting up. If the SD card doesn't work the LED blinks very fast for
3 seconds while starting up.


# GPS
For GPS the Adafruit GPS library is used that is ported to the STM32. It is completely interrupt based
to not use much CPU time. To make this possible I had to change the HAL UART driver in the Drivers/
directory. The new driver is in the customized directory and the official one is excluded from the build.
The file from customized/Inc must be copied into the STM32F1xx_HAL_Driver/Inc

The UART3 interrupt must be enabled in the USART3 and NVIC settings.

If there is no GPS fix the LED is off shortly every second. If there is a GPS fix the LED is on
constantly.


# SD Card
On the SD card a data.csv file while be created that contains all sensor data.
Regenerating the code changes the FATFS driver. Change line 181 in FATFS/Target/ffconf.h to
#define _MAX_SS    512


# Barometer
Connect the barometer to SPI1 at the extension header, CS is pin B3.


# Misc
When regenerating the code from the TestLaunchCode.ioc line 155 in stm32f1xx_hal_conf.h must be changed
to 1U
#define  USE_HAL_UART_REGISTER_CALLBACKS        1U // UART register callback disabled
*/



#include "main.h"
#include "fatfs.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "bmp280.h"
#include <Adafruit_GPS.h>


extern ADC_HandleTypeDef hadc1;

extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi1;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern uint32_t millis;


// global buffer
#define BUF_LENGTH 512
char buffer[BUF_LENGTH];

bool radio_send_complete;
char radio_send_buffer[BUF_LENGTH];


FATFS fs;  // file system
FIL datacsv; //, logtxt;  // file
FRESULT fresult;  // to store the result
UINT br, bw;   // file read/write count
/* capacity related variables */
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

struct bmp280_dev bmp;	// barometer

Adafruit_GPS *gps;



void Send_Uart (char *string);
bool Send_Radio(char *buffer);
void UART_Transmit_Complete_Callback(UART_HandleTypeDef *huart);

bool InitSDCard();

void GetAccelReadings(float &X, float &Y, float &Z);

void InitBarometer();
int GetPressureAndTemperatureReading(double &pressure, double &temp);

void InitGPS();
bool GetGPSReading();


extern "C" int maincpp(void) {
	bool sdCardAvailable = false;
	int i;

	// Set Barometer into SPI mode, this must be done before
	// the SD card is initialized
  // Set CS to high, this is the "not selected" state
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);


	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

	sdCardAvailable = InitSDCard();
	// Toggle the LED fast when no SD card is available
	if(!sdCardAvailable) {
		for(i=0; i<30; i++) {
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
			HAL_Delay(100);
		}
	}
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);

	InitBarometer();
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);

	InitGPS();
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);


	radio_send_complete = true;
	HAL_UART_RegisterCallback(&huart2, HAL_UART_TX_COMPLETE_CB_ID, UART_Transmit_Complete_Callback);


	int16_t led_blink_timer = 1000, sync_counter = 10;
	uint16_t loop_delay = 10;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	/* Infinite loop */
	while (1)
	{
		int64_t loop_start_time = millis;

		int rslt = 0;
		float accelX, accelY, accelZ;
		double pressure, temp;

		// Read the accelerometer
		GetAccelReadings(accelX, accelY, accelZ);

		// Read the barometer and temperature
		rslt = GetPressureAndTemperatureReading(pressure, temp);
		if(rslt != 0) {
			// TODO Barometer ERROR
		}

		snprintf(buffer, sizeof(buffer), "%lu,%6.3f,%6.3f,%6.3f,%f,%f", millis,
				accelX, accelY, accelZ, (float)pressure, (float)temp);

		// Read GPS
		if(GetGPSReading())  {
			if(gps->fix) {
				snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer),
						",%f,%f,%f,%f,%f\r\n",
						gps->latitude, gps->longitude, gps->altitude, gps->speed, gps->angle);
			}
			else {
				snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer),
						",NO FIX,%s,,,\r\n", gps->lastNMEA());
			}
		}
		else {
			snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer),
					",,,,,\r\n");
		}

		//Send_Radio(buffer);

		Send_Uart(buffer);

		if(sdCardAvailable) {
			f_puts(buffer, &datacsv);
		}

		sync_counter--;
		if(sync_counter == 0) {
			f_sync(&datacsv);
			sync_counter = 10;
		}



		int64_t loop_duration = millis - loop_start_time;
		int64_t delay_time = loop_delay - loop_duration;
		if(delay_time > 0)
			HAL_Delay(delay_time);

		if(gps->fix)
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		else {
			led_blink_timer -= loop_delay;
			if(led_blink_timer <= 100) {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

				if(led_blink_timer <= 0)
					led_blink_timer = 2000;
			}
			else {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
			}
		}
	}

	/* Close file */
	if(sdCardAvailable)
		f_close(&datacsv);
}



/* to send the data to the uart */
void Send_Uart (char *string)
{
	HAL_UART_Transmit(&huart1, (uint8_t *) string, strlen (string), 2000);  // transmit in blocking mode
}


bool Send_Radio(char *buffer) {
	if(radio_send_complete) {
		radio_send_complete = false;
		memcpy(radio_send_buffer, buffer, strlen(buffer));
		HAL_UART_Transmit_IT(&huart2, (uint8_t*) radio_send_buffer, strlen(radio_send_buffer));
		return true;
	}

	return false;
}

void UART_Transmit_Complete_Callback(UART_HandleTypeDef *huart) {
	radio_send_complete = true;
}



bool InitSDCard() {
	bool sdCardAvailable = false;

	/* Mount SD Card */
	fresult = f_mount(&fs, "", 0);
	if (fresult != FR_OK){
		Send_Uart ((char*)"error in mounting SD CARD...\r\n");
		sdCardAvailable = false;
	}
	else {
		Send_Uart((char*)"SD CARD mounted successfully...\r\n");
		sdCardAvailable = true;
	}

	if(sdCardAvailable) {
		/* Check free space */
		f_getfree("", &fre_clust, &pfs);

		total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
		if(total == 0)
			return false;

		sprintf (buffer, "SD CARD Total Size: \t%lu\r\n",total);
		Send_Uart(buffer);

		memset(buffer, 0, BUF_LENGTH);

		free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
		sprintf (buffer, "SD CARD Free Space: \t%lu\r\n",free_space);
		Send_Uart(buffer);

		/* Open file to write/ create a file if it doesn't exist */
		fresult = f_open(&datacsv, "data.csv", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		f_puts("TIME,X,Y,Z,P,T,LAT,LONG,H,V,ANG\n", &datacsv);

		f_sync(&datacsv);

		//fresult = f_open(&logtxt, "log.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		//f_puts("SD card mounted and working\n", &logtxt);
	}

	return sdCardAvailable;
}





void GetAccelReadings(float &X, float &Y, float &Z) {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	int32_t intZ = HAL_ADC_GetValue(&hadc1) - 2048;

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	int32_t intY = HAL_ADC_GetValue(&hadc1) - 2048;

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	int32_t intX = HAL_ADC_GetValue(&hadc1) - 2048;

	X = ((float)intX) / 372.0 + 0.250;
	Y = ((float)intY) / 372.0 + 0.250;
  Z = ((float)intZ) / 372.0;
}






int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void print_rslt(const char api_name[], int8_t rslt);

void InitBarometer() {
	// Init barometer
  int8_t rslt;
  struct bmp280_config conf;

  // Map the delay function pointer with the function responsible for implementing the delay
  bmp.delay_ms = HAL_Delay;

  bmp.dev_id = GPIO_PIN_3;
  bmp.read = spi_reg_read;
  bmp.write = spi_reg_write;
  bmp.intf = BMP280_SPI_INTF;

  rslt = bmp280_init(&bmp);
  print_rslt(" bmp280_init status", rslt);

  // Always read the current settings before writing, especially when
  // all the configuration is not modified
  rslt = bmp280_get_config(&conf, &bmp);
  print_rslt(" bmp280_get_config status", rslt);

  // configuring the temperature oversampling, filter coefficient and output data rate
  // Overwrite the desired settings
  conf.filter = BMP280_FILTER_COEFF_2;

  // Pressure oversampling set at 4x
  conf.os_pres = BMP280_OS_4X;

  // Setting the output data rate as 1HZ(1000ms)
  conf.odr = BMP280_ODR_1000_MS;
  rslt = bmp280_set_config(&conf, &bmp);
  print_rslt(" bmp280_set_config status", rslt);

  // Always set the power mode after setting the configuration
  rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
  print_rslt(" bmp280_set_power_mode status", rslt);
}


int GetPressureAndTemperatureReading(double &pressure, double &temp) {
  struct bmp280_uncomp_data ucomp_data;

	// Reading the raw data from sensor
	int rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);
	if(rslt != 0) return rslt;

	rslt  = bmp280_get_comp_pres_double(&pressure, ucomp_data.uncomp_press, &bmp);
	if(rslt != 0) return rslt;

	rslt  = bmp280_get_comp_temp_double(&temp, ucomp_data.uncomp_temp, &bmp);
	if(rslt != 0) return rslt;

	return 0;
}



///*!
// *  @brief Function for writing the sensor's registers through SPI bus.
// *
// *  @param[in] cs           : Chip select to enable the sensor.
// *  @param[in] reg_addr     : Register address.
// *  @param[in] reg_data : Pointer to the data buffer whose data has to be written.
// *  @param[in] length       : No of bytes to write.
// *
// *  @return Status of execution
// *  @retval 0 -> Success
// *  @retval >0 -> Failure Info
// *
// */

int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
//	uint8_t data[8], i;
//
//	for(i=0; i<length; i++) {
//		data[i*2]   = reg_addr + i;
//		data[i*2+1] = reg_data[i];
//	}

	// select the chip
	HAL_GPIO_WritePin(GPIOB, cs, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi1, &reg_addr, 1, 10000);
	HAL_SPI_Transmit(&hspi1, reg_data, length, 10000);

	HAL_GPIO_WritePin(GPIOB, cs, GPIO_PIN_SET);
    return 0;
}

/*!
 *  @brief Function for reading the sensor's registers through SPI bus.
 *
 *  @param[in] cs       : Chip select to enable the sensor.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
	HAL_GPIO_WritePin(GPIOB, cs, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi1, &reg_addr, 1, 10000);
	HAL_SPI_Receive(&hspi1, reg_data, length, 10000);

	HAL_GPIO_WritePin(GPIOB, cs, GPIO_PIN_SET);

    return 0;
}


/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : name of the API whose execution status has to be printed.
 *  @param[in] rslt     : error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void print_rslt(const char api_name[], int8_t rslt)
{
	snprintf(buffer, sizeof(buffer), "%s\r\n", api_name);
	if (rslt != BMP280_OK)
	{
		if (rslt == BMP280_E_NULL_PTR)
		{
			snprintf(buffer, sizeof(buffer), "Error [%d] : Null pointer error\r\n", rslt);
		}
		else if (rslt == BMP280_E_COMM_FAIL)
		{
			snprintf(buffer, sizeof(buffer), "Error [%d] : Bus communication failed\r\n", rslt);
		}
		else if (rslt == BMP280_E_IMPLAUS_TEMP)
		{
			snprintf(buffer, sizeof(buffer), "Error [%d] : Invalid Temperature\r\n", rslt);
		}
		else if (rslt == BMP280_E_DEV_NOT_FOUND)
		{
			snprintf(buffer, sizeof(buffer), "Error [%d] : Device not found\r\n", rslt);
		}
		else
		{
			//For more error codes refer "*_defs.h"
			snprintf(buffer, sizeof(buffer), "Error [%d] : Unknown error code\r\n", rslt);
		}
	}
	Send_Uart(buffer);
}




void InitGPS() {
	gps = Adafruit_GPS::getInstance();

	HAL_Delay(500);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);

	gps->begin(&huart3);

	// turn on RMC (recommended minimum) and GGA (fix data) including altitude
	gps->sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
	// Set the update rate
	gps->sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
	// Request updates on antenna status, comment out to keep quiet
	gps->sendCommand(PGCMD_ANTENNA);

	HAL_Delay(500);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
	// set baud rate of GPS module to 115200
	gps->sendCommand(PMTK_SET_BAUD_115200);
	HAL_Delay(100);
	// set baud rate of uC to 115200
	HAL_UART_DeInit(&huart3);
	huart3.Init.BaudRate = 115200;
	if (HAL_UART_Init(&huart3) != HAL_OK)
		Error_Handler();

	gps->begin(&huart3);
}

bool GetGPSReading() {
	if (gps->newNMEAreceived()) {
		//send_uart(gps->lastNMEA());
		gps->parse(gps->lastNMEA());
		return true;
	}

	return false;
}

