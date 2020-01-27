/*
 * main.cpp
 *
 *  Created on: Jan 21, 2020
 *      Author: jonas
 */

#include "main.h"
#include "fatfs.h"
#include <stdio.h>
#include <string.h>
#include "bmp280.h"


extern ADC_HandleTypeDef hadc1;

extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi1;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern uint32_t millis;

FATFS fs;  // file system
FIL fil;  // file
FRESULT fresult;  // to store the result
#define BUF_LENGTH 512
char buffer[BUF_LENGTH]; // to store data
#define RADIO_BUF_LENGTH 1
uint8_t radioBuffer[RADIO_BUF_LENGTH];

UINT br, bw;   // file read/write count

/* capacity related variables */
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;


/* to send the data to the uart */
void send_uart (char *string)
{
	uint8_t len = strlen (string);
	HAL_UART_Transmit(&huart1, (uint8_t *) string, len, 2000);  // transmit in blocking mode
}

/* to find the size of data in the buffer */
int bufsize (char *buf)
{
	int i=0;
	while (*buf++ != '\0') i++;
	return i;
}

void bufclear (void)  // clear buffer
{
	for (int i=0; i<BUF_LENGTH; i++)
	{
		buffer[i] = '\0';
	}
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
				/* For more error codes refer "*_defs.h" */
			snprintf(buffer, sizeof(buffer), "Error [%d] : Unknown error code\r\n", rslt);
		}
	}
	send_uart(buffer);
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart == &huart1) {
		HAL_UART_Transmit(&huart2, radioBuffer, RADIO_BUF_LENGTH, 1000);
	}
	else if(huart == &huart2) {
		HAL_UART_Transmit(&huart1, radioBuffer, RADIO_BUF_LENGTH, 1000);
	}

	HAL_UART_Receive_IT(huart, radioBuffer, RADIO_BUF_LENGTH);
}


extern "C" int maincpp(void) {
	send_uart((char*) "Hello World\r\n");
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

	int accelZ=0, accelY=0, accelX=0;
	bool sdCardAvailable = false;



/*
	// Init barometer
  int8_t rslt;
  struct bmp280_dev bmp;
  struct bmp280_config conf;
  struct bmp280_uncomp_data ucomp_data;
  uint32_t pres32, pres64;
  double pres;

  // Set CS to high, this is the "not selected" state
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

  // Map the delay function pointer with the function responsible for implementing the delay
  bmp.delay_ms = HAL_Delay;

  bmp.dev_id = GPIO_PIN_4;
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
  print_rslt(" bmp280_set_power_mode status", rslt);*/






	/* Mount SD Card */
	fresult = f_mount(&fs, "", 0);
	if (fresult != FR_OK){
		send_uart ((char*)"error in mounting SD CARD...\r\n");
		sdCardAvailable = true;
	}
	else {
		send_uart((char*)"SD CARD mounted successfully...\r\n");
		sdCardAvailable = false;
	}

	if(sdCardAvailable) {
		/* Check free space */
		f_getfree("", &fre_clust, &pfs);

		total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
		sprintf (buffer, "SD CARD Total Size: \t%lu\n",total);
		send_uart(buffer);
		bufclear();
		free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
		sprintf (buffer, "SD CARD Free Space: \t%lu\n",free_space);
		send_uart(buffer);


		/* Open file to write/ create a file if it doesn't exist */
		fresult = f_open(&fil, "data.csv", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		f_puts("TIME,X,Y,Z\n", &fil);
	}


	HAL_UART_RegisterCallback(&huart1, HAL_UART_RX_COMPLETE_CB_ID, HAL_UART_RxCpltCallback);
	HAL_UART_RegisterCallback(&huart2, HAL_UART_RX_COMPLETE_CB_ID, HAL_UART_RxCpltCallback);

	HAL_UART_Receive_IT(&huart1, radioBuffer, RADIO_BUF_LENGTH);
	HAL_UART_Receive_IT(&huart2, radioBuffer, RADIO_BUF_LENGTH);


	uint8_t sync_counter = 0;

	/* Infinite loop */
	while (1)
	{
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		accelZ = HAL_ADC_GetValue(&hadc1) - 2048;

		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		accelY = HAL_ADC_GetValue(&hadc1) - 2048;

		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		accelX = HAL_ADC_GetValue(&hadc1) - 2048;


		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);


		snprintf(buffer, sizeof(buffer), "%lu,%6.3f,%6.3f,%6.3f\r\n", millis,((float)accelX) / 372.0 + 0.250, ((float)accelY) / 372.0 + 0.250, ((float)accelZ) / 372.0);

		if(sdCardAvailable) {
			f_puts(buffer, &fil);
			if(sync_counter == 20) {
				sync_counter = 0;
				f_sync(&fil);
			}
			sync_counter++;
		}

		HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);

/*
		// Reading the raw data from sensor
		rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);

		// Getting the compensated pressure using 32 bit precision
		rslt = bmp280_get_comp_pres_32bit(&pres32, ucomp_data.uncomp_press, &bmp);

		// Getting the compensated pressure using 64 bit precision
		rslt = bmp280_get_comp_pres_64bit(&pres64, ucomp_data.uncomp_press, &bmp);

		// Getting the compensated pressure as floating point value
		rslt = bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press, &bmp);
		snprintf(buffer, sizeof(buffer), "UP: %ld, P32: %ld, P64: %ld, P64N: %ld, P: %f\r\n",
		                 ucomp_data.uncomp_press,
		                 pres32,
		                 pres64,
		                 pres64 / 256,
		                 pres);

		HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);

*/

		HAL_Delay(200);

		//HAL_UART_Transmit(&huart2, (uint8_t*) "Hello World\r\n", 13, 2000);  // transmit in blocking mode
	}

	/* Close file */
	if(sdCardAvailable)
		f_close(&fil);
}


