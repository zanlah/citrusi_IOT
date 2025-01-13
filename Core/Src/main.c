/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVER_IP "192.168.137.1"  //spremeni to da kaze na sever
#define SERVER_PORT "3000"           //spremeni to na pravi port, ponavadi 80
#define CONNECTION_ID 0
#define DEVICE_ID 1				  //identifikator za napravo
#define API_ENDPOINT "/user_routes/add-gyro-measurement"
#define UART_RX_BUFFER_SIZE 2048
#define MAX_JSON_SIZE 2048
#define MEASUREMENTS_PER_BATCH 20
//#define BUFFER_SIZE 1024

//static char debugBuffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t debugLength = 0;
static volatile uint8_t newDataFlag = 0;
static uint8_t uartRxBuffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t uartRxIndex = 0;

static uint8_t tempByte;
static uint32_t lastATTime = 0;
int connectedToWifi = 0;

#define CTRL_REG1_ADDR   0x20  // Address for CTRL_REG1
#define CTRL_REG4_ADDR   0x23   // Address for CTRL_REG4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
UART_HandleTypeDef huart1;

uint32_t last_button_press = 0;
uint8_t gyro_ready = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// ---------------------------------------------------------------------------
//                          GYRO CODE
// ---------------------------------------------------------------------------

// Pavza, s katero omogocimo pravilno delovanje avtomatskega testa
void pavza(){
  uint32_t counter = 0;
  for(counter=0; counter<600; counter++){
    asm("nop");
  }
}

uint8_t spi1_beriRegister(uint8_t reg)
{
  uint16_t buf_out, buf_in;
  reg |= 0x80; // najpomembnejsi bit na 1
  buf_out = reg; // little endian, se postavi na pravo mesto ....
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
  pavza();
  //HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&buf_out, (uint8_t*)&buf_in, 2, 2); // blocking posiljanje ....
  HAL_SPI_TransmitReceive(&hspi1, &((uint8_t*)&buf_out)[0], &((uint8_t*)&buf_in)[0], 1, 2); // razbito na dva dela, da se podaljsa cas in omogoci pravilno delovanje testa
  pavza();
  HAL_SPI_TransmitReceive(&hspi1, &((uint8_t*)&buf_out)[1], &((uint8_t*)&buf_in)[1], 1, 2); // razbito na dva dela, da se podaljsa cas in omogoci pravilno delovanje testa
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
  pavza();
  return buf_in >> 8; // little endian...
}

void spi1_pisiRegister(uint8_t reg, uint8_t vrednost)
{
  uint16_t buf_out;
  buf_out = reg | (vrednost<<8); // little endian, se postavi na pravo mesto ....
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
  pavza();
  //HAL_SPI_Transmit(&hspi1, (uint8_t*)&buf_out, 2, 2); // blocking posiljanje ....
  HAL_SPI_Transmit(&hspi1, &((uint8_t*)&buf_out)[0], 1, 2); // razbito na dva dela, da se podaljsa cas in omogoci pravilno delovanje testa
  pavza();
  HAL_SPI_Transmit(&hspi1, &((uint8_t*)&buf_out)[1], 1, 2); // razbito na dva dela, da se podaljsa cas in omogoci pravilno delovanje testa
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
  pavza();
}

void spi1_beriRegistre(uint8_t reg, uint8_t* buffer, uint8_t velikost)
{
  reg |= 0xC0; // najpomembnejsa bita na 1
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
  pavza();
  HAL_SPI_Transmit(&hspi1, &reg, 1, 10); // blocking posiljanje....
  pavza();
  HAL_SPI_Receive(&hspi1,  buffer, velikost, velikost); // blocking posiljanje....
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
  pavza();
}

void initL3GD20()
{
  // preverimo ali smo "poklicali" pravi senzor
  uint8_t cip = spi1_beriRegister(0x0F);
  if (cip != 0xD4 && cip != 0xD3)
    for (;;);

  spi1_pisiRegister(0x20, 0x0F);// zbudi ziroskop in omogoci osi
  spi1_pisiRegister(0x22, 0x08);// zbudi ziroskop in omogoci osi
  spi1_pisiRegister(0x23, 0x10);// zbudi ziroskop in omogoci osi
}

void set_responsivness_190Hz() {
	/*
	 * DR (Data Rate) = 01 (190 Hz)
	 * BW = 00 (no low-pass filter)
	 * PD (Power Down) = 1 (Power On)
	 * Zen = 1, Yen = 1, Xen = 1 (Enable all axes)
	 * => 01001111 = 0x6f
	 */
     uint8_t ctrl_reg1_value = 0x6F; // DR=01 (190 Hz)

    // Write to CTRL_REG1
    HAL_SPI_Transmit(&hspi1, (uint8_t[]){CTRL_REG1_ADDR | SPI_SR_TXE, ctrl_reg1_value}, 2, HAL_MAX_DELAY);
}

void Set_Sensitivity_500DPS() {
	/*
	 * FS = 01
	 * BDU, BLE, SIM, ST, and bits [2:0] = 0
	 * => 01000000 = 0x10
	 */
    uint8_t ctrl_reg4_value = 0x10; // FS = 01 for Â±500 dps

    // Write to CTRL_REG4
    HAL_SPI_Transmit(&hspi1, (uint8_t[]){CTRL_REG4_ADDR | SPI_SR_TXE, ctrl_reg4_value}, 2, HAL_MAX_DELAY);

}

uint8_t isButtonPressed() {
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) { // Gumb pritisnjen
        if (HAL_GetTick() - last_button_press > 100) { // 100 ms debounce
            last_button_press = HAL_GetTick();
            return 1;
        }
    }
    return 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == GPIO_PIN_1) {
		gyro_ready = 1;
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
	}
}

// ----------------------------------------------------------------------------

// ---------------------------------------------------------------------------
//                          G Y R O S za malco?
// ---------------------------------------------------------------------------

void appendToJson(char* jsonStr, float value, int isLast) {
    char valueBuffer[32];
    int fullPart = (int)value;
    int decimalPart;

    //obdelam negativen vrednosti da no -1.-2
    if (value < 0) {
        float decimalTemp = (value - (float)fullPart) * -100.0f;
        decimalPart = (int)(decimalTemp + 0.5f);
    } else {
        float decimalTemp = (value - (float)fullPart) * 100.0f;
        decimalPart = (int)(decimalTemp + 0.5f);
    }

    //formatiram ,dve decimalki
    sprintf(valueBuffer, "%d.%02d%s", fullPart, decimalPart, isLast ? "" : ", ");
    strcat(jsonStr, valueBuffer);
}

void sendMeasurementBatch(float* measurements, int count) {
    char jsonData[MAX_JSON_SIZE];

    //ustvarim nov JSon
    sprintf(jsonData, "{\"deviceId\": \"%d\", \"measurements\": [", DEVICE_ID);

    //dodam vsako meritev posebej
    for(int i = 0; i < count; i++) {
        appendToJson(jsonData, measurements[i], i == count - 1);
    }

    //koncam json
    strcat(jsonData, "]}");

    //posljem podatke
    SendData(jsonData);
    SendDebug("Batch sent:");
    SendDebug(jsonData);
}

void SendData(const char* data)
{
    char cmd[128];

    //naredim povezavo
    sprintf(cmd, "AT+CIPSTART=%d,\"TCP\",\"%s\",%s", CONNECTION_ID, SERVER_IP, SERVER_PORT);
    SendCommand(cmd);
    HAL_Delay(1000);

    //pripravim post request
    char request[512];
       sprintf(request,
           "POST %s HTTP/1.1\r\n"
           "Host: %s\r\n"
           "Content-Type: application/json\r\n"
           "Connection: close\r\n"
           "Content-Length: %d\r\n"
           "\r\n"
           "%s",
           API_ENDPOINT, SERVER_IP, strlen(data), data);

    //posljem dolzino sporocila
    sprintf(cmd, "AT+CIPSEND=%d,%d", CONNECTION_ID, strlen(request));
    SendCommand(cmd);
    HAL_Delay(100);

    //posljem dejsnako sporocilo
    HAL_UART_Transmit(&huart1, (uint8_t*)request, strlen(request), 1000);
    HAL_Delay(1000);

    //zaprem povezavo
    sprintf(cmd, "AT+CIPCLOSE=%d", CONNECTION_ID);
    SendCommand(cmd);

    SendDebug("Data sent to server");
}



void SendSensorData(int fullPart, int decimalPart)
{
    char jsonData[128];
    //zapakiram v json
    sprintf(jsonData, "{\"deviceId\": \"%d\", \"gyro\": %d.%02d}",
              DEVICE_ID, fullPart, decimalPart);

    SendDebug("Sending sensor data:");
    SendDebug(jsonData);

    SendData(jsonData);
}

void SendCommand(const char* command)
{
	HAL_GPIO_WritePin(GPIOE, LD6_Pin, GPIO_PIN_SET);
    char cmd[128];
    sprintf(cmd, "%s\r\n", command);

    //posljem debug na pc
    SendDebug("TX -> ");
    SendDebug(cmd);

    //posljem komando na esp
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 1000);

	HAL_GPIO_WritePin(GPIOE, LD6_Pin, GPIO_PIN_RESET);
}


void SetupAPServer(void)
{
	HAL_GPIO_WritePin(GPIOE, LD10_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, LD8_Pin, GPIO_PIN_SET);
    //resetiraj
    SendCommand("AT+RST");
    HAL_Delay(2000);

    //AP
    SendCommand("AT+CWMODE=2");

    //nastavitve AP
    SendCommand("AT+CWSAP=\"ESP32_citrusi\",\"12345678\",1,3");

    //Vec povezav
    SendCommand("AT+CIPMUX=1");

    //zacnem TCP server
    SendCommand("AT+CIPSERVER=1,80");

    //dobim ip
    SendCommand("AT+CIFSR");
}


//posiljanje debug informacij na pc
void SendDebug(const char* message)
{
	  //posljem sporocilo
	    uint8_t result = CDC_Transmit_FS((uint8_t*)message, strlen(message));
	    HAL_Delay(10);

	    //posljem se konec vrstice
	    if (result == USBD_OK)
	    {
	        CDC_Transmit_FS((uint8_t*)"\r\n", 2);
	        HAL_Delay(10);
	    }
}

void ConnectToWiFi(char conn_id, const char* ssid, const char* password)
{
    char response[512];

    HAL_GPIO_WritePin(GPIOE, LD8_Pin, GPIO_PIN_RESET);

    //spremenim mode v AP in WIFI
    SendCommand("AT+CWMODE=3");
    HAL_Delay(2000);


    char wifi_cmd[128];
    sprintf(wifi_cmd, "AT+CWJAP=\"%s\",\"%s\"", ssid, password);
    SendCommand(wifi_cmd);


    HAL_Delay(2000);

   /* SendCommand("AT+CIFSR");
    memset(response, 0, sizeof(response));
    HAL_Delay(1000);
    */
    HAL_GPIO_WritePin(GPIOE, LD10_Pin, GPIO_PIN_SET);
    connectedToWifi = 1;
   /* HAL_UART_Receive(&huart1, (uint8_t*)response, sizeof(response), 1000);

    if(strstr(response, "STAIP,\"")) {

           HAL_GPIO_WritePin(GPIOE, LD9_Pin, GPIO_PIN_SET);
       } else {
           SendDebug("Failed to get IP!");
           HAL_GPIO_WritePin(GPIOE, LD9_Pin, GPIO_PIN_RESET);
       }*/
}


void SendWebPage(char conn_id)
{
	 const char* html = "HTTP/1.1 200 OK\r\n"
				 "Content-Type: text/html\r\n\r\n"
				                       "<html><head>"
				                       "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
				                       "<style>"
				                       "body {display: flex; justify-content: center; align-items: center; height: 100vh; margin: 0;}"
				                       ".container {text-align: center; width: 90%; max-width: 300px;}"
				                       "input {font-size: 18px; padding: 8px; margin: 10px; width: 100%; box-sizing: border-box;}"
				                       "h1 {font-size: 24px;}"
			 	 	 	 	 	          "</style></head>"
		                     "<body>"
		                     "<div class='container'>"
		                     "<h1>Citrusi Wifi setup</h1>"
			 	 	        "<form method='get' action='/connect'>"
		                     "<p><input type='text' name='ssid' placeholder='WiFi Name'></p>"
		                     "<p><input type='password' name='pass' placeholder='Password'></p>"
		                     "<input type='submit' value='Povezi'>"
		                     "</form>"
		                     "</div></body></html>";

		    char lenCmd[32];
		    sprintf(lenCmd, "AT+CIPSEND=%c,%d", conn_id, strlen(html));
		    SendCommand(lenCmd);
		    HAL_Delay(500);


		    HAL_UART_Transmit(&huart1, (uint8_t*)html, strlen(html), 1000);
		    HAL_Delay(500);


		    char closeCmd[32];
		    sprintf(closeCmd, "AT+CIPCLOSE=%c", conn_id);
		    SendCommand(closeCmd);
		    HAL_Delay(500);
}





void extractUrlParams(const char* url, char* ssid, char* password) {

	    char* ssid_start = strstr(url, "ssid=");
	    char* pass_start = strstr(url, "pass=");

	    if(ssid_start && pass_start)
	    {
	        ssid_start += 5;


	        char* ssid_end = strstr(ssid_start, "&");
	        if(!ssid_end) ssid_end = strstr(ssid_start, " ");

	        if(ssid_end) {
	            size_t ssid_len = ssid_end - ssid_start;
	            strncpy(ssid, ssid_start, ssid_len);
	            ssid[ssid_len] = 0;

	              }

	        pass_start += 5;


	        char* pass_end = strstr(pass_start, "&");
	        if(!pass_end) pass_end = strstr(pass_start, " ");

	        if(pass_end) {
	            size_t pass_len = pass_end - pass_start;
	            strncpy(password, pass_start, pass_len);
	            password[pass_len] = 0;
	        }
	   }
}


void ProcessUART(void)
{
    char* ipd = strstr((char*)uartRxBuffer, "+IPD,");
    char* get = strstr((char*)uartRxBuffer, "GET");

    if(ipd && get)
    {
        SendDebug("Dobil buffer:");
        SendDebug((char*)uartRxBuffer);


        char conn_id = ipd[5];
        HAL_GPIO_WritePin(GPIOE, LD4_Pin, GPIO_PIN_SET);

        if(strstr((char*)uartRxBuffer, "connect"))
        {
            char ssid[32] = {0};
            char password[32] = {0};

            //HAL_GPIO_WritePin(GPIOE, LD5_Pin, GPIO_PIN_SET);
            extractUrlParams((char*)uartRxBuffer, ssid, password);

            if(strlen(ssid) > 0 && strlen(password) > 0)
            {
                ConnectToWiFi(conn_id, ssid, password);
            }

        }
        else
        {
            //HAL_GPIO_WritePin(GPIOE, LD9_Pin, GPIO_PIN_SET);
            SendWebPage(conn_id);
        }

        HAL_GPIO_WritePin(GPIOE, LD4_Pin, GPIO_PIN_RESET);
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	  if(huart->Instance == USART1)
	    {
	        //shrani prvi byte
	        if(uartRxIndex < UART_RX_BUFFER_SIZE - 1)
	        {
	            uartRxBuffer[uartRxIndex++] = tempByte;

	            //nastavi zastavico za procesiranje ce dobimo konec vrstice
	            if(tempByte == '\n' || tempByte == '\r')
	            {
	                if(uartRxIndex > 1)
	                {
	                    newDataFlag = 1;
	                }
	            }
	        }
	        else
	        {
	            //resetiramo buffer
	            uartRxIndex = 0;
	        }

	        //resetiraj recepcijo prekinitve
	        HAL_UART_Receive_IT(&huart1, &tempByte, 1);
	    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */





  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  //pokazatelj delovanja, led vsako sekundo unteracije while zanke
  HAL_GPIO_WritePin(GPIOE, LD3_Pin, GPIO_PIN_RESET);


  HAL_GPIO_WritePin(GPIOE, LD8_Pin, GPIO_PIN_RESET);

  //pokaze kdaj se posilja komanda
  HAL_GPIO_WritePin(GPIOE, LD6_Pin, GPIO_PIN_RESET);

//led on ko dobim nove podatke
  HAL_GPIO_WritePin(GPIOE, LD9_Pin, GPIO_PIN_RESET);


  HAL_GPIO_WritePin(GPIOE, LD7_Pin, GPIO_PIN_RESET);


  HAL_GPIO_WritePin(GPIOE, LD10_Pin, GPIO_PIN_RESET);


  HAL_GPIO_WritePin(GPIOE, LD4_Pin, GPIO_PIN_RESET);


  HAL_UART_Receive_IT(&huart1, &tempByte, 1);
  SendDebug("zacenjam ESP32 kot AP...");

     // Setup the access point
     SetupAPServer();

     SendDebug("AP nastavljen, cakam na povezave...");
  lastATTime = HAL_GetTick();


    // -----------------------------------------------------------------
    //                  GYRO CODE
    // -----------------------------------------------------------------

    __HAL_SPI_ENABLE(&hspi1);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

    set_responsivness_190Hz();
    Set_Sensitivity_500DPS();
    initL3GD20();

    // ---------------------------------------------------------------------


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  if(newDataFlag)
	         {
		  	  	  	 HAL_GPIO_WritePin(GPIOE, LD9_Pin, GPIO_PIN_SET);
		  	  	  	// Null terminate the received data
		             uartRxBuffer[uartRxIndex] = 0;

		             SendDebug("Dobil buffer:");
		             SendDebug((char*)uartRxBuffer);

		             //procesiram buffer
		             ProcessUART();

		             //resetiram buffer in zastavico
		             uartRxIndex = 0;
		             newDataFlag = 0;
		             memset(uartRxBuffer, 0, UART_RX_BUFFER_SIZE);
		             HAL_GPIO_WritePin(GPIOE, LD9_Pin, GPIO_PIN_RESET);
	         }

		static uint32_t lastBlink = 0;
		static uint32_t lastGyroSend = 0;
		static float measurements[MEASUREMENTS_PER_BATCH];
		static int measurementIndex = 0;

	    if(connectedToWifi && (HAL_GetTick() - lastGyroSend >= 500)) {
	        lastGyroSend = HAL_GetTick();  // Update last send time

	        HAL_GPIO_TogglePin(GPIOE, LD6_Pin);

	        int16_t xyz[3] = {0};
	        spi1_beriRegistre(0x28, (uint8_t*)xyz, 6);

	        float sensitivity = 0.0175f;
	        float x_dps = xyz[0] * sensitivity;

	        /* int fullPart = (int)(x_dps);
	        int decimalPart;

	       if (x_dps < 0) {
	            float decimalTemp = (x_dps - (float)fullPart) * -100.0f;
	            decimalPart = (int)(decimalTemp + 0.5f);
	        } else {
	            float decimalTemp = (x_dps - (float)fullPart) * 100.0f;
	            decimalPart = (int)(decimalTemp + 0.5f);
	        }

	        SendSensorData(fullPart, decimalPart);
	        */
	        // Store measurement in buffer
	               measurements[measurementIndex] = x_dps;
	               measurementIndex++;

	               //ko je buffer pol posljem celo vsebino
	               if(measurementIndex >= MEASUREMENTS_PER_BATCH) {
	                   sendMeasurementBatch(measurements, measurementIndex);
	                   measurementIndex = 0;  //resetiram stevec
	               }
	    }


	  	  //indikator delovanje, led blink vsako sekundo
	        if(HAL_GetTick() - lastBlink >= 1000)
	        {
	            HAL_GPIO_TogglePin(GPIOE, LD3_Pin);
	            lastBlink = HAL_GetTick();

	        }
	         HAL_Delay(10);
	     }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD7_Pin
                           LD9_Pin LD10_Pin LD8_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD7_Pin
                          |LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
