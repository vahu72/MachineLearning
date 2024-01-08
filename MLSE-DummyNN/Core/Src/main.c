/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_x-cube-ai.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dummynn.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Choix de l'interface de sortie des données.
typedef enum {
  USB_Datalog = 0,
  SDCARD_Datalog
} LoggingInterface;

// Numérotation des threads.
typedef enum {
  THREAD_1 = 0,
  THREAD_2
} ThreadNumber;

// Structure de données pour l'état de l'accéléromètre.
typedef struct {
  uint32_t ms_counter;
  BSP_MOTION_SENSOR_Axes_t acc;
  BSP_MOTION_SENSOR_Axes_t gyro;
} SensorData;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Paramètres de fonctionnement de l'accéléromètre.
#define LSM6DSM_ACC_ODR 52.0f /* ODR = 52Hz */
#define LSM6DSM_ACC_FS 2 /* FS = 2g */
#define LSM6DSM_GYRO_ODR 52.0f /* ODR = 52Hz */
#define LSM6DSM_GYRO_FS 2000 /* FS = 2000dps */
#define DATA_PERIOD_MS     (20)

// Identifiant de message pour commande le démarrage ou d'arrêt de l'acquisition.
#define DATALOG_CMD_STARTSTOP (0x00000007)

// Taille de la file des messages pour la communication entre threads.
#define DATAQUEUE_SIZE ((uint32_t)100)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

RTC_HandleTypeDef hrtc;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* loggingInterface = USB_Datalog     --> Send sensors data via USB */
/* loggingInterface = SDCARD_Datalog  --> Save sensors data on SDCard (enable with double tap) */
static LoggingInterface loggingInterface = USB_Datalog;

static uint32_t exec;

// Indicateur d'interruption de l'accéléromètre.
static volatile uint8_t memsInterrupt = 0;

// Objets représentant les fichiers pour les écritures sur la carte SD.
static char sdPath[4];
static FIL sdFile;
static FATFS sdFatFs;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

// Fonctions de gestion du timer.
static void dataTimerCallback(void const *arg);
static void dataTimerStart(void);
static void dataTimerStop(void);

// Fonctions de gestion de l'accéléromètre LSM6DSM.
static int32_t LSM6DSM_init(void);
static int32_t LSM6DSM_doubleTap(void);
static int32_t LSM6DSM_getData(SensorData *mptr);

// Fonctions de gestion de la carte SD.
static void SD_init(void);
static void SD_deinit(void);
static uint8_t SD_enable(void);
static void SD_disable(void);
static uint8_t SD_write(char *s, uint32_t size);

// Threads.
static void getDataThread(void const *argument);
static void writeDataThread(void const *argument);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

osTimerId dataTimerId;
osTimerDef(dataTimer, dataTimerCallback);

// File de messages pour la communication entre threads.
osMessageQId dataQueueId;
osMessageQDef(dataQueue, DATAQUEUE_SIZE, int);

// Pool de mémoire pour les échanges des données des capteurs entre threads.
osPoolId sensorPoolId;
osPoolDef(sensorPool, DATAQUEUE_SIZE, SensorData);

// Sémaphore pour débloquer le thread d'acquisition sur interruption
// du timer ou de l'accéléromètre.
osSemaphoreId readDataSemId;
osSemaphoreDef(readDataSem);

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
  MX_RTC_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_CRC_Init();
  MX_X_CUBE_AI_Init();
  /* USER CODE BEGIN 2 */

  // En mode USB, éteindre la LED 1.
  if(loggingInterface == USB_Datalog) {
      BSP_LED_Init(LED1);
      BSP_LED_Off(LED1);
  }

  // Activer l'alimentation par USB.
  HAL_PWREx_EnableVddUSB();
  HAL_PWREx_EnableVddIO2();

  // En mode USB, configurer le port USB en interface série (Communication Device Class).
  // En mode SD, initialiser le lecteur de carte SD.
  if(loggingInterface == USB_Datalog) {
      USBD_Init(&USBD_Device, &VCP_Desc, 0);
      USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
      USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
      USBD_Start(&USBD_Device);
  }
  else {
      SD_init();
  }

  // Préparer et démarrer deux threads: un pour l'acquisition (prioritaire) et un pour l'écriture.
  osThreadDef(THREAD_1, getDataThread,   osPriorityAboveNormal, 0, configMINIMAL_STACK_SIZE*4);
  osThreadDef(THREAD_2, writeDataThread, osPriorityNormal,      0, configMINIMAL_STACK_SIZE*4);
  osThreadCreate(osThread(THREAD_1), NULL);
  osThreadCreate(osThread(THREAD_2), NULL);
  osKernelStart();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
    /* USER CODE END WHILE */

  MX_X_CUBE_AI_Process();
    /* USER CODE BEGIN 3 */
    }
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_MSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

// Thread d'acquisition de données.
static void getDataThread(void const *argument) {
  (void) argument;

  sensorPoolId = osPoolCreate(osPool(sensorPool));
  dataQueueId  = osMessageCreate(osMessageQ(dataQueue), NULL);

  // Ce sémaphore sera utilisé pour débloquer ce thread
  // à chaque interruption du timer ou de l'accéléromètre.
  readDataSemId = osSemaphoreCreate(osSemaphore(readDataSem), 1);
  osSemaphoreWait(readDataSemId, osWaitForever);

  // Initialiser les périphériques.
  LSM6DSM_init();

  // L'acquisition démarre immédiatement.
  dataTimerStart();

  for (;;) {
      // Attendre la prochaine interruption du timer.
      osSemaphoreWait(readDataSemId, osWaitForever);

	  // Allouer un bloc de mémoire.
	  SensorData *mptr = osPoolAlloc(sensorPoolId);
	  if(mptr == NULL) {
	      Error_Handler();
	  }
	  // Lire l'état de l'accéléromètre.
	  else if(LSM6DSM_getData(mptr) != BSP_ERROR_NONE){
	      Error_Handler();
	  }
	  // Transmettre les données au thread d'écriture.
	  else if(osMessagePut(dataQueueId, (uint32_t)mptr, osWaitForever) != osOK) {
	      Error_Handler();
	  }
  }
}

// Thread d'écriture des données.
static void writeDataThread(void const *argument) {
  (void) argument;

  MX_X_CUBE_AI_Process();
}

int acquire_and_process_data(void * data)
{
	osEvent evt = osMessageGet(dataQueueId, osWaitForever);
	if (evt.status != osEventMessage) {
		return -1;
	}

	SensorData *rptr = evt.value.p;
	int32_t acc_x = rptr->acc.x;

	float *fdata = (float*)data;
	for (int i = AI_DUMMYNN_IN_1_SIZE - 1; i > 0; i --) {
		fdata[i] = fdata[i - 1];
	}
	fdata[0] = (float)acc_x;

	// Libérer la mémoire utilisée pour les données du message.
	osPoolFree(sensorPoolId, rptr);

	return 0;
}

int post_process(void * data)
{
	float *fdata = (float*)data;
	float out = fdata[0];
	if (out > 0.5) {
	      BSP_LED_On(LED1);
	}
	else {
	      BSP_LED_Off(LED1);
	}

  return 0;
}

/* ------------------------------------------------------------------------- *
 * Fonctions de gestion du timer.
 * ------------------------------------------------------------------------- */

// Libérer le thread d'acquisition à chaque interruption du timer.
static void dataTimerCallback(void const *arg) {
  osSemaphoreRelease(readDataSemId);
}

// Démarrer un timer pour échantillonner l'état de l'accéléromètre.
static void dataTimerStart(void) {
  osStatus  status;

  exec = 1;
  dataTimerId = osTimerCreate(osTimer(dataTimer), osTimerPeriodic, &exec);
  if (dataTimerId)  {
      status = osTimerStart(dataTimerId, DATA_PERIOD_MS);
      if (status != osOK)  {
	  // Timer could not be started
      }
  }
}

// Arrêter le timer.
static void dataTimerStop(void) {
  osTimerStop(dataTimerId);
}

/* ------------------------------------------------------------------------- *
 * Fonctions de gestion de l'accéléromètre.
 * ------------------------------------------------------------------------- */

// Configurer l'accéléromètre LSM6DSM.
static int32_t LSM6DSM_init(void) {
  // Utiliser l'accéléromètre et le gyroscope.
  // Régler l'échelle des valeurs et la fréquence d'échantillonnage.
  BSP_MOTION_SENSOR_Init(LSM6DSM_0, MOTION_ACCELERO | MOTION_GYRO);
  BSP_MOTION_SENSOR_SetOutputDataRate(LSM6DSM_0, MOTION_ACCELERO, LSM6DSM_ACC_ODR);
  BSP_MOTION_SENSOR_SetFullScale(LSM6DSM_0, MOTION_ACCELERO, LSM6DSM_ACC_FS);
  BSP_MOTION_SENSOR_SetOutputDataRate(LSM6DSM_0, MOTION_GYRO, LSM6DSM_GYRO_ODR);
  BSP_MOTION_SENSOR_SetFullScale(LSM6DSM_0, MOTION_GYRO, LSM6DSM_GYRO_FS);

  BSP_MOTION_SENSOR_Enable_Double_Tap_Detection(LSM6DSM_0, BSP_MOTION_SENSOR_INT2_PIN);

  /* At the moment this feature is only implemented for LSM6DSM */
  GPIO_InitTypeDef GPIO_InitStructureInt2;

  /* Enable INT2 GPIO clock */
  BSP_LSM6DSM_INT2_GPIO_CLK_ENABLE();

  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructureInt2.Pin = BSP_LSM6DSM_INT2;
  GPIO_InitStructureInt2.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructureInt2.Speed = GPIO_SPEED_FAST;
  GPIO_InitStructureInt2.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(BSP_LSM6DSM_INT2_GPIO_PORT, &GPIO_InitStructureInt2);

  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(BSP_LSM6DSM_INT2_EXTI_IRQn, 0x08, 0x00);
  HAL_NVIC_EnableIRQ(BSP_LSM6DSM_INT2_EXTI_IRQn);

  return BSP_ERROR_NONE;
}

// Vérifier si une interruption double-tap s'est produite.
static int32_t LSM6DSM_doubleTap(void) {
  BSP_MOTION_SENSOR_Event_Status_t status;
  BSP_MOTION_SENSOR_Get_Event_Status(LSM6DSM_0, &status);
  return status.DoubleTapStatus == 1;
}

// Lire l'état de l'accéléromètre.
static int32_t LSM6DSM_getData(SensorData *mptr) {
  int32_t ret = BSP_ERROR_NONE;
  mptr->ms_counter = HAL_GetTick();

  if (BSP_MOTION_SENSOR_GetAxes(LSM6DSM_0, MOTION_ACCELERO, &mptr->acc) == BSP_ERROR_COMPONENT_FAILURE) {
      mptr->acc.x = 0;
      mptr->acc.y = 0;
      mptr->acc.z = 0;
      ret = BSP_ERROR_COMPONENT_FAILURE;
  }

  if (BSP_MOTION_SENSOR_GetAxes(LSM6DSM_0, MOTION_GYRO, &mptr->gyro) == BSP_ERROR_COMPONENT_FAILURE) {
      mptr->gyro.x = 0;
      mptr->gyro.y = 0;
      mptr->gyro.z = 0;
      ret = BSP_ERROR_COMPONENT_FAILURE;
  }

  return ret;
}


// Détection d'interruption sur l'entrée d'interruption externe.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  memsInterrupt = 1;
  osSemaphoreRelease(readDataSemId);
}

/* ------------------------------------------------------------------------- *
 * Fonctions de gestion de la carte SD.
 * ------------------------------------------------------------------------- */

static void SD_init(void) {
  if(FATFS_LinkDriver(&SD_Driver, sdPath) == 0) {
      /* Register the file system object to the FatFs module */
      if(f_mount(&sdFatFs, (TCHAR const*)sdPath, 0) != FR_OK) {
	  /* FatFs Initialization Error */
	  while(1) {
	      BSP_LED_On(LED1);
	      HAL_Delay(500);
	      BSP_LED_Off(LED1);
	      HAL_Delay(100);
	  }
      }
  }
}

static uint8_t SD_enable(void) {
  static uint16_t sdcard_file_counter = 0;
  char header[] = "T [ms],AccX [mg],AccY [mg],AccZ [mg],GyroX [mdps],GyroY [mdps],GyroZ [mdps]\r\n";
  uint32_t byteswritten; /* written byte count */
  char file_name[30] = {0};

  /* SD SPI CS Config */
  SD_IO_CS_Init();

  sprintf(file_name, "%s%.3d%s", "SensorTile_Log_N", sdcard_file_counter, ".csv");
  sdcard_file_counter++;

  HAL_Delay(100);

  if(f_open(&sdFile, (char const*)file_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
      sdcard_file_counter--;
      return 0;
  }

  if(f_write(&sdFile, (const void*)&header, sizeof(header)-1, (void *)&byteswritten) != FR_OK) {
      return 0;
  }

  return 1;
}

static uint8_t SD_write(char *s, uint32_t size) {
  uint32_t byteswritten;
  if(f_write(&sdFile, s, size, (void*)&byteswritten) != FR_OK) {
      return 0;
  }
  return 1;
}

static void SD_disable(void) {
  f_close(&sdFile);
  SD_IO_CS_DeInit();
}

static void SD_deinit(void) {
  FATFS_UnLinkDriver(sdPath);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
