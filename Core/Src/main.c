/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "rc522.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t okunan_ham[MFRC522_MAX_LEN] ;
uint16_t kart_no[5] ;
uint16_t kayitli_id1[5] ={115,237,74,167,115} ; //ALİ
uint16_t kayitli_id2[5] ={108,101,174,219,124} ; //SALİH
uint16_t kayitli_id3[5] ={131,4,24,173,50} ;   //SAKİNE
uint16_t kayitli_id4[5] ={83,117,183,29,140} ;  //VELİ

uint8_t text1[16];
uint8_t durum ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  extern SPI_HandleTypeDef hspi1;

  // RC522
  extern uint8_t MFRC522_Check(uint8_t* id);
  extern uint8_t MFRC522_Compare(uint8_t* CardID, uint8_t* CompareID);
  extern void MFRC522_WriteRegister(uint8_t addr, uint8_t val);
  extern uint8_t MFRC522_ReadRegister(uint8_t addr);
  extern void MFRC522_SetBitMask(uint8_t reg, uint8_t mask);
  extern void MFRC522_ClearBitMask(uint8_t reg, uint8_t mask);
  extern uint8_t MFRC522_Request(uint8_t reqMode, uint8_t* TagType);
  extern uint8_t MFRC522_ToCard(uint8_t command, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint16_t* backLen);
  extern uint8_t MFRC522_Anticoll(uint8_t* serNum);
  extern void MFRC522_CalulateCRC(uint8_t* pIndata, uint8_t len, uint8_t* pOutData);
  extern uint8_t MFRC522_SelectTag(uint8_t* serNum);
  extern uint8_t MFRC522_Auth(uint8_t authMode, uint8_t BlockAddr, uint8_t* Sectorkey, uint8_t* serNum);
  extern uint8_t MFRC522_Read(uint8_t blockAddr, uint8_t* recvData);
  extern uint8_t MFRC522_Write(uint8_t blockAddr, uint8_t* writeData);
  extern void MFRC522_Init(void);
  extern void MFRC522_Reset(void);
  extern void MFRC522_AntennaOn(void);
  extern void MFRC522_AntennaOff(void);
  extern void MFRC522_Halt(void);


  void Servo_Angle(int angle) //SERVO KÜTÜPHANESİ
  {
  	if(angle<0)
  		angle=0;
  	if(angle>180)
  		angle=180;
  	angle=angle+45;

  	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,angle);

  }

  void ana_erkan()// BU KART OKUTULMADIKÇA EKRANDA YAZACAK FONKSİYON WHİLE DÖNGÜSÜNÜN İÇİNDE KULLANDIK.
  {
	  lcd_clear();
	  lcd_print(1,1,"   TEKNOLOJI    ");
	  lcd_print(2,1,"   FAKULTESI    ");
  }

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  MFRC522_Init();

   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  lcd_init(_LCD_4BIT, _LCD_FONT_5x8, _LCD_2LINE);

        ana_erkan();
  	 	HAL_Delay(3000);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */




	  if( !MFRC522_Request(PICC_REQIDL,okunan_ham) ) // REQUEST FONKSİYONU RFİD YE KART VAR MI YOK MU ONU SORAR DEVAMLI.
	  	 {

	  	      if(! MFRC522_Anticoll(okunan_ham)) // HERHANGİ BİR KART ALGILANDI�?INDA İKİNCİ Bİ KARTI OKUMAZ.
	  	      {
	  	        for(int a = 0 ; a<5; a++) //OKUNAN HAM VERİYİ KART NO'YA AKTARIYOR.
	  	        {
	  	          kart_no[a] = okunan_ham[a];

	  	        }
	  	                         sprintf(text1,"%d",kart_no) ; //KART NO DAKİ SAYILARI LCD YE YAZMAK İÇİN KULLANILAN FONKSİYON.
	  	       	         		 lcd_clear();
	  	       	         		 lcd_print(1,1,"KART OKUNDU");
	  	       	         		 lcd_print(2,1,text1) ;
	  	       	         		 HAL_GPIO_WritePin(buzer_GPIO_Port, buzer_Pin,GPIO_PIN_SET);
	  	       	         		 HAL_Delay(250);
	  	       	         	     HAL_GPIO_WritePin(buzer_GPIO_Port, buzer_Pin,GPIO_PIN_RESET);
	  	       	         		 HAL_Delay(2000) ;



	  	 if( (MFRC522_Compare(kart_no, kayitli_id1) == MI_OK )) //COMPARE FONKSİYONU KAR�?ILA�?TIMRAYI SA�?LAR DO�?RUYSA İF E GİRER.

	  	 {
	  		lcd_clear();
	  		lcd_print(1,1,"    MERHABA");
	  		lcd_print(2,1,"      ALi");
	  		HAL_Delay(1500);
	  		 durum = 1 ;

	  	 }
	  	 else if((MFRC522_Compare(kart_no, kayitli_id2) == MI_OK )) //COMPARE FONKSİYONU KAR�?ILA�?TIMRAYI SA�?LAR DO�?RUYSA İF E GİRER.
	  	 {
	  		lcd_clear();
	  		lcd_print(1,1,"    MERHABA");
	  	    lcd_print(2,1,"     SALiH");
	  	    HAL_Delay(1500);
	  	    durum = 1 ;
	  	 }
	  	 else if((MFRC522_Compare(kart_no, kayitli_id3) == MI_OK )) //COMPARE FONKSİYONU KAR�?ILA�?TIMRAYI SA�?LAR DO�?RUYSA İF E GİRER.
	  		  	 {
	  		  		lcd_clear();
	  		  		lcd_print(1,1,"    MERHABA");
	  		  		lcd_print(2,1,"     SAKiNE");
	  		  	    HAL_Delay(1500);
	  		  	    durum = 1 ;
	  		  	 }
	  	 else if((MFRC522_Compare(kart_no, kayitli_id4) == MI_OK )) //COMPARE FONKSİYONU KAR�?ILA�?TIMRAYI SA�?LAR DO�?RUYSA İF E GİRER.
	  		  	 {
	  		  		lcd_clear();
	  		  		lcd_print(1,1,"    MERHABA");
	  		  	    lcd_print(2,1,"      VELi");
	  		  	    HAL_Delay(1500);
	  		  	    durum = 1 ;
	  		  	 }


	  	 else
	  	 {
	  		 durum = 0 ;

	  		 lcd_clear();
	  	     lcd_print(1,1,"    KAYITSIZ");
	  	     lcd_print(2,1,"      KiSi");
	  	     for(int d=0; d<6; d++) //BUZZER BURADA 3 PULSE YANİ 3 KEZ ÖTER BU DA DÖNGÜDE 6 KEZ TOGGLE A NEDEN OLUR O YÜZDEN 6 YA KADAR SAYILIR.
	  	     {
	  	    	 HAL_GPIO_TogglePin(buzer_GPIO_Port, buzer_Pin);
	  	    	 HAL_Delay(500);
	  	     }
	  	     HAL_GPIO_WritePin(buzer_GPIO_Port, buzer_Pin, GPIO_PIN_RESET);

	  		 for(int b=0 ; b<5 ; b++) // BURADA KAYITSIZ Kİ�?İYKEN KART NO DİZİSİNİN İÇİ SIFIRLANIYOR./KAYITLI Kİ�?İLERİNKİ DURUM 1 İN İÇİNDE SIFIRLANIYOR.
	  		 {
	  			kart_no[b]= 0 ;
	  		 }
	  		 for(int c=0 ; c<16 ; c++) // BURADA KAYITSIZ Kİ�?İYKEN OKUNAN HAM DİZİSİNİN İÇİ SIFIRLANIYOR.
	  		 {
	  		 okunan_ham[c]= 0 ;
	  		 }
              ana_erkan();
	  	 }

	  	      }
	  	 }

	  	  ////////////////////////////////////////////////////////////////////////////////

	  	 if(durum==1)
	  	 {

	  	lcd_print(1,1,"KAPI ACILIYOR:)");
	  	lcd_print(2,1,"  HOSGELDINIZ");
	  	//int i=0;
	  	  for(int i=0;i<=180;i++) // SERVOYU 0 DAN 180 E 10 MS ARALIKLARLA AZAR AZAR DÖNDÜRÜYO.
	  	  	  {
	  	  	    Servo_Angle(i);
	  	  		HAL_Delay(10);
	  	  	  }
	  	  HAL_Delay(5000);
	  	  lcd_clear();
	  	  lcd_print(1,1,"KAPI KAPANIYOR:)");
	  	  lcd_print(2,1,"   GULE GULE");
	  	  for(int i=180;i>=0;i--) // SERVOYU 180 DAN 0 A 10 MS ARALIKLARLA AZAR AZAR DÖNDÜRÜYO.
	  	  	  {
	  	  	    Servo_Angle(i);
	  	  	  	HAL_Delay(10);
	  	  	  }
	  	 // HAL_Delay(2000);
	  	  for(int c=0 ; c<16 ; c++) //BURADA KAYITLI Kİ�?İYKEN KART NO DİZİSİNİN İÇİ SIFIRLANIYOR.
	  	 		 {
	  	 			 okunan_ham[c]= 0 ;
	  	 		 }
	  	for(int e=0 ; e<5 ; e++) //BURADA KAYITLI Kİ�?İYKEN OKUNAN HAM DİZİSİNİN İÇİ SIFIRLANIYOR.
	  		  		 {
	  		  			kart_no[e]= 0 ;
	  		  		 }
	  	   durum = 0 ;
	  	   ana_erkan();
	  	 }

	  	  MFRC522_Halt(); // EN SON OKUNAN KARTTAN SONRA RFİD Yİ TEMİZLEME FONKSİYONU.

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
