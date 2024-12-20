/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
// #include "string.h"
#include "stdio.h"
#include "FEE.h"
#include <stm32f103xb.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// #define TIPE_EMULASI 5
#define NUM_LEDS 4
#define NUM_INTERRUPT 3
#define NUM_BUTTONS 8
#define DEBOUNCE_DELAY 50 // Debounce delay dalam ms
#define EMULASI_QUESTION_MARK 1 // 1 output verbose menggunakan ? 0 dengan verbose delay

// definisi address dari variabel yang dapat diubah oleh $
#define SET_WAKTU_TIMER_OLI_ADDRESS ((uint32_t)0x0800F040)     //$1
#define SET_WAKTU_POMPA_OLI_ON_ADDRESS ((uint32_t)0x0800F050)  //$2
#define TIPE_EMULASI_ADDRESS ((uint32_t)0x0800F000)       //$3
#define AUTO_COUNTER_DELAY_ADDRESS ((uint32_t)0x0800F010) //$4
#define VERBOSE_DELAY_ADDRESS ((uint32_t)0x0800F020)      //$5
#define ARAH_AUTO_COUNTER_ADDRESS ((uint32_t)0x0800F030)  //$6 0 aritnya bertambah 1 artinya berkurang

// #define verbose_delay 200 // Debounce delay dalam ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */
/* ----------------------- variabel yang dapat diubah $ --------------------------- */
uint32_t set_waktu_timer_oli; //$1 waktu dalam menit, default 15 menit
uint32_t set_waktu_pompa_oli_on; //$2 waktu dalam detik, default 5 detik
// 0 = emulasi button / switch
// 1 = emulasi latch
// 2 = emulasi counter auto
// 3 = emulasi counter button
// 4 = emulasi input interrupt pulse xyz
// 5 = emulasi board oli mk2
// 6 = hanya echo perintah dari pc
uint32_t tipe_emulasi; //$3 tergantung nilai yang dibaca di eeprom
uint32_t auto_counter_delay; //$4 default 1000 (1 detik)
uint32_t verbose_delay; //$5 default 0
uint32_t arah_auto_counter; //$6 0 aritnya bertambah 1 artinya berkurang

uint8_t nilai_emulasi_counter = 0;
uint32_t waktu_timer_oli = 0;
uint8_t keadaan_pompa_oli = 0;
uint8_t reset_pompa_oli = 0;
uint8_t tampilkan_verbose = 0;
uint8_t baca_dan_tampilkan_flash = 0;
// nilai counter untuk axis x y dan z
volatile int_fast32_t nilai_counter_axis[3] = { 0, 0, 0 };
uint8_t latch_states = 0;
uint8_t button_states = 0;
// list interrupt pin
GPIO_TypeDef *interrupt_ports[NUM_INTERRUPT] = {
INT_X_GPIO_Port, INT_Y_GPIO_Port,
INT_Z_GPIO_Port };
const uint16_t interrupt_pins[NUM_INTERRUPT] = {
INT_X_Pin, INT_Y_Pin, INT_Z_Pin };
// list led gpio
GPIO_TypeDef *led_ports[NUM_LEDS] = {
LED_1_GPIO_Port, LED_2_GPIO_Port,
LED_3_GPIO_Port, LED_4_GPIO_Port };
const uint16_t led_pins[NUM_LEDS] = {
LED_1_Pin, LED_2_Pin, LED_3_Pin, LED_4_Pin };
// list button gpio
GPIO_TypeDef *button_ports[NUM_BUTTONS] = {
BTN_1_GPIO_Port, BTN_2_GPIO_Port, BTN_3_GPIO_Port,
BTN_4_GPIO_Port, BTN_5_GPIO_Port, BTN_6_GPIO_Port,
BTN_7_GPIO_Port, BTN_8_GPIO_Port };
const uint16_t button_pins[NUM_BUTTONS] = {
BTN_1_Pin,
BTN_2_Pin,
BTN_3_Pin,
BTN_4_Pin,
BTN_5_Pin,
BTN_6_Pin,
BTN_7_Pin,
BTN_8_Pin, };
uint32_t last_button_press_time[NUM_BUTTONS] = { 0 }; // Variabel untuk debounce

uint32_t last_verbose_time = 0;
uint32_t last_auto_counter_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
void USB_CDC_RxHandler(uint8_t*, uint32_t);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// baca nilai pada flash memory
void baca_nilai_pada_flash(void) {
 FEE_ReadData(SET_WAKTU_TIMER_OLI_ADDRESS, &set_waktu_timer_oli, sizeof(uint32_t));
 FEE_ReadData(SET_WAKTU_POMPA_OLI_ON_ADDRESS, &set_waktu_pompa_oli_on, sizeof(uint32_t));
 FEE_ReadData(TIPE_EMULASI_ADDRESS, &tipe_emulasi, sizeof(uint32_t));
 FEE_ReadData(AUTO_COUNTER_DELAY_ADDRESS, &auto_counter_delay, sizeof(uint32_t));
 FEE_ReadData(VERBOSE_DELAY_ADDRESS, &verbose_delay, sizeof(uint32_t));
 FEE_ReadData(ARAH_AUTO_COUNTER_ADDRESS, &arah_auto_counter, sizeof(uint32_t));
}
// program untuk membaca data dari pc
int _write(int file, char *ptr, int len) {
 CDC_Transmit_FS((uint8_t*) ptr, len);
 return len;
}

void update_leds(uint8_t counter) {
 for (uint8_t i = 0; i < NUM_LEDS; i++) {
  (counter & (1 << i)) ?
   HAL_GPIO_WritePin(led_ports[i], led_pins[i], GPIO_PIN_SET) :
   HAL_GPIO_WritePin(led_ports[i], led_pins[i], GPIO_PIN_RESET); // Nyalakan LED
 }
}

void read_buttons_with_debounce(void) {
 for (uint8_t i = 0; i < 8; i++) {
  uint8_t current_state = HAL_GPIO_ReadPin(button_ports[i], button_pins[i]);
  if (!(current_state) == !(button_states & (1 << i))) {
   if (HAL_GetTick() - last_button_press_time[i] >= DEBOUNCE_DELAY) {
    last_button_press_time[i] = HAL_GetTick();
    /* Perbarui status tombol */
    // aktif low, jika tombol ditekan
    if (!current_state) {
     button_states |= (1 << i); // Set bit tombol
     // latch_states[i] = !latch_states[i];
     latch_states ^= (1 << i); // Balikkan bit pada latch_state
    } else button_states &= ~(1 << i); // Clear bit tombol
    // tambah fitur reset pada program emulasi counter
    switch (i) {
    case 0:
     nilai_emulasi_counter++;
     break;
    case 1:
     nilai_emulasi_counter--;
     break;
    case 2:
     nilai_emulasi_counter = 0;
     break;
    case 3:
     for (int i = 0; i < 3; ++i) {
      nilai_counter_axis[i] = 0;
     }
     break;
    default:
     break;
    }
   }
  }
 }
}

uint8_t update_waktu_timer_oli(uint32_t *waktu_timer_oli, uint32_t *set_waktu_timer_oli,
 uint32_t *set_waktu_pompa_oli_on, uint8_t *reset_pompa) //keadaan spindle atau auto mode
{
 uint8_t keadaan_led = 1;
 if (*reset_pompa == 1) {
  *reset_pompa = 0;
  *waktu_timer_oli = HAL_GetTick() - *set_waktu_timer_oli;
 } else if (HAL_GetTick() - *waktu_timer_oli >= *set_waktu_timer_oli) //nyalakan ketika pertama kali dihidupkan
  {
  keadaan_led = 1;
  if (HAL_GetTick() - (*waktu_timer_oli + *set_waktu_pompa_oli_on)
   >= (*set_waktu_timer_oli + *set_waktu_pompa_oli_on)) //reset waktu timer oli
   {
   *waktu_timer_oli = HAL_GetTick(); //set waktu timer oli jika led sudah waktunya mati
   keadaan_led = 0;
   //	    HAL_GPIO_WritePin(led_ports[0], led_ports[0], GPIO_PIN_RESET);
  }
  //	    HAL_GPIO_WritePin(led_ports[0], led_ports[0], GPIO_PIN_SET); // nyalakan led
 } else keadaan_led = 0;
 return keadaan_led;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
 MX_USB_DEVICE_Init();
 MX_RTC_Init();
 MX_FATFS_Init();
 /* USER CODE BEGIN 2 */
 baca_nilai_pada_flash();
 /* USER CODE END 2 */

 /* Infinite loop */
 /* USER CODE BEGIN WHILE */
 while (1) {
//  baca dan tampilkan data yang ada pada flash memory
  if (baca_dan_tampilkan_flash == 1) {
   baca_nilai_pada_flash();
   printf("$1 = %ld\r\n", set_waktu_timer_oli);
   printf("$2 = %ld\r\n", set_waktu_pompa_oli_on);
   printf("$3 = %ld\r\n", tipe_emulasi);
   printf("$4 = %ld\r\n", auto_counter_delay);
   printf("$5 = %ld\r\n", verbose_delay);
   printf("$6 = %ld\r\n", arah_auto_counter);
   baca_dan_tampilkan_flash = 0;
  }

  read_buttons_with_debounce(); // baca nilai tombol

  /* ----------------------- program untuk melakukan komputasi / simulasi tombol --------------------------- */
  // tipe button / simulasi
  switch (tipe_emulasi) {
  case 0: // tipe switch
   update_leds(button_states);
//	    for (uint8_t i = 0; i < NUM_LEDS; i++)
//		(button_states & (1 << i)) ?
//			HAL_GPIO_WritePin(led_ports[i], led_pins[i],
//				GPIO_PIN_SET) :
//			HAL_GPIO_WritePin(led_ports[i], led_pins[i],
//				GPIO_PIN_RESET);
   break;
  case 1: // tipe latch
   update_leds(latch_states);
   break;
  case 2: // tipe auto counter
   if (HAL_GetTick() - last_auto_counter_time >= auto_counter_delay) {
    last_auto_counter_time = HAL_GetTick();
    arah_auto_counter == 0 ?
     update_leds(nilai_emulasi_counter++) : update_leds(nilai_emulasi_counter--);
   }
   break;
  case 3: // tipe button counter
   update_leds(nilai_emulasi_counter);
   break;
  case 4: // tipe emulasi counter xyz. do nothing karena sudah dihandle interrupt
   break;
  case 5: // tipe simulasi board oli mk2b
   if (!(latch_states & (1 << 6))) { //oli tidak habis
    if (reset_pompa_oli == 1) { // dalam keadaan reset pompa oli
     if (update_waktu_timer_oli(&waktu_timer_oli, &set_waktu_timer_oli, &set_waktu_pompa_oli_on,
      &reset_pompa_oli) == 1) keadaan_pompa_oli = 1;
     else keadaan_pompa_oli = 0;
    } else if ((latch_states & (1 << 7))) { //spindle berjalan, auto mode
     if (update_waktu_timer_oli(&waktu_timer_oli, &set_waktu_timer_oli, &set_waktu_pompa_oli_on, 0)
      == 1) keadaan_pompa_oli = 1;
     else keadaan_pompa_oli = 0;
    } else keadaan_pompa_oli = 0;
   } else keadaan_pompa_oli = 0;

   // nyalakan led berdasarkan keadaan tombol
   if (latch_states & (1 << 7)) //m3 aktif
   HAL_GPIO_WritePin(led_ports[3], led_pins[3], GPIO_PIN_SET);
   else HAL_GPIO_WritePin(led_ports[3], led_pins[3], GPIO_PIN_RESET);
   if (latch_states & (1 << 6)) // oli habis
   HAL_GPIO_WritePin(led_ports[2], led_pins[2], GPIO_PIN_SET);
   else HAL_GPIO_WritePin(led_ports[2], led_pins[2], GPIO_PIN_RESET);

   keadaan_pompa_oli == 1 ?
    HAL_GPIO_WritePin(led_ports[0], led_pins[0], GPIO_PIN_SET) :
    HAL_GPIO_WritePin(led_ports[0], led_pins[0], GPIO_PIN_RESET);
   break;

  default:
   break;
  }

  /* ----------------------- program menampilkan karakter dan verbose --------------------------- */
  // tampilkan verbose button
#if EMULASI_QUESTION_MARK == 0
  if (HAL_GetTick() - last_verbose_time >= verbose_delay) {
   last_verbose_time = HAL_GetTick();
#else
  if (tampilkan_verbose == 1) {
   tampilkan_verbose = 0;
#endif
   switch (tipe_emulasi) {
   case 0:
   case 1:
   case 2:
   case 3:
    printf("%d,%d,%d\r\n", button_states, latch_states, nilai_emulasi_counter);
    break;
   case 4:
    printf("%ld,", HAL_GetTick()); // dapatkan nilai waktu untuk mendapatkan nilai akselerasi
    for (uint8_t i = 0; i < NUM_INTERRUPT; ++i) {
     //		i == 0 ? printf("X,") : i == 1 ? printf("Y,") : printf("Z,");
     i != NUM_INTERRUPT - 1 ?
      printf("%d,", nilai_counter_axis[i]) : printf("%d", nilai_counter_axis[i]);
    }
    printf("\r\n"); // enter untuk merapikan verbose
    break;
   case 5:
    printf("O:");
    (latch_states & (1 << 7)) ? printf("M") : 0; // m3 aktif
    (latch_states & (1 << 6)) ? printf("E") : 0; // oli habis
    (keadaan_pompa_oli == 1) ? printf("H") : 0;
    printf("\r\n"); //enter untuk merapikan
    break;
   default:
    printf("perintah tidak valid \r\n"); // enter untuk merapikan verbose
    break;
   }
  }
 }
 /* USER CODE END WHILE */

 /* USER CODE BEGIN 3 */

 /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
 RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
 RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
 RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

 /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
 RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
 RCC_OscInitStruct.HSEState = RCC_HSE_ON;
 RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
 RCC_OscInitStruct.HSIState = RCC_HSI_ON;
 RCC_OscInitStruct.LSIState = RCC_LSI_ON;
 RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
 RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
 RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
 if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
  Error_Handler();
 }

 /** Initializes the CPU, AHB and APB buses clocks
  */
 RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
  | RCC_CLOCKTYPE_PCLK2;
 RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
 RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
 RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
 RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

 if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
  Error_Handler();
 }
 PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_USB;
 PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
 PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
 if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
  Error_Handler();
 }
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

 /* USER CODE BEGIN RTC_Init 0 */

 /* USER CODE END RTC_Init 0 */

 RTC_TimeTypeDef sTime = { 0 };
 RTC_DateTypeDef DateToUpdate = { 0 };

 /* USER CODE BEGIN RTC_Init 1 */

 /* USER CODE END RTC_Init 1 */

 /** Initialize RTC Only
  */
 hrtc.Instance = RTC;
 hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
 hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
 if (HAL_RTC_Init(&hrtc) != HAL_OK) {
  Error_Handler();
 }

 /* USER CODE BEGIN Check_RTC_BKUP */

 /* USER CODE END Check_RTC_BKUP */

 /** Initialize RTC and set the Time and Date
  */
 sTime.Hours = 0x0;
 sTime.Minutes = 0x0;
 sTime.Seconds = 0x0;

 if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {
  Error_Handler();
 }
 DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
 DateToUpdate.Month = RTC_MONTH_JANUARY;
 DateToUpdate.Date = 0x1;
 DateToUpdate.Year = 0x0;

 if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK) {
  Error_Handler();
 }
 /* USER CODE BEGIN RTC_Init 2 */

 /* USER CODE END RTC_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
 GPIO_InitTypeDef GPIO_InitStruct = { 0 };
 /* USER CODE BEGIN MX_GPIO_Init_1 */
 /* USER CODE END MX_GPIO_Init_1 */

 /* GPIO Ports Clock Enable */
 __HAL_RCC_GPIOC_CLK_ENABLE();
 __HAL_RCC_GPIOD_CLK_ENABLE();
 __HAL_RCC_GPIOA_CLK_ENABLE();
 __HAL_RCC_GPIOB_CLK_ENABLE();

 /*Configure GPIO pin Output Level */
 HAL_GPIO_WritePin(GPIOC, LED_4_Pin | LED_3_Pin, GPIO_PIN_RESET);

 /*Configure GPIO pin Output Level */
 HAL_GPIO_WritePin(GPIOA, LED_2_Pin | LED_1_Pin, GPIO_PIN_RESET);

 /*Configure GPIO pins : LED_4_Pin LED_3_Pin */
 GPIO_InitStruct.Pin = LED_4_Pin | LED_3_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

 /*Configure GPIO pins : LED_2_Pin LED_1_Pin */
 GPIO_InitStruct.Pin = LED_2_Pin | LED_1_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

 /*Configure GPIO pins : INT_X_Pin INT_Y_Pin INT_Z_Pin */
 GPIO_InitStruct.Pin = INT_X_Pin | INT_Y_Pin | INT_Z_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

 /*Configure GPIO pins : BTN_1_Pin BTN_5_Pin BTN_6_Pin BTN_7_Pin
  BTN_8_Pin */
 GPIO_InitStruct.Pin = BTN_1_Pin | BTN_5_Pin | BTN_6_Pin | BTN_7_Pin | BTN_8_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
 GPIO_InitStruct.Pull = GPIO_PULLUP;
 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

 /*Configure GPIO pins : BTN_2_Pin BTN_3_Pin BTN_4_Pin */
 GPIO_InitStruct.Pin = BTN_2_Pin | BTN_3_Pin | BTN_4_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
 GPIO_InitStruct.Pull = GPIO_PULLUP;
 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

 /* EXTI interrupt init*/
 HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
 HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

 /* USER CODE BEGIN MX_GPIO_Init_2 */
 /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// untuk menerima pesan dari pc
void USB_CDC_RxHandler(uint8_t *Buf, uint32_t Len) {
 /* Pastikan string berakhir dengan null-terminator */
 Buf[Len] = '\0';

 /* Periksa apakah perintah dimulai dengan '$' */
 if (Buf[0] == '$') {
  char *token = strtok((char*) Buf, "="); // Pisahkan berdasarkan '='
  if (token != NULL && strcmp(token, "$1") == 0) {
   /* Ambil nilai setelah '=' */
   char *value = strtok(NULL, "=");
   if (value != NULL) {
    /* Ubah nilai variabel mode_emulasi */
    set_waktu_timer_oli = atoi(value);
    FEE_WriteData(SET_WAKTU_TIMER_OLI_ADDRESS, &set_waktu_timer_oli, sizeof(uint32_t));
    printf("waktu oli diatur ke %ld\n", set_waktu_timer_oli);
   }
  } else if (token != NULL && strcmp(token, "$2") == 0) {
   char *value = strtok(NULL, "=");
   if (value != NULL) {
    set_waktu_pompa_oli_on = atoi(value);
    FEE_WriteData(SET_WAKTU_POMPA_OLI_ON_ADDRESS, &set_waktu_pompa_oli_on, sizeof(uint32_t));
    printf("waktu oli on diatur ke %ld\n", set_waktu_pompa_oli_on);
   }
  } else if (token != NULL && strcmp(token, "$3") == 0) {
   char *value = strtok(NULL, "=");
   if (value != NULL) {
    tipe_emulasi = atoi(value);
    FEE_WriteData(TIPE_EMULASI_ADDRESS, &tipe_emulasi, sizeof(uint32_t));
    printf("Mode Emulasi diatur ke %ld\n", tipe_emulasi);
   }
  } else if (token != NULL && strcmp(token, "$4") == 0) {
   char *value = strtok(NULL, "=");
   if (value != NULL) {
    auto_counter_delay = atoi(value);
    FEE_WriteData(AUTO_COUNTER_DELAY_ADDRESS, &auto_counter_delay, sizeof(uint32_t));
    printf("auto counter delay diatur ke %ld\n", auto_counter_delay);
   }
  } else if (token != NULL && strcmp(token, "$5") == 0) {
   char *value = strtok(NULL, "=");
   if (value != NULL) {
    verbose_delay = atoi(value);
    FEE_WriteData(VERBOSE_DELAY_ADDRESS, &verbose_delay, sizeof(uint32_t));
    printf("verbose delay diatur ke %ld\n", verbose_delay);
   }
  } else if (token != NULL && strcmp(token, "$6") == 0) {
   char *value = strtok(NULL, "=");
   if (value != NULL) {
    arah_auto_counter = atoi(value);
    FEE_WriteData(ARAH_AUTO_COUNTER_ADDRESS, &arah_auto_counter, sizeof(uint32_t));
    printf("arah auto counter diatur ke %ld\n", arah_auto_counter);
   }
  } else if (token != NULL && strcmp(token, "$$") == 0) {
   // perlihatkan apa yang ada pada flash memory (simpanan pengaturan)
   baca_dan_tampilkan_flash = 1;
  } else printf("Perintah tidak valid\n");
 } else if (Buf[0] == 'Q') {
  reset_pompa_oli = 1;
  printf("pompa oli diaktifkan manual\n");
 } else if (Buf[0] == '?') {
  tampilkan_verbose = 1;
 } else printf("Perintah tidak valid\n");

}

// untuk callback fungsi interrupt pulse driver x y dan z
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
 for (uint8_t i = 0; i < NUM_INTERRUPT; ++i)
  if (GPIO_Pin == interrupt_pins[i]) // If The INT Source Is EXTI Line9 (A9 Pin)
  HAL_GPIO_ReadPin(button_ports[i], button_pins[i]) == 1 ?
   nilai_counter_axis[i]++ : nilai_counter_axis[i]--;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
 /* USER CODE BEGIN Error_Handler_Debug */
 /* User can add his own implementation to report the HAL error return state */
 __disable_irq();
 while (1) {
 }
 /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
