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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
#include "stdio.h"
#include "FEE.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//0 = emulasi button
//1 = emulasi latch
//2 = emulasi counter auto
//3 = emulasi counter button
//4 = emulasi input interrupt pulse xyz
//5 = hanya echo perintah dari pc
//#define TIPE_EMULASI 5
#define NUM_LEDS 4
#define NUM_INTERRUPT 3
#define NUM_BUTTONS 8
#define DEBOUNCE_DELAY 50 // Debounce delay dalam ms

//definisi address dari variabel yang dapat diubah oleh $
#define TIPE_EMULASI_ADDRESS 		((uint32_t)0x0800F000) //$1
#define AUTO_COUNTER_DELAY_ADDRESS 	((uint32_t)0x0800F010) //$2
#define VERBOSE_DELAY_ADDRESS 		((uint32_t)0x0800F020) //$3
#define ARAH_AUTO_COUNTER_ADDRESS 	((uint32_t)0x0800F030) //$4 0 aritnya bertambah 1 artinya berkurang
//#define verbose_delay 200 // Debounce delay dalam ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ----------------------- variabel yang dapat diubah $ --------------------------- */
uint8_t tipe_emulasi = 0; //$1
uint32_t auto_counter_delay = 200; //$2
uint32_t verbose_delay = 250; //$3
uint8_t arah_auto_counter = 0; //$4 0 aritnya bertambah 1 artinya berkurang

uint8_t tipe_emulasi_address_read;
uint8_t nilai_emulasi_counter = 0;
//nilai counter untuk axis x y dan z
volatile int_fast32_t nilai_counter_axis[3] =
    {
    0, 0, 0
    };
uint8_t latch_states = 0;
uint8_t button_states = 0;
// list interrupt pin
GPIO_TypeDef *interrupt_ports[NUM_INTERRUPT] =
    {
    INT_X_GPIO_Port, INT_Y_GPIO_Port,
    INT_Z_GPIO_Port
    };
const uint16_t interrupt_pins[NUM_INTERRUPT] =
    {
    INT_X_Pin, INT_Y_Pin, INT_Z_Pin
    };
// list led gpio
GPIO_TypeDef *led_ports[NUM_LEDS] =
    {
    LED_1_GPIO_Port, LED_2_GPIO_Port,
    LED_3_GPIO_Port, LED_4_GPIO_Port
    };
const uint16_t led_pins[NUM_LEDS] =
    {
    LED_1_Pin, LED_2_Pin, LED_3_Pin, LED_4_Pin
    };
//list button gpio
GPIO_TypeDef *button_ports[NUM_BUTTONS] =
    {
    BTN_1_GPIO_Port, BTN_2_GPIO_Port, BTN_3_GPIO_Port,
    BTN_4_GPIO_Port, BTN_5_GPIO_Port, BTN_6_GPIO_Port,
    BTN_7_GPIO_Port, BTN_8_GPIO_Port
    };
const uint16_t button_pins[NUM_BUTTONS] =
    {
    BTN_1_Pin, BTN_2_Pin, BTN_3_Pin,
    BTN_4_Pin, BTN_5_Pin, BTN_6_Pin, BTN_7_Pin, BTN_8_Pin,
    };
uint32_t last_button_press_time[NUM_BUTTONS] =
    {
    0
    }; // Variabel untuk debounce

uint32_t last_verbose_time = 0; // Variabel untuk debounce
uint32_t last_auto_counter_time = 0; // Variabel untuk debounce
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// program untuk membaca data dari pc
int _write(int file, char *ptr, int len)
    {
    CDC_Transmit_FS((uint8_t*) ptr, len);
    return len;
    }

void update_leds(uint8_t counter)
    {
    for (uint8_t i = 0; i < NUM_LEDS; i++)
	{
	if (counter & (1 << i))
	    HAL_GPIO_WritePin(led_ports[i], led_pins[i], GPIO_PIN_SET); // Nyalakan LED
	else
	    HAL_GPIO_WritePin(led_ports[i], led_pins[i], GPIO_PIN_RESET); // Nyalakan LED
	}
    }

void read_buttons_with_debounce(void)
    {
    for (uint8_t i = 0; i < 8; i++)
	{
	uint8_t current_state = HAL_GPIO_ReadPin(button_ports[i],
		button_pins[i]);
	if (current_state != button_states)
	    {
	    if (HAL_GetTick() - last_button_press_time[i] >= DEBOUNCE_DELAY)
		{
		last_button_press_time[i] = HAL_GetTick();
		/* Perbarui status tombol */
		if (!current_state)
		    {
		    button_states |= (1 << i);  // Set bit tombol
//		    latch_states[i] = !latch_states[i];
		    if (button_states & (1 << i)) // Periksa apakah tombol kedua ditekan
			latch_states ^= (1 << i); // Balikkan bit pada latch_state
		    }
		else
		    button_states &= ~(1 << i); // Clear bit tombol

//		    tambah fitur reset pada program emulasi counter
		switch (i)
		    {
		case 0:
		    nilai_emulasi_counter++;
		    break;
		case 1:
		    nilai_emulasi_counter--;
		    break;
		case 2:
		    nilai_emulasi_counter = 0;
		    break;
		default:
		    break;
		    }
		}
	    }
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
    MX_USB_DEVICE_Init();

    /* USER CODE BEGIN 2 */
    FEE_ReadData(TIPE_EMULASI_ADDRESS, &tipe_emulasi_address_read, sizeof(int32_t));
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
	{
	read_buttons_with_debounce(); // baca nilai tombol

	/* ----------------------- program untuk melakukan komputasi tombol --------------------------- */
//tipe button
	if (tipe_emulasi == 0)
	    for (uint8_t i = 0; i < NUM_LEDS; i++)
		(button_states & (1 << i)) ?
			HAL_GPIO_WritePin(led_ports[i], led_pins[i],
				GPIO_PIN_SET) :
			HAL_GPIO_WritePin(led_ports[i], led_pins[i],
				GPIO_PIN_RESET);
//tipe latch
	else if (tipe_emulasi == 1)
	    for (uint8_t i = 0; i < NUM_LEDS; i++)
		(latch_states & (1 << i)) ?
			HAL_GPIO_WritePin(led_ports[i], led_pins[i],
				GPIO_PIN_SET) :
			HAL_GPIO_WritePin(led_ports[i], led_pins[i],
				GPIO_PIN_RESET);
//tipe auto counter
	else if (tipe_emulasi == 2)
	    {
	    if (HAL_GetTick() - last_auto_counter_time >= auto_counter_delay)
		{
		last_auto_counter_time = HAL_GetTick();
		arah_auto_counter == 0 ?
			update_leds(nilai_emulasi_counter++) :
			update_leds(nilai_emulasi_counter--);
		}
	    }
//tipe counter button
	else if (tipe_emulasi == 3)
	    update_leds(nilai_emulasi_counter);

	/* ----------------------- program menampilkan karakter dan verbose --------------------------- */
// tampilkan verbose button
	if (tipe_emulasi != 4)
	    {
	    if (HAL_GetTick() - last_verbose_time >= verbose_delay)
		{
		last_verbose_time = HAL_GetTick();
		printf("B,");
		for (int i = 0; i < NUM_BUTTONS; ++i)
		    {
		    printf("%d", button_states);
		    }
		printf(",L");
		for (int i = 0; i < NUM_BUTTONS; ++i)
		    {
		    printf("%d", latch_states);
		    }
		printf(",C,%d\r\n", nilai_emulasi_counter);
		printf(",E,%d\r\n", tipe_emulasi_address_read);
		printf("\r\n"); //enter untuk merapikan verbose
		}
	    }
	else
	    {
	    for (uint8_t i = 0; i < NUM_INTERRUPT; ++i)
		{
		i == 0 ? printf("X,") : i == 1 ? printf("Y,") : printf("Z,");
		i != NUM_INTERRUPT - 1 ?
			printf("%d,", nilai_counter_axis[i]) :
			printf("%d", nilai_counter_axis[i]);
//		NUM_INTERRUPT - i == 1 ? printf(" ") : printf(",");
		}
	    printf("\r\n"); //enter untuk merapikan verbose
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
void SystemClock_Config(void)
    {
    RCC_OscInitTypeDef RCC_OscInitStruct =
	{
	0
	};
    RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{
	0
	};
    RCC_PeriphCLKInitTypeDef PeriphClkInit =
	{
	0
	};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
	Error_Handler();
	}

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	    | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
	Error_Handler();
	}
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
	Error_Handler();
	}
    }

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
    {
    GPIO_InitTypeDef GPIO_InitStruct =
	{
	0
	};
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
    GPIO_InitStruct.Pin = BTN_1_Pin | BTN_5_Pin | BTN_6_Pin | BTN_7_Pin
	    | BTN_8_Pin;
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
void USB_CDC_RxHandler(uint8_t *Buf, uint32_t Len)
    {
    /* Pastikan string berakhir dengan null-terminator */
    Buf[Len] = '\0';

    /* Periksa apakah perintah dimulai dengan '$' */
    if (Buf[0] == '$')
	{
	char *token = strtok((char*) Buf, "="); // Pisahkan berdasarkan '='
	if (token != NULL && strcmp(token, "$1") == 0)
	    {
	    /* Ambil nilai setelah '=' */
	    char *value = strtok(NULL, "=");
	    if (value != NULL)
		{
		/* Ubah nilai variabel mode_emulasi */
		tipe_emulasi = atoi(value);

		/* Kirim konfirmasi kembali ke PC */
		char response[64];
		snprintf(response, sizeof(response),
			"Mode Emulasi diatur ke %d\n", tipe_emulasi);
		CDC_Transmit_FS((uint8_t*) response, strlen(response));
		}
	    }
	else if (token != NULL && strcmp(token, "$2") == 0)
	    {
	    /* Ambil nilai setelah '=' */
	    char *value = strtok(NULL, "=");
	    if (value != NULL)
		{
		/* Ubah nilai variabel mode_emulasi */
		auto_counter_delay = atoi(value);

		/* Kirim konfirmasi kembali ke PC */
		char response[64];
		snprintf(response, sizeof(response),
			"auto counter delay diatur ke %ld\n",
			auto_counter_delay);
		CDC_Transmit_FS((uint8_t*) response, strlen(response));
		}
	    }
	else if (token != NULL && strcmp(token, "$3") == 0)
	    {
	    /* Ambil nilai setelah '=' */
	    char *value = strtok(NULL, "=");
	    if (value != NULL)
		{
		/* Ubah nilai variabel mode_emulasi */
		verbose_delay = atoi(value);

		/* Kirim konfirmasi kembali ke PC */
		char response[64];
		snprintf(response, sizeof(response),
			"verbose delay diatur ke %ld\n", verbose_delay);
		CDC_Transmit_FS((uint8_t*) response, strlen(response));
		}
	    }
	else if (token != NULL && strcmp(token, "$3") == 0)
	    {
	    /* Ambil nilai setelah '=' */
	    char *value = strtok(NULL, "=");
	    if (value != NULL)
		{
		/* Ubah nilai variabel mode_emulasi */
		arah_auto_counter = atoi(value);

		/* Kirim konfirmasi kembali ke PC */
		char response[64];
		snprintf(response, sizeof(response),
			"verbose delay diatur ke %d\n", arah_auto_counter);
		CDC_Transmit_FS((uint8_t*) response, strlen(response));
		}
	    }
	else
	    {
	    /* Kirim pesan error jika perintah tidak valid */
	    const char *error_msg = "Perintah tidak valid\n";
	    CDC_Transmit_FS((uint8_t*) error_msg, strlen(error_msg));
	    }
	}
    else
	{
	/* Echo kembali data jika bukan perintah */
	CDC_Transmit_FS(Buf, Len);
	}
    }

// untuk callback fungsi interrupt pulse driver x y dan z
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
    {
    for (uint8_t i = 0; i < NUM_INTERRUPT; ++i)
	if (GPIO_Pin == interrupt_pins[i]) // If The INT Source Is EXTI Line9 (A9 Pin)
	    HAL_GPIO_ReadPin(button_ports[i], button_pins[i]) == 1 ?
		    nilai_counter_axis[i]++ : nilai_counter_axis[0]--;
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
