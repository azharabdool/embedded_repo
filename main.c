/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file : main.c
* @brief : Main program body
******************************************************************************
* @attention
*
* Copyright (c) 2023 STMicroelectronics.
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
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32f0xx.h"
#include <lcd_stm32f0.c>
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TODO: Add values for below variables
#define NS 201 // Number of samples in LUT
#define TIM2CLK 8000000 // STM Clock frequency
#define F_SIGNAL 10 // Frequency of output analog signal
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;
/* USER CODE BEGIN PV */
// TODO: Add code for global variables, including LUTs
int i =0;
int pTick=0;
int tick;
uint32_t Sin_LUT[NS] =
{512,528,544,560,576,592,607,623,639,654,670,685,700,715,729,744,758,772,786,799,812,825,838,850,862,873,8
84,895,906,916,925,935,943,952,960,967,974,981,987,993,998,1003,1007,1011,1014,1017,1019,1021,1022,1023,1
023,1023,1022,1021,1019,1017,1014,1011,1007,1003,998,993,987,981,974,967,960,952,943,935,925,916,906,895,
884,873,862,850,838,825,812,799,786,772,758,744,729,715,700,685,670,654,639,623,607,592,576,560,544,528,51
2,495,479,463,447,431,416,400,384,369,353,338,323,308,294,279,265,251,237,224,211,198,185,173,161,150,139,1
28,117,107,98,88,80,71,63,56,49,42,36,30,25,20,16,12,9,6,4,2,1,0,0,0,1,2,4,6,9,12,16,20,25,30,36,42,49,56,63,71,80
,88,98,107,117,128,139,150,161,173,185,198,211,224,237,251,265,279,294,308,323,338,353,369,384,400,416,431,
447,463,479,495,511};
uint32_t saw_LUT[NS] =
{0,5,10,15,20,26,31,36,41,46,51,56,61,66,72,77,82,87,92,97,102,107,113,118,123,128,133,138,143,148,153,159,164
,169,174,179,184,189,194,199,205,210,215,220,225,230,235,240,246,251,256,261,266,271,276,281,286,292,297,30
2,307,312,317,322,327,332,338,343,348,353,358,363,368,373,379,384,389,394,399,404,409,414,419,425,430,435,4
40,445,450,455,460,465,471,476,481,486,491,496,501,506,512,517,522,527,532,537,542,547,552,558,563,568,573,
578,583,588,593,598,604,609,614,619,624,629,634,639,644,650,655,660,665,670,675,680,685,691,696,701,706,71
1,716,721,726,731,737,742,747,752,757,762,767,772,777,783,788,793,798,803,808,813,818,824,829,834,839,844,8
49,854,859,864,870,875,880,885,890,895,900,905,910,916,921,926,931,936,941,946,951,957,962,967,972,977,982,
987,992,997,1003,1008,1013,1018,1023};
uint32_t triangle_LUT[NS] =
{0,10,20,31,41,51,61,72,82,92,102,113,123,133,143,153,164,174,184,194,205,215,225,235,246,256,266,276,286,29
7,307,317,327,338,348,358,368,379,389,399,409,419,430,440,450,460,471,481,491,501,512,522,532,542,552,563,5
73,583,593,604,614,624,634,644,655,665,675,685,696,706,716,726,737,747,757,767,777,788,798,808,818,829,839,
849,859,870,880,890,900,910,921,931,941,951,962,972,982,992,1003,1013,1023,1013,1003,992,982,972,962,951,9
41,931,921,910,900,890,880,870,859,849,839,829,818,808,798,788,777,767,757,747,737,726,716,706,696,685,675,
665,655,644,634,624,614,604,593,583,573,563,552,542,532,522,512,501,491,481,471,460,450,440,430,419,409,39
9,389,379,368,358,348,338,327,317,307,297,286,276,266,256,246,235,225,215,205,194,184,174,164,153,143,133,1
23,113,102,92,82,72,61,51,41,31,20,10,0};
// TODO: Equation to calculate TIM2_Ticks
uint32_t TIM2_Ticks = TIM2CLK/(NS*F_SIGNAL); // How often to write new LUT value
uint32_t DestAddress = (uint32_t) &(TIM3->CCR3); // Write LUT TO TIM3->CCR3 to modify PWM duty cycle
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void EXTI0_1_IRQHandler(void);
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
/**
* @brief The application entry point.
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
init_LCD();
/* USER CODE END Init */
/* Configure the system clock */
SystemClock_Config();
/* USER CODE BEGIN SysInit */
/* USER CODE END SysInit */
/* Initialize all configured peripherals */
MX_GPIO_Init();
MX_DMA_Init();
MX_TIM2_Init();
MX_TIM3_Init();
/* USER CODE BEGIN 2 */
// TODO: Start TIM3 in PWM mode on channel 3
HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
// TODO: Start TIM2 in Output Compare (OC) mode on channel 1.
HAL_TIM_OC_Start(&htim2,TIM_CHANNEL_1);
// TODO: Start DMA in IT mode on TIM2->CH1; Source is LUT and Dest is TIM3->CCR3; start with Sine LUT
HAL_DMA_Start_IT(&hdma_tim2_ch1, Sin_LUT, DestAddress, NS);
// TODO: Write current waveform to LCD ("Sine")
delay(3000);
lcd_command(CLEAR);
lcd_putstring("Sine");
// TODO: Enable DMA (start transfer from LUT to CCR)
__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
/* USER CODE END 2 */
/* Infinite loop */
/* USER CODE BEGIN WHILE */
while (1)
{
/* USER CODE END WHILE */
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
LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
{
}
LL_RCC_HSI_Enable();
/* Wait till HSI is ready */
while(LL_RCC_HSI_IsReady() != 1)
{
}
LL_RCC_HSI_SetCalibTrimming(16);
LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
/* Wait till System clock is ready */
while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
{
}
LL_SetSystemCoreClock(8000000);
/* Update the time base */
if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
{
Error_Handler();
}
}
/**
* @brief TIM2 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM2_Init(void)
{
/* USER CODE BEGIN TIM2_Init 0 */
/* USER CODE END TIM2_Init 0 */
TIM_ClockConfigTypeDef sClockSourceConfig = {0};
TIM_MasterConfigTypeDef sMasterConfig = {0};
TIM_OC_InitTypeDef sConfigOC = {0};
/* USER CODE BEGIN TIM2_Init 1 */
/* USER CODE END TIM2_Init 1 */
htim2.Instance = TIM2;
htim2.Init.Prescaler = 0;
htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
htim2.Init.Period = TIM2_Ticks - 1;
htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
{
Error_Handler();
}
sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
{
Error_Handler();
}
if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
{
Error_Handler();
}
sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
{
Error_Handler();
}
sConfigOC.OCMode = TIM_OCMODE_TIMING;
sConfigOC.Pulse = 0;
sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
{
Error_Handler();
}
/* USER CODE BEGIN TIM2_Init 2 */
/* USER CODE END TIM2_Init 2 */
}
/**
* @brief TIM3 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM3_Init(void)
{
/* USER CODE BEGIN TIM3_Init 0 */
/* USER CODE END TIM3_Init 0 */
TIM_ClockConfigTypeDef sClockSourceConfig = {0};
TIM_MasterConfigTypeDef sMasterConfig = {0};
TIM_OC_InitTypeDef sConfigOC = {0};
/* USER CODE BEGIN TIM3_Init 1 */
/* USER CODE END TIM3_Init 1 */
htim3.Instance = TIM3;
htim3.Init.Prescaler = 0;
htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
htim3.Init.Period = 1023;
htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
{
Error_Handler();
}
sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
{
Error_Handler();
}
if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
{
Error_Handler();
}
sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
{
Error_Handler();
}
sConfigOC.OCMode = TIM_OCMODE_PWM1;
sConfigOC.Pulse = 0;
sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
{
Error_Handler();
}
/* USER CODE BEGIN TIM3_Init 2 */
/* USER CODE END TIM3_Init 2 */
HAL_TIM_MspPostInit(&htim3);
}
/**
* Enable DMA controller clock
*/
static void MX_DMA_Init(void)
{
/* DMA controller clock enable */
__HAL_RCC_DMA1_CLK_ENABLE();
/* DMA interrupt init */
/* DMA1_Channel4_5_IRQn interrupt configuration */
HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);
}
/**
* @brief GPIO Initialization Function
* @param None
* @retval None
*/
static void MX_GPIO_Init(void)
{
LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */
/* GPIO Ports Clock Enable */
LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
/**/
LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);
/**/
LL_GPIO_SetPinPull(Button0_GPIO_Port, Button0_Pin, LL_GPIO_PULL_UP);
/**/
LL_GPIO_SetPinMode(Button0_GPIO_Port, Button0_Pin, LL_GPIO_MODE_INPUT);
/**/
EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
EXTI_InitStruct.LineCommand = ENABLE;
EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
LL_EXTI_Init(&EXTI_InitStruct);
/* USER CODE BEGIN MX_GPIO_Init_2 */
HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
/* USER CODE END MX_GPIO_Init_2 */
}
/* USER CODE BEGIN 4 */
void EXTI0_1_IRQHandler(void)
{
// TODO: Debounce using HAL_GetTick()
// TODO: Disable DMA transfer and abort IT, then start DMA in IT mode with new LUT and re-enable
transfer
// HINT: Consider using C's "switch" function to handle LUT changes
tick = HAL_GetTick();
if ((tick - pTick) > 100)
{
__HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC1);
HAL_DMA_Abort_IT(&hdma_tim2_ch1);
if (i==0){
i=i+1;
delay(3000);
lcd_command(CLEAR);
lcd_putstring("Sawtooth");
HAL_DMA_Start_IT(&hdma_tim2_ch1, saw_LUT, DestAddress, NS);
__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
}
else if(i==1){
i=i+1;
delay(3000);
lcd_command(CLEAR);
lcd_putstring("Triangle");
HAL_DMA_Start_IT(&hdma_tim2_ch1, triangle_LUT, DestAddress, NS);
__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
}
else if(i==2){
i=0;
delay(3000);
lcd_command(CLEAR);
lcd_putstring("Sine");
HAL_DMA_Start_IT(&hdma_tim2_ch1, Sin_LUT, DestAddress, NS);
__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
}
}
HAL_GPIO_EXTI_IRQHandler(Button0_Pin); // Clear interrupt flags
}
/* USER CODE END 4 */
/**
* @brief This function is executed in case of error occurrence.
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
#ifdef USE_FULL_ASSERT
/**
* @brief Reports the name of the source file and the source line number
* where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
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