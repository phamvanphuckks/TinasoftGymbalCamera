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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tracking.h"
#include "keyboard.h"
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
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */

#define PUTCHAR_PROTOTYPE int fputc(int s, FILE *f)
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart3, (uint8_t*)&s,1,10);
    return s;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int state = 0;
int count = 0;
uint8_t buffer[256];
uint8_t a[512];
uint8_t data = 0;
uint8_t old_data = 0;
uint8_t new_data = 0;
int len = 0;
int package_complete = 0;
uint8_t mode=0;

#define MAX 1024
uint8_t cqueue_arr[MAX];
int front = -1;
int rear  = -1;

uint8_t keyIdxPress[6];

uint8_t isEmpty()
{
    if (front == -1)
       return 1;
    else
       return 0;
}/*End of isEmpty()*/

uint8_t isFull()
{
	if ((front == 0 && rear == MAX - 1) || (front == rear + 1))
		return 1;
	else
		return 0;
}/*End of isFull()*/

uint8_t del()
{
    uint8_t item;
    if( isEmpty() )
    {
        printf("\nQueue Underflow\n");                
    }
    item = cqueue_arr[front];
    if(front == rear) /* queue has only one element */
    {
        front = -1;
        rear  = -1;
    }
    else if(front==MAX-1)
        front = 0;
    else
        front = front+1;
    return item;
}/*End of del()*/

void insert(uint8_t item)
{
	if (isFull())
	{
        //del();
        //printf("\nQueue Overflow\n");
        return;
	}
	if (front == -1)
		front = 0;

	if (rear == MAX - 1)/*rear is at last position of queue*/
		rear = 0;
	else
		rear = rear + 1;
	cqueue_arr[rear] = item;
}/*End of insert()*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)	
    {
        insert(data);
        HAL_UART_Receive_IT(&huart2,&data, 1);

    }
}
void printHex(uint8_t* a, int len)
{
    int i = 0;
    while (len - i)
    {
        printf("%x ", a[i]);
        i++;
    }
    printf("\n");
}
void SBUS_write(uint16_t* channels)
{
    static uint8_t packet[25];
    /* assemble the SBUS packet */
    // SBUS header
    packet[0] = _sbusHeader;
    // 16 channels of 11 bit data
    if (channels) {
        packet[1] = (uint8_t) ((channels[0] & 0x07FF));
        packet[2] = (uint8_t) ((channels[0] & 0x07FF)>>8 | (channels[1] & 0x07FF)<<3);
        packet[3] = (uint8_t) ((channels[1] & 0x07FF)>>5 | (channels[2] & 0x07FF)<<6);
        packet[4] = (uint8_t) ((channels[2] & 0x07FF)>>2);
        packet[5] = (uint8_t) ((channels[2] & 0x07FF)>>10 | (channels[3] & 0x07FF)<<1);
        packet[6] = (uint8_t) ((channels[3] & 0x07FF)>>7 | (channels[4] & 0x07FF)<<4);
        packet[7] = (uint8_t) ((channels[4] & 0x07FF)>>4 | (channels[5] & 0x07FF)<<7);
        packet[8] = (uint8_t) ((channels[5] & 0x07FF)>>1);
        packet[9] = (uint8_t) ((channels[5] & 0x07FF)>>9 | (channels[6] & 0x07FF)<<2);
        packet[10] = (uint8_t) ((channels[6] & 0x07FF)>>6 | (channels[7] & 0x07FF)<<5);
        packet[11] = (uint8_t) ((channels[7] & 0x07FF)>>3);
        packet[12] = (uint8_t) ((channels[8] & 0x07FF));
        packet[13] = (uint8_t) ((channels[8] & 0x07FF)>>8 | (channels[9] & 0x07FF)<<3);
        packet[14] = (uint8_t) ((channels[9] & 0x07FF)>>5 | (channels[10] & 0x07FF)<<6);
        packet[15] = (uint8_t) ((channels[10] & 0x07FF)>>2);
        packet[16] = (uint8_t) ((channels[10] & 0x07FF)>>10 | (channels[11] & 0x07FF)<<1);
        packet[17] = (uint8_t) ((channels[11] & 0x07FF)>>7 | (channels[12] & 0x07FF)<<4);
        packet[18] = (uint8_t) ((channels[12] & 0x07FF)>>4 | (channels[13] & 0x07FF)<<7);
        packet[19] = (uint8_t) ((channels[13] & 0x07FF)>>1);
        packet[20] = (uint8_t) ((channels[13] & 0x07FF)>>9 | (channels[14] & 0x07FF)<<2);
        packet[21] = (uint8_t) ((channels[14] & 0x07FF)>>6 | (channels[15] & 0x07FF)<<5);
        packet[22] = (uint8_t) ((channels[15] & 0x07FF)>>3);
    }
    // flags
    packet[23] = 0x00;
    // footer
    packet[24] = _sbusFooter;	
    // write packet
    HAL_UART_Transmit(&huart5,packet,25,100);
    //_bus->write(packet,25);
}

/*static SLStatus getAllFromSla()
{
	s32 sleepMs = 30;

// Request state from SLA
u8 buf[80];
s32 len;
//	s32 cameraIndex = 2;
// Get the version information
	len = SLFIPGetVersionNumber(buf);  HAL_UART_Transmit(&huart2,buf, len,10);  HAL_Delay(sleepMs);
	len = SLFIPGetImageSize(buf,2);
	HAL_UART_Transmit(&huart2,buf, len,10);
	HAL_Delay(sleepMs);
	len = SLFIPGetOverlayMode(buf);
	HAL_UART_Transmit(&huart2,buf, len,10);
	HAL_Delay(sleepMs);
	len = SLFIGetCurrentOverlayObject(buf);  HAL_UART_Transmit(&huart2,buf, len,10);  HAL_Delay(sleepMs);
	printf("OK");
	 // Reset parameters
//len = SLFIPResetAllParameters(buf, 0);  HAL_UART_Transmit(&huart2,buf, len,10);  HAL_Delay(sleepMs);
HAL_Delay(100);
return SLA_SUCCESS;
}
*/
/* write SBUS packets */



//*
uint8_t IsStateKB(u8 key)
{
    for(uint8_t idx = 0; idx < 6; idx++)
    {
        if(keyIdxPress[idx] == key) return 1;
    }
    return 0;
}

void stopGymbal(void)
{
    uint16_t channels[16] = {1023, 1023, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024};
    SBUS_write(&channels[0]);    
}

void zoomInGymbal(void)
{
    uint16_t channels[16] = {1023, 1023, 1024, 1700, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024};
    SBUS_write(&channels[0]);    
}

void zoomOutGymbal(void)
{
    uint16_t channels[16] = {1023, 1023, 1024, 600, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024};
    SBUS_write(&channels[0]);    
}
/* USER CODE END 0 */

/**
* @brief  The application entry point.
* @retval int
*/
int main(void)
{
    /* USER CODE BEGIN 1 */
    u8 packet[25];
    GCPid PID;
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */
    GcInitState (&PID);
    s32 numTracks = 0;
    u8 stateKB = 0;
    SLFIPTrackRes tracks[6];

    uint16_t channels[16] = {1023, 1023, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024};
    //channels[3] = 500;
    SBUS_write(&channels[0]);
    //HAL_Delay(100);
    //channels[3] = 1700;

    SBUS_write(&channels[0]);
    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_UART5_Init();
    /* USER CODE BEGIN 2 */
    HAL_UART_Receive_IT(&huart2,&data, 1); 
        //printf("\nok");
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    //printf("\nHELLO\n");
    //getAllFromSla();
    //int old=0; 
    while (1)
    {
        channels[0]=1023;
        channels[1]=1023;

        while(isEmpty()==0)
        {	
            if (KeyPressed())
            {
                //printf("\n pressed");
                switch (KeyCode())
                {
                    case 12:
                        //printf("\n no.12");
                        zoomInGymbal();
                        break;
                    case 14:
                        //printf("\n no.14");
                        zoomOutGymbal();   
                        break;
                    case 6:
                        mode = 1;
                        break;
                    case 10:
                        mode = 2;
                        stateKB = 0;
                        break;
                    default: 
                        stateKB = 0;
                        break; 
                }
            } 

            old_data = new_data;
            new_data = del();

            if (old_data == 0x51 && new_data == 0xAC)
            {
                memset(buffer, 0, 256);
                state = 2;
                count = 2;
                buffer[0] = old_data;
                buffer[1] = new_data;
            }
            else if (state == 2 )
            {
                len = new_data;
                state = 3;
                count = 3;
                buffer[2] = new_data;
            }
            else if (count >= 3)
            {
                count++;
                buffer[count - 1] = new_data;
                //u8 buf1[]= {0x51,0xAC,0x13,0x4E,0x80,0x07,0x38,0x04,0x80,0x07,0x38,0x04,0x00,0x00,0x00,0x00,0x80,0x07,0x38,0x04,0x02,0x4E};
                if (count == len + 3)
                {
                    package_complete = 1;
                    //printHex(buffer, len + 3);
                    switch(mode){
                        case 1:
                            if(buffer[3]==0x43)
                            {
                                cbTrackingPosition(buffer, &PID, channels);
                                channels[3] = 1023;
                                SBUS_write(&channels[0]);
                            }
                            break;
                        case 2:
                            if (buffer[3] == 0x51)
                            {
                                numTracks=0;
                                //	printf("\n numtrack: %d", numTracks);
                                mycbTrackPositions(tracks, buffer, &numTracks, keyIdxPress); // keyIdxPress[1,3,5]
                                //	printf("\n numtrack: %d", numTracks);
                                if( KeyPressed())
                                {
                                    //printf("\n pressed");
                                    switch (KeyCode())
                                    {
                                        case 0:
                                         //   printf("\n no.1");
                                            stateKB = 1;
                                            break;
                                        case 1:
                                         //   printf("\n no.2");
                                            stateKB = 2;
                                            break;
                                        case 2:
                                         //   printf("\n no.3");
                                            stateKB = 3;
                                            break;
                                        case 4:
                                         //   printf("\n no.4");
                                            stateKB = 4;
                                            break;
                                        case 5:
                                           // printf("\n no.5");
                                            stateKB = 5;
                                            break;
                                        case 12:
                                            //printf("\n no.12");
                                            zoomInGymbal();
                                            break;
                                        case 13:
                                            stateKB=0;
                                            break;
                                        case 14:
                                            //printf("\n no.14");
                                            zoomOutGymbal();   
                                            break;
                                        default: 
                                           // stateKB = 0;
                                            break;
                                   }// end switch                                                                                                                                                                            
                                }
                                else
                                {
                                   stopGymbal();
                                }

                                if(numTracks > 0)
                                {
                                    switch (stateKB)
                                    {
                                        case 0:
                                            break;
                                        case 1:
                                            if(IsStateKB(1))
                                            {
                                                TrackingControl(tracks[1], &PID, channels);
                                                SBUS_write(&channels[0]);
                                            }
                                            break;
                                        case 2:
                                            if(IsStateKB(2))
                                            {
                                                TrackingControl(tracks[2], &PID, channels);
                                                SBUS_write(&channels[0]);
                                            }
                                            break;
                                        case 3:
                                            if(IsStateKB(3))
                                            {
                                                TrackingControl(tracks[3], &PID, channels);
                                                SBUS_write(&channels[0]);
                                            }
                                            break;
                                        case 4:
                                            if(IsStateKB(4))
                                            {
                                                TrackingControl(tracks[4], &PID, channels);
                                                SBUS_write(&channels[0]);
                                            }
                                            break;
                                        case 5:
                                            if(IsStateKB(5))
                                            {
                                                TrackingControl(tracks[5], &PID, channels);
                                                SBUS_write(&channels[0]);
                                            }
                                            break;
                                        default:
                                            stopGymbal();
                                    }// end switch
                                }
                                else
                                {
                                    stateKB = 0;
                                    stopGymbal();
                                }
                            } // end if (buffer[3]==0x51)
                            break;
                   default: 
                       break;
                 }// end switch
                    state = 0;
                    count = 0;
                    len = 0;
                }
            }// end else if (count >= 3)
        }// end while(isEmpty()==0)
    }// end while
}
/* USER CODE END 3 */
/**
* @brief System Clock Configuration
* @retval None
*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
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
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
* @brief UART5 Initialization Function
* @param None
* @retval None
*/
static void MX_UART5_Init(void)
{

/* USER CODE BEGIN UART5_Init 0 */

/* USER CODE END UART5_Init 0 */

/* USER CODE BEGIN UART5_Init 1 */

/* USER CODE END UART5_Init 1 */
huart5.Instance = UART5;
huart5.Init.BaudRate = 100000;
huart5.Init.WordLength = UART_WORDLENGTH_8B;
huart5.Init.StopBits = UART_STOPBITS_2;
huart5.Init.Parity = UART_PARITY_EVEN;
huart5.Init.Mode = UART_MODE_TX_RX;
huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
huart5.Init.OverSampling = UART_OVERSAMPLING_16;
if (HAL_UART_Init(&huart5) != HAL_OK)
{
	Error_Handler();
}
/* USER CODE BEGIN UART5_Init 2 */

/* USER CODE END UART5_Init 2 */

}

/**
* @brief USART2 Initialization Function
* @param None
* @retval None
*/
static void MX_USART2_UART_Init(void)
{

/* USER CODE BEGIN USART2_Init 0 */

/* USER CODE END USART2_Init 0 */

/* USER CODE BEGIN USART2_Init 1 */

/* USER CODE END USART2_Init 1 */
huart2.Instance = USART2;
huart2.Init.BaudRate = 57600;
huart2.Init.WordLength = UART_WORDLENGTH_8B;
huart2.Init.StopBits = UART_STOPBITS_1;
huart2.Init.Parity = UART_PARITY_NONE;
huart2.Init.Mode = UART_MODE_TX_RX;
huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
huart2.Init.OverSampling = UART_OVERSAMPLING_16;
if (HAL_UART_Init(&huart2) != HAL_OK)
{
	Error_Handler();
}
/* USER CODE BEGIN USART2_Init 2 */

/* USER CODE END USART2_Init 2 */

}

/**
* @brief USART3 Initialization Function
* @param None
* @retval None
*/
static void MX_USART3_UART_Init(void)
{

/* USER CODE BEGIN USART3_Init 0 */

/* USER CODE END USART3_Init 0 */

/* USER CODE BEGIN USART3_Init 1 */

/* USER CODE END USART3_Init 1 */
huart3.Instance = USART3;
huart3.Init.BaudRate = 57600;
huart3.Init.WordLength = UART_WORDLENGTH_8B;
huart3.Init.StopBits = UART_STOPBITS_1;
huart3.Init.Parity = UART_PARITY_NONE;
huart3.Init.Mode = UART_MODE_TX_RX;
huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
huart3.Init.OverSampling = UART_OVERSAMPLING_16;
if (HAL_UART_Init(&huart3) != HAL_OK)
{
	Error_Handler();
}
/* USER CODE BEGIN USART3_Init 2 */

/* USER CODE END USART3_Init 2 */

}

/**
* @brief GPIO Initialization Function
* @param None
* @retval None
*/
static void MX_GPIO_Init(void)
{
GPIO_InitTypeDef GPIO_InitStruct = {0};

/* GPIO Ports Clock Enable */
__HAL_RCC_GPIOC_CLK_ENABLE();
__HAL_RCC_GPIOH_CLK_ENABLE();
__HAL_RCC_GPIOA_CLK_ENABLE();
__HAL_RCC_GPIOB_CLK_ENABLE();
__HAL_RCC_GPIOD_CLK_ENABLE();

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

/*Configure GPIO pins : PD4 PD5 PD6 PD7 */
GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/*Configure GPIO pins : PB3 PB4 PB5 PB6 */
GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_PULLUP;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
