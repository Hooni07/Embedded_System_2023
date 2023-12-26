#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_dma.h"
#include "lcd.h"
#include "misc.h"
#include "stdbool.h"

// Define GPIO pins for buzzer
#define BUZZER_PIN GPIO_Pin_3
#define BUZZER_PORT GPIOA

int i = 0, j = 0;
int value = -1;
int password_index = -1;
int keypad_input[4] = {-1, -1, -1, -1}; // ????? ??? ????
int password[4] = {1, 2, 3, 4};
int timePassed = 0;
int pulse = 500;
int comOPen = 0;
char LCD_output[5] = {'\0', '\0', '\0', '\0', '\0'};   // LCD output¿ë ¹öÆÛ
bool isOpen = false;
bool motion = false;
bool prevDoorClosed = true;
bool curDoorClosed = true;
bool bluetoothUser = false;


uint16_t prescale; // ?????? ?????? PWM ??????? ???? Prescaler ??
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;

volatile uint32_t ADC_Value[2];

/* function prototype*/
void RCC_Configure(void);
// Door Action
void openDoor(void);
void closeDoor(void);
// keypad
void getKeyPressed();
void IncorrectPasswordSound(void);
void Check_Password();
// GPIO_Configure
void GPIO_Configure(void);
// EXTI_Configure
void EXTI_Configure(void);
// NVIC_Configure
void NVIC_Configure(void);
// USART_Init(Using Bluetooth)
void USART1_Init(void);
void USART2_Init(void);
// IRQHandler
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void EXTI4_IRQHandler(void); // Manual Open Button
// Human
void Human_Configure(void);
void TIM4_IRQHandler(void);
// PWM_Configure(Using Servo)
void PWM_Configure1(void);
void PWM_Configure2(void);
// Beep
void beep(void);
// Delay
void delay(int);
/* ----------------------------------------------------------- */
void RCC_Configure(void)
{
  /* Keypad */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  
   // Configure I2C
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  
  /* LED & Port D RCC Enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

  /* For Bluetooth */
  /* USART1, USART2 TX/RX port clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
      
  /* USART1, USART2 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  /* For Sensor */
  // human sensor
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // TIM4
  // magnetic sensor
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  /* For Button */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  /* For Servo */
  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // TIM2
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // Port B RCC ENABLE
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // TIM3
  
  /* piezo PC8 */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); // Port C
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // TIM2
  

  /* Alternate Function IO clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

}


/* GPIO */
void GPIO_Configure(void)
{
  /* GPIO Bluetooth */
  GPIO_InitTypeDef GPIO_InitStructure;
  //TX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //RX
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU | GPIO_Mode_IPD;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* USART2 pin setting */
  GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);
  //TX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  //RX
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU | GPIO_Mode_IPD;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  // Bluetooth isConnect
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  
  //GPIO Keypad
  GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // Low(0) When Pressed
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  // output
  GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_11 | GPIO_Pin_4);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    
  //GPIO LED
  GPIO_InitTypeDef GPIO_InitStructure2;
  GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOD, &GPIO_InitStructure2);
  
  
  //GPIO Button
  GPIO_InitTypeDef GPIO_InitStructure3;
  GPIO_InitStructure3.GPIO_Pin =  GPIO_Pin_4;
  GPIO_InitStructure3.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure3.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &GPIO_InitStructure3);

  GPIO_InitStructure3.GPIO_Pin =  GPIO_Pin_10;
  GPIO_InitStructure3.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure3.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOD, &GPIO_InitStructure3);
  
  
  //GPIO Human_Sensor
  GPIO_InitTypeDef GPIO_InitStructure4;
  GPIO_InitStructure4.GPIO_Pin =  GPIO_Pin_2;
  GPIO_InitStructure4.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure4.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOD, &GPIO_InitStructure4);
  
  
  //GPIO Magnetic
  GPIO_InitTypeDef GPIO_InitStructure5;
  GPIO_InitStructure5.GPIO_Pin =  GPIO_Pin_1;
  GPIO_InitStructure5.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure5.GPIO_Mode = GPIO_Mode_IPU;    // analog in
  GPIO_Init(GPIOB, &GPIO_InitStructure5);
  
  
  //GPIO Servo Motor
  GPIO_InitTypeDef GPIO_InitStructure6;
  GPIO_InitStructure6.GPIO_Pin = GPIO_Pin_0; // TIM3 Pin
  GPIO_InitStructure6.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure6.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure6);
  
  //GPIO Piezo Buzzer
  GPIO_InitTypeDef GPIO_InitStructure7;
  GPIO_InitStructure7.GPIO_Pin = BUZZER_PIN;
  GPIO_InitStructure7.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure7.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(BUZZER_PORT, &GPIO_InitStructure7);
}

void EXTI_Configure(void){
    EXTI_InitTypeDef EXTI_InitStructure;

    // Manual Open Button
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}

// Bluetooth
/* USART_Init */
void USART1_Init(void)
{
    USART_InitTypeDef USART1_InitStructure;

    // Enable the USART1 peripheral
    USART_Cmd(USART1, ENABLE);

    // TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
    USART1_InitStructure.USART_BaudRate = 9600;
    USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART1_InitStructure.USART_StopBits = USART_StopBits_1;
    USART1_InitStructure.USART_Parity = USART_Parity_No;
    USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init(USART1, &USART1_InitStructure);
    // TODO: Enable the USART1 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

}

void USART2_Init(void)
{
    USART_InitTypeDef USART2_InitStructure;

    // Enable the USART2 peripheral
    USART_Cmd(USART2, ENABLE);

    // TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
    USART2_InitStructure.USART_BaudRate = 9600;
    USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART2_InitStructure.USART_StopBits = USART_StopBits_1;
    USART2_InitStructure.USART_Parity = USART_Parity_No;
    USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init(USART2, &USART2_InitStructure);

    // TODO: Enable the USART2 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

}

/* NVIC */
void NVIC_Configure(void){
  NVIC_InitTypeDef NVIC_InitStructure;
	
  // TODO: fill the arg you want
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  // USART1
  // 'NVIC_EnableIRQ' is only required for USART setting
  NVIC_EnableIRQ(USART1_IRQn);
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0; // TODO
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // TODO
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // USART2
  // 'NVIC_EnableIRQ' is only required for USART setting
  NVIC_EnableIRQ(USART2_IRQn);
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // TODO
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // TODO
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // TIM4 Interrupt - Magnetic, Human Sensor, BluetoothUser
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  // TIM4
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // Manual Open Button
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;  //
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void USART1_IRQHandler() {
    uint16_t word;
    if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET){
        // the most recent received data by the USART1 peripheral
        word = USART_ReceiveData(USART1);

        // TODO implement
        USART_SendData(USART2, word);
        // clear 'Read data register not empty' flag
        USART_ClearITPendingBit(USART1,USART_IT_RXNE);
    }
}

void USART2_IRQHandler() {
    uint16_t word;
    if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET){
        // the most recent received data by the USART2 peripheral
        word = USART_ReceiveData(USART2);

        // TODO implement
        USART_SendData(USART1, word);
        // clear 'Read data register not empty' flag
        USART_ClearITPendingBit(USART2,USART_IT_RXNE);
    }
}

void TIM4_IRQHandler(){
     if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
         if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2) == Bit_RESET) {
             motion = true;
         }
         else if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2) == Bit_SET){
             motion = false;
         }

         if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == Bit_SET){
             char LCD_output2[5] = {'O', 'P', 'E', 'N', '\0'};
             LCD_ShowString(10, 130, LCD_output2, BLACK, WHITE);
             prevDoorClosed = curDoorClosed;
             curDoorClosed = false;
         }
         else if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == Bit_RESET) {
             char LCD_output2[5] = {'C', 'L', 'O', 'S', 'E'};
             LCD_ShowString(10, 130, LCD_output2, BLACK, WHITE);
             prevDoorClosed = curDoorClosed;
             curDoorClosed = true;
         }

         if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4) == Bit_RESET){
             bluetoothUser = true;
         }
         else if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4) == Bit_SET){
             bluetoothUser = false;
         }
     }
         TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
}

// Manual Open Button
void EXTI4_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line4) != RESET) {
        if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4)== Bit_RESET) {

            // when door locked
            if (pulse == 500){
                openDoor();
            }

            // when door unlocked
            else if (pulse == 2300){
                closeDoor();
            }

            delay(100);
        }
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}

// Human Sensor
void Human_Configure(void){
    prescale = (uint16_t) (SystemCoreClock / 10000);
    TIM_TimeBaseStructure.TIM_Period = 10000;
    TIM_TimeBaseStructure.TIM_Prescaler = prescale;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

/* PWM_Configure close */
/*void PWM_Configure1() {
  prescale = (uint16_t) (SystemCoreClock / 10000);
  TIM_TimeBaseStructure.TIM_Period = 10000;
  TIM_TimeBaseStructure.TIM_Prescaler = prescale;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_ARRPreloadConfig(TIM2, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
}*/

void PWM_Configure2() {
  prescale = (uint16_t) (SystemCoreClock / 1000000);
  TIM_TimeBaseStructure.TIM_Period = 20000;
  TIM_TimeBaseStructure.TIM_Prescaler = prescale;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = pulse; // us
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
}


/*  Keypad */
void getKeyPressed() {

  GPIO_ResetBits(GPIOA, GPIO_Pin_4);
  GPIO_SetBits(GPIOA, GPIO_Pin_11);
  GPIO_SetBits(GPIOA, GPIO_Pin_2);
  GPIO_SetBits(GPIOA, GPIO_Pin_1);
  while (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7)) {
    value = '#';
    delay(5);
  }
  while (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)) {
    value = 0;
    delay(5);
  }

  while (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5)) {
    value = '*';
    delay(5);
  }

  GPIO_SetBits(GPIOA, GPIO_Pin_4);
  GPIO_ResetBits(GPIOA, GPIO_Pin_11);
  GPIO_SetBits(GPIOA, GPIO_Pin_2);
  GPIO_SetBits(GPIOA, GPIO_Pin_1);
  while (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7)) {
    value = 9;
    delay(5);
  }
  
  while (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)) {
    value = 8;
    delay(5);
  }
  while (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5)) {
    value = 7;
    delay(5);
  }

  GPIO_SetBits(GPIOA, GPIO_Pin_4);
  GPIO_SetBits(GPIOA, GPIO_Pin_11);
  GPIO_ResetBits(GPIOA, GPIO_Pin_2);
  GPIO_SetBits(GPIOA, GPIO_Pin_1);
  while (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7)) {
    value = 6;
    delay(5);
  }
  while (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)) {
    value = 5;
    delay(5);
  }
  while (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5)) {
    value = 4;
    delay(5);
  }

  GPIO_SetBits(GPIOA, GPIO_Pin_4);
  GPIO_SetBits(GPIOA, GPIO_Pin_11);
  GPIO_SetBits(GPIOA, GPIO_Pin_2);
  GPIO_ResetBits(GPIOA, GPIO_Pin_1);
  while (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7)) {
    value = 3;
    delay(5);
  }
  while (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)) {
    value = 2;
    delay(5);
  }
  while (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5)) {
    value = 1;
    delay(5);
  }

  if (value >= 0 && value < 10) {
    password_index = (password_index + 1) % 4;
    keypad_input[password_index] = value;
  }

    for (i = 0; i<4 ; i++) {
        if(keypad_input[i] == -1)
            LCD_output[i]= '\0';
        else {
            LCD_output[i] = keypad_input[i] + 48;
        }
    }

    LCD_ShowString(10, 100, LCD_output, BLACK, WHITE);

}


void Check_Password() {

  if (value == '*') {
    password_index = -1;
    for (i = 0; i < 4; i++) {
      // Incorrect Case
      if (keypad_input[i] != password[i]) {
        // Reset input data
        for (j = 0; j < 4; j++) {
          keypad_input[j] = -1;
        }
        IncorrectPasswordSound();
        return;
      }
    }

    // Correct Case
    for (j = 0; j < 4; j++) {
       keypad_input[j] = -1;
    }
    openDoor();
  }
}

void beep(){
    GPIO_ResetBits(GPIOA, GPIO_Pin_3);
    delay(1000000);
    GPIO_SetBits(GPIOA, GPIO_Pin_3);
    delay(1000000);
}

void openDoor(){
    pulse = 2300;
    // Servo Control
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = pulse; // us
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);

    isOpen = true;
    beep();
}

void closeDoor(){
    pulse = 500;
    // Servo Control
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = pulse ; // us
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);

    isOpen = false;
    beep();
    beep();
}

void IncorrectPasswordSound(void) {
    beep();
    beep();
    beep();
}

/* Delay */
void delay(int time)
{
    for(int k=0; k<time; k++){
    }
}

void changePW(){
    for (j = 0; j < 4; j++) {
        keypad_input[j] = -1;
    }
    int buf_index = -1;

    while(1){
        value = -1;

        getKeyPressed();

        if (value >= 0 && value < 10){
            buf_index = (buf_index + 1) % 4;
            password[buf_index] = value;
        }

        if (value == '*'){
            break;
        }
    }

    for (j = 0; j < 4; j++) {
        keypad_input[j] = -1;
    }
}

// implement main function
int main(void){
  SystemInit();
  RCC_Configure();
  GPIO_Configure();
  Human_Configure();
  USART1_Init();
  USART2_Init();
  EXTI_Configure();
  NVIC_Configure();

  PWM_Configure2();
  
  LCD_Init();

  LCD_Clear(WHITE);

  //LED_Control(0);
  while (1) {
      GPIO_SetBits(BUZZER_PORT, BUZZER_PIN);
      TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
      LCD_ShowNum(10, 150, timePassed, 5, BLACK, WHITE);
      value = -1;
      // Door close case
      if (curDoorClosed == true && motion == true) {
          // check human
          if (bluetoothUser == true && isOpen == false) {
              openDoor();
          } else {
              getKeyPressed();
              Check_Password();
          }
      }

      if (prevDoorClosed == false && curDoorClosed == true) {
          delay(20000000);
          closeDoor();
      }

      else if (prevDoorClosed == true && curDoorClosed == true && isOpen == true){
          if (bluetoothUser == false){
              timePassed += 1;
              if (timePassed == 3000){
                  closeDoor();
                  timePassed = 0;
              }
          }
          else if (bluetoothUser == true && motion == false){
              timePassed += 1;
              if (timePassed == 1000){
                  closeDoor();
                  timePassed = 0;
              }
          }
      }

      if (curDoorClosed == false){
          char LCD_changePW[5] = {'C', 'H', 'A', 'N', 'G'};
          LCD_ShowString(10, 170, LCD_changePW, BLACK, WHITE);
          getKeyPressed();
          if (value == '#'){
              changePW();
          }
          char LCD_ChangeEnd[5] = {'A', 'A', 'A', 'A', 'A'};
          LCD_ShowString(10, 190, LCD_ChangeEnd, BLACK, WHITE);
      }

  }
    
    
  return 0;
}