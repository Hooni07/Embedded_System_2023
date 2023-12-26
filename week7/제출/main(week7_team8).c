
#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"

#include "misc.h"

/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void EXTI_Configure(void);
void USART1_Init(void);
void NVIC_Configure(void);

void EXTI2_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

void Delay(void);

void sendDataUART1(uint16_t data);
void setDirection(char direction);

char mode = 'a';
char prev_mode = 'a';
int led_idx = 0;
int index =  0;
char msg[] = "TEAM08\r\n";
//---------------------------------------------------------------------------------------------------

void RCC_Configure(void)
{
   // TODO: Enable the APB2 peripheral clock using the function 'RCC_APB2PeriphClockCmd'
   
   /* UART TX/RX port clock enable */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
   /* Button S1, S2, S3 port clock enable */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
   /* LED port clock enable */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
   /* USART1 clock enable */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);   
   /* Alternate Function IO clock enable */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

   // TODO: Initialize the GPIO pins using the structure 'GPIO_InitTypeDef' and the function 'GPIO_Init'
   
    

    /* Button KEY1, KEY2, KEY3 pin setting */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU | GPIO_Mode_IPD;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU | GPIO_Mode_IPD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* LED pin setting*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
   
    /* UART pin setting */
    //TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // PUSH PULL 이었을 때, 통신이 안됨
    GPIO_Init(GPIOA, &GPIO_InitStructure);
   //RX
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU | GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void EXTI_Configure(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;

   // TODO: Select the GPIO pin (Joystick, button) used as EXTI Line using function 'GPIO_EXTILineConfig'
   // TODO: Initialize the EXTI using the structure 'EXTI_InitTypeDef' and the function 'EXTI_Init'
   
    /* Button 1(PC4) is pressed */
   GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Button 2 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10);
    EXTI_InitStructure.EXTI_Line = EXTI_Line10;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
     
    
   /* Button 3 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);
    EXTI_InitStructure.EXTI_Line = EXTI_Line13;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
   // NOTE: do not select the UART GPIO pin used as EXTI Line here
}

void USART1_Init(void)
{
   USART_InitTypeDef USART1_InitStructure;

   // Enable the USART1 peripheral
   USART_Cmd(USART1, ENABLE);
   
   // TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
   USART1_InitStructure.USART_BaudRate = 9600;
    USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART1_InitStructure.USART_Parity = USART_Parity_No;
    USART1_InitStructure.USART_StopBits = USART_StopBits_1;
    USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART1_InitStructure);
   
   // TODO: Enable the USART1 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
   USART_ITConfig(USART1 , USART_IT_RXNE , ENABLE);
}

void NVIC_Configure(void) {

    NVIC_InitTypeDef NVIC_InitStructure;
    
    // TODO: fill the arg you want
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

   // TODO: Initialize the NVIC using the structure 'NVIC_InitTypeDef' and the function 'NVIC_Init'
    //Button S1
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    // Button S2 , ButtonS3
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // UART1
   
   // 'NVIC_EnableIRQ' is only required for USART setting
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void USART1_IRQHandler() {
   uint16_t word;
    if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET){
       // the most recent received data by the USART1 peripheral
        word = USART_ReceiveData(USART1);

        // TODO implement
        if (word == 'a') {
            // A 동작
            setDirection('a');
        } else if (word == 'b') {
            // B 동작
            setDirection('b');
        }
        // clear 'Read data register not empty' flag
       USART_ClearITPendingBit(USART1,USART_IT_RXNE);
    }
}

void EXTI4_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line4) != RESET) {
        if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4)== Bit_RESET) {
     
        //mode = 0;
        setDirection('a');
        
        }
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}

void EXTI9_5_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line10) != RESET) {
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) == Bit_RESET) {
      
        //mode=1;
        setDirection('b');
        
        }
        EXTI_ClearITPendingBit(EXTI_Line10);
    }
}


void EXTI15_10_IRQHandler(void) { // when the button is pressed
    if (EXTI_GetITStatus(EXTI_Line10) != RESET) {
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) == Bit_RESET) {
      
        //mode=1;
        setDirection('b');
        
        }
        EXTI_ClearITPendingBit(EXTI_Line10);
    }
    
    if (EXTI_GetITStatus(EXTI_Line13) != RESET) {
        if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == Bit_RESET) {
         // TODO implement
       
            char *tmp = &msg[0];
            while (*tmp !='\0') {
                sendDataUART1(*tmp);
                tmp++;
            }
            
         EXTI_ClearITPendingBit(EXTI_Line13);
        }
    }
}

// TODO: Create Joystick interrupt handler functions


void Delay(void) {
   int i;

   for (i = 0; i < 2000000; i++) {}
}

void sendDataUART1(uint16_t data) {
   /* Wait till TC is set */
   while ((USART1->SR & USART_SR_TC) == 0);
   USART_SendData(USART1, data);
}

void setDirection(char direction) {
  if (direction == 'a'){
    mode = 'a';
    
  }
  else if (direction == 'b'){
    mode = 'b';
  }
}

void ledIndex(void) {
    if(mode == 'a'){
        led_idx++;
    }
    
    else if(mode == 'b') {
        if(led_idx == 0){
            led_idx = 3;
        }
        else{
            led_idx--;
        }
    }
}


int main(void)
{

    SystemInit();

    RCC_Configure();

    GPIO_Configure();

    EXTI_Configure();

    USART1_Init();

    NVIC_Configure();
  
    unsigned led_array[4] = {
      GPIO_Pin_2,
      GPIO_Pin_3,
      GPIO_Pin_4,
      GPIO_Pin_7,
    };
    
    
    
    while (1) {
       // TODO: implement 
        
        index = led_idx % 4;
        if (GPIO_ReadOutputDataBit(GPIOD,led_array[index])==0){
          GPIO_SetBits(GPIOD, led_array[index]);
        }
        else {
          GPIO_ResetBits(GPIOD, led_array[index]);
        }
        
        
        if (prev_mode != mode) {
              prev_mode = mode;
        }
        
        else{
            ledIndex();
        }

       // Delay
       Delay();
       
    }
    return 0;
}