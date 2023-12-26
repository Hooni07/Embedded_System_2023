//main.c
#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "lcd.h"
#include "touch.h"
#include "misc.h"

/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void NVIC_Configure(void);
void TIM2_IRQHandler(void);
void PWM_Configure1(void);
void PWM_Configure2(void);
void delay(void);


int color[12] = {WHITE,CYAN,BLUE,RED,MAGENTA,LGRAY,GREEN,YELLOW,BROWN,BRRED,GRAY};
//---------------------------------------------------------------------------------------------------

int ledCount = 0;
int flag = 1; // flag = 1 (OFF), flag = 0 (ON)
int k = 1500;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
uint16_t prescale;

void RCC_Configure(void) {
  /* LCD */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); // Port C RCC ENABLE
  /* PWM */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // TIM2
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // Port B RCC ENABLE
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // TIM3
  /* LED */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); // Port D RCC ENABLE
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

void GPIO_Configure(void) {
  GPIO_InitTypeDef GPIO_InitStructure3;
  GPIO_InitStructure3.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // LED1, LED2 ENABLE
  GPIO_InitStructure3.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure3.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure3);

  /* PWM */
  GPIO_InitTypeDef GPIO_InitStructure2;
  GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_0; // TIM3 Pin
  GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure2);
}

void NVIC_Configure(void) {
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitTypeDef NVIC_InitStructure;
  /* Enable TIM2 Global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void TIM2_IRQHandler(void) {
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
    /* Mode ON */
    if (flag == 0){
      if (ledCount % 2 == 0) {
        GPIO_SetBits(GPIOD, GPIO_Pin_2);
      }
      else {
        GPIO_ResetBits(GPIOD, GPIO_Pin_2); 
      }

      if(ledCount == 0) {
        GPIO_ResetBits(GPIOD, GPIO_Pin_3);
      }
      else if(ledCount == 5) {
        GPIO_SetBits(GPIOD, GPIO_Pin_3);
      }

      ledCount++;
      ledCount %= 10;
      k += 100;
      if (k == 2300) {
        k = 700;
      }
          
      TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
      TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
      TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
      TIM_OCInitStructure.TIM_Pulse = k ; // us
      TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    }
    
    /* Mode OFF */
    else {
      k -= 100;
      if (k == 700){
        k = 2300;
      }
          
      TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
      TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
      TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
      TIM_OCInitStructure.TIM_Pulse = k ; // us
      TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }
}

void PWM_Configure1() {
  prescale = (uint16_t) (SystemCoreClock / 10000);
  TIM_TimeBaseStructure.TIM_Period = 10000;
  TIM_TimeBaseStructure.TIM_Prescaler = prescale;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_ARRPreloadConfig(TIM2, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
}

void PWM_Configure2() {
  prescale = (uint16_t) (SystemCoreClock / 1000000);
  TIM_TimeBaseStructure.TIM_Period = 20000;
  TIM_TimeBaseStructure.TIM_Prescaler = prescale;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 1500; // us
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
}

void delay() {
  for(int j = 0; j < 5000000; j++) {
    continue;
  }
}

int main(void) {
  SystemInit();
  RCC_Configure();
  GPIO_Configure();
  PWM_Configure1();
  PWM_Configure2();
  NVIC_Configure();

  LCD_Init();
  Touch_Configuration();
  Touch_Adjust();

  LCD_Clear(WHITE);
  uint16_t pos_temp[2];
 
  LCD_ShowString(40, 10, "THU_Team08", MAGENTA, WHITE); // 팀명 출력
  LCD_ShowString(90, 50, "OFF", RED, WHITE); // 디폴트로 OFF 
  LCD_DrawRectangle(40, 80, 80, 120); // 사각형 출력
  LCD_ShowString(50,90, "BTN", MAGENTA, WHITE); // "Button" 글자 출력

  while (1) {
    Touch_GetXY(&pos_temp[0], &pos_temp[1], 1); // 터치 좌표 받아서 배열에 입력
    Convert_Pos(pos_temp[0],pos_temp[1],&pos_temp[0],&pos_temp[1]); // 받은 좌표를 LCD 크기에 맞게 변환
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); // interrupt ENABLE

    /* If Button Touch event Occured*/
    if ((uint16_t)40 < pos_temp[0] && pos_temp[0] < (uint16_t)80 && \
        (uint16_t)80 < pos_temp[1] && pos_temp[1] < (uint16_t)120) {
      /* If current Mode is OFF Change Mode to ON */
      if (flag) {
        LCD_ShowString(90, 50, "    ", RED, WHITE); 
        LCD_ShowString(40, 50, "ON", RED, WHITE);
      }

      /* If current Mode is ON Change Mode to OFF */
      else {
        LCD_ShowString(90, 50, "OFF", RED, WHITE); 
        LCD_ShowString(40, 50, "    ", RED, WHITE); 
      }

      flag++;
      flag %= 2;
    }
    delay();
  }
  return 0;
  
}