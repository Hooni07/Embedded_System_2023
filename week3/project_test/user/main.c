#include "stm32f10x.h"

#define RCC_APB2_ENR *(volatile unsigned int *)0x40021018

#define GPIOA_CRL *(volatile unsigned int *)0x40010800
#define GPIOA_BSRR *(volatile unsigned int *)0x40010810
#define GPIOA_BRR *(volatile unsigned int *)0x40010814
#define GPIOA_IDR *(volatile unsigned int *)0x40010808

#define GPIOB_CRH *(volatile unsigned int *)0x40010C04
#define GPIOB_BSRR *(volatile unsigned int *)0x40010C10
#define GPIOB_BRR *(volatile unsigned int *)0x40010C14
#define GPIOB_IDR *(volatile unsigned int *)0x40010C08

#define GPIOC_CRL *(volatile unsigned int *)0x40011000
#define GPIOC_CRH *(volatile unsigned int *)0x40011004
#define GPIOC_BSRR *(volatile unsigned int *)0x40011010
#define GPIOC_BRR *(volatile unsigned int *)0x40011014
#define GPIOC_IDR *(volatile unsigned int *)0x40011008

#define GPIOD_CRL *(volatile unsigned int *)0x40011400  
#define GPIOD_BSRR *(volatile unsigned int *)0x40011410
#define GPIOD_BRR *(volatile unsigned int *)0x40011414
#define GPIOD_IDR *(volatile unsigned int *)0x40011408
#define GPIOD_ODR *(volatile unsigned int *)0x4001140C

void delay(uint32_t);

void delay(__IO uint32_t nCount){
  for(; nCount != 0; nCount--){}
}



int main(void)
{
 
  RCC_APB2_ENR = 0x3C; // RCC clock enable 정의
  
  // LED(PD2, PD3, PD4, PD7) 및 버튼(1:PC4, 2:PB10, 3:PC13, 4:PA0) 초기화
  GPIOD_CRL = 0x44444444;  // LED 초기화
  GPIOC_CRH = 0x44444444;  // Button
  GPIOC_CRL = 0x44444444;  // Button1 초기화
  GPIOB_CRH = 0x44444444;  
  GPIOA_CRL = 0x44444444;

  GPIOD_CRL = 0x10011100; // LED(PD2, PD3, PD4, PD7) 
 
  GPIOD_BSRR = 0x00000000; // all led reset
 
  int led1 = 0, led2 = 0, led3 = 0, led4 = 0; // led state
 
  while(1) {
    // press button1
    if (~GPIOC_IDR & 0x10){
      // led on
      if (led1 == 1){ 
        GPIOD_BSRR |= 0x240000;
        led1 = 0;
      }
      // led off
      else {
        GPIOD_BSRR |=  0x04;
        led1 = 1;
      }
      delay(1000000);
    }
   

    // press button2
    if (~GPIOB_IDR & 0x400) {
      // led on
      if (led2 == 1){
        
        GPIOD_BSRR |= 0x80000;
        led2 = 0;
      }
      // led off
      else {
         GPIOD_BSRR |= 0x08;
         led2 = 1;
      }
      delay(1000000);
    }
   
    // press button3
    if (~GPIOC_IDR & 0x2000) {
      // led on
      if (led3 == 1){
        GPIOD_BSRR |= 0x100000;
        led3 = 0;
      }
      // led off
      else {
         GPIOD_BSRR |=  0x10;
         led3 = 1;
      }
      delay(1000000);
    }
   
    // press button4
    if (~GPIOA_IDR & 0x01){
      // led on
      if (led4 == 1){
        GPIOD_BSRR |= 0x800000;
        led4 = 0;
      }
      // led off
      else {
         GPIOD_BSRR |=  0x80;
         led4 = 1;
      }
      delay(1000000);
    }

  }
  return 0;
 }