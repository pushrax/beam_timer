#include "application.h"
#include "stm32_it.h"
#include "tractortimer.h"

uint16_t display_serial = D0, display_rclk = D1, display_srclk = D2,
         exit_in = D3, up_in = D4, set_in = D5;

/*

# micro socket pinout

       - 0    19 - NC
       - 1    18 -
   SER - 2    17 -
  RCLK - 3    16 -
 SRCLK - 4
       - 5
       - 6
       - 7
   Vcc - 8    15 -
  EXIT - 9    14 -
    UP - 10   13 - GND
   SET - 11   12 - NC

# segment bits

   -4-
  1   5
  |   |
   -3-
  0   6
  |   |
   -2-

# misc bits

 (todo)

*/

const uint16_t SCC_prescaler_us = (uint16_t)(SystemCoreClock / 1000000) - 1;

inline void pinHigh(uint16_t pin) {
  PIN_MAP[pin].gpio_peripheral->BSRR = PIN_MAP[pin].gpio_pin;
}

inline void pinLow(uint16_t pin) {
  PIN_MAP[pin].gpio_peripheral->BRR = PIN_MAP[pin].gpio_pin;
}


uint8_t display_segment_byte_mapping[] = {
  0b01110111,
  0b01100000,
  0b00111101,
  0b01111100,
  0b01101010,
  0b01011110,
  0b01011111,
  0b01110000,
  0b01111111,
  0b01111110,
  0,

  0b00011011,
  0b01111011,
  0b00000011,
  0b00000111
};

inline void write_segments(uint8_t segments) {
  for (int i = 0; i < 8; i++) {
    pinLow(display_srclk);
    if (segments & 1) pinLow(display_serial);
    else pinHigh(display_serial);
    segments >>= 1;
    pinHigh(display_srclk);
  }
}

inline void write_digit(int n) {
  write_segments(display_segment_byte_mapping[n]);
}

uint16_t centiseconds = 0, seconds = 0, minutes = 0;

extern "C" void TIM4_irq_handler(void)
{
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

    centiseconds++;
    if (centiseconds >= 100) {
      centiseconds = 0;
      seconds++;
    }
    if (seconds >= 60) {
      seconds = 0;
      minutes++;
    }

    pinLow(display_rclk);
    write_digit(minutes / 10);
    write_digit(minutes % 10);
    write_digit(seconds / 10);
    write_digit(seconds % 10);
    write_digit(centiseconds / 10);
    write_digit(centiseconds % 10);
    pinHigh(display_rclk);
  }

  // if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)
  // {
  //   TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
  // }
}

void loop()
{
  // Counter
  // for (int i = 0; i <= 199999; i++) {
  //   if (digitalRead(exit_in) == LOW) break;
  //   if (digitalRead(set_in) == HIGH) i = 0;
  //   if (digitalRead(up_in) == HIGH) i = 100000;
  //   pinLow(display_rclk);
  //   int t = i;
  //   for (int k = 0; k < 6; k++) {
  //     int d = t / 100000;
  //     if (k == 0 && d == 0) write_digit(10);
  //     else write_digit(d);
  //     t -= d * 100000;
  //     t *= 10;
  //   }
  //   pinHigh(display_rclk);
  //   delay(10);
  // }

  // FAIL
  // for (int i = 0; i < 6; i++) {
  //   pinLow(display_rclk);
  //   write_digit(10);
  //   write_digit2(i < 2 ? i : i == 5 ? 2 : i + 2);
  //   write_digit(11);
  //   write_digit(12);
  //   write_digit(13);
  //   write_digit(14);
  //   pinHigh(display_rclk);
  //   delay(50);
  // }
}

void setup()
{
  Wiring_TIM4_Interrupt_Handler = TIM4_irq_handler;

  pinMode(display_rclk, OUTPUT);
  pinMode(display_srclk, OUTPUT);
  pinMode(display_serial, OUTPUT);
  pinMode(exit_in, INPUT);
  pinMode(up_in, INPUT);
  pinMode(set_in, INPUT);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  TIM_TimeBaseInitTypeDef timer;
  timer.TIM_Prescaler = SCC_prescaler_us;
  timer.TIM_CounterMode = TIM_CounterMode_Up;
  timer.TIM_Period = 10000;
  timer.TIM_ClockDivision = TIM_CKD_DIV1;
  timer.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM4, &timer);

  NVIC_InitTypeDef nvic;
  nvic.NVIC_IRQChannel = TIM4_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 0;
  nvic.NVIC_IRQChannelSubPriority = 1;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);
  TIM_Cmd(TIM4, ENABLE);

  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  // TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);
}
