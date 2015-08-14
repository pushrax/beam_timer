#include "application.h"
#include "stm32_it.h"
#include "beam_timer.h"

SYSTEM_MODE(SEMI_AUTOMATIC);

uint16_t display_serial = D0, display_rclk = D1, display_srclk = D2,
         exit_in = D3, up_in = D6, set_in = D5,
         recv0_out = A0, recv0_in = A1, recv1_out = A4, recv1_in = A5;

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

# receivers

order: orange (A0), green (A4), blue (A6)

*/

const uint16_t SCC_prescaler_us = (uint16_t)(SystemCoreClock / 1000000) - 1;

inline void pinHigh(uint16_t pin)
{
    PIN_MAP[pin].gpio_peripheral->BSRR = PIN_MAP[pin].gpio_pin;
}

inline void pinLow(uint16_t pin)
{
    PIN_MAP[pin].gpio_peripheral->BRR = PIN_MAP[pin].gpio_pin;
}

const uint8_t display_segment_byte_mapping[] =
{
    0b01110111, // 0
    0b01100000, // 1
    0b00111101, // 2
    0b01111100, // 3
    0b01101010, // 4
    0b01011110, // 5
    0b01011111, // 6
    0b01110000, // 7
    0b01111111, // 8
    0b01111110, // 9
    0,

    0b01111011, // A
    0b01001111, // b
    0b00010111, // C
    0b01101101, // d
    0b00011111, // E
    0b00011011, // F
    0b01010111, // G
    0b01101011, // H
    0b00000011, // I
    0b01100101, // J
    0b00001111, // k
    0b00000111, // L
    0b01110011, // M
    0b01001001, // n
    0b01110111, // O
    0b00111011, // P
    0b01111010, // q
    0b00001001, // r
    0b01011110, // S
    0b00010011, // t
    0b01100111, // U
    0b01100110, // V
    0b00100110, // W
    0b00101001, // X
    0b01101010, // Y
    0b00111101  // Z
};

enum States
{
    STOPPED,
    WAITING,
    TIMING
};

enum Modes
{
    CLOCK,
    RACE,
    DEBUG1,
    DEBUG2,
    TEXT
};

uint16_t centiseconds = 0, seconds = 0, minutes = 0;
uint16_t last_centiseconds = 0, last_seconds = 0, last_minutes = 0;
int state = STOPPED, mode = TEXT, display_last = 0, samples = 0;

String text = "Connecting";

uint8_t extraLeds = 0;
uint8_t amLed = 1, pmLed = 1 << 1, colonLed = 1 << 3, dotLed = 1 << 4;

int setText(String t) {
    text = t;
    return 0;
}

inline void write_segments(uint8_t index, uint8_t segments)
{
    for (int i = 0; i < 7; i++)
    {
        pinLow(display_srclk);
        if (segments & 1) pinLow(display_serial);
        else pinHigh(display_serial);
        segments >>= 1;
        pinHigh(display_srclk);
    }
    pinLow(display_srclk);
    if (extraLeds & (1 << index)) pinLow(display_serial);
    else pinHigh(display_serial);
    pinHigh(display_srclk);
}

inline void write_digit(uint8_t index, int n)
{
    write_segments(index, display_segment_byte_mapping[n]);
}

void increment_cs()
{
    centiseconds++;
    if (centiseconds >= 100)
    {
        centiseconds = 0;
        seconds++;
    }
    if (seconds >= 60)
    {
        seconds = 0;
        minutes++;
    }
}

void write_time(uint16_t centiseconds, uint16_t seconds, uint16_t minutes)
{
    if (mode == DEBUG1 || mode == DEBUG2) return;
    pinLow(display_rclk);
    write_digit(0, minutes < 10 ? 10 : minutes / 10);
    write_digit(1, minutes % 10);
    write_digit(2, seconds / 10);
    write_digit(3, seconds % 10);
    write_digit(4, centiseconds / 10);
    write_digit(5, centiseconds % 10);
    pinHigh(display_rclk);
}

String get_time_string(uint16_t centiseconds, uint16_t seconds, uint16_t minutes)
{
    String output = String(minutes, DEC) + ":";
    output += String(seconds / 10, DEC) + String(seconds % 10, DEC) + ".";
    output += String(centiseconds / 10, DEC) + String(centiseconds % 10, DEC);
    return output;
}

void write_number(int t)
{
    pinLow(display_rclk);
    for (int k = 0; k < 6; k++)
    {
        int d = t / 100000;
        if (k == 0 && d == 0) write_digit(k, 10);
        else write_digit(k, d);
        t -= d * 100000;
        t *= 10;
    }
    pinHigh(display_rclk);
}

void write_string(int start, const char* str)
{
    pinLow(display_rclk);
    bool done = false;
    for (int i = 0; i < start; i++)
    {
        write_segments(i, 0);
    }
    for (int i = start; i < 6; i++)
    {
        if (i < 0) i = 0;
        int k = i - start;
        if (str[k] == 0) done = true;
        if (done || i == 0)
        {
            write_segments(i, 0);
            continue;
        }
        if (str[k] >= 'A' && str[k] <= 'Z')
        {
            write_segments(i, display_segment_byte_mapping[str[k] - 'A' + 11]);
        }
        else if (str[k] >= 'a' && str[k] <= 'z')
        {
            write_segments(i, display_segment_byte_mapping[str[k] - 'a' + 11]);
        }
        else if (str[k] >= '0' && str[k] <= '9')
        {
            write_segments(i, display_segment_byte_mapping[str[k] - '0']);
        }
        else if (str[k] == '-')
        {
            write_segments(i, 1 << 3);
        }
        else if (str[k] == '_')
        {
            write_segments(i, 1 << 2);
        }
        else
        {
            write_segments(i, 0);
        }
    }
    pinHigh(display_rclk);
}

int poll_receiver(int pin)
{
    int val = analogRead(pin);
    if (mode == DEBUG1 || mode == DEBUG2) write_number(val);
    return val >= 800;
}

void poll_receivers()
{
    if (mode == DEBUG1)
    {
        poll_receiver(recv0_in);
        return;
    }
    else if (mode == DEBUG2)
    {
        poll_receiver(recv1_in);
        return;
    }
    else if (mode == RACE)
    {
        if (state == STOPPED) return;

        if (state == TIMING && display_last == 0 && !poll_receiver(recv1_in))
        {
            last_minutes = minutes;
            last_seconds = seconds;
            last_centiseconds = centiseconds;
            display_last = 200;
        }
        else if (state == WAITING && !poll_receiver(recv0_in))
        {
            state = TIMING;
            samples = 0;
        }
    }
}

extern "C" void TIM4_irq_handler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        if (mode == RACE && state == TIMING)
        {
            increment_cs();
        }

        if (mode == RACE && display_last)
        {
            if (display_last > 0) display_last--;
            write_time(last_centiseconds, last_seconds, last_minutes);
        }
        else if (mode == TEXT)
        {
            if (text.length() <= 5)
            {
                write_string(1, text.c_str());
            }
            else
            {
                display_last--;
                write_string(display_last / 40 + 6, text.c_str());
                if (display_last < ((text.length() + 6) * -40))
                {
                    display_last = 0;
                }
            }
        }
        else
        {
            write_time(centiseconds, seconds, minutes);
        }
    }
}

extern "C" void TIM3_irq_handler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        if (mode == RACE || mode == DEBUG1 || mode == DEBUG2) {
            pinHigh(recv0_out);
            pinHigh(recv1_out);
            poll_receivers();
        } else {
            pinLow(recv0_out);
            pinLow(recv1_out);
        }
    }
}

void finalize_time()
{
    state = STOPPED;
    display_last = -1;
    if (last_centiseconds > 0 || last_seconds > 0 || last_minutes > 0)
    {
        String message = "Lap completed: " + get_time_string(last_centiseconds, last_seconds, last_minutes);
        Spark.publish("slack-message", message, 60, PRIVATE);
    }
}

unsigned long debounce = 0;

void check_buttons()
{
    if (debounce > millis()) return;
    if (mode == RACE)
    {
        // Button C
        if (digitalRead(set_in) == HIGH)
        {
            finalize_time();
            debounce = millis() + 1000;
        }
        if (digitalRead(up_in) == HIGH) // Button B
        {
            centiseconds = seconds = minutes = 0;
            last_centiseconds = last_seconds = last_minutes = 0;
            state = WAITING;
            samples = 0;
            display_last = 0;
        }
    }
    if (digitalRead(exit_in) == LOW) // Button A
    {
        mode++;
        if (mode > TEXT) mode = CLOCK;
        if (mode == RACE)
        {
            centiseconds = seconds = minutes = 0;
            state = STOPPED;
            samples = 0;
            display_last = 0;
        }
        debounce = millis() + 250;
    }
}

unsigned long nextSync = 0;
bool shouldConnect = true;
bool connected = false;

void loop()
{
    if (shouldConnect)
    {
        shouldConnect = false;
        Spark.connect();
        return;
    }
    if (Spark.connected() && !connected)
    {
        connected = true;

        text = "Shopify";
        if (mode == TEXT) mode = CLOCK;
        Time.zone(-4);
        Spark.function("text", setText);
    }
    if (mode == RACE)
    {
        extraLeds = colonLed;
    }
    else if (mode == CLOCK)
    {
        minutes = Time.hourFormat12();
        seconds = Time.minute();
        centiseconds = Time.second();
        if (Time.isAM()) {
            extraLeds |= amLed;
            extraLeds &= ~pmLed;
        } else {
            extraLeds &= ~amLed;
            extraLeds |= pmLed;
        }
        if (centiseconds & 1) {
            extraLeds |= colonLed | dotLed;
        } else {
            extraLeds &= ~(colonLed | dotLed);
        }

        if (nextSync < millis() && Spark.connected())
        {
            Spark.syncTime();
            nextSync = millis() + (60 * 60 * 1000); // 1 hour
        }
    }
    else if (mode == DEBUG1)
    {
        extraLeds = amLed;
    }
    else if (mode == DEBUG2)
    {
        extraLeds = pmLed;
    }
    else
    {
        extraLeds = 0;
    }
    if (mode != RACE && mode != DEBUG1 && mode != DEBUG2) {
        pinLow(recv0_out);
        pinLow(recv1_out);
    }

    check_buttons();
}

void setup()
{
    Wiring_TIM3_Interrupt_Handler = TIM3_irq_handler;
    Wiring_TIM4_Interrupt_Handler = TIM4_irq_handler;

    pinMode(display_rclk, OUTPUT);
    pinMode(display_srclk, OUTPUT);
    pinMode(display_serial, OUTPUT);
    pinMode(exit_in, INPUT);
    pinMode(up_in, INPUT);
    pinMode(set_in, INPUT);

    pinMode(recv0_out, OUTPUT);
    pinMode(recv0_in, INPUT);
    pinMode(recv1_out, OUTPUT);
    pinMode(recv1_in, INPUT);
    pinLow(recv0_out);
    pinLow(recv1_out);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef poll_timer;
    poll_timer.TIM_Prescaler = SCC_prescaler_us;
    poll_timer.TIM_CounterMode = TIM_CounterMode_Up;
    poll_timer.TIM_Period = 1000;
    poll_timer.TIM_ClockDivision = TIM_CKD_DIV1;
    poll_timer.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &poll_timer);

    TIM_TimeBaseInitTypeDef timer;
    timer.TIM_Prescaler = SCC_prescaler_us;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    timer.TIM_Period = 10000;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    timer.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &timer);

    NVIC_InitTypeDef poll_nvic;
    poll_nvic.NVIC_IRQChannel = TIM3_IRQn;
    poll_nvic.NVIC_IRQChannelPreemptionPriority = 0;
    poll_nvic.NVIC_IRQChannelSubPriority = 1;
    poll_nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&poll_nvic);
    TIM_Cmd(TIM3, ENABLE);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = TIM4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    TIM_Cmd(TIM4, ENABLE);

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    // TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);
}
