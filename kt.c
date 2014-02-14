// -*- coding: windows-1251 -*-

// Февраль 2014г. Константин Ильяшенко <kx13@ya.ru>

// Макросы:
// F_CPU - частота процессора
// MCU - тип микроконтроллера
// определены в Makefile

#include <avr/cpufunc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <inttypes.h>
#include <util/delay.h>

#define TIMER_FREQ 100UL // Требуемая частота таймера (Герц)
#define TIMER_PRESCALER 1 // Установить значение делителя таймера в TCCR1B (Bits 2:0)

#define MAX_TIMER  (F_CPU / TIMER_PRESCALER) / TIMER_FREQ // максимальное значение таймера в режиме CTC
#if MAX_TIMER > UINT16_MAX
// значение MAX_TIMER слишком большое, необходимо увеличить TIMER_PRESCALER.
# error "MAX_TIMER too large, need increase TIMER_PRESCALER."
#endif

#define low(x)  ((x) & 0xFF)
#define high(x) (((x)>>8) & 0xFF)

#define set_bit(port,bit)   port |= _BV(bit)
#define reset_bit(port,bit) port &= ~(_BV(bit))

#define STOP_BEEP() reset_bit(TCCR0A,COM0B0)
#define START_BEEP() set_bit(TCCR0A,COM0B0)

const uint16_t MAIN_TIMER_MAX = 60 * (TIMER_FREQ -1); // количество отчсетов таймера за 1 минуту
const uint16_t REBEEP_TIMER_MAX = 15 * TIMER_FREQ; // пауза между повторными сигналами

const uint8_t MAX_COUNT_VALUE = 99;

const uint8_t KEY_TIME = 1;    // время задержки при нажатии на клавиши (n/TIMER_FREQ)*4 (сек.)
const uint8_t SAVE_TIME = 100; // так как проверяется после 4 проходов по строкам

const uint8_t KEY_FREQ = 200; // частоты пищалки для разных типов событий
const uint8_t END_FREQ = 80;
const uint8_t SAVE_FREQ = 150;

const uint8_t SHOW_TIME = 1; // время свечения одного сегмента индикатора

enum KEYS {HASH_KEY = -1, STAR_KEY = -2};
enum NO_ACTION {NO_KEY_PRESSED = -4, NOT_USED = 5, STOP = -1};
enum BEEP_STATUS { BEEP_ON, BEEP_OFF};
enum YES_NO { NO = 0, YES = 1};
enum STATES { STATE_COUNTING, STATE_WAIT};
enum KEY_ACTIONS { ACTION_NONE, ACTION_PRESSED, ACTION_SAVE};

typedef struct beep_struct
{
  int8_t enable; // состояние вывода звука (включен или выключен)
  uint8_t position; // позиция в массиве звука
  const int8_t *sound; //выбраный массив звука
  uint8_t play; // счетчик длительности звука
  uint8_t repeat; // флаг повторного включения звука
  uint16_t pause; // пауза между повторами
} CurrentBeep;

typedef struct led_struct
{
  uint8_t first_digit;
  uint8_t second_digit;
} Led;

typedef struct key_struct
{
  int8_t pressed; // номер нажатой кнопки
  int8_t used;    // кнопка была использована
  int8_t action; // действие для нажатой кнопки
} Key;

typedef struct counter_struct
{
  uint8_t current; // значение счетчика
  uint8_t last; // предыдущее значение
  uint8_t index; // номер выбранного счетчика
  uint8_t finished; // флаг окончания счета
  uint16_t fraction; // отсчитывает доли для current
} Counter;


//Звуки для разных событий.
//Формат:
//Положительные числа задают длительность звучения, отрицательные - длительность паузы,
//Специальное значение STOP означает конец звучания.
//Время звучания определяется как n/TIMER_FREQ.
//Т.е если TIMER_FREQ=100, то при n=100 будет пищать 1 секунду
//максимальное значение числа: от -127 до +127.
//Если необходима большая длительность можно поставить несколько значений звучания подряд.
const int8_t key_beep[] = { 2, STOP};
const int8_t end_beep[] = { 55, -10,  55, -10, 55, -10, 55, -10, 55, STOP};
const int8_t save_beep[] = { 10, -10,  10, STOP};

//значения таймера по умолчанию
const uint8_t timer_preset_default[10] PROGMEM  = { 99, 10, 20, 30, 40, 50, 60, 70, 80, 90 };
//значения таймера в EEPROM по умолчанию. Загружаются в контроллер отдельной командой
uint8_t EEMEM timer_preset[10] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

const int8_t keyboard_decoder[4][3] PROGMEM = {{ 1, 2, 3}, { 4, 5, 6}, {7, 8, 9}, {STAR_KEY, 0, HASH_KEY}};
const int8_t led_digits[] PROGMEM = { 64, 121, 36, 48, 25, 18, 2, 120, 0, 16};

volatile CurrentBeep beep;
volatile Key key;
volatile Counter counter;

inline void led_set(Led *led)
{
  led->first_digit =  pgm_read_byte( &led_digits[counter.current / 10]);
  led->second_digit = pgm_read_byte( &led_digits[counter.current % 10]);
}

inline static void led_show(Led led)
{
  PORTB = led.second_digit;
  set_bit(PORTB,PINB7);
  _delay_ms(SHOW_TIME);

  PORTB =  led.first_digit;
  set_bit(PORTD,PIND6);
  _delay_ms(SHOW_TIME);
  reset_bit(PORTD,PIND6);
}

inline void start_beep(const int8_t* set_beep, uint8_t freq)
{
  beep.position = 1;
  beep.sound = set_beep;
  beep.play = beep.sound[0];
  OCR0A = freq;
  set_bit(TCCR0A,COM0B0);
  beep.enable = BEEP_ON;
}

inline void check_beep(void)
{
  if (beep.enable != BEEP_ON)
    return;

  if(beep.play > 0)
  {
    beep.play--;
    return;
  }
  
  int8_t current = beep.sound[beep.position];
  int8_t pause_flag = current & 0x80;
  int8_t delay;

  if (current == STOP)
  {
    beep.enable = BEEP_OFF;
    STOP_BEEP();
    return;
  }

  if (pause_flag)
  {
    STOP_BEEP();
    delay = -1*current;
  }
  else 
  {
    START_BEEP();
    delay = current;
  }
  
  beep.play = delay;
  beep.position++;

}

inline static void scan_keyboard(void)
{
  static const uint8_t START_ROW = 1; // номера битов порта для сканирования строк клавиатуры
  static const uint8_t END_ROW = 4;

  static int8_t current_key = NO_KEY_PRESSED ;
  static int8_t last_key = NO_KEY_PRESSED ;
  static uint8_t pressed_time = 0;
  static int8_t current_row = START_ROW ;

  reset_bit(PORTD, current_row);

  if (current_row == START_ROW)
    current_key = NO_KEY_PRESSED;

  uint8_t column0 = (~PIND) & 0x01;
  uint8_t column1 = (~PINA) & 0x01;
  uint8_t column2 = (~PINA) & 0x02;

  int8_t shift = -1;
  if(column0)
    shift = 0;
  else if (column1)
    shift = 1;
  else if(column2)
    shift = 2;
  if ( shift >= 0)
    current_key = pgm_read_byte(&keyboard_decoder[current_row-1][shift]);
  
  set_bit(PORTD, current_row);
  current_row++;

  if (current_row < END_ROW + 1)
    return;
  
  if (current_key != NO_KEY_PRESSED && pressed_time <  UINT8_MAX)
    pressed_time++;

  if( current_key == NO_KEY_PRESSED)
  {
    if(pressed_time > KEY_TIME && pressed_time < SAVE_TIME)
      key.action = ACTION_PRESSED;

      pressed_time = 0;
      key.used = NOT_USED;
  }
  else if (pressed_time > SAVE_TIME && key.used == NOT_USED)
  {
    key.action = ACTION_SAVE;
  }
  
  current_row = START_ROW;
  key.pressed = last_key;
  last_key = current_key;

}

inline uint8_t is_digit_key(void)
{
  return key.pressed >= 0 && key.pressed <= 9;
}

uint8_t key_pressed(void)
{
  if(key.action != ACTION_PRESSED)
  {
    return NO;
  }

  if(is_digit_key())
  {
    counter.current = eeprom_read_byte(&timer_preset[key.pressed]);
    if ((counter.current > MAX_COUNT_VALUE) || (counter.current == 0))
    {
      counter.current = pgm_read_byte(&timer_preset_default[key.pressed]);
    }
  }
  else if(key.pressed == STAR_KEY)
  {
    counter.current--;
    if (counter.current == 0)
    {
      counter.current = MAX_COUNT_VALUE;
    }
  }
  else if(key.pressed == HASH_KEY)
  {
    counter.current++;
    if(counter.current > MAX_COUNT_VALUE)
    {
      counter.current = 1;
    }
  }
  else
  {
    return NO;
  }

  start_beep(key_beep, KEY_FREQ);
  counter.fraction = 0;        
  counter.finished = NO;
  key.action = ACTION_NONE;
    
  return YES;
}

inline void end_count(void)
{
  start_beep(end_beep, END_FREQ);
  beep.pause = 0;      
  beep.repeat = NO;
}

ISR (TIMER1_COMPA_vect)
{
  if(counter.current > 0)
  {
    if (counter.fraction == MAIN_TIMER_MAX)
    {
      counter.fraction = 0;
      counter.current--;
    }
    else
      counter.fraction++;
  }
  
  if(counter.finished == YES)
  {
    if (beep.pause == REBEEP_TIMER_MAX)
    {
      beep.repeat = YES;
      beep.pause = 0;
    }
    else
      beep.pause++;
  }
  
  return;
}

ISR (TIMER1_COMPB_vect)
{
  scan_keyboard();
  check_beep();
}

int main(void)
{
  //все ножки на вход
  DDRA = 0x00;
  //вклчаем подтягивающие резисторы
  PORTA = 0xFF;

  // все ножки на выход
  DDRB = 0xFF;
  // везде лог.0
  PORTB = 0x00 ;


  // указаные дорожки на выход.
  DDRD = _BV(PIND1) | _BV(PIND2) | _BV(PIND3) |_BV(PIND4) | _BV(PIND5)  | _BV(PIND6);
  // на 0 ножку уст. резистор, ножки 1-4 перевести в лог.1.
  PORTD = _BV(PIND0) | _BV(PIND1) |_BV(PIND2) | _BV(PIND3)  | _BV(PIND4);

  // CTC mode
  TCCR0A =  _BV(WGM01);
  //prescaler at 8
  TCCR0B = _BV(CS01);
  TCNT0 = 0x00;
  // max value
  OCR0A = 130;
  OCR0B = 0;

  TCCR1A = 0x00;
  TCCR1B =_BV(WGM12) | _BV(CS10); //CTC mode, prescaler 1
  TCNT1H = 0x00;
  TCNT1L = 0x00;
  ICR1H = 0x00;
  ICR1L = 0x00;
  OCR1AH = high(MAX_TIMER);
  OCR1AL = low(MAX_TIMER);
  OCR1BH = 0x00;
  OCR1BL = 0x00;

  // прерывания от таймеров
  TIMSK = _BV(OCIE1A) | _BV(OCIE1B);

  // выключаем Universal Serial Interface
  USICR=0x00;

  // выключаем Analog Comparator 
  ACSR=0x80;

  
  beep.enable = BEEP_OFF;

  key.pressed = NO_KEY_PRESSED;
  key.action = ACTION_NONE;
  key.used = NOT_USED;

  // сразу после включения таймер будет периодически пищать
  // чтобы напомнить, что он включен
  counter.current =  0;
  counter.finished = YES;
  counter.last = counter.current;

  beep.repeat = NO;
  beep.pause = 0;        

  Led led_display;
  led_set(&led_display);

  int8_t state = STATE_WAIT;
  sei();
  while (1)
  {
    switch(state)
    {
    case STATE_WAIT:
      if(beep.repeat == YES)
      {
        end_count();
      }
      
      if(key_pressed())
        state = STATE_COUNTING;
      break;
     
    case STATE_COUNTING:
      key_pressed();

      if( key.action == ACTION_SAVE)
      {
        if(is_digit_key()) 
        {
          start_beep(save_beep,SAVE_FREQ);
          eeprom_write_byte(&timer_preset[key.pressed], counter.current);
        }
        key.used = key.pressed;
        key.action = ACTION_NONE;
      }
      
      if(counter.current == 0)
      {
        end_count();        
        counter.finished = YES;
        state = STATE_WAIT;
      }
      break;
      
    default:
      state = STATE_WAIT;
    }
    
    if(counter.current != counter.last)
    {
      led_set(&led_display);
      counter.last = counter.current;
    }

    led_show(led_display);
  }
  return 0;
}
