// -*- coding: windows-1251 -*-
#define AVR_ATtiny2313
#define F_CPU 1000000UL   // 1 MHz

#include <avr/cpufunc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <inttypes.h>
#include <util/delay.h>

#define set_bit(port,bit)   port |= _BV(bit)
#define reset_bit(port,bit) port &= ~(_BV(bit))
#define invert_bit(port,bit) port = _BV(bit);

#define STOP_COUNT()  reset_bit(TIMSK,OCIE1A);
#define START_COUNT() set_bit(TIMSK,OCIE1A);

const uint8_t MAX_COUNT_VALUE = 99;
const uint8_t MAX_FRAC = 100;
const uint8_t START_ROW = 1;
const uint8_t END_ROW = 4;

const uint8_t KEY_FREQ = 250;
const uint8_t END_FREQ = 80;
const uint8_t SAVE_FREQ = 150;

const uint8_t KEY_TIME = 5;
const uint8_t SHIFT_TIME = 100;
const uint8_t SAVE_TIME = 200;

const uint8_t SHOW_TIME = 4;

enum KEYS {HASH_KEY = -1, STAR_KEY = -2, SAVE_KEY = -3, NO_KEY_PRESSED = -4};
enum NO_ACTION { NO_COUNT = 100, NO_DIGIT = 10, STOP = -1};
enum BEEP_STATUS { BEEP_ON, BEEP_OFF};

typedef struct current_beep
{
  int8_t status;
  uint8_t position;
  const int8_t *melody;
  uint8_t timer;
} CurrentBeep;

typedef struct led_struct
{
  uint8_t first_digit;
  uint8_t second_digit;
} Led;

typedef struct key_struct
{
  int8_t pressed;
  int8_t used;
} Key;


const int8_t keyboard_decoder[4][3] PROGMEM = {{ 1, 2, 3}, { 4, 5, 6}, {7, 8, 9}, {STAR_KEY, 0, HASH_KEY}};
const int8_t led_digits[] PROGMEM = { 64, 121, 36, 48, 25, 18, 2, 120, 0, 16, 63};
const int8_t key_beep[] =  { 2, STOP};
const int8_t end_beep[] =  { 5, -20,  5, -20, 5, STOP};
const int8_t save_beep[] = { 10, -10,  5, STOP};

const uint8_t timer_preset_default[] PROGMEM  = { 99, 10, 20, 30, 40, 50, 60, 70, 80, 90 , 97 , 12 ,13 };
uint8_t EEMEM timer_preset[10];

volatile uint8_t counter = NO_COUNT;
volatile uint8_t frac = 0;

volatile CurrentBeep beep;
volatile Key key;


void led_set(Led *led)
{
  if (counter == NO_COUNT)
  {
    led->first_digit = NO_DIGIT;
    led->second_digit  = NO_DIGIT;
  }
  else
  {
    led->first_digit =  counter / 10;
    led->second_digit = counter % 10;
  }
}

void led_show(Led led)
{
  PORTB = pgm_read_byte( &led_digits[led.first_digit]);
  set_bit(PORTD,PIND6);
  _delay_ms(SHOW_TIME);
  reset_bit(PORTD,PIND6);
  PORTB = pgm_read_byte( &led_digits[led.second_digit]);
  set_bit(PORTB,PINB7);
  _delay_ms(SHOW_TIME);
}


inline void start_beep(const int8_t* set_beep, uint8_t freq)
{
  beep.position = 1;
  beep.melody = set_beep;
  beep.timer = beep.melody[0];
  OCR0A = freq;
  set_bit(TCCR0A,COM0B0);
  beep.status = BEEP_ON;
}

inline void check_beep(void)
{
  if (beep.status != BEEP_ON)
    return;

  if(beep.timer > 0)
    beep.timer--;
  else
  {
    int8_t current = beep.melody[beep.position];
    int8_t pause_flag = current & 0x80;
    int8_t delay;
    if (current == STOP)
    {
      beep.status = BEEP_OFF;
      reset_bit(TCCR0A,COM0B0);
      return;
    }
    if ( pause_flag)
    {
      reset_bit(TCCR0A,COM0B0);
      delay = ~current +1;
    }
    else // beep
    {
      set_bit(TCCR0A,COM0B0);
      delay = current;
    }
    beep.timer = delay;
    beep.position++;
  }
}

inline static void scan_keyboard(void)
{
  static int8_t current_key = NO_KEY_PRESSED ;
  static uint8_t pressed_time = 0;
  static int8_t current_row = START_ROW ;

  uint8_t column0;
  uint8_t column1;
  uint8_t column2;  

  reset_bit(PORTD,current_row);

  if (current_row ==START_ROW)
    current_key = NO_KEY_PRESSED;

  column0 = (~PIND) & 0x01;
  column1 = (~PINA) & 0x01;
  column2 = (~PINA) & 0x02;

  if( current_row == END_ROW && ( column0 && column2))
    current_key = SAVE_KEY;
  else
  {
    int8_t shift = -1;
    if( column0)
      shift = 0;
    else if ( column1 )
      shift = 1;
    else if( column2 )
      shift = 2;
    if ( shift >= 0)
      current_key = pgm_read_byte(&keyboard_decoder[current_row-1][shift]);
  }
  
  if (current_key != NO_KEY_PRESSED && pressed_time <  UINT8_MAX)
    pressed_time++;

  set_bit(PORTD,current_row);

  current_row++;

  if (current_row == END_ROW + 1)
  {
    if( current_key == NO_KEY_PRESSED)
    {
      pressed_time = 0;
      key.used = NO_KEY_PRESSED;
      key.pressed = NO_KEY_PRESSED;
    }
    else if ( current_key != key.used)
    {
      uint8_t accept = 0;
      if (current_key == SAVE_KEY && pressed_time >=  SAVE_TIME)
        accept = 1;
      else if ((current_key == STAR_KEY || current_key == HASH_KEY) && pressed_time >=  SHIFT_TIME)
        accept = 1;
      else if (pressed_time >=  KEY_TIME )
        accept = 1;

      if (accept)
      {
        key.used = NO_KEY_PRESSED;
        key.pressed = current_key;
      }
    }

    current_row = START_ROW;
  }
}

int8_t set_counter(void)
{
  frac = 0;
  uint8_t value;
  value = eeprom_read_byte(&timer_preset[key.pressed]);
  if ( ( value > MAX_COUNT_VALUE) ||  (value == 0 ) )
    value = pgm_read_byte(&timer_preset_default[key.pressed]);
  return value;
}
  
ISR (TIMER1_COMPA_vect)
{
  if ( frac == MAX_FRAC)
  {
    frac = 0;
    counter--;
  }
  frac++;
  return;
}

ISR (TIMER1_COMPB_vect)
{
  scan_keyboard();
  check_beep();
}


int main(void)
{
  PORTA=0xFF;
  DDRA=0x00;

  PORTB=0x00 ;
  DDRB=0xFF;

  PORTD= _BV(PIND0) | _BV(PIND1) |_BV(PIND2) | _BV(PIND3)  | _BV(PIND4);
  DDRD = _BV(PIND1) | _BV(PIND2) | _BV(PIND3) |_BV(PIND4) | _BV(PIND5)  | _BV(PIND6);


  // CTC mode
  TCCR0A =  1 << WGM01 ;
  //prescaler at 8
  TCCR0B = 1 << CS01;
  TCNT0 = 0x00;
  // max value
  OCR0A = 130;
  OCR0B = 0;

  TCCR1A= 0x00;
  TCCR1B =_BV(WGM12) |  _BV(CS10); //CTC mode, prescaler 1
  TCNT1H=0x00;
  TCNT1L=0x00;
  ICR1H=0x00;
  ICR1L=0x00;
  OCR1AH=0x27; //1000
  OCR1AL=0x10;
  OCR1BH=0x00;
  OCR1BL=0xFF;

  TIMSK = _BV(OCIE1B);
  //


  // Universal Serial Interface initialization
  USICR=0x00;

  // Analog Comparator initialization
  ACSR=0x80;

  beep.status = BEEP_OFF;

  key.pressed = NO_KEY_PRESSED;
  key.used = NO_KEY_PRESSED;

  counter =  NO_COUNT;
  int8_t last_counter = counter;
  Led led_display;
  led_set(&led_display);
  // Global enable interrupts
  sei();
  while (1)
  {
    if(counter == 0)
    {
      STOP_COUNT();
      counter = NO_COUNT;
      led_set(&led_display);
      last_counter = counter;
      start_beep(end_beep, END_FREQ);
    }

    if(key.pressed  > NO_KEY_PRESSED && key.used == NO_KEY_PRESSED)
    {
      start_beep(key_beep,KEY_FREQ);
      if(key.pressed >= 0)
      {
        counter =  set_counter();
        START_COUNT();
      }
      else if(key.pressed == SAVE_KEY)
      {
        start_beep(save_beep,SAVE_FREQ); 
      }
      else if(key.pressed == HASH_KEY && counter > 1)
        counter--;
      else if(key.pressed == STAR_KEY && counter < MAX_COUNT_VALUE)
        counter++;
      key.used = key.pressed ;
    }
 
    if(counter != last_counter)
    {
      led_set(&led_display);
      last_counter = counter;
    }
    led_show(led_display);
  }
  return 0;
}

 


