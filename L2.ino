#include <LiquidCrystal.h>

#define CPU_F 16000000
#define PRESCALER 1024
#define TEMP_CHANNEL 0

LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

volatile int hours_counter = 18;
volatile int minutes_counter = 30;
volatile int seconds_counter = 0;

void timer1_init();
void adc_init();
void printTime();

int main()
{  
  
  lcd.begin(16, 2);

  timer1_init();
  adc_init();
  
  sei();
  
  while(1)
  {
    
  }
}

void timer1_init()
{
  TCCR1B |= (1 << WGM12); // CTC
  TCCR1B |= (1 << CS12) | (1 << CS10); // prescaler 1024
  TIMSK1 |= (1 << OCIE1A); // Output Compare A
  OCR1A = (CPU_F/PRESCALER - 1); // 1 second interrupt
}

void adc_init()
{
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // ADC Prescaler
  ADMUX |= (1 << REFS0);
  ADCSRA |= (1 << ADEN); 
  ADCSRA |= (1 << ADIE); 
}

void printTime()
{
    seconds_counter++;

  if(seconds_counter == 60)
  {
    seconds_counter = 0;
    minutes_counter++;
  }

  if(minutes_counter == 60)
  {
    minutes_counter = 0;
    hours_counter++;
  }
  
  if(hours_counter == 24)
  {
    hours_counter = 0;
  }

  String message = "Ora ";

  if(hours_counter < 10)
  {
    message += "0";
  }
  message += String(hours_counter) + ":";
  
  if(minutes_counter < 10)
  {
    message += "0";
  }
  message += String(minutes_counter) + ":";
  
  if(seconds_counter < 10)
  {
    message += "0";
  }
  message += String(seconds_counter);

  lcd.setCursor(0, 1);
  lcd.print(message);
}

ISR(TIMER1_COMPA_vect)
{
  printTime();

  ADMUX &= ~0x0F; // reset the current channel
  ADMUX |= TEMP_CHANNEL; // setting the channel
  ADCSRA |= (1 << ADSC); // adc interrupt
}

ISR(ADC_vect)
{
  int can_value = ADC;
  long voltage_value = (long)can_value* 5000/1023.0;
  int temp = voltage_value / 10;

  String message = "Temp = ";
  message += String(temp) + "'C";
  
  lcd.setCursor(0, 0);
  lcd.print(message);
  // lcd.print("    ");
}



