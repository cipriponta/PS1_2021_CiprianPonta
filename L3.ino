#include <LiquidCrystal.h>

// Define Statements
#define CPU_F 16000000
#define PRESCALER 1024
#define TEMP_CHANNEL 0

#define BUTTON_OK     (1 << PB3)
#define BUTTON_CANCEL (1 << PB2)
#define BUTTON_PREV   (1 << PB1)
#define BUTTON_NEXT   (1 << PB0)

// Type Definitions
enum button_t
{
  EV_OK       = 0,
  EV_CANCEL   = 1,
  EV_PREV     = 2,
  EV_NEXT     = 3,
  EV_NONE     = 4,
  EV_MAX_NUM  = 5
};

enum menu_t
{
  MAIN_MENU_WELCOME   = 0,
  MAIN_MENU_T_SET     = 1,
  MAIN_MENU_T_HEAT    = 2,
  MAIN_MENU_T_KEEP    = 3,
  MAIN_MENU_T_COOL    = 4,
  MAIN_MENU_KP        = 5,
  MAIN_MENU_KI        = 6,
  MAIN_MENU_KD        = 7,
  MAIN_MENU_MAX_NUM   = 8,
  SUB_MENU_T_SET      = 9,
  SUB_MENU_T_HEAT     = 10,
  SUB_MENU_T_KEEP     = 11,
  SUB_MENU_T_COOL     = 12,
  SUB_MENU_KP         = 13,
  SUB_MENU_KI         = 14,
  SUB_MENU_KD         = 15,
  SUB_MENU_MAX_NUM    = 16
};

// Global Variables
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

volatile button_t current_button_state = EV_NONE;
volatile menu_t current_menu_state = MAIN_MENU_WELCOME;

int t_set = 30;
int t_heat = 30;
int t_keep = 30;
int t_cool = 30;
int kp = 1;
int ki = 1;
int kd = 1;

// Function Signatures
void timer1_init();
void adc_init();
void button_init();

void print_menu();

void nothing();
void enter_menu();
void exit_menu();
void go_home();
void next_menu();
void prev_menu();

void inc_t_set();
void dec_t_set();
void inc_t_heat();
void dec_t_heat();
void inc_t_keep();
void dec_t_keep();
void inc_t_cool();
void dec_t_cool();
void inc_kp();
void dec_kp();
void inc_ki();
void dec_ki();
void inc_kd();
void dec_kd();

void (*state_machine[SUB_MENU_MAX_NUM][EV_MAX_NUM])() = 
{
  {nothing,     nothing,    prev_menu,    next_menu },  // MAIN_MENU_WELCOME
  {enter_menu,  go_home,    prev_menu,    next_menu },  // MAIN_MENU_T_SET
  {enter_menu,  go_home,    prev_menu,    next_menu },  // MAIN_MENU_T_HEAT
  {enter_menu,  go_home,    prev_menu,    next_menu },  // MAIN_MENU_T_KEEP
  {enter_menu,  go_home,    prev_menu,    next_menu },  // MAIN_MENU_T_COOL
  {enter_menu,  go_home,    prev_menu,    next_menu },  // MAIN_MENU_T_KP
  {enter_menu,  go_home,    prev_menu,    next_menu },  // MAIN_MENU_T_KI
  {enter_menu,  go_home,    prev_menu,    next_menu },  // MAIN_MENU_T_KD
  {nothing,     nothing,    nothing,      nothing   },  // MAIN_MENU_MAX_NUM
  {exit_menu,   exit_menu,  dec_t_set,    inc_t_set },  // SUB_MENU_T_SET
  {exit_menu,   exit_menu,  dec_t_heat,   inc_t_heat},  // SUB_MENU_T_HEAT
  {exit_menu,   exit_menu,  dec_t_keep,   inc_t_keep},  // SUB_MENU_T_KEEP
  {exit_menu,   exit_menu,  dec_t_cool,   inc_t_cool},  // SUB_MENU_T_COOL
  {exit_menu,   exit_menu,  dec_kp,       inc_kp    },  // SUB_MENU_KP
  {exit_menu,   exit_menu,  dec_ki,       inc_ki    },  // SUB_MENU_KI
  {exit_menu,   exit_menu,  dec_kd,       inc_kd    }  // SUB_MENU_KD
};

// Main Function
int main()
{  
  Serial.begin(9600);
  lcd.begin(16, 2);

  timer1_init();
  adc_init();
  button_init();
  
  sei();
  
  while(1)
  {
    
  }
}

// Function Definitions
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

void button_init()
{
  DDRB &= ~((1 << PB3) | (1 << PB2) | (1 << PB1) | (1 << PB0)); // buttons pins; 
  PCICR |= (1 << PCIE0); // enable pin interrupt for the b port
  PCMSK0 |= (1 << PCINT0) | (1 << PCINT1) | (1 << PCINT2) | (1 << PCINT3); // enable the interrupts for arduino pins 8, 9, 10, 11
}

void print_menu()
{
  switch(current_menu_state)
  {
    case MAIN_MENU_T_SET:
    lcd.setCursor(0, 0);
    lcd.print("MENU:           ");
    lcd.setCursor(0, 1);
    lcd.print("CHANGE T_SET           ");
    break;

    case MAIN_MENU_T_HEAT:
    lcd.setCursor(0, 0);
    lcd.print("MENU:           ");
    lcd.setCursor(0, 1);
    lcd.print("CHANGE T_HEAT           ");
    break;

    case MAIN_MENU_T_KEEP:
    lcd.setCursor(0, 0);
    lcd.print("MENU:           ");
    lcd.setCursor(0, 1);
    lcd.print("CHANGE T_KEEP           ");
    break;

    case MAIN_MENU_T_COOL:
    lcd.setCursor(0, 0);
    lcd.print("MENU:           ");
    lcd.setCursor(0, 1);
    lcd.print("CHANGE T_COOL           ");
    break;

    case MAIN_MENU_KP:
    lcd.setCursor(0, 0);
    lcd.print("MENU:           ");
    lcd.setCursor(0, 1);
    lcd.print("CHANGE KP           ");
    break;

    case MAIN_MENU_KI:
    lcd.setCursor(0, 0);
    lcd.print("MENU:           ");
    lcd.setCursor(0, 1);
    lcd.print("CHANGE KI           ");
    break;

    case MAIN_MENU_KD:
    lcd.setCursor(0, 0);
    lcd.print("MENU:           ");
    lcd.setCursor(0, 1);
    lcd.print("CHANGE KD           ");
    break;

    case SUB_MENU_T_SET:
    lcd.setCursor(0, 0);
    lcd.print("SUBMENU T_SET:           ");
    lcd.setCursor(0, 1);
    lcd.print("T_SET = ");
    lcd.print(t_set);
    lcd.print("         ");
    break;

    case SUB_MENU_T_HEAT:
    lcd.setCursor(0, 0);
    lcd.print("SUBMENU T_HEAT:           ");
    lcd.setCursor(0, 1);
    lcd.print("T_HEAT = ");
    lcd.print(t_heat);
    lcd.print("         ");
    break;

    case SUB_MENU_T_KEEP:
    lcd.setCursor(0, 0);
    lcd.print("SUBMENU T_KEEP:           ");
    lcd.setCursor(0, 1);
    lcd.print("T_KEEP = ");
    lcd.print(t_keep);
    lcd.print("         ");
    break;

    case SUB_MENU_T_COOL:
    lcd.setCursor(0, 0);
    lcd.print("SUBMENU T_COOL:           ");
    lcd.setCursor(0, 1);
    lcd.print("T_COOL = ");
    lcd.print(t_cool);
    lcd.print("         ");
    break;

    case SUB_MENU_KP:
    lcd.setCursor(0, 0);
    lcd.print("SUBMENU KP:           ");
    lcd.setCursor(0, 1);
    lcd.print("KP = ");
    lcd.print(kp);
    lcd.print("         ");
    break;

    case SUB_MENU_KI:
    lcd.setCursor(0, 0);
    lcd.print("SUBMENU KI:           ");
    lcd.setCursor(0, 1);
    lcd.print("KI = ");
    lcd.print(ki);
    lcd.print("         ");
    break;

    case SUB_MENU_KD:
    lcd.setCursor(0, 0);
    lcd.print("SUBMENU KD:           ");
    lcd.setCursor(0, 1);
    lcd.print("KD = ");
    lcd.print(kd);
    lcd.print("         ");
    break;
    
    case MAIN_MENU_WELCOME:
    default:
    lcd.setCursor(0, 0);
    lcd.print("PS1 PROJECT           ");
    lcd.setCursor(0, 1);
    lcd.print("               ");
    break;
  }
}

void nothing()
{
  
}

void enter_menu()
{
  current_menu_state = (menu_t)((int)current_menu_state + 8);
}

void exit_menu()
{
  current_menu_state = (menu_t)((int)current_menu_state - 8);
}

void go_home()
{
  current_menu_state = MAIN_MENU_WELCOME;
}

void next_menu()
{
  current_menu_state = (menu_t)((int)current_menu_state + 1);
  if(current_menu_state == MAIN_MENU_MAX_NUM)
  {
    current_menu_state = 0;
  }
}

void prev_menu()
{
  current_menu_state = (menu_t)((int)current_menu_state - 1);
  if(current_menu_state == -1)
  {
    current_menu_state = MAIN_MENU_MAX_NUM - 1;
  }
}

void inc_t_set()
{
  t_set++;
}

void dec_t_set()
{
  t_set--;
}

void inc_t_heat()
{
  t_heat++;
}

void dec_t_heat()
{
  t_heat--;
}

void inc_t_keep()
{
  t_keep++;
}

void dec_t_keep()
{
  t_keep--;
}

void inc_t_cool()
{
  t_cool++;
}

void dec_t_cool()
{
  t_cool--;
}

void inc_kp()
{
  kp++;
}

void dec_kp()
{
  kp--;
}

void inc_ki()
{
  ki++;
}

void dec_ki()
{
  ki--;
}

void inc_kd()
{
  kd++;
}

void dec_kd()
{
  kd--;;
}

// Interruption Routines
ISR(TIMER1_COMPA_vect)
{
  ADMUX &= ~0x0F; // reset the current channel
  ADMUX |= TEMP_CHANNEL; // setting the channel
  ADCSRA |= (1 << ADSC); // adc interrupt

  Serial.println(current_button_state);
  
  if(current_button_state != EV_NONE)
  {
    state_machine[current_menu_state][current_button_state]();
  }
  print_menu();
}

ISR(ADC_vect)
{
  int can_value = ADC;
  long voltage_value = (long)can_value* 5000/1023.0;
  int temp = voltage_value / 10;
}

ISR(PCINT0_vect)
{
  current_button_state = EV_NONE;
  if(PINB & BUTTON_OK)
  {
    current_button_state = EV_OK;
  }

  if(PINB & BUTTON_CANCEL)
  {
    current_button_state = EV_CANCEL;
  }

  if(PINB & BUTTON_PREV)
  {
    current_button_state = EV_PREV;
  }

  if(PINB & BUTTON_NEXT)
  {
    current_button_state = EV_NEXT;
  }
}

