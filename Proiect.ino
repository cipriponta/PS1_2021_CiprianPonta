#include <LiquidCrystal.h>

// Define Statements
#define CPU_F 16000000
#define PRESCALER 64
#define TEMP_CHANNEL 0
#define SAMPLE_TIME 0.1

#define BUTTON_OK     (1 << PB4)
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
	MAIN_MENU_WELCOME   	= 0,
	MAIN_MENU_T_SET			= 1,
	MAIN_MENU_T_HEAT    	= 2,
	MAIN_MENU_T_KEEP    	= 3,
	MAIN_MENU_T_COOL    	= 4,
	MAIN_MENU_KP        	= 5,
	MAIN_MENU_KI        	= 6,
	MAIN_MENU_KD        	= 7,
	MAIN_MENU_PID			= 8,
	MAIN_MENU_MAX_NUM   	= 9,
	SUB_MENU_T_SET      	= 10,
	SUB_MENU_T_HEAT     	= 11,
	SUB_MENU_T_KEEP     	= 12,
	SUB_MENU_T_COOL     	= 13,
	SUB_MENU_KP         	= 14,
	SUB_MENU_KI         	= 15,
	SUB_MENU_KD         	= 16,
	SUB_MENU_PID			= 17,
	SUB_MENU_MAX_NUM    	= 18
};

// Global Variables
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

volatile button_t current_button_state = EV_NONE;
volatile menu_t current_menu_state = MAIN_MENU_WELCOME;

volatile int t_set = 35;
volatile int t_heat = 30;
volatile int t_keep = 30;
volatile int t_cool = 30;
volatile double kp = 100;
volatile double ki = 0.01;
volatile double kd = 0.1;

volatile int timer1_counter = 0;
volatile double temperature = 0;

// Function Signatures
void timer1_init();
void timer2_init();
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

void start_pid_control();
void stop_pid_control();

void (*state_machine[SUB_MENU_MAX_NUM][EV_MAX_NUM])() = 
{
	{nothing,		nothing,    prev_menu,	next_menu 	},  // MAIN_MENU_WELCOME
	{enter_menu,  	go_home,    prev_menu,  next_menu 	},  // MAIN_MENU_T_SET
	{enter_menu,  	go_home,    prev_menu,  next_menu 	},  // MAIN_MENU_T_HEAT
	{enter_menu,  	go_home,    prev_menu,  next_menu 	},  // MAIN_MENU_T_KEEP
	{enter_menu,  	go_home,    prev_menu,  next_menu 	},  // MAIN_MENU_T_COOL
	{enter_menu,  	go_home,    prev_menu,  next_menu 	},  // MAIN_MENU_KP
	{enter_menu,  	go_home,    prev_menu,  next_menu 	},  // MAIN_MENU_KI
	{enter_menu,  	go_home,    prev_menu,  next_menu 	},  // MAIN_MENU_KD
	{enter_menu,	go_home,	prev_menu, 	next_menu	},	// MAIN_MENU_PID
	{nothing,     	nothing,    nothing,    nothing   	},  // MAIN_MENU_MAX_NUM
	{exit_menu,   	exit_menu,  dec_t_set,  inc_t_set 	},  // SUB_MENU_T_SET
	{exit_menu,  	exit_menu,  dec_t_heat, inc_t_heat	},  // SUB_MENU_T_HEAT
	{exit_menu,   	exit_menu,  dec_t_keep, inc_t_keep	},  // SUB_MENU_T_KEEP
	{exit_menu,   	exit_menu,  dec_t_cool, inc_t_cool	},  // SUB_MENU_T_COOL
	{exit_menu,   	exit_menu,  dec_kp,     inc_kp    	},  // SUB_MENU_KP
	{exit_menu,   	exit_menu,  dec_ki,     inc_ki    	},  // SUB_MENU_KI
	{exit_menu,   	exit_menu,	dec_kd,     inc_kd    	},  // SUB_MENU_KD
	{exit_menu,		exit_menu,	nothing,	nothing		}	// SUB_MENU_PID
};

// Main Function
int main()
{  
	Serial.begin(9600);
	lcd.begin(16, 2);

	timer1_init();
	timer2_init();
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
	TCCR1B |= (1 << CS11) | (1 << CS10); // prescaler 64
	TIMSK1 |= (1 << OCIE1A); // Output Compare A
	OCR1A = (CPU_F / PRESCALER - 1) / 10; // 0.1 second interrupt
}

void timer2_init()
{
	DDRB |= (1 << PB3); // OC2A pin
	TCCR2A |= (1 << COM2A1); // non inverting mode
	TCCR2A |= (1 << WGM21) | (1 << WGM20); // fast pwm
	TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS10); // prescaler = 1024
	OCR2A = 255;
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
	DDRB &= ~(BUTTON_OK | BUTTON_CANCEL | BUTTON_NEXT | BUTTON_PREV); // buttons pins; 
	PCICR |= (1 << PCIE0); // enable pin interrupt for the b port
	PCMSK0 |= (1 << PCINT0) | (1 << PCINT1) | (1 << PCINT2) | (1 << PCINT4); // enable the interrupts for arduino pins 8, 9, 10, 12
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
		
		case MAIN_MENU_PID:
		lcd.setCursor(0, 0);
		lcd.print("MENU:           ");
		lcd.setCursor(0, 1);
		lcd.print("PID CONTROL           ");
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
		
		case SUB_MENU_PID:
		lcd.setCursor(0, 0);
		lcd.print("SUBMENU PID:           ");
		lcd.setCursor(0, 1);
		lcd.print("Temp = ");
		lcd.print(temperature);
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
	current_menu_state = (menu_t)((int)current_menu_state + 9);
}

void exit_menu()
{
	current_menu_state = (menu_t)((int)current_menu_state - 9);
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
	kp += 0.1;
}

void dec_kp()
{
	kp -= 0.1;
}

void inc_ki()
{
	ki += 0.1;
}

void dec_ki()
{
	ki -= 0.1;
}

void inc_kd()
{
	kd += 0.1;
}

void dec_kd()
{
	kd -= 0.1;;
}

void start_pid_control()
{	
	static double error_sum = 0;
	static double prev_error = 0;
	
	double error = (double)t_set - temperature;
	error_sum += (error * SAMPLE_TIME);
	double derivative = (error - prev_error) / SAMPLE_TIME;
	double output = kp * error + ki * error_sum + kd * derivative;
	prev_error = error;
	
	if(output < 0)
	{
		OCR2A = 0;
	}
	else if(output > 255)
	{
		OCR2A = 255;
	}
	else
	{
		OCR2A = (int)output;
	}
	
	// String message = "";
	// message += "error: " + String(error) + ", error_sum: " + String(error_sum) + ", derivative: " + String(derivative) +
				// ", output:" + String(output) + ", prev_error:" + String(prev_error) + ", OCR2A: " + String(OCR2A);
	// Serial.println(message);
}

void stop_pid_control()
{
	OCR2A = 0;
}

// Interruption Routines
ISR(TIMER1_COMPA_vect)
{
	timer1_counter++;
	
	if(timer1_counter == 10)
	{
		timer1_counter = 0;
	}
	
	// every 0.1 seconds 
	ADMUX &= ~0x0F; // reset the current channel
	ADMUX |= TEMP_CHANNEL; // setting the channel
	ADCSRA |= (1 << ADSC); // adc interrupt
	if(current_menu_state == SUB_MENU_PID)
	{
		start_pid_control();
	}
	else
	{	
		stop_pid_control();
	}
	
	// every 0.5 seconds 
	if(timer1_counter % 5 == 0)
	{
		if(current_button_state != EV_NONE)
		{
			state_machine[current_menu_state][current_button_state]();
		}
		print_menu();
	}
  
	// every 1 second
	if(timer1_counter == 0)
	{
		Serial.print(temperature);
		Serial.print(" ");
		Serial.println(t_set);
	}
}

ISR(ADC_vect)
{
	int can_value = ADC;
	double voltage_value = (double)can_value * 5000.0/1023.0;
	temperature = voltage_value / 10.0;
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

