#include <LiquidCrystal.h>

// Define Statements
#define CPU_F 16000000
#define PRESCALER 64
#define TEMP_CHANNEL 0
#define SAMPLE_TIME 0.1
#define KCR 255.0
#define PCR 0.5

#define BUTTON_OK    	(1 << PB4)
#define BUTTON_CANCEL 	(1 << PB2)
#define BUTTON_PREV   	(1 << PB1)
#define BUTTON_NEXT   	(1 << PB0)

#define MEM_T_SET_H		((unsigned int)(0x00))
#define MEM_T_SET_L		((unsigned int)(0x01))
#define MEM_T_HEAT_H	((unsigned int)(0x02))
#define MEM_T_HEAT_L	((unsigned int)(0x03))
#define MEM_T_KEEP_H	((unsigned int)(0x04))
#define MEM_T_KEEP_L	((unsigned int)(0x05))
#define MEM_T_COOL_H	((unsigned int)(0x06))
#define MEM_T_COOL_L	((unsigned int)(0x07))
#define MEM_KP_H		((unsigned int)(0x08))
#define MEM_KP_L		((unsigned int)(0x09))
#define MEM_KI_H		((unsigned int)(0x0A))
#define MEM_KI_L		((unsigned int)(0x0B))
#define MEM_KD_H		((unsigned int)(0x0C))
#define MEM_KD_L		((unsigned int)(0x0D))

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

volatile unsigned int initial_t_set = 0;
volatile unsigned int initial_t_heat = 0;
volatile unsigned int initial_t_keep = 0;
volatile unsigned int initial_t_cool = 0;
volatile double initial_kp = 0;			
volatile double initial_ki = 0;			
volatile double initial_kd = 0;			

volatile unsigned int t_set = 0;		// t_set = 35
volatile unsigned int t_heat = 0;		// t_heat = 120
volatile unsigned int t_keep = 0;		// t_keep = 60
volatile unsigned int t_cool = 0;		// t_cool = 600
volatile double kp = 0;					// System Constraint: kp * 100 < 65536, kp = 0.6 * KCR (153.00)
volatile double ki = 0;					// System Constraint: ki * 100 < 65536, ki = 0.5 * PCR (0.25)
volatile double kd = 0;					// System Constraint: kd * 100 < 65536, kd = 0.125 * PCR (0.06)

volatile int timer1_counter = 0;
volatile double temperature = 0;
volatile double initial_temperature = 0;
volatile double setpoint = 0;
volatile double pid_mode_counter = 0;

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
void calculate_setpoint();

void EEPROM_write(unsigned int address, unsigned char data);
void EEPROM_read(unsigned int address, unsigned char *data);

void EEPROM_read_all();
void EEPROM_write_all();
void EEPROM_write_t_set();
void EEPROM_write_t_heat();
void EEPROM_write_t_keep();
void EEPROM_write_t_cool();
void EEPROM_write_kp();
void EEPROM_write_ki();
void EEPROM_write_kd();

void (*state_machine[SUB_MENU_MAX_NUM][EV_MAX_NUM])() = 
{
	{nothing,		nothing,    prev_menu,	next_menu 	},	// MAIN_MENU_WELCOME
	{enter_menu,  	go_home,    prev_menu,  next_menu 	},  // MAIN_MENU_T_SET
	{enter_menu,  	go_home,    prev_menu,  next_menu 	},  // MAIN_MENU_T_HEAT
	{enter_menu,  	go_home,    prev_menu,  next_menu 	},  // MAIN_MENU_T_KEEP
	{enter_menu,  	go_home,    prev_menu,  next_menu 	},  // MAIN_MENU_T_COOL
	{enter_menu,  	go_home,	prev_menu,  next_menu 	},  // MAIN_MENU_KP
	{enter_menu,  	go_home,    prev_menu,  next_menu 	},  // MAIN_MENU_KI
	{enter_menu,  	go_home,    prev_menu,  next_menu 	},  // MAIN_MENU_KD
	{enter_menu,	go_home,	prev_menu, 	next_menu	},	// MAIN_MENU_PID
	{nothing,     	nothing,   	 nothing,   nothing   	},  // MAIN_MENU_MAX_NUM
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
	
	sei();
	
	timer1_init();
	timer2_init();
	adc_init();
	button_init();
	
	EEPROM_read_all();
	
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
	EEPROM_write_all();
	
	static double error_sum = 0;
	static double prev_error = 0;
	
	double error = setpoint - temperature;
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

void calculate_setpoint()
{
	if(current_menu_state == SUB_MENU_PID)
	{
		pid_mode_counter++;
		
		if(pid_mode_counter <= t_heat)
		{
			setpoint = initial_temperature + ((double)t_set - initial_temperature)/((double)t_heat) * (double)pid_mode_counter;
		}
		else if(pid_mode_counter <= t_heat + t_keep)
		{
			setpoint = (double)t_set;
		}
		else if(pid_mode_counter <= t_heat + t_keep + t_cool)
		{
			setpoint = t_set - ((double)t_set - initial_temperature)/((double)t_cool) * ((double)(pid_mode_counter - t_heat - t_keep));
		}
		else
		{
			setpoint = initial_temperature;
		}
	}
	else
	{
		pid_mode_counter = 0;
	}
}

void EEPROM_write(unsigned int address, unsigned char data)
{
	while(EECR & (1<<EEPE));	// Wait for completion of previous write
	EEAR = address;				// Set up address register
	EEDR = data;				// Set up data register
	EECR |= (1<<EEMPE);			// Enable master write
	EECR |= (1<<EEPE);			// Start EEPROM write
}

void EEPROM_read(unsigned int address, unsigned char *data)
{
	while(EECR & (1<<EEPE)); 	// Wait for completion of previous write
	EEAR = address;				// Set up address register
	EECR |= (1 << EERE);		// Start EEPROM read
	*data = EEDR;				// Return data through pointer
}

void EEPROM_read_all()
{
	cli();
	
	unsigned char t_set_h;
	unsigned char t_set_l;
	
	unsigned char t_heat_h;
	unsigned char t_heat_l;
	
	unsigned char t_keep_h;
	unsigned char t_keep_l;
	
	unsigned char t_cool_h;
	unsigned char t_cool_l;
	
	unsigned char kp_h;
	unsigned char kp_l;
	
	unsigned char ki_h;
	unsigned char ki_l;
	
	unsigned char kd_h;
	unsigned char kd_l;
	
	EEPROM_read(MEM_T_SET_H, &t_set_h);
	EEPROM_read(MEM_T_SET_L, &t_set_l);
	initial_t_set = (((unsigned int)(t_set_h)) << 8) + (unsigned int)(t_set_l);
	t_set = initial_t_set;
	
	// Serial.println("READ T_SET:");
	// Serial.println(t_set);
	// Serial.println(t_set_h);
	// Serial.println(t_set_l);
	
	EEPROM_read(MEM_T_HEAT_H, &t_heat_h);
	EEPROM_read(MEM_T_HEAT_L, &t_heat_l);
	initial_t_heat = (((unsigned int)(t_heat_h)) << 8) + (unsigned int)(t_heat_l);
	t_heat = initial_t_heat;
	
	// Serial.println("READ T_HEAT:");
	// Serial.println(t_heat);
	// Serial.println(t_heat_h);
	// Serial.println(t_heat_l);
	
	EEPROM_read(MEM_T_KEEP_H, &t_keep_h);
	EEPROM_read(MEM_T_KEEP_L, &t_keep_l);
	initial_t_keep = (((unsigned int)(t_keep_h)) << 8) + (unsigned int)(t_keep_l);
	t_keep = initial_t_keep;
	
	// Serial.println("READ T_KEEP:");
	// Serial.println(t_keep);
	// Serial.println(t_keep_h);
	// Serial.println(t_keep_l);
	
	EEPROM_read(MEM_T_COOL_H, &t_cool_h);
	EEPROM_read(MEM_T_COOL_L, &t_cool_l);
	initial_t_cool = (((unsigned int)(t_cool_h)) << 8) + (unsigned int)(t_cool_l);
	t_cool = initial_t_cool;
	
	// Serial.println("READ T_COOL:");
	// Serial.println(t_cool);
	// Serial.println(t_cool_h);
	// Serial.println(t_cool_l);
	
	EEPROM_read(MEM_KP_H, &kp_h);
	EEPROM_read(MEM_KP_L, &kp_l);
	initial_kp = ((double)((((unsigned int)(kp_h)) << 8) + (unsigned int)(kp_l)))/100.00;
	kp = initial_kp;
	
	// Serial.println("READ KP:");
	// Serial.println(kp);
	// Serial.println(kp_h);
	// Serial.println(kp_l);
	
	EEPROM_read(MEM_KI_H, &ki_h);
	EEPROM_read(MEM_KI_L, &ki_l);
	initial_ki = ((double)((((unsigned int)(ki_h)) << 8) + (unsigned int)(ki_l)))/100.00;
	ki = initial_ki;
	
	// Serial.println("READ KI:");
	// Serial.println(ki);
	// Serial.println(ki_h);
	// Serial.println(ki_l);
	
	EEPROM_read(MEM_KD_H, &kd_h);
	EEPROM_read(MEM_KD_L, &kd_l);
	initial_kd = ((double)((((unsigned int)(kd_h)) << 8) + (unsigned int)(kd_l)))/100.00;
	kd = initial_kd;
	
	// Serial.println("READ KD:");
	// Serial.println(kd);
	// Serial.println(kd_h);
	// Serial.println(kd_l);
	
	sei();
}

void EEPROM_write_all()
{	
	if(t_set != initial_t_set)
	{
		cli();
		EEPROM_write_t_set();
		initial_t_set = t_set;
		sei();
	}
	
	if(t_heat != initial_t_heat)
	{
		cli();
		EEPROM_write_t_heat();
		initial_t_heat = t_heat;
		sei();
	}
	
	if(t_keep != initial_t_keep)
	{
		cli();
		EEPROM_write_t_keep();
		initial_t_keep = t_keep;
		sei();
	}
	
	if(t_cool != initial_t_cool)
	{
		cli();
		EEPROM_write_t_cool();
		initial_t_cool = t_cool;
		sei();
	}
	
	if(kp != initial_kp)
	{
		cli();
		EEPROM_write_kp();
		initial_kp = kp;
		sei();
	}
	
	if(ki != initial_ki)
	{
		cli();
		EEPROM_write_ki();
		initial_ki = ki;
		sei();
	}
	
	if(kd != initial_kd)
	{
		cli();
		EEPROM_write_kd();
		initial_kd = kd;
		sei();
	}
}

void EEPROM_write_t_set()
{
	unsigned char t_set_h;
	unsigned char t_set_l;
	
	t_set_h = (unsigned char)((t_set & 0xFF00) >> 8);
	t_set_l = (unsigned char)(t_set & 0x00FF);
	
	Serial.println("WRITE T_SET:");
	Serial.println(t_set_h);
	Serial.println(t_set_l);
	
	EEPROM_write(MEM_T_SET_H, t_set_h);
	EEPROM_write(MEM_T_SET_L, t_set_l);
}

void EEPROM_write_t_heat()
{
	unsigned char t_heat_h;
	unsigned char t_heat_l;
	
	t_heat_h = (unsigned char)((t_heat & 0xFF00) >> 8);
	t_heat_l = (unsigned char)(t_heat & 0x00FF);
	
	Serial.println("WRITE T_HEAT:");
	Serial.println(t_heat_h);
	Serial.println(t_heat_l);
	
	EEPROM_write(MEM_T_HEAT_H, t_heat_h);
	EEPROM_write(MEM_T_HEAT_L, t_heat_l);
}

void EEPROM_write_t_keep()
{
	unsigned char t_keep_h;
	unsigned char t_keep_l;
	
	t_keep_h = (unsigned char)((t_keep & 0xFF00) >> 8);
	t_keep_l = (unsigned char)(t_keep & 0x00FF);
	
	Serial.println("WRITE T_KEEP:");
	Serial.println(t_keep_h);
	Serial.println(t_keep_l);
	
	EEPROM_write(MEM_T_KEEP_H, t_keep_h);
	EEPROM_write(MEM_T_KEEP_L, t_keep_l);
}

void EEPROM_write_t_cool()
{
	unsigned char t_cool_h;
	unsigned char t_cool_l;
	
	t_cool_h = (unsigned char)((t_cool & 0xFF00) >> 8);
	t_cool_l = (unsigned char)(t_cool & 0x00FF);
	
	Serial.println("WRITE T_COOL:");
	Serial.println(t_cool_h);
	Serial.println(t_cool_l);
	
	EEPROM_write(MEM_T_COOL_H, t_cool_h);
	EEPROM_write(MEM_T_COOL_L, t_cool_l);
}

void EEPROM_write_kp()
{
	unsigned char kp_h;
	unsigned char kp_l;
	unsigned int kp_int;
	
	kp_int = (unsigned int)(kp * 100);
	
	kp_h = (unsigned char)((kp_int & 0xFF00) >> 8);
	kp_l = (unsigned char)(kp_int & 0x00FF);
	
	Serial.println("WRITE KP:");
	Serial.println(kp_int);
	Serial.println(kp_h);
	Serial.println(kp_l);
	
	EEPROM_write(MEM_KP_H, kp_h);
	EEPROM_write(MEM_KP_L, kp_l);
}

void EEPROM_write_ki()
{
	unsigned char ki_h;
	unsigned char ki_l;
	unsigned int ki_int;
	
	ki_int = (unsigned int)(ki * 100);
	
	ki_h = (unsigned char)((ki_int & 0xFF00) >> 8);
	ki_l = (unsigned char)(ki_int & 0x00FF);
	
	Serial.println("WRITE KI:");
	Serial.println(ki_int);
	Serial.println(ki_h);
	Serial.println(ki_l);
	
	EEPROM_write(MEM_KI_H, ki_h);
	EEPROM_write(MEM_KI_L, ki_l);
}

void EEPROM_write_kd()
{
	unsigned char kd_h;
	unsigned char kd_l;
	unsigned int kd_int;
	
	kd_int = (unsigned int)(kd * 100);
	
	kd_h = (unsigned char)((kd_int & 0xFF00) >> 8);
	kd_l = (unsigned char)(kd_int & 0x00FF);
	
	Serial.println("WRITE KD:");
	Serial.println(kd_int);
	Serial.println(kd_h);
	Serial.println(kd_l);
	
	EEPROM_write(MEM_KD_H, kd_h);
	EEPROM_write(MEM_KD_L, kd_l);
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
		initial_temperature = temperature;
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
		Serial.print("initial_temperature = ");
		Serial.print(initial_temperature);
		Serial.print(", temperature = ");
		Serial.print(temperature);
		Serial.print(", setpoint = ");
		Serial.print(setpoint);
		Serial.print(", t_set = ");
		Serial.println(t_set);
		
		calculate_setpoint();
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

