#include <avr/io.h>
#include <avr/interrupt.h>

#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)
#define FOSC 20000000 //clock speed
//#define BAUD 125000
#define MYUBRR 24//FOSC/8/(BAUD + 1)
#define DShim (int)12500
#define Bounce 245
void USART_Init(unsigned int);
void ADC_Init(void);
void timer_counter1_INIT();
int main (void)
{
	volatile unsigned char tmp = 0;
	volatile unsigned int i = 0;
	//volatile unsigned char j = 0;
	volatile unsigned char sum = 0;
	volatile unsigned char a1 = 0;
	volatile unsigned char dataUsart = 0;
	volatile unsigned char temperature = 0;
	volatile unsigned char counterUsart = 0;
	volatile unsigned char n = 0;
	volatile unsigned char podsvet = 0;
	volatile unsigned char coef = 0;
	volatile unsigned int cnt_shim = 0;
	volatile unsigned char buttons = 0;
	volatile unsigned char temp_cnt = 0;
	volatile unsigned char temp_buf;
	//Init data direction registors
	DDRC = (1 << DDC0) | (1 << DDC1) | (1 << DDC5);
	DDRB = (1 << DDB6) | (1 << DDB4) | (7 << DDB0);//comments on paper
	DDRD = (0 << DDD4) | (0 << DDD0) | (0 << DDD1) | (0 << DDD2) | (0 << DDD3) | (0 << DDD5);// kn
	PORTD = (1 << PD4) | (1 << PD0);// pull-up to PD0, PD4;
	//DDRD = (0 << DDD1) | (0 << DDD2) | (0 << DDD3) | (0 << DDD5);
	PORTC |= (1 << PC1) | (1 << PC0); //| (1 << PC3) | (1 << PC4);
		
	unsigned char slovo[32] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
								0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
								0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01};
	//CKSEL3 = 0; CKSEL2 = 1; CKSEL1 = 1; 
	//pull-up_resistors_deactivated default and activated on unused pins
	ADC_Init();
	USART_Init(MYUBRR);
	timer_counter1_INIT();
	//read coef from eeprom
	EEAR = 0x0000;
	EECR = 0x01;
	coef = EEDR;
	while(1)
	{
	//ispravnost obogreva-----------------------------
	if(PIND & 0b01000000)
		slovo[29] = 0x01;
	else
		slovo[29] = 0x00;
	//------------------------------------------------

	//drebezg-----------------------------------------
	buttons = (PIND & 0b00101110);
	if (tmp == buttons){
		i++;
		if(i == Bounce){
			a1 = (~tmp);		
			slovo[8] = ((a1 & 0b00000010) >> 1);
			slovo[9] = ((a1 & 0b00000100) >> 2);		
			slovo[10] = ((a1 & 0b00001000) >> 3);
			slovo[11] = ((a1 & 0b00100000) >> 5) ;
			i = 0;
		}
	}
	else{
		i = 0;
		tmp = buttons;

	}/*
	if(j == 50){
		a1 = (~tmp);		
		slovo[8] = ((a1 & 0b00000010) >> 1);
		slovo[9] = ((a1 & 0b00000100) >> 2);		
		slovo[10] = ((a1 & 0b00001000) >> 3);
		slovo[11] = ((a1 & 0b00100000) >> 5) ;
		j = 0;
	}*/
	//temperatura ADC-------------------------
	ADCSRA |= (1 << ADSC);
	while((ADCSRA & 0b01000000)){};
		temp_buf = ADCH;
	if(temperature == temp_buf){
		temp_cnt++;
		if(temp_cnt == Bounce){
			slovo[12] = (0b10000000 & temperature) >> 7;
			slovo[13] = (0b01000000 & temperature) >> 6;
			slovo[14] = (0b00100000 & temperature) >> 5;
			slovo[15] = (0b00010000 & temperature) >> 4;
			slovo[16] = (0b00001000 & temperature) >> 3;
			slovo[17] = (0b00000100 & temperature) >> 2;
			slovo[18] = (0b00000010 & temperature) >> 1;
			slovo[19] = (0b00000001 & temperature);
			temp_cnt = 0;
		}
	}
	else{
		temp_cnt = 0;
		temperature = temp_buf;
	}
	
	if(temperature <= 123)//vkl obogreva
	{
		slovo[20] = 0x00;
		PORTB = (PINB & ~(1 << PB2));

	}
	else if (temperature >= 135)//otkl obogreva
	{
		slovo[20] = 0x01;
		PORTB = (PINB & ~(1 << PB2));
	}
	//-----------------------------------------

	///pereda4a slovo--------------------------
	for(n = 0; n < 32; n++)
	{			
			PORTC = PINC & 0b11111101;// falling c_rtm			
			PORTC = ((PINC & 0b11111110) | slovo[n]);
			 _NOP();_NOP();
			//PORTC = (PINC ; //);
			sum = sum * sum; sum = sum * sum; sum = sum * sum; sum = sum * sum; sum = sum * sum; sum = sum * sum;
			_NOP(); _NOP(); 
			PORTC = PINC | 0b00000010;// rising c_rtm
			slovo[31] ^= slovo[n];
			sum = sum * sum; sum = sum * sum; sum = sum * sum; sum = sum * sum; sum = sum * sum; 
			_NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); 
	}	
	slovo[31] = 0x01;
	PORTC |= 0x01;
	PORTB |= (1 << PB0);
	PORTB &= ~(1 << PB0);
	//-------------------------------------------

	//priem usart--------------------------------
	if(UCSR0A & (1 << RXC0))
	{						
			counterUsart = 0;	
			//dataUsart = UDR0;
			PORTB = PINB | 0x01;	
			while (UCSR0A & (1<<RXC0) ){
				if((((1 << FE0) & UCSR0A) == 0) && (((1 << UPE0) & UCSR0A) == 0)){
					dataUsart = UDR0;			 
				/*unsigned char var = dataUsart;
				for(unsigned char c = 0; c < 8; c++){
					PORTB = (PINB & 0xFE) | ((var) & ~0xFE);
			 		var = var >> 1;*/
			 	}else{
					volatile unsigned char trash = UDR0;
				}
			}			 
			 PORTB = PINB | 0x01;
			 PORTB = PINB & ~0x01;
			//PORTC = PORTC & ~(1 << PC5);
			slovo[29] = 0x01;
			
	}
	else
	{		
		counterUsart++;
		if(counterUsart == 255)
		{
			counterUsart = 0;
			slovo[29] = 0x00;
			dataUsart = 0xFF;
		}
	}
	//-------------------------------------------

	// podsvet-----------------------------------	
	if(cnt_shim == DShim){ //12500
		if(podsvet != dataUsart){
			podsvet = dataUsart;
			/*PORTC = PINC | (1 << PC5);
			PORTC = PINC & ~(1 << PC5);*/
			if((dataUsart & 0b01000000) >> 6)//day
			{
				ICR1 = ((dataUsart) & 0b00111111) * coef + 64;
			}
			else//night
			{
				ICR1 = (dataUsart) & 0b00111111;
			}
		}
	}else{
		cnt_shim++;
	}
	//-------------------------------------------
	/*for(char n = 0; n < 250; n++)
	{sum = sum * sum; sum = sum * sum; sum = sum * sum; sum = sum * sum; sum = sum * sum; sum = sum * sum; sum = sum * sum;
			_NOP();_NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP();
	}*/
		
  }
}

void ADC_Init(void)
{
	ADMUX = (1 << REFS0) | (1 << ADLAR) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0);//0b01100111; //choise_voltage_regerence 2b, result to hight register, zero, 7adc pin;
//	DIDR0 = 0x00;
	ADCSRA = (1 << ADEN) | (0 << ADATE) | (7 << ADPS0);//10000111;	
}

/*void USART_Init(void) //Usart_initialization sync //unsigned int ubrr
{
    UCSR0B = (1 << RXEN0);// 0b00001000; //EN_ //UBRR0H = (unsigned char)(ubrr>>8); //BAUD RATE //	UBRR0L = (unsigned char)ubrr;
	UCSR0C = (1 << UMSEL00) | (1 << UPM00) | (1 << UPM01) | (1 << UCSZ01) | (1 << UCSZ00) | (0 << UCPOL0);//0b01000111; //FORMAT_DATA p.s. parity?
}*/
void USART_Init(unsigned int ubrr) //Usart_initialization async //unsigned int ubrr
{
	UBRR0L = (unsigned char)ubrr;
	UBRR0H = (unsigned char)(ubrr>>8);
    UCSR0B = (1 << RXEN0);// 0b00001000; //EN_ //UBRR0H = (unsigned char)(ubrr>>8); //BAUD RATE //	UBRR0L = (unsigned char)ubrr;
	UCSR0C = (0 << UMSEL00) | (0 << UPM00) | (1 << UPM01) | (1 << UCSZ01) | (1 << UCSZ00) | (0 << UCPOL0);//0b01000111; //FORMAT_DATA p.s. parity?
}
void timer_counter1_INIT()
{
		TCCR1A = (1 << COM1A1);
		TCCR1B = (1 << WGM13) | (1 << CS11) | (1 << CS10);
		//ICR1 = 0x03FF;
		//ICR1 = 63;
		//OCR1A = 0x3FE;
}

/*ISR(TIMER0_COMPA_vect)
{
	//PORTC |= (PORTC & 0b11111101) | (0b00000010 & ~PORTC);
	PORTC = (PORTC & 0b11111100) | (~PORTC & 0b00000010) | (((char)(slovo32 >> n)) & 0x01);
	n--;
}*/

