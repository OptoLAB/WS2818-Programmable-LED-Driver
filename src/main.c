/*
 * mini_AT_WS2812.c
 *
 * Created: 7/24/2019 12:29:27 PM
 * Author : Jovan
 */ 
#define F_CPU	16000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)
	
#define BUAD	9600
#define BRC		((F_CPU/16/BUAD) - 1)

unsigned char rx_data;

#define MAX_NUM_LEDS 99
#define COLOR_DEPTH 24

unsigned long LEDs[MAX_NUM_LEDS];

//colors
#define COLOR(r,g,b) (unsigned long)(((unsigned long)g<<16)|((unsigned int)r<<8)|(b))
#define RED		0x00FF00
#define GREEN	0xFF0000
#define BLUE	0x0000FF
#define YELLOW	0xFFFF00
#define CYAN	0xFF00FF
#define MAGENTA	0x00FFFF
#define WHITE	0xFFFFFF
#define BLACK	0x000000

unsigned int NUM_OF_LEDS=24;

#define RX_BUFF_SIZE 0x04
#define RX_BUFF_MASK 0x03
unsigned char rx_i=0;
unsigned char rx_buff[RX_BUFF_SIZE];

unsigned char MODE=0, WL_S=100, WL_V=100;
unsigned int WL_H=0;
//-------------------------------------------------------------------------------------------------------
// UART
//-------------------------------------------------------------------------------------------------------

void configUART(void)
{
	UBRR0H = (BRC >> 8);
	UBRR0L =  BRC;
	
	UCSR0B = (1 << RXEN0)  | (1 << RXCIE0) | (1 << TXEN0);;
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

	DDRB|= (1 << PORTB0);
}

void UART_putChar(char c)
{
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c; 
}

ISR(USART_RX_vect)
{
	rx_data=UDR0;
	
	rx_buff[rx_i]=rx_data;
	rx_i++;
	rx_i&=RX_BUFF_MASK;
	
	if(rx_buff[1]=='M')  // SET MODE
	{
		rx_buff[1]=0;
		MODE=atoi((char*)rx_buff);  // [0-5]
		rx_i=0;
		
	}
	else if(MODE)		
	{
		switch(rx_buff[3])   // UDPADTE HSV
		{
			case 'H':	rx_buff[3]=0;				// HUE
						WL_H=atoi((char*)rx_buff);
						rx_i=0;
						break;
			case 'S':	rx_buff[3]=0;				//SATURATION
						WL_S=atoi((char*)rx_buff);
						rx_i=0;
						break;
			case 'V':	rx_buff[3]=0;				//VALUE
						WL_V=atoi((char*)rx_buff);
						rx_i=0;
						break;
			case 'N':	rx_buff[3]=0;				//NUMBER OF LEDs
						NUM_OF_LEDS=atoi((char*)rx_buff);
						rx_i=0;
			break;						
		}
	}
}

//-------------------------------------------------------------------------------------------------------
// ADC
//-------------------------------------------------------------------------------------------------------

void configADC(void)
{
	    ADMUX = (1<<REFS0);
	    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

uint16_t readADC(uint8_t ch)
{
	ch &= 0b00000111;
	ADMUX = (ADMUX & 0xF8)|ch;
	
	ADCSRA |= (1<<ADSC);
	
	while(ADCSRA & (1<<ADSC));
	
	return (ADC);
}

//-------------------------------------------------------------------------------------------------------
// LEDs
//-------------------------------------------------------------------------------------------------------

void delay_ms(unsigned int count) {
	while(count--) {
		_delay_ms(1);
	}
}

void RGBLed_OneBit()
{
	cli();
	sbi(PORTB,PORTB2);
	_NOP();
	_NOP();
	_NOP();
	_NOP();
	_NOP();
	_NOP();
	_NOP();
	_NOP();
	_NOP();
	cbi(PORTB,PORTB2);
	sei();

}

void RGBLed_ZeroBit()
{
	cli();
	sbi(PORTB,PORTB2);
	_NOP();
	_NOP();
	_NOP();
	_NOP();
	cbi(PORTB,PORTB2);
	sei(); 
}

void resetLEDS(void)
{
	cbi(PORTB,PORTB2);
	delay_ms(2);
}

void updateLEDs(void)
{
	unsigned int i,j;
	resetLEDS();
	for(j=0;j<NUM_OF_LEDS;j++)
	{
		for(i=0;i<COLOR_DEPTH;i++)
		{
			if((LEDs[j]&(0x800000>>i))==0) RGBLed_ZeroBit();
			else RGBLed_OneBit();
		}
	}
}

void fillLEDs(unsigned long color)
{
	unsigned int i;
	for(i=0;i<NUM_OF_LEDS;i++)LEDs[i]=color;
	updateLEDs();	
}

void sweepLEDs(unsigned long color, unsigned char dir, const unsigned int period)
{
	unsigned int i;
	for(i=0;i<NUM_OF_LEDS;i++)
	{
		if(dir)LEDs[i]=color;
		else LEDs[NUM_OF_LEDS-1-i]=color;
		updateLEDs();
		delay_ms(period);
	}
}

void shiftLEDs(unsigned char dir, unsigned int steps, unsigned int period)
{
	unsigned int i,j;
	unsigned long temp;
	for(j=0;j<steps;j++)
	{
		if(dir)
		{
			temp=LEDs[NUM_OF_LEDS-1];
			for(i=0;i<NUM_OF_LEDS-1;i++) LEDs[NUM_OF_LEDS-1-i]=LEDs[NUM_OF_LEDS-2-i];
			LEDs[0]=temp;
		}
		else
		{
			temp=LEDs[0];
			for(i=0;i<NUM_OF_LEDS-1;i++) LEDs[i]=LEDs[i+1];
			LEDs[NUM_OF_LEDS-1]=temp;
		}
		updateLEDs();
		delay_ms(period);
	}
}

void initLEDs(void)
{
	DDRB|=(1 << PORTB2);
	fillLEDs(BLACK);
}

void HSVtoRGB(unsigned int H, unsigned char S, unsigned char  V, unsigned char  *R, unsigned char  *G, unsigned char  *B)
{
	int i;
	float h, s, v;
	float r ,g, b; 
	float f, p, q, t; 

	h=(float)H/60;			
    s=(float)S/100;
    v=(float)V/100;
	
	i = floor( h );
	f = h - i;			
	p = v * ( 1 - s );
	q = v * ( 1 - s * f );
	t = v * ( 1 - s * ( 1 - f ) );
	switch( i ) {
		case 0:
			r = v;
			g = t;
			b = p;
			break;
		case 1:
			r = q;
			g = v;
			b = p;
			break;
		case 2:
			r = p;
			g = v;
			b = t;
			break;
		case 3:
			r = p;
		    g = q;
			b = v;
			break;
		case 4:
			r = t;
			g = p;
			b = v;
			break;
		default:		// case 5:
			r = v;
			g = p;
			b = q;
			break;
	}
    
    *R=r*255; 
    *G=g*255;
    *B=b*255;
}

void RBBCMY_patern(unsigned char S, unsigned char V)
{
	unsigned char i, R, G, B, N;
	unsigned int H=0;
	
	N=NUM_OF_LEDS/6;
	
	HSVtoRGB(H,S,V,&R,&G,&B);
	for(i=0;i<N;i++) LEDs[i]=COLOR(R,G,B);
	H+=60;
	HSVtoRGB(H,S,V,&R,&G,&B);
	for(i=0;i<N;i++) LEDs[i+(1*N)]=COLOR(R,G,B);
	H+=60;
	HSVtoRGB(H,S,V,&R,&G,&B);
	for(i=0;i<N;i++) LEDs[i+(2*N)]=COLOR(R,G,B);
	H+=60;
	HSVtoRGB(H,S,V,&R,&G,&B);
	for(i=0;i<N;i++) LEDs[i+(3*N)]=COLOR(R,G,B);
	H+=60;
	HSVtoRGB(H,S,V,&R,&G,&B);
	for(i=0;i<N;i++) LEDs[i+(4*N)]=COLOR(R,G,B);
	H+=60;
	HSVtoRGB(H,S,V,&R,&G,&B);
	for(i=0;i<N;i++) LEDs[i+(5*N)]=COLOR(R,G,B);
}

//-------------------------------------------------------------------------------------------------------
// MAIN
//-------------------------------------------------------------------------------------------------------
int main(void)
{
	unsigned int  H=0; 
	unsigned char S, V, dir=0; 
	unsigned char R,  G,  B;
	unsigned long ADC_H, ADC_S, ADC_V;
	
	for(rx_i=0;rx_i<RX_BUFF_SIZE;rx_i++)rx_buff[rx_i]=0;
	rx_i=0;
	configADC();
	configUART();
	initLEDs();
	

	delay_ms(500);
	
	sei();

    while (1) 
    {

		switch(MODE)
		{
			case 0: ADC_H=readADC(1);						 //manual control via pots
					ADC_S=readADC(2);
					ADC_V=readADC(4);			
					H=(ADC_H*360)/1024;
					S=(ADC_S*100)/1024;
					V=(ADC_V*100)/1024;
					HSVtoRGB(H,S,V,&R,&G,&B);
					fillLEDs(COLOR(R,G,B));
					break;
			case 1: HSVtoRGB(WL_H,WL_S,WL_V,&R,&G,&B);			//remote control	
					fillLEDs(COLOR(R,G,B));
					break;
			case 2: HSVtoRGB(H,WL_S,WL_V,&R,&G,&B);				// sweep
					sweepLEDs(COLOR(R,G,B),dir,WL_H>>2);
					H+=60;
					if(H>=360)H=0;
					dir=~dir;
					break;
			case 3: RBBCMY_patern(WL_S, WL_V);					//rotate
					shiftLEDs(1,NUM_OF_LEDS,WL_H>>2);
					break;
			case 4: H=rand()%360;								//random
					HSVtoRGB(H,WL_S,WL_V,&R,&G,&B);
					fillLEDs(COLOR(R,G,B));
					delay_ms(WL_H<<1);
					break;	
			case 5: H+=1;
					if(H>=360)H=0;
					HSVtoRGB(H,WL_S,WL_V,&R,&G,&B);				//spectrum 
					fillLEDs(COLOR(R,G,B));
					delay_ms(WL_H>>2);
					break;			
		}
    }
}

