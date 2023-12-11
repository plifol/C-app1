/*
 *
 */ 

#define VER_MAJOR 1
#define VER_MINOR 1

#define F_CPU	7372800
#define BAUD	460800

#define UART_BUFF_SIZE 32

#include <stddef.h>
#include <stdio.h>
#include <util/setbaud.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

static volatile enum _enumUartState
{
	UartIdle,
	UartRecvBeg,
	UartRecvEnd,
	UartTransBeg,
	UartTransEnd
}
UartState = UartIdle;

#define LED_PORT	PORTB
#define LED_F1		( 1 << PB5 )
#define LED_C1		( 1 << PB4 )
#define LED_V25		( 1 << PB3 )
#define LED_IO		( 1 << PB2 )

#define ERR89_PORTA	( 1 << PA5 )
#define C_INT_PORTA	( 1 << PA4 )
#define C4_PORTA	( 1 << PA3 )
#define C3_PORTA	( 1 << PA2 )
#define C2_PORTA	( 1 << PA1 )
#define C1_PORTA	( 1 << PA0 )

#define ERR89_PORTC	( 1 << PINC2 )
#define C_INT_PORTC	( 1 << PINC3 )
#define C1_PORTC	( 1 << PINC4 )
#define C2_PORTC	( 1 << PINC5 )
#define C3_PORTC	( 1 << PINC6 )
#define C4_PORTC	( 1 << PINC7 )

#define C1_LIMIT	10
#define C2_LIMIT	10
#define C3_LIMIT	10
#define C4_LIMIT	10
#define C_INT_LIMIT	40
#define V25_LIMIT	40

#define C1_RESOK	( 1 << 0 )
#define C2_RESOK	( 1 << 1 )
#define C3_RESOK	( 1 << 2 )
#define C4_RESOK	( 1 << 3 )
#define CINT_RESOK	( 1 << 4 )
#define V25_RESOK	( 1 << 5 )
#define IO_RESOK	( 1 << 6 )
#define ERR_89_RES	( 1 << 7 )

static uint8_t C1_4_PORTA[ 4 ] = { C1_PORTA, C2_PORTA, C3_PORTA, C4_PORTA };
static uint8_t C1_4_PORTC[ 4 ] = { C1_PORTC, C2_PORTC, C3_PORTC, C4_PORTC };
static uint8_t C1_4_LIMIT[ 4 ] = { C1_LIMIT, C2_LIMIT, C3_LIMIT, C4_LIMIT };
static uint8_t C1_4_RESOK[ 4 ] = { C1_RESOK, C2_RESOK, C3_RESOK, C4_RESOK };

static volatile uint8_t IndexC1 = 0;
static volatile uint8_t Log25V  = 0;
static volatile uint8_t LogCint = 0;
static volatile uint8_t LogC1_4[ 4 ] = {[ 0 ... 3 ] = 0 };

static volatile uint8_t SendLen  = 0;
static volatile uint8_t SendNum  = 0;
static volatile uint8_t LogData  = 0;
static volatile uint8_t CmdData  = 0;
static volatile uint8_t OutPortA = 0;
static uint8_t InBuff [ UART_BUFF_SIZE ];
static uint8_t OutBuff[ UART_BUFF_SIZE ];

static uint8_t Crc( uint8_t* aBuffer, size_t aSize )
{
	uint8_t res = 0;
	
	while( aSize-- )
	{
		res += *aBuffer;
		aBuffer++;
	}

	return( ~res );
}

ISR( USART1_RX_vect )
{
	static uint8_t udr;
	static uint8_t len;
	static uint8_t num;

	udr = UDR1;

	if( UartIdle == UartState )
	{
		switch( udr )
		{
			case 0xA0: len = 2; break;
			case 0xA1: len = 3; break;
			case 0xA2:
			case 0xA3:
			case 0xA4: len = 2; break;

			default  : len = 0;
		}

		if( 0 < len )
			UartState = UartRecvBeg,
			InBuff[ num = 0 ] = udr,
			num++,
			len--;
	}
	else
	{
		if( 0 < len )
			InBuff[ num ] = udr,
			num++, len--;
		
		if( 0 == len )
		{
			num--;

			if( Crc( InBuff, num ) == InBuff[ num ])
				LogData |= IO_RESOK;
			else
				LogData &= ~IO_RESOK;

			UartState = UartRecvEnd;
		}
	}
}

ISR( USART1_UDRE_vect )
{
	cli();

	UDR1 = OutBuff[ SendNum ];
	SendNum++;

	if( SendNum == SendLen )
		UCSR1B  &= ~( 1 << UDRIE1 ),
		UCSR1B  |=  ( 1 << TXCIE1 ),
		UartState = UartTransEnd;		

	sei();
}

ISR( USART1_TX_vect )
{
	cli();
	UartState = UartIdle;
	UCSR1B   &= ~( 1 << TXCIE1 );
	sei();
}

#define  IS_LIMIT( __val__, __limit__, __dlimit__ ) ( __val__ >= ( __limit__-__dlimit__ )) && ( __val__ < ( __limit__+__dlimit__ ))

ISR( TIMER3_COMPA_vect ) // 800 ms
{
	ETIMSK &= ~( 1 << OCIE3A );	// timer3 compare reg A INT OFF
	TCCR3B  = 0;				// timer3 STOP
	TCNT3   = 0;				// timer3 value 0
	TCCR3B  = ( 1 << CS32 );	// start Timer3 div 256
	ETIMSK |= ( 1 << OCIE3A );	// timer3 compare reg A INT ON

	cli();

	if( IS_LIMIT( Log25V, V25_LIMIT, 2 ))
		LED_PORT |= LED_V25,
		LogData  |= V25_RESOK;
	else
		LED_PORT &= ~LED_V25,
		LogData  &= ~V25_RESOK;

	Log25V = 0;

	if( IS_LIMIT( LogCint, C_INT_LIMIT, 2 ))
		LED_PORT |= LED_F1,
		LogData  |= CINT_RESOK;
	else
		LED_PORT &= ~LED_F1,
		LogData  &= ~CINT_RESOK;

	LogCint = 0;

	for( int i = 0; 4 > i; ++i )
	{
		if( IS_LIMIT( LogC1_4[ i ], C1_4_LIMIT[ i ], 2 ))
			LogData |= C1_4_RESOK[ i ];
		else
			LogData &= ~C1_4_RESOK[ i ];
		
		LogC1_4[ i ] = 0;
	}

	sei();
	
	if( 0 == ( LogData & 0x0F ))
		LED_PORT &= ~LED_C1;
	else
		if( 0x0F == ( LogData & 0x0F ))
			LED_PORT |= LED_C1;
		else
			LED_PORT ^= LED_C1;
}

ISR( TIMER1_COMPB_vect )
{
	switch( ++IndexC1 )
	{
		case 1 ... 3: break;
		default: IndexC1 = 0;
	}

	TIMSK  &= ~( 1 << OCIE1B );	// INT Off for compare reg B
	TCCR1B = 0;					// Timer Stop
	TCNT1  = 0;					// timer value 0
	EIMSK |= ( 1 << INT0 );		// INT0 on
}

ISR( TIMER1_COMPA_vect )
{
	uint8_t pinc = PINC;
	uint8_t cInt = 0;
	uint8_t c1_4 = 0;

	if(( pinc & C_INT_PORTC ) && ( CmdData & C_INT_PORTA ))
		PORTA &= ~C_INT_PORTA, // C1-INT Down
		cInt  = 1;

	if(( pinc & C1_4_PORTC[ IndexC1 ]) && ( CmdData & C1_4_PORTA[ IndexC1 ]))
		PORTA &= ~C1_4_PORTA[ IndexC1 ], // C1_1...4 Down
		c1_4 = 1;

	asm( "nop" );

	pinc = PINC;

	if( cInt && ( !( pinc & C_INT_PORTC )))
		LogCint++;

	if( c1_4 && ( !( pinc & C1_4_PORTC[ IndexC1 ])))
		LogC1_4[ IndexC1 ]++;

	OutPortA = 0;
	TIMSK   &= ~( 1 << OCIE1A );	// Int Off for compare reg A
}

ISR( INT0_vect )
{
	if( CmdData & C_INT_PORTA )
		OutPortA |= C_INT_PORTA; // C1-INT Up

	if( CmdData & C1_4_PORTA[ IndexC1 ])
		OutPortA |= C1_4_PORTA[ IndexC1 ]; // C1-1...4 Up

	PORTA |= OutPortA;
	Log25V++;

	TCCR1B =  ( 1 << CS11   )|( 1 << CS10   );	// start Timer1 div 64
	TIMSK |=  ( 1 << OCIE1A )|( 1 << OCIE1B );	// Int On for compare reg A and reg B
	EIMSK &= ~( 1 << INT0 );					// INT0 off
}

static void setup( void )
{
	//--- init ports( 1-write 0-read ) ---
	DDRA  = ( 1 << DDA7 )|( 1 << DDA6 )|( 1 << DDA5 )|( 1 << DDA4 )|( 1 << DDA3 )|( 1 << DDA2 )|( 1 << DDA1 )|( 1 << DDA0 );
	DDRC  = ( 0 << DDC7 )|( 0 << DDC6 )|( 0 << DDC5 )|( 0 << DDC4 )|( 0 << DDC3 )|( 0 << DDC2 )|( 0 << DDC1 )|( 0 << DDC0 );
	DDRB  = ( 0 << DDB7 )|( 0 << DDB6 )|( 1 << DDB5 )|( 1 << DDB4 )|( 1 << DDB3 )|( 1 << DDB2 )|( 0 << DDB1 )|( 0 << DDB0 );
	PORTB = ( 0 << PB7  )|( 0 << PB6  )|( 0 << PB5  )|( 0 << PB4  )|( 0 << PB3  )|( 0 << PB2  )|( 0 << PB1  )|( 0 << PB0  );

	//--- init timers ---
	TIMSK  = 0, ETIMSK = 0;					// INT Off For All timers
	OCR1A  =  2 * ( F_CPU / 64 ) / 1000;	// Timer1 for C1-INT C1-1...4 Down( 2ms )
	OCR1B  = 16 * ( F_CPU / 64 ) / 1000;	// Timer1 for INT0 enadle( 16ms )
	TCCR3B = ( 1 << CS32 );					// Timer3 div 256
	OCR3A  = 800 * ( F_CPU / 256 ) / 1000;	// timer3 period 800 ms
	ETIMSK |= 1 << OCIE3A;					// INT ON timer3 compare reg A
	
	//--- Init Extern INT
	EIMSK = 0;				// Off all Extern INT
	EICRA = ( 1 << ISC01 );	// config INT0 1 -> 0
	EIMSK = ( 1 << INT0 );	// On Extern INT0

	//--- Init UART ---
#if defined UBRR1H
	UBRR1H = UBRRH_VALUE;
	UBRR1L = UBRRL_VALUE;
#endif
#if USE_2X
	UCSR1A |= ( 1 << U2X1 );
#else
	UCSR1A &= ~( 1 << U2X1 );
#endif
	UCSR1B = ( 1 << RXCIE1 )|( 1 << RXEN1 )|( 1 << TXEN1 );
	UCSR1C = ( 1 << UCSZ11 )|( 1 << UCSZ10 );
}

int main(void)
{
	cli();
	setup();
	sei();

    while (1) 
    {
		_delay_us( 1. );

		if( UartRecvEnd == UartState )
		{
			switch( InBuff[ 0 ])
			{
				case 0xA0:
					OutBuff[ 0 ] = 0xC0;
					OutBuff[ 1 ] = CmdData;
					OutBuff[ 2 ] = LogData;
					SendLen = 4;
					break;

				case 0xA1:
					CmdData = InBuff[ 1 ];

					if( CmdData & ERR89_PORTA ) PORTA |= ERR89_PORTA; else PORTA &= ~ERR89_PORTA;
					_delay_us( 1. );
					if( PINC & ERR89_PORTC ) LogData |= ERR_89_RES; else LogData &= ~ERR_89_RES;

					OutBuff[ 0 ] = 0xC1;
					OutBuff[ 1 ] = CmdData;
					OutBuff[ 2 ] = LogData;
					SendLen = 4;
					break;

				case 0xA2:
					OutBuff[ 0 ] = 0xC2;
					OutBuff[ 1 ] = VER_MAJOR;
					OutBuff[ 2 ] = VER_MINOR;
					SendLen = 4;
					break;

				case 0xA3:
					OutBuff[ 0 ] = 0xC3;
					sprintf(( char* )&( OutBuff[ 1 ]), "%s %s", __DATE__, __TIME__ );
					SendLen = 23;
					break;
				
				case 0xA4:
					OutBuff[ 0 ] = 0xC4;
					sprintf(( char* )&( OutBuff[ 1 ]), "%s", "Name" );
					SendLen = 22;
					break;

				default:
					continue;
			}

			OutBuff[ SendLen - 1 ] = Crc( OutBuff, SendLen - 1 );

			cli();
			UartState = UartTransBeg;
			SendNum = 0;
			UCSR1B |= ( 1 << UDRIE1 );
			sei();
			LED_PORT ^= LED_IO;
		}
    }

	return( 0 );
}
