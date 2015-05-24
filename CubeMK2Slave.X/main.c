// TODO
// 1. Need to clear SPI rx buffer or interrupts if no data received in certain period of time - when reprogramming master, the slave locks up
//      likely due to garbage being spewed into or overflow of SPIBUF

#define _SUPPRESS_PLIB_WARNING

/* PIC32 Model PIC32MX250F128B */
#include <p32xxxx.h>    // pic32 library functions
#include <plib.h>       // peripheral library functions
#include <stdlib.h>
#include <stdint.h>

// Configuration Bits
#pragma config POSCMOD      = HS            // Primary oscillator using high speed crystal mode
#pragma config FNOSC        = PRIPLL        // Internal Fast RC oscillator (4 MHz) w/ PLL
#pragma config FPLLIDIV     = DIV_1         // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL      = MUL_20        // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV     = DIV_2         // Divide After PLL (now 40 MHz)
#pragma config FPBDIV       = DIV_1         // Divide core clock by 2 for peripheral bus (=20MHz Fpb)
#pragma config FWDTEN       = OFF           // Watchdog Timer Disabled
#pragma config ICESEL       = ICS_PGx1      // ICE/ICD Comm Channel Select
#pragma config JTAGEN       = OFF           // Disable JTAG
#pragma config FSOSCEN      = OFF           // Disable Secondary Oscillator

// main clock at 40MHz
#define SYS_FREQ    ( 40000000L )
#define SDTI        0x100       // data pin = RB8
#define SCKI        0x80        // clock pin = RB7
#define ROW_Q0      0x01
#define ROW_Q1      0x02
#define ROW_Q2      0x04
#define ROW_Q3      0x08
#define ROW_Q4      0x200
#define ROW_ALL_Q   ( ROW_Q0 | ROW_Q1 | ROW_Q2 | ROW_Q3 | ROW_Q4 )
#define LED_TOTAL   8           // this is temporary for testing and will change when I scale the project up
#define COUNT_MAX   ( LED_TOTAL / 4 )
#define MAX_ROW     4           // 0 inclusive
#define MAX_COL     7           //   0 inclusive


volatile int activeRow = 0;
volatile int bufferFull = 0;
// [ COL NUM ][ LED_NUM ][ RGB_NUM ]
volatile uint16_t colorBuff1[ 8 ][ 8 ][ 3 ];
/*** Might use the below buffers for double-buffering later ***/
// volatile uint16_t colorBuff2[ 8 ][ 8 ][ 3 ];
// uint16_t (*rxBuff)[ 8 ][ 3 ] = colorBuff1;
// uint16_t (*pBuff)[ 8 ][ 3 ] = colorBuff1;

// write cmd, params, BC for R, G, & B
uint8_t data_buff[ 4 ] = { 0x94, 0x50, 0x0F, 0xFF };
int dummy;

inline void ShiftWord( uint16_t word ) {
    int i;
    for ( i = 15; i >= 0; i-- ) {
        if ( ( word >> i ) & 0x01 ) {
            LATBSET = SDTI;
        }
        LATBSET = SCKI;
        LATBCLR = SCKI | SDTI;
    }
}

inline void ShiftByte( uint8_t byte ) {
    int i;
    for ( i = 7; i >= 0; i-- ) {
        if ( ( byte >> i ) & 0x01 )  {
            LATBSET = SDTI;
        }
        LATBSET = SCKI;
        LATBCLR = SCKI | SDTI;
    }
}

inline void Refresh() {
    int i, j, k;
    // Because we need to retransmit the entire 224-bit packet
    // for each TLC5971 in cascade, and thus each 4th RGB LED, I've wrapped the
    // refresh logic up in a convenient loop that can be configured to
    // easily adjust for variable cascade configurations
    for ( k = ( COUNT_MAX - 1 ); k >= 0; k-- ) {    // need to shift MSB out first for proper sequencing
        int iMin = 4*k;
        int iMax = iMin + 4;
        // Send header packet (MSB) first - 4 bytes
        ShiftByte( data_buff[ 0 ] );    // command 25h - latch
        ShiftByte( data_buff[ 1 ] );    // optional config params
        ShiftByte( data_buff[ 2 ] );    // BC control - B
        ShiftByte( data_buff[ 3 ] );    // BC control - G, R

        // Send color data packet - 24 bytes (12 words) - MSB SHIFTED FIRST
         for ( i = ( iMax - 1 ); i >= iMin; i-- ) {
            for ( j = 2; j >= 0; j-- ) {    // this line is OK
                ShiftWord( colorBuff1[ activeRow ][ i ][ j ] );
            }
        }
    }
    // Short delay before next refresh to allow internal latch to fire
    // in the TLC5971 (dwell for period 8x last SCKI strobe per documentation)
    // Also allows transistors to fully switch off before switching rows
    for( j = 0; j < 16; j++ ) {
       Nop();
    }
}

// Timer 2 interrupt handler
void __ISR ( _TIMER_2_VECTOR, ipl6 ) TMR2IntHandler( void ) {
    // Turn off all row select transistors   
    LATBSET = ROW_ALL_Q;

    // Update TLC5971 driver latches while all PNPs are OFF
    Refresh();

    // Mux
    switch ( activeRow ) {
        case 0:
            LATBCLR = ROW_Q0;
            break;
        case 1:
            LATBCLR = ROW_Q1;
            break;
        case 2:
            LATBCLR = ROW_Q2;
            break;
        case 3:
            LATBCLR = ROW_Q3;
            break;
        case 4:
            LATBCLR = ROW_Q4;
            break;

        default:
            break;
    }

   if ( ++activeRow > 4 ) {
        activeRow = 0;
   }
    
    mT2ClearIntFlag();
}

int main() {
    int row, col;
    uint16_t ledDesc, addr, color;
    uint32_t rxData;

    SYSTEMConfig( SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE );
    INTEnableSystemMultiVectoredInt();
    ANSELB = 0;
    // Set PORTB tristate pin modes
    TRISBbits.TRISB0 = 0;       // Row select Q0
    TRISBbits.TRISB1 = 0;       // Row select Q1
    TRISBbits.TRISB2 = 0;       // Row select Q2
    TRISBbits.TRISB3 = 0;       // Row select Q3
    TRISBbits.TRISB5 = 1;       // SDI1
    TRISBbits.TRISB7 = 0;       // PWM SCKI
    TRISBbits.TRISB8 = 0;       // PWM STDI
    TRISBbits.TRISB9 = 0;       // Row select Q4
    TRISBbits.TRISB13 = 0;      // SDO1

    // Set peripheral pin mappings
    PPSUnLock;
    PPSOutput( 3, RPB13, SDO1 );
    PPSInput( 2, SDI1, RPB5 );
    PPSLock;

    // Initialize Outputs - startup with row select PNP's off
    LATBSET = 0x20F;
    // channel 1, slave mode, 32 bit, clkDiv = 512 -> 40000000 / 714 = 56 kbaud
    SpiChnOpen( 1, SPI_OPEN_MODE32 | SPI_OPEN_SMP_END, 714 );

   // x_sec = (PR2 + 1) * TMR_PS / Fpb ->
   // where PR2 = period, PS = prescale, Fpb = peripheral bus frequency
        // x_sec = (PR2 + 1) * 8 / 40000000
        // -> PR2 = ( x_sec * 40000000 / 8 ) - 1
        // -> PR2 =  xxxx

   OpenTimer2( T2_ON | T2_PS_1_8 | T2_SOURCE_INT, 2600 );
   ConfigIntTimer2( T2_INT_ON | T2_INT_PRIOR_6 );
  
    srand( ReadCoreTimer() );
/*
    int test_rData[8] = {
            0x80003492,
            0x80013492,
            0x80023492,
            0x80033492,
            0x80043492,
            0x80053492,
            0x80063492,
            0x80073492
        };
 */

    while ( 1 ) {
        rxData = SpiChnGetC( 1 );    // Receive data on the slave channel when SPIBUF is ready
        SpiChnPutC( 1, rxData );     // Relay back data to master on next clock

        // decode data from packet and populate array
        // 32-bit packet structure: [ 2 bits color_desc ] [ 14 bits addr ] [ 16 bits LED value ]
        ledDesc = ( rxData >> 30 ) & 0x03;
        addr = ( rxData >> 16 ) & 0x3FFF;    // need to mask out the 14 addr bits
        color = rxData & 0xFFFF;             // need to mask out the 16 color bits (LSB)
        // convert to row/col matrix from linear address (range 0-39)
        col = addr % 8;
        row = addr / 8;

        // set the color of the referenced RGB
        if ( ledDesc == 0x00 ) {
            colorBuff1[ row ][ col ][ 0 ] = color;
        }

        if ( ledDesc == 0x01 ) {
            colorBuff1[ row ][ col ][ 1 ] = color;
        }

        if ( ledDesc == 0x02 ) {
            colorBuff1[ row ][ col ][ 2 ] = color;
        }
    }

    return ( EXIT_SUCCESS );
}