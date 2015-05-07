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
#define LED_TOTAL   8           // this is temporary for testing and will change when I scale the project up
#define COUNT_MAX   ( LED_TOTAL / 4 )

int col = 0;
uint32_t rData;
int bufferFull = 0;

// [ COL NUM ][ LED_NUM ][ RGB_NUM ]
uint16_t colorBuff1[ 8 ][ 8 ][ 3 ];
uint16_t colorBuff2[ 8 ][ 8 ][ 3 ];
uint16_t (*rxBuff)[ 8 ][ 3 ] = colorBuff1;
uint16_t (*pBuff)[ 8 ][ 3 ] = colorBuff1;
uint16_t ledDesc, addr, color;
// write cmd, params, BC for R, G, & B
uint8_t data_buff[ 4 ] = { 0x94, 0x50, 0x0F, 0xFF };
int dummy;

// Timer 2 interrupt handler
void __ISR ( _TIMER_2_VECTOR, ipl6 ) TMR2IntHandler( void ) {
        Refresh();
        if ( col++ > 0 ) {  // enforce column stays at zero for now
            col = 0;
        }
    mT2ClearIntFlag();
}

void ShiftWord( uint16_t word ) {
    int i;
    for ( i = 15; i >= 0; i-- ) {
        if ( ( word >> i ) & 0x01 ) {
            LATBSET = SDTI;
        }
        LATBSET = SCKI;
        LATBCLR = SCKI | SDTI;
    }
}

void ShiftByte( uint8_t byte ) {
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
        for ( i = iMin; i < iMax; i++ ) {
            for ( j = 2; j >= 0; j-- ) {
                ShiftWord( pBuff[ col ][ i ][ j ] );
            }
        }
    }
    // Short delay before next refresh to allow internal latch to fire
    // in the TLC5971 (dwell for period 8x last SCKI strobe per documentation)
    for( j = 0; j < 8; j++ ) {
            Nop();
    }
}

int main() {
    // the usual optimization macro
    SYSTEMConfig( SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE );
    INTEnableSystemMultiVectoredInt();
    // AD0PCFG = 0xFFFF;
    ANSELB = 0;
    TRISBbits.TRISB5 = 1;       // set RB5 as output
    TRISBbits.TRISB7 = 0;       // set RB5 as output
    TRISBbits.TRISB8 = 0;       // set RB5 as output
    TRISBbits.TRISB13 = 0;       // set RB5 as output
    // Startup with all outputs OFF
    LATBCLR = 0xFFFF;

    // Set peripheral pin mappings
    PPSUnLock;                        
    PPSOutput( 3, RPB13, SDO1 );
    PPSInput( 2, SDI1, RPB5 );
    PPSLock;

    CloseTimer2();

    // channel 1, slave mode, 16 bit - need to verify BAUD rate
    SpiChnOpen( 1, SPI_OPEN_MODE32 | SPI_OPEN_SMP_END, 512 );

   // for overall refresh rate of 150 hz, with 8 passes = 1200 Hz, or 833 uS
   // We want to find required PR2 for given delay time...
   // x_sec = (PR2 + 1) * TMR_PS / Fpb ->
   // where PR2 = period, PS = prescale, Fpb = peripheral bus frequency
        // .00083 = (PR2 + 1) * 8 / 20000000
        // -> PR2 = ( .00083 * 40000000 / 8 ) - 1
        // -> PR2 = 414

   // Set timer on, prescaler = 1:8, PR2 = 414
    OpenTimer2( T2_ON | T2_PS_1_8 | T2_SOURCE_INT, 414 );
    ConfigIntTimer2( T2_INT_ON | T2_INT_PRIOR_6 );
    mT2SetIntPriority( 6 );      // set timer2 int priority
    mT2ClearIntFlag();           // clear interrupt flag before startup
    mT2IntEnable( 1 );           // enable timer2 interrupts

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

        rData = SpiChnGetC( 1 );    // Receive data on the slave channel
        SpiChnPutC( 1, rData );     // Relay back data to master on next clock
        
        // decode data from packet and populate array
        // 32-bit packet structure: [ 2 bits color_desc ] [ 14 bits addr ] [ 16 bits LED value ]
        ledDesc = ( rData >> 30 ) & 0x03;
        addr = ( rData >> 16 ) & 0x3FFF;    // need to mask out the 14 addr bits
        color = rData & 0xFFFF;             // need to mask out the 16 color bits (LSB)

        if ( ledDesc == 0x00 ) {
            pBuff[ 0 ][ addr ][ 0 ] = color;
        }

        if ( ledDesc == 0x01 ) {
            pBuff[ 0 ][ addr ][ 1 ] = color;
        }

        if ( ledDesc == 0x02 ) {
            pBuff[ 0 ][ addr ][ 2 ] = color;
        }

        // experimenting with double buffering...TODO
        /*
        if ( ( addr >= 7 ) && !bufferFull ) {
            bufferFull = 1;
            if ( rxBuff == colorBuff1 ) {
                rxBuff = colorBuff2;
                pBuff = colorBuff1;
            } else {
                rxBuff = colorBuff1;
                pBuff = colorBuff2;
            }
        }
         */

    }

    return ( EXIT_SUCCESS );
}