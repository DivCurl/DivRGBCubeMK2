/* PIC32 Model PIC32MX250F128B */
#define _SUPPRESS_PLIB_WARNING

#include <p32xxxx.h>    // pic32 library functions
#include <plib.h>       // peripheral library functions - deprecated in future XC32 versions
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
#define MAX_LED     40
#define MAX_ROW     4
#define MAX_COL     7
#define R_DESC      0x0
#define G_DESC      0x1
#define B_DESC      0x2

typedef struct {
    uint16_t r;
    uint16_t g;
    uint16_t b;
} rgb_t;

uint32_t txBuff = 0;
uint32_t rData;
uint16_t colorBuff1[ 8 ][ 8 ][ 3 ];
rgb_t rgbColorBuff[ 8 ][ 8 ];

int row = 0;
int seq = 0;
int count = 0;
int rgbAngle = 0;
int idxColor = 0;
int brightVal = 0;

// takes 8-bit RGB input value and maps to 16-bit value
uint16_t R_Downsample( uint8_t input ) {
    // 16-bit = total of 65535 values, 8-bit total of 255 values
    // For even distribution, 65535 / 255 = 257
    // -> output = input * 257
    return (uint16_t) ( input * 257 );
}

// note: valid angle must be between 0 & 360 deg - no bounds checking
rgb_t R_GetColorByAngle( float angle ) {
    rgb_t color;
    // 0 deg = 255, 0 0
    // 60 deg = 255, 255, 0
    // 120 deg = 0, 255, 0
    // 180 deg = 0, 255, 255
    // 240 deg = 0, 0 255
    // 300 deg = 255, 0, 255
    // delta-phi = 60 degrees / delta-x = 255

    // while loop to handle possible multiples of 360
    while ( angle >= 360 ) {
        angle = angle - 360;
    }

    // increase g value, hold red at max
    if ( ( angle >= 0 ) && ( angle < 60 ) ) {
        color.r = 0xFFFF;
        color.g = angle * 1092.25;
        color.b = 0;
        return color;
    }

    // decrease red value, hold g at max
    if ( ( angle >= 60 ) && ( angle < 120 ) ) {
        color.r = 0xFFFF - (angle-60) * 1092.25;
        color.g = 0xFFFF;
        color.b = 0;
        return color;
    }

    // increase blue value, hold g at max
    if ( ( angle >= 120 ) && ( angle < 180 ) ) {
        color.r = 0;
        color.g = 0xFFFF;
        color.b = (angle-120) * 1092.25;
        return color;
    }

    // decrease G value, hold B at max
    if ( ( angle >= 180 ) && ( angle < 240 ) ) {
        color.r = 0;
        color.g = 0xFFFF - (angle-180) * 1092.25;
        color.b = 0xFFFF;
        return color;
    }

    // increase R value, hold B at max
    if ( ( angle >= 240 ) && ( angle < 300 ) ) {
        color.r = (angle-240) * 1092.25;
        color.g = 0;
        color.b = 0xFFFF;
        return color;
    }

    // decrease B value, hold R at max
    if ( ( angle >= 300 ) && ( angle < 360 ) ) {
        color.r = 0xFFFF;
        color.g = 0;
        color.b = 0xFFFF - (angle-300) * 1092.25;
        return color;
    }

}

// Timer 2 interrupt handler - fires every 1ms
void __ISR ( _TIMER_2_VECTOR, ipl6 ) TMR2IntHandler( void ) {
    rgb_t tmpColor;
    // update brightness level every 50 mS
    // 256 total values (0-255) thus 12.8 seconds for full refresh @ 50 mS rate
    if ( ++count == 30 ) {   // 1 mS * 50 counts = 50mS
        // when full brightness is reached...

        if ( rgbAngle++ > 360 ) {
            rgbAngle = 0;
        }

        for ( idxColor = 0; idxColor < 8; idxColor++ ) {
            tmpColor = R_GetColorByAngle( rgbAngle + ( (idxColor+1)*45 ) );
            colorBuff1[ 0 ][ idxColor ][ 0 ] = tmpColor.r;
            colorBuff1[ 0 ][ idxColor ][ 1 ] = tmpColor.g;
            colorBuff1[ 0 ][ idxColor ][ 2 ] = tmpColor.b;
        }
        count = 0;
    }

    mT2ClearIntFlag();
}

int main() {
    int i, row, col;
    SYSTEMConfig( SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE );
    INTEnableSystemMultiVectoredInt();
    ANSELB = 0;     // set all analog pins to digital mode
    TRISBbits.TRISB5 = 1;       // set RB5 as output
    TRISBbits.TRISB7 = 0;       // set RB7 as output
    TRISBbits.TRISB8 = 0;       // set RB8 as output
    TRISBbits.TRISB13 = 0;       // set RB13 as output
    // Startup with all outputs OFF
    LATBCLR = 0xFFFF;

    PPSUnLock;
    PPSOutput( 3, RPB13, SDO1 );
    PPSInput( 2, SDI1, RPB5 );
    PPSLock;

    CloseTimer2();
    // initialize first LED to red

    // channel 1, slave mode, 16 bit - need to verify BAUD rate
    // bitrate=srcClk/(2*(SPIBRG+1)) The input parametes srcClkDiv specifies the srcClk divisor term (2*(SPIBRG+1)), so the BRG is calculated as SPIBRG=srcClkDiv/2-1.
    // with div = 1024 -> 20000000-1
    SpiChnOpen( 1, SPI_OPEN_MSTEN | SPI_OPEN_MODE32 | SPI_OPEN_SMP_END, 714 );  // with clkDiv = 512, 78.125kbps

   // for overall refresh rate of 1 mS
   // We want to find required PR2 for given delay time...
   // x_sec = (PR2 + 1) * TMR_PS / Fpb ->
   // where PR2 = period, PS = prescale, Fpb = peripheral bus frequency
        // .001 = (PR2 + 1) * 256 / 40000000
        // -> PR2 = ( .001 * 40000000 / 256 ) - 1
        // -> PR2 = 155

    OpenTimer2( T2_ON | T2_PS_1_256 | T2_SOURCE_INT, 155 );
    ConfigIntTimer2( T2_INT_ON | T2_INT_PRIOR_6 );
    mT2SetIntPriority( 6 );      // set timer2 int priority
    mT2ClearIntFlag();           // clear interrupt flag before startup
    mT2IntEnable( 1 );           // enable timer2 interrupts

    srand( ReadCoreTimer() );

    // Slight delay before starting data xfer
    for ( i = 0; i < 50000; i++ ) {
        Nop();
    }

    // packet format:
    // [ 2 bits color_desc ] [ 14 bits addr ] [ 16 bits LED value ]
    // color_desc is the color descriptor for the current LED: 00 = R, 01 = G, 10 = B, 11 = not used
    // addr is the current linear address in the cube space. 14 bits = 16384 total unique addresses
    // LED value is the PWM brightnes value (0-65535) of the current LED

    while ( 1 ) {
        row = col = 0;
        // convert to row/col matrix from linear address
        for ( i = 0; i < MAX_LED; i++ ) {
            // increment row every time we overflow the max column

            // build packet and transmit R data
            txBuff = colorBuff1[ row ][ col ][ 0 ];     // R data (16 bits)
            txBuff |= ( i << 16 );                  // address data (14 bits)
            txBuff |= ( R_DESC << 30 );             // LED descriptor (2 bits)
            SpiChnPutC( 1, txBuff );

            // build packet and transmit G data
            txBuff = colorBuff1[ row ][ col ][ 1 ];     // G data (16 bits)
            txBuff |= ( i << 16 );                  // address data (14 bits)
            txBuff |= ( G_DESC << 30 );             // LED descriptor (2 bits)
            SpiChnPutC( 1, txBuff );

            // build packet and transmit B data
            txBuff = colorBuff1[ row ][ col ][ 2 ];     // B data (16 bits)
            txBuff |= ( i << 16 );                  // address data (14 bits)
            txBuff |= ( B_DESC << 30 );             // LED descriptor (2 bits)
            SpiChnPutC( 1, txBuff );

            if ( col++ == MAX_COL ) {
                row++;
                col = 0;
            }
           
        }
    }

    return ( EXIT_SUCCESS );
}