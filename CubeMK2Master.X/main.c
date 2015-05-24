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

enum {
    AN_TEST,
    AN_TEXT
};

volatile uint16_t colorBuff[ 8 ][ 8 ][ 3 ];
volatile rgb_t rgbColorBuff[ 8 ][ 8 ];
volatile int count = 0;
volatile int rgbAngle = 0;
volatile int idxColor = 0;
volatile int brightVal = 0;
volatile int gDelay = 0;
int delayDn = 0;
int anim = 0;

const unsigned char ascii[455] = {
	0x00,0x00,0x00,0x00,0x00,0x00,0x5f,0x5f,0x00,0x00,	//  !
	0x00,0x03,0x00,0x03,0x00,0x14,0x7f,0x14,0x7f,0x14,	// "#
	0x24,0x2a,0x7f,0x2a,0x12,0x23,0x13,0x08,0x64,0x62,	// $%
	0x36,0x49,0x55,0x22,0x50,0x00,0x05,0x03,0x00,0x00,	// &'
	0x00,0x1c,0x22,0x41,0x00,0x00,0x41,0x22,0x1c,0x00,	// ()
	0x14,0x08,0x3e,0x08,0x14,0x08,0x08,0x3e,0x08,0x08,	// *+
	0x00,0x50,0x30,0x00,0x00,0x08,0x08,0x08,0x08,0x08,	// ,-
	0x00,0x60,0x60,0x00,0x00,0x20,0x10,0x08,0x04,0x02,	// ./
	0x3e,0x51,0x49,0x45,0x3e,0x00,0x42,0x7f,0x40,0x00,	// 01
	0x42,0x61,0x51,0x49,0x46,0x21,0x41,0x45,0x4b,0x31,	// 23
	0x18,0x14,0x12,0x7f,0x10,0x27,0x45,0x45,0x45,0x39,	// 45
	0x3c,0x4a,0x49,0x49,0x30,0x01,0x71,0x09,0x05,0x03,	// 67
	0x36,0x49,0x49,0x49,0x36,0x06,0x49,0x49,0x29,0x1e,	// 89
	0x00,0x36,0x36,0x00,0x00,0x00,0x56,0x36,0x00,0x00,	// :;
	0x08,0x14,0x22,0x41,0x00,0x14,0x14,0x14,0x14,0x14,	// <=
	0x00,0x41,0x22,0x14,0x08,0x02,0x01,0x51,0x09,0x06,	// >?
	0x32,0x49,0x79,0x41,0x3e,0x7e,0x11,0x11,0x11,0x7e,	// @A
	0x7f,0x49,0x49,0x49,0x36,0x3e,0x41,0x41,0x41,0x22,	// BC
	0x7f,0x41,0x41,0x22,0x1c,0x7f,0x49,0x49,0x49,0x41,	// DE
	0x7f,0x09,0x09,0x09,0x01,0x3e,0x41,0x49,0x49,0x7a,	// FG
	0x7f,0x08,0x08,0x08,0x7f,0x00,0x41,0x7f,0x41,0x00,	// HI
	0x20,0x40,0x41,0x3f,0x01,0x7f,0x08,0x14,0x22,0x41,	// JK
	0x7f,0x40,0x40,0x40,0x40,0x7f,0x02,0x0c,0x02,0x7f,	// LM
	0x7f,0x04,0x08,0x10,0x7f,0x3e,0x41,0x41,0x41,0x3e,	// NO
	0x7f,0x09,0x09,0x09,0x06,0x3e,0x41,0x51,0x21,0x5e,	// PQ
	0x7f,0x09,0x19,0x29,0x46,0x46,0x49,0x49,0x49,0x31,	// RS
	0x01,0x01,0x7f,0x01,0x01,0x3f,0x40,0x40,0x40,0x3f,	// TU
	0x1f,0x20,0x40,0x20,0x1f,0x3f,0x40,0x38,0x40,0x3f,	// VW
	0x63,0x14,0x08,0x14,0x63,0x07,0x08,0x70,0x08,0x07,	// XY
	0x61,0x51,0x49,0x45,0x43,0x00,0x7f,0x41,0x41,0x00,	// Z[
	0x02,0x04,0x08,0x10,0x20,0x00,0x41,0x41,0x7f,0x00,	// \]
	0x04,0x02,0x01,0x02,0x04,0x40,0x40,0x40,0x40,0x40,	// ^_
	0x00,0x01,0x02,0x04,0x00,0x20,0x54,0x54,0x54,0x78,	// `a
	0x7f,0x48,0x44,0x44,0x38,0x38,0x44,0x44,0x44,0x20,	// bc
	0x38,0x44,0x44,0x48,0x7f,0x38,0x54,0x54,0x54,0x18,	// de
	0x08,0x7e,0x09,0x01,0x02,0x0c,0x52,0x52,0x52,0x3e,	// fg
	0x7f,0x08,0x04,0x04,0x78,0x00,0x44,0x7d,0x40,0x00,	// hi
	0x20,0x40,0x44,0x3d,0x00,0x7f,0x10,0x28,0x44,0x00,	// jk
	0x00,0x41,0x7f,0x40,0x00,0x7c,0x04,0x18,0x04,0x78,	// lm
	0x7c,0x08,0x04,0x04,0x78,0x38,0x44,0x44,0x44,0x38,	// no
	0x7c,0x14,0x14,0x14,0x08,0x08,0x14,0x14,0x18,0x7c,	// pq
	0x7c,0x08,0x04,0x04,0x08,0x48,0x54,0x54,0x54,0x20,	// rs
	0x04,0x3f,0x44,0x40,0x20,0x3c,0x40,0x40,0x20,0x7c,	// tu
	0x1c,0x20,0x40,0x20,0x1c,0x3c,0x40,0x30,0x40,0x3c,	// vw
	0x44,0x28,0x10,0x28,0x44,0x0c,0x50,0x50,0x50,0x3c,	// xy
	0x44,0x64,0x54,0x4c,0x44				// z
};

// Assigns the input RGB color to the specified voxel
void V_Set( int x, int y, rgb_t color ) {
    colorBuff[ x ][ y ][ 0 ] = color.r;
    colorBuff[ x ][ y ][ 1 ] = color.g;
    colorBuff[ x ][ y ][ 2 ] = color.b;
}

void V_Clear( int x, int y ) {
    colorBuff[ x ][ y ][ 0 ] = 0;
    colorBuff[ x ][ y ][ 1 ] = 0;
    colorBuff[ x ][ y ][ 2 ] = 0;
}

void F_Getchar (char chr, unsigned char *dst) {
    uint8_t i;
    // our bitmap font starts at ascii char 32 (decimal, space char). This ensures the input
    // ascii character is aligned with the start of the bitmap.
    chr -= 32;

    // loop through each byte of the array and store slice in dst[i] until entire char is built
    for ( i = 0; i < 5; i++ ) {
		dst[i] = ascii[ ( chr * 5 ) + i ] ;
    }
}

void F_SendText(const char *str, rgb_t color) {
    uint16_t x, y, i;
    static uint16_t shifts;
    unsigned char chr[5];   // stores the current character

        // Loop through each character in the string until null byte is hit
        while ( *str ) {
            // Get the current character in the pointer and move it into chr array ( 5 bytes )

            /*  !!! adapt code below to displaye one column of text at a time and scroll right !!! */
            F_Getchar( *str++, chr );

            for (x = 0; x < 5; x++) {
                for (y = 0; y < 8; y++)	{
                    if (chr[x] & (0x80 >> y)) {
                        V_Set(x, 0, color);
                    }
                }
            }

            /*
            for (i = 0; i < 5; i++) {
                // block and delay for a moment
    //            D_Shift('Y', SHIFT_OUT);
            }
             */
        }   // end while

    // make sure last character is shifted out of the cube once
    // we break out of the while loop
    for (i = 0; i < 8; i++) {
      // block and delay for a moment
      // D_Shift('Y', SHIFT_OUT);
    }
}

// takes 8-bit RGB input value and maps to 16-bit value
uint16_t R_Downsample( uint8_t input ) {
    // 16-bit = total of 65535 values, 8-bit total of 255 values
    // For even distribution, 65535 / 255 = 257
    // -> output = input * 257
    return ( uint16_t ) ( input * 257 );
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

void A_Test() {
    rgb_t tmpColor;
    
    if ( (rgbAngle += 5 ) > 360 ) {
            rgbAngle = 0;
    }

    for ( idxColor = 0; idxColor < 8; idxColor++ ) {
        tmpColor = R_GetColorByAngle( rgbAngle + ( ( idxColor + 1) * 25 ) );
        colorBuff[ 0 ][ idxColor ][ 0 ] = tmpColor.r;
        colorBuff[ 0 ][ idxColor ][ 1 ] = tmpColor.g;
        colorBuff[ 0 ][ idxColor ][ 2 ] = tmpColor.b;

        tmpColor = R_GetColorByAngle( rgbAngle + ( ( idxColor + 2) * 25 ) );
        colorBuff[ 1 ][ idxColor ][ 0 ] = tmpColor.r;
        colorBuff[ 1 ][ idxColor ][ 1 ] = tmpColor.g;
        colorBuff[ 1 ][ idxColor ][ 2 ] = tmpColor.b;

        tmpColor = R_GetColorByAngle( rgbAngle + ( ( idxColor + 3) * 25 ) );
        colorBuff[ 2 ][ idxColor ][ 0 ] = tmpColor.r;
        colorBuff[ 2 ][ idxColor ][ 1 ] = tmpColor.g;
        colorBuff[ 2 ][ idxColor ][ 2 ] = tmpColor.b;

        tmpColor = R_GetColorByAngle( rgbAngle + ( ( idxColor + 4) * 25 ) );
        colorBuff[ 3 ][ idxColor ][ 0 ] = tmpColor.r;
        colorBuff[ 3 ][ idxColor ][ 1 ] = tmpColor.g;
        colorBuff[ 3 ][ idxColor ][ 2 ] = tmpColor.b;

        tmpColor = R_GetColorByAngle( rgbAngle + ( ( idxColor + 5) * 25 ) );
        colorBuff[ 4 ][ idxColor ][ 0 ] = tmpColor.r;
        colorBuff[ 4 ][ idxColor ][ 1 ] = tmpColor.g;
        colorBuff[ 4 ][ idxColor ][ 2 ] = tmpColor.b;

    }
}

void D_msDelay ( unsigned int delay ) {
    OpenTimer1( T1_ON | T1_PS_1_256, 0xFFFF );
    
    while ( delay-- ) {
      // t x 1ms loop
        WriteTimer1( 0 );
        while ( ReadTimer1() < ( SYS_FREQ / 256 / 1000 ) );
    }

    CloseTimer1();
}

// Timer 2 interrupt handler - fires every 1ms
void __ISR ( _TIMER_2_VECTOR, ipl6 ) TMR2IntHandler( void ) {
    // update brightness level every 50 mS
    // 256 total values (0-255) thus 12.8 seconds for full refresh @ 50 mS rate
    if ( count < gDelay ) {
        count++;
    }
    
    mT2ClearIntFlag();
}

int main() {
    int i, row, col;
    uint32_t txBuff, rxData;
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
        // convert to row/col matrix from linear address
        for ( i = 0; i < MAX_LED; i++ ) {
            col = i % 8;
            row = i / 8;

            // build packet and transmit R data
            txBuff = colorBuff[ row ][ col ][ 0 ];     // R data (16 bits)
            txBuff |= ( i << 16 );                  // address data (14 bits)
            txBuff |= ( R_DESC << 30 );             // LED descriptor (2 bits)
            SpiChnPutC( 1, txBuff );

            // build packet and transmit G data
            txBuff = colorBuff[ row ][ col ][ 1 ];     // G data (16 bits)
            txBuff |= ( i << 16 );                  // address data (14 bits)
            txBuff |= ( G_DESC << 30 );             // LED descriptor (2 bits)
            SpiChnPutC( 1, txBuff );

            // build packet and transmit B data
            txBuff = colorBuff[ row ][ col ][ 2 ];     // B data (16 bits)
            txBuff |= ( i << 16 );                  // address data (14 bits)
            txBuff |= ( B_DESC << 30 );             // LED descriptor (2 bits)
            SpiChnPutC( 1, txBuff );
           
        }
    }

    return ( EXIT_SUCCESS );
}