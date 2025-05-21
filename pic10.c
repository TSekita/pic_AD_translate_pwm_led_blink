// PIC16F18857 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT1  // Power-up default value for COSC bits (HFINTOSC (1MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (FSCM timer disabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (VBOR = 1.9V on LF, 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (PPSLOCK can only be cleared and set once)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable bit (Stack Overflow/Underflow does not cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (1:65536 divider; WDTPS controlled by software)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open; software control)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config WRT = OFF        // UserNVM self-write protection bits (Write protection off)
#pragma config SCANE = not_available// Scanner Enable bit (Scanner module not available)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin is MCLR)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (Program memory code protection disabled)
#pragma config CPD = OFF        // DataNVM code protection bit (Data EEPROM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define _XTAL_FREQ 1000000 // Internal oscillator frequency in Hz

void ADC_Init(void);
unsigned int ADC_Read(unsigned char channel);
void pwm6_init(void);
void pwm6_set_duty(uint16_t duty);

void main(void) {
    ADC_Init();
    pwm6_init();

    while (1) {
        unsigned int adcVal = ADC_Read(0); // Read ADC from AN0
        pwm6_set_duty(adcVal);             // Apply to PWM
        __delay_ms(10);
    }
}

void ADC_Init(void) {
    // Set AN0 as analog input
    TRISAbits.TRISA0 = 1;     // Set as input
    ANSELAbits.ANSA0 = 1;     // Enable analog function

    // Set ADCON1: clock source and reference voltage
    ADCON0bits.ADFRM0 = 1;    // Right justified (10-bit)
    ADCLKbits.ADCCS = 0x3F;   // ADC clock = Fosc/128
    ADREFbits.ADNREF = 0;     // Negative reference = Vss
    ADREFbits.ADPREF = 0b00;  // Positive reference = Vdd

    // Initial channel selection (AN0), ADC enabled
    ADPCHbits.ADPCH = 0b00000; // Select AN0
    ADCON0bits.ADON = 1;       // Enable ADC module
}

unsigned int ADC_Read(unsigned char channel) {
    // Select channel (0 to 35)
    ADPCHbits.ADPCH = channel;
    __delay_us(10); // Wait for channel to stabilize

    ADCON0bits.ADGO = 1; // Start conversion
    while (ADCON0bits.ADGO); // Wait for completion

    // Get result (10-bit right justified)
    return (unsigned int)((ADRESH << 8) | ADRESL);
}

void pwm6_init(void)
{
    // Set PWM output pin
    TRISAbits.TRISA4 = 0;       // RA4 output
    ANSELAbits.ANSA4 = 0;       // RA4 digital mode

    // PPS setting (PWM6 -> RA4)
    PPSLOCK = 0x55;             // Unlock sequence step 1
    PPSLOCK = 0xAA;             // Unlock sequence step 2
    PPSLOCKbits.PPSLOCKED = 0;  // Unlock PPS

    RA4PPS = 0x0E;              // Assign PWM6 to RA4
    __delay_us(10);

    PPSLOCK = 0x55;             // Lock sequence step 1
    PPSLOCK = 0xAA;             // Lock sequence step 2
    PPSLOCKbits.PPSLOCKED = 1;  // Lock PPS

    // Timer2 setting (PWM clock source)
    PR2 = 0xF9;                 // PWM period setting (max 255 = 0xFF)
    T2CONbits.T2CKPS = 0b000;   // Prescaler 1:1
    T2CLKCONbits.CS = 0b0001;   // Clock source = FOSC/4 = 1MHz / 4 = 250kHz
    PIR4bits.TMR2IF = 0;        // Clear timer flag
    T2CONbits.TMR2ON = 1;       // Start Timer2

    while (!PIR4bits.TMR2IF);   // Wait for Timer2 to stabilize

    // PWM6 setup
    PWM6DCH = 0;                // Initialize high duty register
    PWM6DCL = 0;                // Initialize low duty register
    PWM6CONbits.PWM6POL = 0;    // Active high
    PWM6CONbits.PWM6EN = 1;     // Enable PWM6
}

void pwm6_set_duty(uint16_t duty)
{
    if (duty > 1023) duty = 1023;
    PWM6DCH = (uint8_t)(duty >> 2);              // High 8 bits
    PWM6DCL = (uint8_t)((duty & 0x03) << 6);     // Low 2 bits
}
