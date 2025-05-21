
// PIC16F18857 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT1  // Power-up default value for COSC bits (HFINTOSC (1MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (FSCM timer disabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will not cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config WRT = OFF        // UserNVM self-write protection bits (Write protection off)
#pragma config SCANE = not_available// Scanner Enable bit (Scanner module is not available for use)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (Program Memory code protection disabled)
#pragma config CPD = OFF        // DataNVM code protection bit (Data EEPROM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define _XTAL_FREQ 1000000 //Internal oscillator Hz

void ADC_Init(void);
unsigned int ADC_Read(unsigned char channel);
void pwm6_init(void);
void pwm6_set_duty(uint16_t duty);

void main(void) {
    ADC_Init();
    pwm6_init();

    while (1) {
        unsigned int adcVal = ADC_Read(0); // AN0からADC読み取り
        pwm6_set_duty(adcVal);              // PWMに反映
        __delay_ms(10);
    }
}

void ADC_Init(void) {
    // AN0をアナログ入力に設定
    TRISAbits.TRISA0 = 1;     // 入力に設定
    ANSELAbits.ANSA0 = 1;    // アナログ機能を有効

    // ADCON1設定：クロックソースと基準電圧
    ADCON0bits.ADFRM0 = 1;    // 右詰め（10bit）
    ADCLKbits.ADCCS = 0x3F; // ADCクロック = Fosc/64
    ADREFbits.ADNREF = 0;  // 負基準 = Vss
    ADREFbits.ADPREF = 0b00; // 正基準 = Vdd

    // 初期チャネル選択（AN0）、ADC無効
    ADPCHbits.ADPCH = 0b00000; // AN0選択
    ADCON0bits.ADON = 1;      // ADCモジュールを有効化
}

unsigned int ADC_Read(unsigned char channel) {
    // チャネル選択（0～35）
    ADPCHbits.ADPCH = channel;
    __delay_us(10); // チャネル切替後の安定待ち

    ADCON0bits.GO_nDONE = 1; // 変換開始
    while (ADCON0bits.GO_nDONE); // 完了待ち

    // 結果取得（10ビット右詰め）
    return (unsigned int)((ADRESH << 8) | ADRESL);
}

void pwm6_init(void)
{
     // PWM out pin setting
    TRISAbits.TRISA4 = 0;       // RA4 output
    ANSELAbits.ANSA4 = 0;       // RA4 digital
    
    // PPS setting (PWM6 -> RA4)
    PPSLOCK = 0x55;             // charm
    PPSLOCK = 0xAA;             // charm
    PPSLOCKbits.PPSLOCKED = 0;  // PPS UNLOCKED

    RA4PPS = 0x0E; // RA4 assign PWM6
    __delay_us(10);

    PPSLOCK = 0x55;             // charm
    PPSLOCK = 0xAA;             // charm
    PPSLOCKbits.PPSLOCKED = 1;  // PPS LOCKED

    // Timer2 setting (PWM clock)
    PR2 = 0xF9;                 // PWM period setting MAX 255(0xFF)
    T2CONbits.T2CKPS = 0b000;    // prescaler 1:1
    T2CLKCONbits.CS = 0b0001;  // FOSC/4 = 1MHz / 4 = 250kHz
    PIR4bits.TMR2IF = 0;        // clear flag
    T2CONbits.TMR2ON = 1;       // Timer start

    while (!PIR4bits.TMR2IF);   // Timer2 waiting for stability

    // PWM6 setting
    PWM6DCH = 0;                // duty init higher rank
    PWM6DCL = 0;                // duty init lower rank
    PWM6CONbits.PWM6POL = 0;    // active high
    PWM6CONbits.PWM6EN = 1;     // PWM6 enabled
}

void pwm6_set_duty(uint16_t duty)
{
    if (duty > 1023) duty = 1023;
    PWM6DCH = (uint8_t)(duty >> 2);
    PWM6DCL = (uint8_t)((duty & 0x03) << 6);
}
