#define _XTAL_FREQ 20000000

#include <xc.h>
#include <stdlib.h>
#include <stdint.h>

// BEGIN CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)
//END CONFIG

volatile unsigned int rise_time, current_period, timer_count, target_period, min_period, pot_output, target_direction, current_direction, b_on;

// interrupt service routine
void __interrupt () isr() {
    // ccp1 interrupt (velocity measurement)
    if (CCP1IF) {
        CCP1IF = 0; // clear flag
        timer_count = 0; // reset stop timer
        current_direction = b_on; // measure direction through quadrature interface
        current_period = CCPR1 - rise_time; // measure period as the time between two rising edges
        rise_time = CCPR1;
    }
    // ccp2 interrupt (quadrature interface)
    if (CCP2IF) {
        CCP2IF = 0;
        // CCP2M0 = 1: capture rising edge, = 0: capture falling edge
        if (CCP2M0) {
            b_on = 1;
            CCP2M0 = 0;
        } else {
            b_on = 0;
            CCP2M0 = 1;
        }
    }
    // timer 0 interrupt (stop detection)
    if (TMR0IF){
        TMR0IF = 0;
        timer_count++;
        if (timer_count == 38){
            current_period = 65535; // motor has stopped
            timer_count = 0;
        }
    }
}

void main() {
    // initialize ports and custom peripherals
    TRISB = 0b00000000; // RB0: Bang-bang, RB1,RB2: Direction
    TRISC = 0b00000110;
    // configuration registers
    INTCON = 0b11000000; // enable all unmasked and peripheral interrupts
    OPTION_REG = 0b00000111; // timer 0: internal oscillator, 1:256 prescaler
    T1CON = 0b00110001; // internal oscillator, 1:8 prescaler, enabled
    CCP1CON = 0b00000101; // capture on every rising edge
    CCP2CON = 0b00000101; // capture on every rising edge
    // initial values
    TMR0 = 0;
    timer_count = 0;
    TMR0IE = 1;
    CCP1IE = 1;
    CCP2IE = 1;
    min_period = 21000; // maximum velocity = lowest possible period
    current_period = 65535; // lowest velocity = highest possible period, assume the motor starts at rest
    RB0 = 0;
    target_direction = 0; // 0 = clockwise, 1 = counterclockwise
    while (1) {
        // change motor orientation
        if (target_direction){
            if(!(RB1 == 1 && RB2 == 0)) { // check if already set
                RB0 = 0;
                __delay_ms(5); // delay for motor acceleration constraints
                RB1 = 1; // RB1 and RB2 control the high power relays for the motor
                RB2 = 0;
                __delay_ms(5);
            }
        } else {
            if(!(RB1 == 0 && RB2 == 1)) {
                RB0 = 0;
                __delay_ms(5);
                RB1 = 0;
                RB2 = 1;
                __delay_ms(5);
            }
        }
        if(target_period < min_period) {
            target_period = min_period; // limit the max velocity
        }
        // bang-bang controller
        if (current_period < target_period) {
            RB0 = 0;
        }
        else {
            RB0 = 1;
        }
    }
}
