#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 1000000UL

#pragma config OSC = INTIO2, WDT = OFF, LVP = OFF
#pragma config PBADEN = DIG
#pragma config CCP2MX = RC1
#pragma config MCLRE = OFF

#define DSHIFT 4
#define LCD_RS  LATEbits.LATE0
#define LCD_RW  LATEbits.LATE1
#define LCD_E   LATEbits.LATE2

#define STOP_LED LATBbits.LATB1

volatile uint8_t  stopped = 0;
volatile uint8_t  lcd_status_dirty = 1;
volatile uint8_t  int0_lock = 0;

volatile uint16_t duty_current = 0;
volatile uint16_t duty_target  = 0;
volatile uint16_t duty_step    = 1;
volatile uint16_t delay_ms     = 0;
volatile uint16_t delay_ticks  = 1;
volatile uint16_t tick_counter = 0;

volatile uint16_t step_base = 0;
volatile uint16_t step_bonus_ticks = 0;
volatile uint16_t bonus_progress = 0;

static inline void lcd_put4_raw(uint8_t nib){
    uint8_t t0 = INTCONbits.TMR0IE; INTCONbits.TMR0IE = 0;
    LATD = (LATD & ~(0x0Fu << DSHIFT)) | ((nib & 0x0F) << DSHIFT);
    LCD_E = 1; __delay_us(5); LCD_E = 0; __delay_us(50);
    INTCONbits.TMR0IE = t0;
}
static void lcd_cmd4(uint8_t b){ LCD_RS=0; LCD_RW=0; lcd_put4_raw(b>>4); lcd_put4_raw(b&0x0F); }
static void lcd_dat4(uint8_t b){ LCD_RS=1; LCD_RW=0; lcd_put4_raw(b>>4); lcd_put4_raw(b&0x0F); }
static void lcd_goto(uint8_t r,uint8_t c){ lcd_cmd4(0x80 + (r?0x40:0x00) + c); }
static void lcd_print(const char*s){ while(*s) lcd_dat4(*s++); }
static void lcd_uint4(uint16_t v){ char b[5]; b[4]='\0'; for(int i=3;i>=0;i--){ b[i]='0'+(v%10); v/=10;} lcd_print(b); }

static void lcd_init4(void){
    TRISD=0x00; TRISE=0x00; LATD=0x00; LATE=0x00; LCD_RW=0;  __delay_ms(40);
    LCD_RS=0; LCD_RW=0;
    lcd_put4_raw(0x03); __delay_ms(5);
    lcd_put4_raw(0x03); __delay_ms(5);
    lcd_put4_raw(0x03); __delay_ms(5);
    lcd_put4_raw(0x02); __delay_ms(1);
    lcd_cmd4(0x28); lcd_cmd4(0x08);
    lcd_cmd4(0x01); __delay_ms(2);
    lcd_cmd4(0x06); lcd_cmd4(0x0C);
    lcd_goto(0,0);  lcd_print("Max:");
    lcd_goto(1,0);  lcd_print("Delay:");
    lcd_goto(1,12); lcd_print("RUN ");
}

static void adc_init(void){
    ADCON1 = 0x0C;
    ADCON2 = 0b10001001;
    ADCON0 = 0x01;
    TRISAbits.TRISA0 = 1;
    TRISAbits.TRISA1 = 1;
}
static uint16_t adc_read(uint8_t ch){
    ADCON0bits.CHS = ch & 0x0F; __delay_us(5);
    ADCON0bits.GO = 1; while(ADCON0bits.GO);
    return ((uint16_t)ADRESH<<8) | ADRESL;
}

static void pwm_init(void){
    TRISCbits.TRISC1 = 0;
    PR2   = 249;
    T2CON = 0b00000100;
    CCP2CON = 0b00001100;
}
static void pwm_set(uint16_t duty10){
    if(duty10>1023) duty10=1023;
    uint32_t counts = (uint32_t)duty10 * 1000u + 511u;
    counts /= 1023u;
    uint16_t reg = ((uint16_t)counts) << 2;
    CCPR2L = (reg >> 2) & 0xFF; CCP2CONbits.DC2B = reg & 0x03;
}

static void tmr0_init(void){
    T0CON = 0b00000001;
    TMR0H = 0xFD; TMR0L = 0xCF;
    INTCON2bits.TMR0IP = 0;
    INTCONbits.TMR0IF = 0;
    INTCONbits.TMR0IE = 1;
    T0CONbits.TMR0ON = 1;
}

static void int0_init(void){
    TRISBbits.TRISB0 = 1;
    TRISBbits.TRISB1 = 0; STOP_LED = 0;
    INTCON2bits.INTEDG0 = 0;
    INTCONbits.INT0IF = 0;
    INTCONbits.INT0IE = 1;
}

void __interrupt(high_priority) isr_high(void){
    if (INTCONbits.INT0IF){
        INTCONbits.INT0IF = 0;
        if (int0_lock == 0){
            stopped ^= 1;
            lcd_status_dirty = 1;
            if (stopped){
                pwm_set(0);
                STOP_LED = 1;
            } else {
                STOP_LED = 0;
                tick_counter = 0;
                bonus_progress = 0;
                duty_current = 0;
            }
            int0_lock = 5;
        }
    }
}

void __interrupt(low_priority) isr_low(void){
    if (INTCONbits.TMR0IF){
        INTCONbits.TMR0IF = 0;
        TMR0H = 0xFD; TMR0L = 0xCF;

        if (int0_lock) int0_lock--;

        if(!stopped){
            if (delay_ms == 0){
                duty_current = duty_target;
                tick_counter = 0;
                bonus_progress = 0;
            } else {
                uint16_t add = step_base;
                if (bonus_progress < step_bonus_ticks){
                    add++; bonus_progress++;
                }
                duty_current += add;
                if (duty_current > duty_target) duty_current = duty_target;

                tick_counter++;
                if (tick_counter >= delay_ticks){
                    tick_counter = 0;
                    duty_current = 0;
                    bonus_progress = 0;
                }
            }
            pwm_set(duty_current);
        }
    }
}

int main(void){
    OSCCONbits.IRCF = 0b100;
    OSCCONbits.SCS  = 0b10;

    TRISEbits.PSPMODE = 0;
    ADCON1 = 0x0C;

    lcd_init4();
    adc_init();
    pwm_init();
    tmr0_init();
    int0_init();

    RCONbits.IPEN = 1;
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;

    while(1){
        if (lcd_status_dirty){
            lcd_goto(1,12); lcd_print(stopped ? "STOP" : "RUN ");
            lcd_status_dirty = 0;
        }

        if (!stopped){
            uint16_t a0 = adc_read(0);
            uint16_t a1 = adc_read(1);

            duty_target = a0;

            uint16_t new_delay_ms = (uint16_t)(((uint32_t)a1 * 10023u + 511u) / 1023u);
            uint16_t new_delay_ticks = (new_delay_ms == 0) ? 1 : (uint16_t)((new_delay_ms + 9u) / 10u);

            uint8_t t0 = INTCONbits.TMR0IE; INTCONbits.TMR0IE = 0;

            delay_ms    = new_delay_ms;
            delay_ticks = new_delay_ticks;

            if (delay_ms == 0){
                step_base = duty_target;
                step_bonus_ticks = 0;
            } else if (duty_target == 0){
                step_base = 0;
                step_bonus_ticks = 0;
                bonus_progress = 0;
                duty_current = 0;
                tick_counter = 0;
            } else {
                step_base = (uint16_t)(duty_target / delay_ticks);
                step_bonus_ticks = (uint16_t)(duty_target % delay_ticks);
            }

            INTCONbits.TMR0IE = t0;

            lcd_goto(0,5);  lcd_uint4(duty_target);
            lcd_goto(1,6);  lcd_uint4(delay_ms);
        }

        __delay_ms(80);
    }
}
