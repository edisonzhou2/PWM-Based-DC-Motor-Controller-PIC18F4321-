# DC Motor Control System (PIC18F4321)

An embedded system project that controls a DC motor’s speed using PWM, with dual ADC inputs, LCD feedback, and an interrupt-driven emergency stop. Built using the **PIC18F4321 microcontroller** and programmed in **C (XC8)** with **MPLAB X IDE**.

---

## 🔧 Features

- **Real-time motor speed control** using **Pulse Width Modulation (PWM)**.  
- **Dual ADC inputs**:
  - Potentiometer 1 → Controls maximum motor speed.  
  - Potentiometer 2 → Adjusts acceleration delay (0.5–10 seconds).  
- **Timer0 interrupt (10 ms)** controls the duty cycle ramping from 0% to the maximum speed.  
- **Interrupt-driven emergency stop** button (INT0) immediately halts motor operation.  
- **LCD display** shows real-time values for max speed, delay, and current state (RUN/STOP).  
- **Green LED** indicates PWM intensity; **Red LED** lights when the system is stopped.

---

## ⚙️ Hardware Components

| Component | Description |
|------------|--------------|
| **Microcontroller** | PIC18F4321 |
| **LCD Display** | 16x2 HD44780-compatible |
| **Motor** | DC brushed motor |
| **Transistor** | NPN (e.g., TIP120) or N-channel MOSFET (e.g., IRLZ44N) |
| **Flyback Diode** | 1N4007 or 1N5819 |
| **Potentiometers (x2)** | 10 kΩ – used for max speed and delay control |
| **Push Button** | Used for emergency stop interrupt |
| **LEDs** | Green (PWM indicator), Red (STOP indicator) |
| **Power Supply** | 5V for MCU/LCD, separate motor power recommended |

---

## 🔌 Pin Configuration (Recommended)

| Function | PIC Pin | Description |
|-----------|----------|--------------|
| LCD Data (D4–D7) | RB4–RB7 | 4-bit interface |
| LCD RS / E | RD0 / RD1 | LCD control lines |
| PWM Output | RC2 / CCP1 | Controls transistor gate/base |
| ADC Inputs | RA0, RA1 | Potentiometer wipers |
| Emergency Button | RB0 / INT0 | High-priority interrupt input |
| Green LED | RB1 | Mirrors PWM duty cycle |
| Red LED | RB2 | STOP indicator |

> ⚠️ **Tip:** Disconnect the LCD backlight if the power supply resets under load.

---

## 🧠 Firmware Overview

### Configuration Bits
```c
#pragma config FOSC = HS     // High-speed oscillator
#pragma config WDT = OFF     // Watchdog Timer off
#pragma config LVP = OFF     // Low-voltage programming off
#pragma config PBADEN = OFF  // PORTB<4:0> as digital
#pragma config MCLRE = ON    // MCLR enabled
#pragma config BOR = ON      // Brown-out Reset enabled
#pragma config PWRT = ON     // Power-up Timer enabled
