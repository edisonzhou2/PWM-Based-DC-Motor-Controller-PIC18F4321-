# DC Motor Control System (PIC18F4321)

An embedded system project that controls a DC motor’s speed using **Pulse Width Modulation (PWM)**, with **dual ADC inputs**, a **16×2 LCD display**, and an **interrupt-driven emergency stop**.  
Built using the **PIC18F4321 microcontroller** and programmed in **C (XC8)** within **MPLAB X IDE**.

---

## 🔧 Features

- **PWM-based speed control** using **CCP2 (RC1)** output  
- **Dual ADC channels** for analog control:
  - **RA0 (AN0)** → maximum speed  
  - **RA1 (AN1)** → ramp-up delay (0 – 10 s)
- **Timer0 interrupt (~9 ms)** handles PWM duty-cycle ramping  
- **INT0 pushbutton interrupt** provides an emergency stop/resume  
- **16×2 LCD** shows current max speed, delay, and system state (“RUN” / “STOP”)  
- **Red LED (RB1)** lights when the motor is stopped  

---

## ⚙️ Hardware Components

| Component | Description |
|------------|--------------|
| **Microcontroller** | PIC18F4321 |
| **Display** | 16×2 LCD (HD44780-compatible) |
| **Motor Driver** | NPN transistor or logic-level N-MOSFET |
| **Flyback Diode** | 1N4007 / 1N5819 |
| **Potentiometers (×2)** | 10 kΩ — for max speed and delay control |
| **Push Button** | Emergency stop (INT0) |
| **LEDs** | Red LED for STOP indicator |
| **Power Supply** | 5 V logic + motor supply (with common ground) |

---

## 🧩 Pin Configuration

| Function | PIC18F4321 Pin | Code Symbol | Notes |
|-----------|----------------|--------------|-------|
| PWM Output | **RC1 / CCP2** | CCP2 | Controls motor transistor |
| LCD D4 – D7 | **RD4 – RD7** | LATD (nibble shift = 4) | 4-bit data bus |
| LCD RS | **RE0** | LATE0 | Command/data select |
| LCD RW | **RE1** | LATE1 | Always 0 (write) |
| LCD E | **RE2** | LATE2 | Enable strobe |
| Emergency Button | **RB0 / INT0** | — | Falling-edge interrupt |
| STOP LED | **RB1** | LATB1 | Active HIGH when stopped |
| ADC (Max) | **RA0 / AN0** | — | Potentiometer A |
| ADC (Delay) | **RA1 / AN1** | — | Potentiometer B |

> 🧠 The LCD uses **PORT D** for data (upper nibble) and **PORT E** for control pins.

---

## 🕒 Timing and Operation

- **Clock:** Internal 1 MHz oscillator (`_XTAL_FREQ 1000000UL`)  
- **Timer0:** Reload = `TMR0H = 0xFD`, `TMR0L = 0xCF` → ≈ 9 ms ISR  
- **PWM:** CCP2 on RC1, `PR2 = 249`, `T2CON = 0b00000100` (prescale 1:1)

Each Timer0 interrupt:
1. Increments the PWM duty cycle (`duty_current`) toward the ADC-set maximum (`duty_target`).  
2. After reaching max, resets to 0 and repeats based on the selected delay (`delay_ms`).  
3. Emergency stop (INT0) immediately sets duty = 0 and lights the red LED.

---

## 🧠 Firmware Summary

| Routine | Purpose |
|----------|----------|
| **`adc_init()` / `adc_read()`** | Initialize and read 10-bit analog values from AN0 and AN1 |
| **`pwm_init()` / `pwm_set()`** | Configure CCP2 module and update duty cycle |
| **`lcd_init4()` / `lcd_cmd4()` / `lcd_dat4()`** | Initialize LCD in 4-bit mode and print text |
| **`tmr0_init()`** | Configure Timer0 for periodic (~9 ms) interrupts |
| **`int0_init()`** | Set up emergency stop interrupt (INT0) |
| **`isr_high()`** | Handles pushbutton STOP/RESUME toggle |
| **`isr_low()`** | Handles PWM ramping logic |
| **`main()`** | Initializes peripherals, updates LCD with speed/delay readings |

---

## 🧰 Tools & Environment

- **IDE:** MPLAB X IDE  
- **Compiler:** Microchip XC8  
- **Debugger/Programmer:** PICkit 4 or compatible  
- **Simulation:** MPLAB SIM (optional)  

---

## 🧪 System Behavior

| Input | Effect |
|-------|--------|
| **Pot A (RA0)** | Sets max PWM duty cycle (motor speed) |
| **Pot B (RA1)** | Sets total ramp time (delay 0 – 10 s) |
| **INT0 Button** | Toggles motor STOP/RESUME |
| **LCD Display** | Shows “Max”, “Delay”, and “RUN/STOP” |
| **Red LED (RB1)** | Lit = STOP active |

---

## 📘 Learning Outcomes

- Implement PWM motor control using a PIC microcontroller  
- Interface ADCs and LCDs for real-time parameter monitoring  
- Configure multiple interrupts (Timer0 + INT0)  
- Apply structured embedded C programming for timing-critical tasks  


