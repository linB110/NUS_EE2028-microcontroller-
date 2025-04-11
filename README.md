# NUS_EE2028 Microcontroller Projects

STM32L475VG board and peripheral interfacing coursework projects.

---

## 📌 Introduction

**Board:** STM32L475VG Discovery Kit  
**IDE:** STM32CubeIDE  
**Language:** Pure C and ARM Assembly

**Used hardware peripherals:**

1. HTS221 – Temperature and Humidity Sensor  
2. LIS3MDL – 3-Axis Magnetometer  
3. LSM6DSL – 3D Accelerometer and Gyroscope  
4. LPS22HB – Barometer  
5. VL53L0X – Gesture Detection Sensor  
6. Grove LED Matrix  
7. Grove OLED Display  
8. Grove Buzzer  
9. Grove 5-Way Switch  

---

## 🧪 Assignment 1: ARM Assembly and Register Control

**Objective:**  
Hands-on experience with ARM assembly and low-level hardware manipulation by directly accessing STM32 registers.

**Key Features:**

- Coding primarily in **ARM Assembly**
- Direct **register-level** manipulation (no HAL)
- Flow control implementation (loops, conditionals)
- Integration with C code for higher-level logic

---

## 🧩 Assignment 2: Peripheral Integration and User Interface

**Objective:**  
Build a full embedded application that interacts with multiple sensors and user interfaces.

**Key Features:**

- Developed entirely in **C using STM32 HAL**
- Interface with both **on-board and external devices**
- Real-time display updates on **OLED**
- **LED Matrix** and **buzzer** control for feedback
- **EXTI interrupts** used for switch and sensor inputs
- User-friendly interactive interface

---

## 🛠️ How to Build & Flash

1. Open **STM32CubeIDE**
2. Import the assignment project folder:
   - `Assign1` for Assignment 1
   - `Assignment2` for Assignment 2
3. Connect the STM32L475VG board via USB
4. Click **Build** → **Run** to flash the code

> Make sure the board drivers and CubeIDE are properly installed.

---

## 👤 Author

Developed as part of EE2028 Microcontroller Programming  
National University of Singapore (NUS)

student : Lin Huang Ting 

