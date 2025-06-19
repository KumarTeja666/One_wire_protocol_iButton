# One_wire_protocol_iButton

# 🔌 STM32 1-Wire Protocol (iButton Interface)

This project implements the 1-Wire communication protocol using **STM32 HAL** to interface with 1-Wire devices like the **iButton (DS1990A)**.

---

## 📦 Features

- Bit-level 1-Wire protocol implementation (read/write/reset)
- Manual open-drain simulation using GPIO direction control
- Accurate timing using `delay_us()` function
- Compatible with STM32CubeIDE and HAL

---

## 🧰 Hardware Requirements

| Component        | Description                         |
|------------------|-------------------------------------|
| MCU              | STM32F303RE (or any STM32 HAL-based)|
| 1-Wire Device    | iButton (DS1990A)                   |
| Pull-up Resistor | 4.7kΩ between data and 3.3V or 5V   |
| Power Supply     | 3.3V or 5V (match iButton spec)     |
| Common GND       | Shared GND between STM32 and iButton|

---

## 🔌 Wiring and GPIO Configuration

- GPIO Pin: PA1 (configurable)
- Mode:
  - Output → Write LOW
  - Input  → Release (simulate open-drain)
- Speed: Medium or High
- Pull: No pull-up/down (external used)

---
