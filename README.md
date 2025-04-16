# 🏭 CintaClasificadora

Proyecto de control de una cinta transportadora con clasificación automática de cajas según altura, usando C++ y Qt para la interfaz gráfica, y un microcontrolador ATmega328P como sistema embebido.

---

## 🚀 Descripción

Este sistema clasifica cajas en tiempo real mediante:

- Un sensor ultrasónico HC-SR04 para medir la altura.
- Sensores infrarrojos TCRT5000 para detectar la posición de las cajas.
- Servomotores que empujan las cajas hacia diferentes sectores según su categoría (A, B o C).
- Comunicación serial entre el microcontrolador y la PC usando protocolo UNER.
- Una interfaz Qt que permite visualizar y controlar el proceso.

---

## 🛠️ Tecnologías utilizadas

| Componente             | Descripción                              |
|------------------------|------------------------------------------|
| Qt 6.5 / Qt Creator    | Interfaz gráfica                         |
| C++                    | Lógica de control                        |
| MinGW 64-bit           | Compilador en Windows                    |
| ATmega328P             | Microcontrolador principal               |
| USART / Protocolo UNER | Comunicación entre PC y microcontrolador |
| Git + GitHub           | Control de versiones                     |

---

## 🧩 Estructura del Proyecto

