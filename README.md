# CintaClasificadora · Sistema de Clasificación Industrial

<div align="center">

![Platform](https://img.shields.io/badge/Plataforma-ATmega328P-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![Language](https://img.shields.io/badge/Lenguaje-C%2B%2B-00599C?style=for-the-badge&logo=cplusplus&logoColor=white)
![GUI](https://img.shields.io/badge/GUI-Qt%206.5-41CD52?style=for-the-badge&logo=qt&logoColor=white)
![Status](https://img.shields.io/badge/Estado-Funcional-brightgreen?style=for-the-badge)

<br/>

> **Sistema embebido + interfaz de escritorio para clasificación automática de cajas:**
> detección ultrasónica en tiempo real, lógica de desvío por servomotores,
> telemetría serial con protocolo UNER y dashboard Qt —
> **sin errores de clasificación · sin intervención manual · sin parar la cinta.**

<br/>

[![LinkedIn](https://img.shields.io/badge/Tadeo_Mendelevich-0A66C2?style=for-the-badge&logo=data:image/svg%2bxml;base64,PHN2ZyByb2xlPSJpbWciIHZpZXdCb3g9IjAgMCAyNCAyNCIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj48cGF0aCBmaWxsPSJ3aGl0ZSIgZD0iTTIwLjQ0NyAyMC40NTJoLTMuNTU0di01LjU2OWMwLTEuMzI4LS4wMjctMy4wMzctMS44NTItMy4wMzctMS44NTMgMC0yLjEzNiAxLjQ0NS0yLjEzNiAyLjkzOXY1LjY2N0g5LjM1MVY5aDMuNDE0djEuNTYxaC4wNDZjLjQ3Ny0uOSAxLjYzNy0xLjg1IDMuMzctMS44NSAzLjYwMSAwIDQuMjY3IDIuMzcgNC4yNjcgNS40NTV2Ni4yODZ6TTUuMzM3IDcuNDMzYTIuMDYyIDIuMDYyIDAgMCAxLTIuMDYzLTIuMDY1IDIuMDY0IDIuMDY0IDAgMSAxIDIuMDYzIDIuMDY1em0xLjc4MiAxMy4wMTlIMy41NTVWOWgzLjU2NHYxMS40NTJ6TTIyLjIyNSAwSDEuNzcxQy43OTIgMCAwIC43NzQgMCAxLjcyOXYyMC41NDJDMCAyMy4yMjcuNzkyIDI0IDEuNzcxIDI0aDIwLjQ1MUMyMy4yIDI0IDI0IDIzLjIyNyAyNCAyMi4yNzFWMS43MjlDMjQgLjc3NCAyMy4yIDAgMjIuMjIyIDBoLjAwM3oiLz48L3N2Zz4=&logoColor=white)](https://www.linkedin.com/in/tadeo-mendelevich/)
[![GitHub](https://img.shields.io/badge/tadeomendelevich-181717?style=for-the-badge&logo=github&logoColor=white)](https://github.com/tadeomendelevich)

</div>

---

## ¿Qué hace este proyecto?

Sistema de clasificación industrial con dos capas: un **ATmega328P** que lee sensores y actúa en tiempo real, y una **aplicación Qt** en PC que visualiza y comanda el proceso. Todo coordinado por protocolo UNER sobre USART.

<div align="center">

### 🏭 Sistema en funcionamiento

![Cinta clasificadora en funcionamiento](cintaClasificadora.gif)

</div>

| Módulo | Descripción |
|--------|-------------|
| 📏 **HC-SR04** | Mide la altura de cada caja con pulsos ultrasónicos — disparo y captura por interrupciones, sin polling |
| 🔦 **TCRT5000 ×3** | Detecta la posición de la caja en la cinta: entrada, zona de clasificación y salida |
| ⚙️ **Servomotores** | Empujan la caja al sector correcto (A, B o C) según el rango de altura medido |
| 📡 **Protocolo UNER** | Trama binaria estructurada sobre USART — sincronización, tipo, payload y checksum |
| 🖥️ **Interfaz Qt** | Dashboard en tiempo real: estado de la cinta, categoría detectada, historial de clasificaciones y control manual |
| 🔁 **Lógica embebida** | Máquina de estados que gestiona la secuencia medir → clasificar → desviar → resetear, sin bloqueos |

---

## Arquitectura

```
PC (Qt 6.5 / C++)
│
├── SerialWorker ──► QSerialPort ──► Parser UNER ──► Señales Qt ──► Dashboard
│
└── Control manual ──► Comandos UNER ──► USART ──────────────────► ATmega328P

ATmega328P (firmware C)
│
├── HC-SR04 ──► Input Capture / Timer1 ──► altura_cm ──► Categoría A/B/C
│
├── TCRT5000 ──► INT0 / INT1 / PCINT ──► posición de caja en cinta
│
├── Servos ──────────────────────────────────────────► PWM Timer2
│
└── Máquina de estados ──► USART ──► Trama UNER ──► PC
```

> La medición ultrasónica usa Input Capture sobre Timer1 para calcular el tiempo de vuelo
> con precisión de microsegundos — sin `delay()`, sin busy-wait.

---

## Lo más interesante del código

#### 📏 Medición sin bloqueos
El trigger del HC-SR04 se dispara por overflow de timer. El echo se captura por Input Capture en ambos flancos. El firmware nunca espera activamente — la CPU está disponible para atender la cinta mientras el sonar "vuela".

#### 🔁 Máquina de estados robusta
Cada caja pasa por estados bien definidos: `ESPERA → DETECCIÓN → MEDICIÓN → CLASIFICACIÓN → DESVÍO → RESET`. Las transiciones están guiadas por los sensores IR, no por tiempo — el sistema se adapta a cualquier velocidad de cinta.

#### 📡 Protocolo UNER binario
Las tramas tienen estructura fija: `[START | ID | TIPO | PAYLOAD[N] | CHECKSUM]`. El parser en Qt reconstruye paquetes fragmentados por el buffer serial — robusto ante latencia y jitter de la USART.

#### 🖥️ Interfaz Qt desacoplada del hardware
El `SerialWorker` corre en un hilo separado. La UI principal nunca bloquea esperando datos del puerto — los datos llegan por señales Qt al thread principal cuando están listos.

#### 🔧 Categorización configurable
Los umbrales de altura para las categorías A, B y C se envían desde la PC al microcontrolador en tiempo de ejecución. No hace falta recompilar el firmware para ajustar la clasificación.

---

## Hardware

| Componente | Detalle |
|------------|---------|
| MCU | ATmega328P @ 16 MHz |
| Sensor de altura | HC-SR04 — rango 2 cm a 400 cm, resolución ~3 mm |
| Sensores de posición | TCRT5000 × 3 — detección IR reflectivo |
| Actuadores | Servomotores × 3 (uno por categoría) — PWM 50 Hz |
| Comunicación | USART @ 9600 bps — protocolo UNER binario |
| PC / GUI | Qt 6.5, compilado con MinGW 64-bit en Windows |

---

## Estructura del proyecto

```
firmware/
├── main.c          — Máquina de estados, loop principal
├── hcsr04.c        — Driver ultrasónico con Input Capture
├── servo.c         — Control PWM de servomotores
├── uner.c          — Protocolo de telemetría binario
└── tcrt5000.c      — Detección de posición por IR
qt_app/
├── mainwindow.cpp  — Dashboard y lógica de interfaz
├── serialworker.cpp — Hilo de comunicación serial
└── unerparser.cpp  — Parser de tramas UNER
```

---

<div align="center">

**Tadeo Mendelevich** · Ingeniería en Sistemas · UNER — Concordia, Entre Ríos

[![LinkedIn](https://img.shields.io/badge/LinkedIn-Conectar-0A66C2?style=flat&logo=linkedin)](https://www.linkedin.com/in/tadeo-mendelevich/)
[![GitHub](https://img.shields.io/badge/GitHub-tadeomendelevich-181717?style=flat&logo=github)](https://github.com/tadeomendelevich)

</div>
