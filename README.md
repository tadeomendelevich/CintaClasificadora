🏭 Cinta Clasificadora Inteligente

[Platform] [Language] [GUI] [Status]

  Sistema de clasificación automática en tiempo real basado en sensores
  industriales, control embebido y visualización en PC. Diseño orientado
  a robustez, determinismo y escalabilidad.

------------------------------------------------------------------------

🚀 ¿Qué hace este proyecto?

Sistema completo de automatización industrial a pequeña escala que
permite clasificar cajas por altura en tiempo real, combinando sensado,
lógica embebida y visualización externa.

El flujo de operación es:

Detección → Medición → Clasificación → Actuación → Visualización

Cada caja es analizada en movimiento y desviada automáticamente a su
categoría correspondiente (A, B o C) sin intervención humana.

------------------------------------------------------------------------

🧠 Arquitectura del sistema

Sensores ───────────────► ATmega328P ───────────────► Servos │ │ │ │
└────► Protocolo UNER ─────┘ │ │
└──────────────────────────────────────► Qt GUI (PC)

-   El microcontrolador ejecuta la lógica en tiempo real
-   La PC actúa como sistema de monitoreo y control
-   Comunicación serial robusta mediante protocolo propio

------------------------------------------------------------------------

⚙️ Componentes del sistema

  Módulo            Función
  ----------------- -----------------------------------------
  📏 HC-SR04        Medición de altura de las cajas
  📍 TCRT5000       Detección de presencia y posición
  ⚙️ Servomotores   Clasificación física por empuje lateral
  🧠 ATmega328P     Control lógico en tiempo real
  🖥️ Qt GUI         Visualización, control y debugging

------------------------------------------------------------------------

🔥 Lo más interesante del proyecto

⚡ Clasificación en tiempo real

El sistema procesa cada caja en movimiento sin detener la cinta,
garantizando flujo continuo.

🎯 Lógica determinística embebida

Toda la toma de decisiones corre en el microcontrolador, asegurando
tiempos de respuesta predecibles.

🔌 Protocolo UNER

Comunicación serial estructurada entre PC y sistema embebido: - envío de
estados
- monitoreo en vivo
- control externo

🖥️ Interfaz Qt profesional

Aplicación en PC para: - visualizar el estado del sistema
- debuggear sensores
- validar el comportamiento en tiempo real

🧱 Diseño modular

Cada componente (sensado, control, actuación, GUI) está desacoplado,
permitiendo escalar o reemplazar partes fácilmente.

------------------------------------------------------------------------

🛠️ Tecnologías utilizadas

  Componente            Descripción
  --------------------- -----------------------------------
  Qt 6.5 / Qt Creator   Interfaz gráfica multiplataforma
  C++                   Lógica de control y GUI
  MinGW 64-bit          Toolchain en entorno Windows
  ATmega328P            Sistema embebido principal
  USART + UNER          Comunicación serial estructurada
  Git + GitHub          Versionado y gestión del proyecto

------------------------------------------------------------------------

📊 Aplicaciones reales

Este sistema es una versión simplificada de soluciones usadas en:

-   🏭 Líneas de clasificación industrial
-   📦 Centros logísticos y packaging
-   🥫 Industria alimenticia
-   🚚 Sistemas de distribución automatizados

------------------------------------------------------------------------

🧑‍💻 Autor

Tadeo Mendelevich
Ingeniería en Sistemas · UNER — Concordia, Entre Ríos

[LinkedIn] [GitHub]
