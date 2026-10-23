# rp2040 — Controlador de vuelo para quadcopter

Firmware de controlador de vuelo para un **quadcopter en configuración X**, escrito en C sobre una **Raspberry Pi Pico (RP2040)** con el Pico SDK. Vuela en **modo Acro (control de velocidad angular)**: la palanca del control pide una velocidad de giro y el PID la sigue usando el giroscopio.

Qué hace:

- Lee un IMU **MPU6500** por SPI + DMA, sincronizado por su pin de interrupción DRDY.
- Recibe los canales del control por **Crossfire (CRSF)** a 420 000 baudios, con DMA.
- Controla los 4 motores con **DShot** generado por PIO (una máquina de estados por motor).
- Envía **telemetría** de batería y modo de vuelo por el mismo enlace CRSF.
- Permite **ajustar los PID en vuelo** (con los motores desarmados) desde el control.

> ⚠️ **Seguridad:** probá siempre con las **hélices quitadas**. Este firmware está en desarrollo y los PID todavía se están tuneando.

---

## Contenido

1. [Hardware y conexiones](#hardware-y-conexiones)
2. [Cómo funciona](#cómo-funciona)
3. [Armado y seguridad](#armado-y-seguridad)
4. [Configuración](#configuración)
5. [Compilar y cargar](#compilar-y-cargar)
6. [Ajuste de PID desde el control](#ajuste-de-pid-desde-el-control)
7. [Enlace de radio con ESP32](#enlace-de-radio-con-esp32)
8. [Depuración y calibración del giroscopio](#depuración-y-calibración-del-giroscopio)
9. [Estructura del repositorio](#estructura-del-repositorio)
10. [Pendientes](#pendientes)
11. [Licencia](#licencia)

---

## Hardware y conexiones

| Función | Pin(es) RP2040 | Detalle |
|---|---|---|
| MPU6500 (SPI1) | GP10 SCK, GP11 MOSI, GP12 MISO, GP13 CS | Escritura de registros a 1 MHz, lectura de datos a 8 MHz |
| MPU6500 DRDY | GP14 | Interrupción en flanco de subida: marca "muestra lista" |
| Receptor CRSF (UART0) | GP0 TX, GP1 RX | 420 000 baudios, 8N1 |
| Motores (DShot) | GP18, GP19, GP20, GP21 | Motor 1 a 4, en ese orden |
| Voltaje de batería | GP26 (ADC0) | Divisor resistivo, ver `config/battery.h` |
| Sensor de corriente | GP27 (ADC1) | Opcional, desactivado por defecto |
| Consola de depuración | USB | Salida de `printf` por USB (UART deshabilitada) |

**Numeración de motores** (vista desde arriba, frente del dron hacia adelante):

```
              ▲ frente

   M4 (adelante-izq)   M2 (adelante-der)
               ╲         ╱
                 ╲     ╱
                   ╳
                 ╱     ╲
               ╱         ╲
   M3 (atrás-izq)      M1 (atrás-der)
```

M1 y M4 forman una diagonal; M2 y M3, la otra. El sentido de giro de las hélices se define con `MIXER_YAW_CW_DIAGONAL_M1_M4` en `src/control/mixer.h` (1 = la diagonal M1+M4 gira en sentido horario visto desde arriba).

**Notas de montaje**

- Los pines del MPU están escritos directamente en `main.c` (estructura `mpu_config_t`), y los del UART de CRSF en `src/hal/rx/crossfire.c`. De `config/pinout.h` el firmware usa `MOTOR_BASE_PIN`.
- `src/hal/mpu/mpu.c` reordena y cambia el signo de los ejes del IMU (rotación de 90° en yaw) para que coincidan con el marco del dron. Si montás el sensor distinto, es ahí (y en `RC_SIGN_*` de `config/rcMap.h`) donde hay que corregirlo.
- Las salidas DShot son señales de 3,3 V directas del RP2040 hacia los ESC.

---

## Cómo funciona

### Ciclo principal

`main.c` corre un ciclo cooperativo a **1 kHz** (`LOOP_FREQ_HZ`). El tiempo de un ciclo completo ronda los 250 µs, así que el RP2040 queda con bastante margen. En cada vuelta:

1. **Receptor:** procesa los bytes nuevos de CRSF (`crsf_update`) y, si llegó un pedido de cambio de PID, lo aplica.
2. **IMU:** si el MPU avisó por DRDY, lee acelerómetro y giroscopio en una sola ráfaga DMA, mide el `dt` real desde la muestra anterior y ejecuta el control (`attitude_update` → `mixer_mix`).
3. **Motores:** aplica la lógica de armado y envía el valor a cada ESC por DShot.
4. **Batería y telemetría:** muestrea el ADC y reparte las tramas CRSF de telemetría.
5. **Cadencia:** espera hasta un instante absoluto (`next_loop_us`), sin acumular error entre ciclos.

El `dt` que usan los PID y los filtros es el **medido** entre muestras; el valor nominal (`NOMINAL_LOOP_DT_S`) solo sirve de respaldo en la primera muestra o ante una pausa anormal.

### Cadena de control (`src/core/attitude.c`)

```
Canales RC ──► curva de rates ──► velocidad deseada (rad/s) ─┐
                                                              ├─► PID ──► mixer ──► DShot
Giroscopio ──► filtro pasa-bajos (80 Hz) ──► velocidad real ─┘
```

- **Rates estilo Betaflight** (`config/rcMap.h`): `RC_RATE` 1.0, `RC_EXPO` 0.20, `RC_SUPER_RATE` 0.70. Con la tasa base de 200 °/s el máximo alcanzable es ≈ 667 °/s. Hay una verificación en compilación (`_Static_assert`) que impide configurar un rate máximo demasiado cerca del fondo de escala del giroscopio.
- **PID por eje** (`src/control/pid.c`), uno para roll, pitch y yaw, con:
  - **P** sobre el error.
  - **I** con anti-windup (limita el término integral y recorta el acumulador).
  - **D sobre la medición**, no sobre el error, para evitar el *derivative kick*.
  - **Feedforward** (`kff`) sobre la derivada del setpoint, filtrada a 40 Hz y con tope por eje.
- **Valores iniciales** (definidos en `attitude_init()`): roll y pitch con Kp 38, Ki 0.11, Kd 0.008; yaw con Kp 38, Ki 0.10, Kd 0; feedforward 0.25 en los tres ejes. Límite de salida de ±500 (±300 en yaw).
- **Mapeo de throttle:** el canal CRSF 172–1811 se traduce linealmente a valores DShot desde `DSHOT_MIN_THROTTLE` (60) hasta 2047.

### Mixer (`src/control/mixer.c`)

Mezcla throttle, roll, pitch y yaw para los 4 motores, con prioridades:

1. **Roll y pitch tienen prioridad máxima.** Si juntos no caben en el rango disponible, se escalan.
2. **Yaw usa solo el margen que sobra** después de roll/pitch.
3. Si el conjunto se sale del rango `[DSHOT_MIN_THROTTLE, 2047]`, se **desplaza** completo en lugar de recortar motor por motor, así no se deforma la mezcla de roll/pitch.

### IMU (`src/hal/mpu`)

- MPU6500 con giroscopio a **±1000 °/s** (32.8 LSB/°/s) y filtro digital interno (DLPF) de 184 Hz; salida a 1 kHz.
- Lectura por **DMA** (canales TX y RX en paralelo), con acelerómetro y giroscopio en una única ráfaga.
- **Calibración de sesgo en el arranque:** promedia 1000 muestras con el dron quieto. Si hay movimiento (pico a pico mayor a 250 LSB) descarta el resultado y lo informa por USB.
- Los datos se entregan en punto fijo **Q16.16** (`src/math/fixed_point.h`).

### Receptor CRSF (`src/hal/rx`)

- Recepción por **DMA** hacia un búfer circular de 256 bytes; un parser de estados valida longitud y **CRC8** (polinomio 0xD5).
- Decodifica los **16 canales de 11 bits** (rango 172–1811, centro 992).
- **Failsafe:** si pasan 200 ms sin paquetes válidos, el enlace se declara perdido y los canales se ponen en 0.
- La transmisión de telemetría también va por DMA: las funciones `crsf_send_*` solo encolan la trama y no bloquean el ciclo.

**Mapa de canales** (`config/rcMap.h`): CH0 Roll, CH1 Pitch, CH2 Throttle, CH3 Yaw, CH4 Armado.

### Motores DShot (`src/hal/esc`)

- Un programa **PIO** (`dshot.pio`) genera la señal; se usan 4 máquinas de estados de `pio0`, una por motor, en pines consecutivos desde `MOTOR_BASE_PIN`.
- La temporización apunta a **DShot300**: el reloj de las máquinas de estados se divide a 8,1 MHz y cada bit dura 30 ciclos (≈ 270 kbit/s).
- `dshot_init()` incluye una secuencia de arranque de ~1 s enviando throttle 0.

### Telemetría (`src/telemetry`, `src/hal/battery`)

Se envía por CRSF, repartida para no saturar el enlace:

| Trama | Frecuencia |
|---|---|
| Batería (tensión, corriente, mAh consumidos, % restante) | 5 Hz |
| Modo de vuelo (`ACRO`, o `DISARM` si está desarmado) | 1 Hz |

La batería se mide a 50 Hz con un filtro exponencial. El **número de celdas se detecta al encender** a partir de la tensión (se asume ≤ 4,2 V por celda) y el porcentaje restante se estima entre 3,40 V y 4,20 V por celda. Si no hay sensor de corriente (`BATTERY_CURRENT_SENSOR_ENABLED 0`), se reporta 0 A.

---

## Armado y seguridad

- **Armar:** requiere enlace CRSF activo, el interruptor de armado (CH4) por encima de 1500 y el **throttle abajo** (CH2 menor a 175).
- **Desarmar:** al bajar el interruptor de armado, o **de inmediato si se pierde el enlace** (failsafe de 200 ms). Al desarmar se ponen los motores en 0 y se reinician los PID.
- **Con los motores desarmados** la salida a los ESC es siempre throttle 0.
- **Armado:** los motores nunca bajan de `DSHOT_MIN_THROTTLE` (60) mientras el dron esté armado.
- `dshot_arm()` y `dshot_disarm()` envían ceros durante ~100 ms; es un bloqueo breve y deliberado.
- Los **cambios de PID por radio se rechazan con los motores armados**.

---

## Configuración

Todo lo ajustable está en `config/`:

| Archivo | Qué controla |
|---|---|
| `loopConfig.h` | Frecuencia del ciclo (`LOOP_FREQ_HZ`, 1000 por defecto). No conviene pasar de 1 kHz: la salida del sensor también está fijada en 1 kHz. |
| `gyro.h` | Fondo de escala y sensibilidad del giroscopio, filtros internos del MPU (DLPF), parámetros de calibración. |
| `rcMap.h` | Mapeo de canales, sentido de cada eje y rates (rate, expo, super rate). |
| `battery.h` | Pines ADC, divisor de tensión, sensor de corriente y rango de tensión por celda. |
| `pinout.h` | Pin base de los motores (`MOTOR_BASE_PIN`). |
| `debugConfig.h` | Activa o desactiva el menú de depuración del giroscopio (`GYRO_DEBUG_ENABLED`). |

Otros valores que se cambian en el código fuente:

- Ganancias iniciales de PID, límites y feedforward → `attitude_init()` y constantes al inicio de `src/core/attitude.c`.
- Filtro de velocidades angulares (`ATTITUDE_RATE_FILTER_CUTOFF_HZ`, 80 Hz) → `src/core/attitude.c`.
- Sentido de giro de las hélices → `MIXER_YAW_CW_DIAGONAL_M1_M4` en `src/control/mixer.h`.

**Importante:** `GYRO_SENSITIVITY_LSB_PER_DPS` debe corresponder al `GYRO_FULL_SCALE_DPS` elegido; si no, se ignora y se usa el valor nominal. Cada sensor se desvía unos puntos porcentuales del nominal (ver [depuración](#depuración-y-calibración-del-giroscopio)).

---

## Compilar y cargar

**Requisitos**

- CMake ≥ 3.13 y un compilador ARM (`arm-none-eabi-gcc`).
- **Pico SDK 2.2.0**. Lo más cómodo es la extensión *Raspberry Pi Pico* de VS Code, que instala todo en `~/.pico-sdk`.
  - Si existe `~/.pico-sdk/sdk/2.2.0`, el proyecto lo usa.
  - Si no, descarga el SDK desde GitHub dentro del proyecto (carpeta `.pico-sdk/`).
- Placa configurada: `pico`.

**Compilación**

```bash
mkdir build
cd build
cmake ..
make -j4
```

Se genera `build/rp2040.uf2`.

**Carga**

1. Mantené apretado **BOOTSEL** en la Pico y conectala por USB.
2. Copiá `build/rp2040.uf2` a la unidad que aparece.

Para ver los mensajes de depuración, abrí el puerto serie USB que expone la Pico (el firmware espera 2 s al arrancar para que alcances a conectarte).

---

## Ajuste de PID desde el control

Los PID se pueden cambiar sin recompilar, **solo con los motores desarmados**.

**Trama de pedido** (control → FC), tipo CRSF `0x60`, payload de 18 bytes:

| Bytes | Campo |
|---|---|
| 1 | `seq`: número de secuencia |
| 1 | `axis`: 0 roll, 1 pitch, 2 yaw |
| 4 | `kp` (float, big-endian) |
| 4 | `ki` (float, big-endian) |
| 4 | `kd` (float, big-endian) |
| 4 | `kff` (float, big-endian) |

**Trama de respuesta** (FC → control), tipo CRSF `0x61`: `seq`, `axis`, `status`.

| `status` | Significado |
|---|---|
| 0 | OK, valores aplicados |
| 1 | Rechazado: motores armados |
| 2 | Rechazado: valor fuera de rango |
| 3 | Rechazado: eje inválido |

**Rangos aceptados:** `kp` 0–50, `ki` 0–5, `kd` 0–5, `kff` 0–5 (no se aceptan valores negativos ni no finitos). Al aplicar un cambio se reinicia el integrador de ese eje.

Los cambios **no se guardan en memoria**: al reiniciar la placa vuelven los valores de `attitude_init()`.

---

## Enlace de radio con ESP32

En `working examples/` están los dos sketches de Arduino que forman el enlace de radio del proyecto:

- **`emisor_corrected/`** (ESP32 en el control): lee los joysticks y el interruptor de armado, aplica calibración de extremos, zona muerta y filtro, y envía los canales por **ESP-NOW**. También expone un puerto **Bluetooth serie** para configurar los PID y consultar la telemetría.
- **`receptor_corrected/`** (ESP32 a bordo del dron): recibe los canales por ESP-NOW y los reenvía a la FC como tramas **CRSF a 420 000 baudios**; devuelve al emisor la telemetría que genera la FC y reenvía los pedidos de PID.

**Conexión receptor ↔ RP2040** (cruzada):

| ESP32 (receptor) | RP2040 |
|---|---|
| GPIO 17 (TX) | GP1 (RX) |
| GPIO 16 (RX) | GP0 (TX) |
| GND | GND |

**Comandos por Bluetooth serie** (emisor):

| Comando | Acción |
|---|---|
| `set_pid:<eje>,<kp>,<ki>,<kd>[,<kff>]` | Encola el cambio de PID. Si se omite `kff`, se mantiene el último enviado desde el emisor. |
| `get_pid:<eje>` | Muestra los últimos valores confirmados por la FC. Es una caché del emisor, no una lectura directa de la FC: hasta el primer cambio confirmado muestra valores iniciales propios del sketch, que pueden no coincidir con los de `attitude_init()`. |
| `get_telemetry` | Muestra la última telemetría recibida. |

El emisor reintenta el envío de PID hasta 10 veces, cada 100 ms, hasta recibir el `ack` de la FC (`pid_ok` / `pid_err`).

---

## Depuración y calibración del giroscopio

El menú de giroscopio está **desactivado por defecto** porque consume cerca del 16 % del tiempo del ciclo y sus impresiones pueden bloquear el USB. Para activarlo, definí `GYRO_DEBUG_ENABLED` en 1 (en `config/debugConfig.h` o con `-DGYRO_DEBUG_ENABLED=1` al compilar) y abrí la consola USB.

| Tecla | Acción |
|---|---|
| `h` o `?` | Ayuda |
| `z` | Recalibrar el sesgo (dron quieto; bloqueado si está armado) |
| `1` / `2` / `3` | Ángulo de referencia: 90°, 180° o 360° |
| `r` | Iniciar/terminar una medición de giro |
| `a` | Aplicar en caliente la sensibilidad sugerida |
| `+` / `-` | Ajuste fino de la sensibilidad |
| `f` | Cambiar el fondo de escala (invalida la calibración; bloqueado si está armado) |
| `d` | Reiniciar la deriva acumulada |
| `p` | Pausar/reanudar las impresiones |

**Cómo medir la sensibilidad real de tu giroscopio:**

1. Con el dron quieto, verificá que la deriva mostrada sea cercana a 0 °/s; si no, recalibrá con `z`.
2. Elegí el ángulo de referencia con `1`, `2` o `3`.
3. Pulsá `r`, girá el dron ese ángulo exacto sobre **un solo eje** y volvé a pulsar `r`.
4. El menú imprime el ángulo integrado y la sensibilidad sugerida. Con `a` la aplicás en caliente y con `z` recalibrás para probarla.
5. Copiá el valor final en `GYRO_SENSITIVITY_LSB_PER_DPS` (`config/gyro.h`).

---

## Estructura del repositorio

```
.
├── CMakeLists.txt            Proyecto CMake (Pico SDK 2.2.0)
├── pico_sdk_import.cmake     Importador del SDK
├── config/                   Parámetros ajustables (ver "Configuración")
├── src/
│   ├── main.c                Ciclo principal y lógica de armado
│   ├── core/                 Rates, filtro de velocidades y ejecución del PID por eje
│   ├── control/              PID y mixer
│   ├── hal/
│   │   ├── mpu/              Driver del MPU6500 (SPI + DMA, calibración)
│   │   ├── rx/               Receptor y telemetría CRSF
│   │   ├── esc/              DShot por PIO
│   │   └── battery/          Medición de batería por ADC
│   ├── telemetry/            Planificador de tramas de telemetría
│   ├── math/                 Aritmética de punto fijo Q16.16
│   └── debug/                Menú de depuración del giroscopio
├── working examples/
│   ├── emisor_corrected/     Sketch ESP32 del control (ESP-NOW + Bluetooth)
│   └── receptor_corrected/   Sketch ESP32 a bordo (ESP-NOW ↔ CRSF)
├── todo.txt                  Lista de pendientes
└── licence                   Licencia MIT
```

---

## Pendientes

- Reconfigurar el MPU y el ciclo para trabajar a 2 o 4 kHz.
- Un protocolo Crossfire más rápido, para reducir los cálculos necesarios.
- Usar más de un filtro pasa-bajos: PT1 (200–250 Hz) y PT2 (400–500 Hz), suponiendo un dron rígido y limpio.
- Tunear los PID.

---

## Licencia

Este proyecto está bajo la licencia MIT. Mirá el archivo [`licence`](licence) para más detalles.
