# HEXAPOD EDO

El robot se controla desde MATLAB. MATLAB resuelve la cinemática inversa del ciclo de marcha y transmite los ángulos de las 18 articulaciones por WiFi; el ESP32 actúa como puente y la OpenCM9.04 los traduce a posiciones de los servomotores:

```
     MATLAB    ──>     ESP32     ──>         OpenCM9.04       ──> 18× Dynamixel AX-12A
 (IK + marcha)     (puente WiFi)     (calibración + syncWrite)
```

---

## Estructura de carpetas

```
ESP32/                                   Firmware del puente WiFi → UART
├── hexapodo_puente_esp32/               ★ Versión final
└── codigotesisv1/                       Primera versión (histórico)
│
Opencm/                                  Firmware de la OpenCM9.04
├── hexapodo_caminata_opencm/            ★ Versión final (recibe la marcha)
├── hexapodo_home/                       Posición HOME y calibración de patas
└── Opencm.ino                           Código heredado (referencia)
│
MATLAB/                                  Simulación y control
├── robotat_hexapod_main_functions/      ★ Librería Robotat del hexápodo
├── sim.m, sim_giro.m                    Simulación de avance y de giro
├── caminata_esp32.m, giro_esp32.m       Primeras pruebas de marcha en hardware
```

Las carpetas marcadas con ★ contienen el código que corre en la versión final del robot.

---

## `ESP32/` — Puente WiFi → UART

### `hexapodo_puente_esp32/hexapodo_puente_esp32.ino` ★

Recibe por TCP (puerto 80) las tramas que envía MATLAB y las reenvía línea por línea a la OpenCM por `Serial2`. El ESP32 no interpreta el JSON: solo actúa como puente, lo que reduce la latencia. Mantiene el socket abierto durante toda la sesión y responde `ok` por cada trama recibida para que MATLAB no se adelante.

| Parámetro | Función |
|---|---|
| `HEXAPOD_ID` | Identidad del robot (31 a 36). En el laboratorio define la IP fija `192.168.50.(200 + ID)`. |
| `RED_LABORATORIO` | `1` = red Robotat con IP fija · `0` = otra red por DHCP. |
| `DEBUG_ECHO` | `1` imprime cada trama en el monitor serie (dejar en `0` al caminar). |


**Conexión física:** ESP32 GPIO17 (TX2) → RX de Serial2 de la OpenCM · GPIO16 (RX2) ← TX · GND común.

### `codigotesisv1/codigotesisv1.ino`

Primera versión del puente. Recibía un JSON con seis cadenas (`q1s`…`q6s`), cerraba la conexión después de cada mensaje y separaba los valores en el ESP32. Se conserva como referencia histórica y fue reemplazado por `hexapodo_puente_esp32`.

---

## `Opencm/` — Firmware de la OpenCM9.04

### `hexapodo_caminata_opencm/hexapodo_caminata_opencm.ino` ★

Firmware final. Al encender verifica que los 18 servos respondan (si falta alguno, aborta), lleva las seis patas a HOME y queda a la espera de tramas desde el ESP32.

- **Traducción de ángulos:** MATLAB envía ángulos del *modelo* en decigrados; la placa los convierte a unidades crudas del AX-12A con su propia tabla de calibración:
  `raw = RAW_HOME + SIGN · (ángulo − MODEL_HOME_DEG) · 1023/300`.
  De esta forma, la calibración vive en un solo lugar.
- **Escritura simultánea:** usa `syncWrite`, de modo que los 18 servos reciben su posición en un solo paquete.
- **Límites de seguridad:** `RAW_DEV_MAX` recorta cualquier desviación excesiva respecto a HOME.
- **Velocidades:** `MOVING_SPEED_HOME` (lenta, para ir a HOME) y `MOVING_SPEED_WALK` (durante la marcha).
- **Watchdog:** si no llegan tramas en `LINK_TIMEOUT_MS` (1 s), mantiene la última pose. Con `HOME_ON_TIMEOUT = 1` regresa a HOME.

### `hexapodo_home/hexapodo_home.ino`

Sketch de puesta a punto. Lleva las seis patas a HOME, pata por pata, e imprime las posiciones alcanzadas. Se usa para calibrar `JOINT_OFFSET_DEG` y `JOINT_SIGN` de cada pata antes de caminar. Sus valores de HOME son los que se trasladaron a `RAW_HOME` en el firmware de caminata.

### `Opencm.ino`

Código heredado del trabajo anterior (Luis Salazar). Recibía valores crudos 0–1023 y escribía servo por servo con `ping` + `goalPosition` + `delay`. Se conserva como referencia.

**Mapeo de servos** (fila = pata, columnas = coxa, fémur, tibia):

| Pata | IDs | Lado |
|---|---|---|
| 1 | 1, 2, 3 | A |
| 2 | 4, 5, 6 | A |
| 3 | 7, 8, 9 | A |
| 4 | 10, 11, 12 | B (espejado) |
| 5 | 13, 14, 15 | B (espejado) |
| 6 | 16, 17, 18 | B (espejado) |

---

## `MATLAB/` — Simulación y control

### `robotat_hexapod_main_functions/` ★ Librería Robotat del hexápodo

| Archivo | Descripción |
|---|---|
| `robotat_hexapod_connect.m` | Conecta con el hexápodo (`connect(31)` en el laboratorio o `connect(31, 'IP')` en otra red). Resuelve **una sola vez** la cinemática inversa del ciclo de marcha, calcula la compensación radial `kleg` para el giro y los límites de velocidad `robot.vmin/vmax` y `robot.wmin/wmax`. |
| `robotat_hexapod_advance_gait.m` | Marcha trípode hacia adelante o atrás a una velocidad en m/s. |
| `robotat_hexapod_turn_gait.m` | Giro sobre su propio eje, a izquierda o derecha, a una velocidad en °/s. |
| `robotat_hexapod_step.m` | Avanza la fase del ciclo según el tiempo transcurrido (`tic`/`toc`, variables `persistent`) y envía la trama de 18 ángulos. |
| `robotat_hexapod_stream.m` | Reproduce el ciclo completo durante un tiempo fijo. |
| `robotat_hexapod_disconnect.m` | Detiene el robot y cierra la conexión. |
| `robotat_connect.m`, `robotat_disconnect.m`, `robotat_get_pose.m`, `robotat_trvisualize.m`, `q2eul.m`, `q2rot.m` | Funciones estándar del Robotat para leer la pose del sistema OptiTrack. |
| `main_hexapod_test.m` | Script de pruebas por secciones: avance, retroceso, avance por distancia, giro por ángulo, cuadrado, velocidad variable, baile y desconexión. |
| `laboratorio11.m` | Adaptación del Laboratorio 11 de MT3005 (control de robots móviles) al hexápodo: navegación hacia un marcador meta con controlador LQR (linealización por realimentación), alternando entre giro y avance. |

### Scripts de simulación (carpeta `MATLAB/`)

Todos modelan cada pata como una cadena de 3 GDL con Robotics Toolbox (`'Rz(q1) Ty(L1) Rx(q2) Ty(L2) Rx(q3) Tz(L3)'` → `DHFactor` → `SerialLink`) y colocan las seis patas alrededor del cuerpo con transformaciones `SE3`.

| Archivo | Descripción |
|---|---|
| `sim.m` | Simulación de la marcha trípode de avance: trayectoria del pie con `mstraj`, IK con `ikine` y animación de las seis patas. Incluye la gráfica de la trayectoria del extremo de la pata. |
| `sim_giro.m` | Simulación del giro sobre su propio eje (vector `sgn` con todas las patas en el mismo sentido). |
| `caminata_esp32.m` | Primera prueba de marcha en hardware: calcula el ciclo y lo transmite al ESP32 a una frecuencia fija. Tiene modo `simOnly` para graficar sin conectar. |
| `giro_esp32.m` | Equivalente de `caminata_esp32.m` para el giro. |

> `caminata_esp32.m` y `giro_esp32.m` fueron el paso previo a la librería; para operar el robot se recomienda usar `robotat_hexapod_main_functions/`.

---

## Protocolo MATLAB → OpenCM

Una línea JSON por trama, terminada en `\n`:

| Trama | Significado |
|---|---|
| `{"q":[q1,…,q18]}` | Ángulos del modelo en **decigrados** (enteros). El índice *k* corresponde al servo con ID *k*. |
| `{"c":"home"}` | Ir a HOME. |
| `{"c":"relax"}` | Apagar el torque. |
| `{"c":"torque"}` | Encender el torque. |

El ESP32 saluda con `READY` al conectarse y responde `ok` por cada línea.

---

## Parámetros que deben coincidir entre MATLAB y el firmware

| MATLAB | OpenCM (`hexapodo_caminata_opencm.ino`) | Nota |
|---|---|---|
| `qHome = [0 0.4 -0.3]` rad | `MODEL_HOME_DEG = {0, 22.918, -17.189}` | Si cambia uno, hay que cambiar el otro. |
| Vector `sgn` (espejo de la coxa en patas 4–6) | `JOINT_SIGN` | Invertir el sentido en **un solo** lado; si se invierte en ambos, se cancela. |
| `robotat_hexapod_connect(ID)` | `HEXAPOD_ID` en el ESP32 | Mismo número (31–36). |

---

## Puesta en marcha rápida

1. Asignar los IDs 1–18 a los servos y verificar su baud rate (1 Mbps) con DynamixelWorkbench.
2. Cargar `hexapodo_home.ino` en la OpenCM y verificar la calibración de cada pata.
3. Cargar `hexapodo_caminata_opencm.ino` en la OpenCM.
4. Configurar `HEXAPOD_ID` y `RED_LABORATORIO` y cargar `hexapodo_puente_esp32.ino` en el ESP32.
5. En MATLAB, agregar `robotat_hexapod_main_functions/` al *path* y ejecutar por secciones `main_hexapod_test.m`.

---

## Dependencias

**MATLAB**
- Robotics Toolbox for MATLAB de Peter Corke (RTB 9): `DHFactor`, `SerialLink`, `SE3`, `mstraj`, `transl`.
- Control System Toolbox (`lqr`), solo para `laboratorio11.m`.

**Arduino IDE**
- Paquete de placas ESP32 (Espressif).
- Paquete de placas OpenCM9.04 (ROBOTIS), que incluye `DynamixelWorkbench`.
- `ArduinoJson` (firmware de la OpenCM y `codigotesisv1`).
