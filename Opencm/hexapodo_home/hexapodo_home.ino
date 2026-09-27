/*******************************************************************************
 * Hexápodo - Posicionamiento de las 6 PATAS en configuración HOME
 * -----------------------------------------------------------------------------
 * OpenCM9.04 + OpenCM 485 EXP, 18x AX-12A en bus TTL (Serial3, 1 Mbps)
 *
 * Mapeo de servos (lineal):
 *   Pata 1: 1, 2, 3      Pata 4: 10, 11, 12
 *   Pata 2: 4, 5, 6      Pata 5: 13, 14, 15
 *   Pata 3: 7, 8, 9      Pata 6: 16, 17, 18
 *   Cada pata = [coxa, fémur, tibia]
 *
 * LADOS:
 *   Patas 1-2-3  -> lado A (el de la pata 2 que YA calibraste)
 *   Patas 4-5-6  -> lado B (espejado, PENDIENTE de calibrar)
 *
 * La calibración de la pata 2 (OFFSET {0,0,-55}, SIGN {+1,+1,-1}) se replica
 * a las patas 1 y 3 porque comparten lado y montaje. Las patas 4-5-6 quedan
 * con la MISMA plantilla como punto de partida: hay que ajustarlas viendo el
 * robot, igual que hiciste con la 2.
 *
 * PRECAUCIÓN: la primera vez, levantá el robot o sostené las patas del lado B.
 * Con la calibración del lado A aplicada al lado espejado, esas patas van a
 * apuntar mal hasta que corrijas signos y offsets.
 ******************************************************************************/

#include <DynamixelWorkbench.h>

#define DEVICE_NAME  "3"        // Puertos de la OpenCM 485 EXP
#define BAUDRATE     1000000    // Baudrate de fábrica del AX-12A

const uint8_t NUM_LEGS   = 6;
const uint8_t NUM_JOINTS = 3;

// --- IDs de cada pata: fila = pata, columna = [coxa, fémur, tibia] ----------
const uint8_t LEG_IDS[NUM_LEGS][NUM_JOINTS] = {
  {  1,  2,  3 },   // Pata 1  (lado A)
  {  4,  5,  6 },   // Pata 2  (lado A)  <- la que calibraste
  {  7,  8,  9 },   // Pata 3  (lado A)
  { 10, 11, 12 },   // Pata 4  (lado B, espejado)
  { 13, 14, 15 },   // Pata 5  (lado B, espejado)
  { 16, 17, 18 }    // Pata 6  (lado B, espejado)
};

// --- Ángulos HOME en grados (desde tu modelo de MATLAB) ---------------------
// Mismos para las 6 patas: es la MISMA postura cinemática. Las diferencias de
// montaje entre lados se resuelven con SIGN/OFFSET, NO cambiando estos ángulos.
const float HOME_DEG[NUM_JOINTS] = { 0.0, -30.0, 45.0 };

// --- Calibración por pata ---------------------------------------------------
// OFFSET: grados entre el cero cinemático y el centro del servo.
// SIGN:   +1 si el giro del servo coincide con el modelo, -1 si está invertido.
//
// Patas 1-3: valores YA validados en la pata 2 (replicados al resto del lado A).
// Patas 4-6: plantilla = copia del lado A. AJUSTAR viendo el robot.
//            Lo más probable es que la COXA (y quizá el fémur) necesiten SIGN
//            invertido por el espejado. Empezá volteando JOINT_SIGN[·][0].
const float JOINT_OFFSET_DEG[NUM_LEGS][NUM_JOINTS] = {
  {  0.0,  0.0, -55.0 },   // Pata 1  (lado A, validado)
  {  0.0,  0.0, -55.0 },   // Pata 2  (lado A, validado)
  {  0.0,  0.0, -55.0 },   // Pata 3  (lado A, validado)
  {  0.0,  0.0, -55.0 },   // Pata 4  (lado B) <- CALIBRAR
  {  0.0,  0.0, -55.0 },   // Pata 5  (lado B) <- CALIBRAR
  {  0.0,  0.0, -55.0 }    // Pata 6  (lado B) <- CALIBRAR
};

const int8_t JOINT_SIGN[NUM_LEGS][NUM_JOINTS] = {
  { +1, +1, -1 },   // Pata 1  (lado A, validado)
  { +1, +1, -1 },   // Pata 2  (lado A, validado)
  { +1, +1, -1 },   // Pata 3  (lado A, validado)
  { +1, +1, -1 },   // Pata 4  (lado B) <- CALIBRAR (probá coxa -1)
  { +1, +1, -1 },   // Pata 5  (lado B) <- CALIBRAR (probá coxa -1)
  { +1, +1, -1 }    // Pata 6  (lado B) <- CALIBRAR (probá coxa -1)
};

// Velocidad de movimiento (Moving Speed). 0 = MÁXIMA sin control (peligroso).
// Rango útil 1-1023. Con 18 servos moviéndose a la vez, mantené esto bajo.
const int32_t MOVING_SPEED = 80;

DynamixelWorkbench dxl_wb;

// --- Conversión grados -> unidades crudas del AX-12A ------------------------
// 1024 pasos (0..1023) sobre 300 grados; centro mecánico (150°) = valor 512.
int32_t degToRaw(float deg)
{
  float raw = 512.0f + deg * (1023.0f / 300.0f);
  if (raw < 0.0f)    raw = 0.0f;
  if (raw > 1023.0f) raw = 1023.0f;
  return (int32_t)(raw + 0.5f);
}

// Aplica calibración de una articulación y devuelve el valor crudo destino.
int32_t homeRawFor(uint8_t leg, uint8_t joint)
{
  float deg = JOINT_SIGN[leg][joint] * HOME_DEG[joint]
            + JOINT_OFFSET_DEG[leg][joint];
  return degToRaw(deg);
}

void setup()
{
  Serial.begin(57600);
  delay(3000);                 // En vez de while(!Serial), por el CDC de la OpenCM

  const char *log;
  bool result = false;
  uint16_t model_number = 0;

  // 1. Abrir el bus -----------------------------------------------------------
  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
  if (result == false) {
    Serial.println(log);
    Serial.println("Fallo al inicializar el bus");
    return;
  }
  Serial.print("Bus inicializado a ");
  Serial.println(BAUDRATE);

  // 2. Verificar que los 18 servos respondan ----------------------------------
  //    Si alguno falla, abortamos: mover el robot con una pata muerta lo
  //    desbalancea y puede dañar la mecánica.
  Serial.println("Verificando los 18 servos...");
  bool todos_ok = true;
  for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
      uint8_t id = LEG_IDS[leg][j];
      if (dxl_wb.ping(id, &model_number, &log) == false) {
        Serial.print("  NO responde ID ");
        Serial.print(id);
        Serial.print(" (pata ");
        Serial.print(leg + 1);
        Serial.println(")");
        todos_ok = false;
      }
    }
  }
  if (!todos_ok) {
    Serial.println("Faltan servos. Abortando por seguridad.");
    return;
  }
  Serial.println("Los 18 servos responden.");

  // 3. Modo articulación con velocidad limitada, para todos -------------------
  for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
      uint8_t id = LEG_IDS[leg][j];
      if (dxl_wb.jointMode(id, MOVING_SPEED, 0, &log) == false) {
        Serial.print("Fallo jointMode en ID ");
        Serial.println(id);
        Serial.println(log);
        return;
      }
    }
  }
  Serial.println("Modo articulacion activado en las 6 patas.");

  // 4. Enviar HOME. Mandamos pata por pata para poder frenar rápido si algo
  //    se ve mal en las patas del lado B durante la primera calibración.
  Serial.println("Moviendo a HOME...");
  for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
    Serial.print("  Pata ");
    Serial.print(leg + 1);
    Serial.print(leg >= 3 ? " (lado B) -> " : " (lado A) -> ");

    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
      uint8_t id  = LEG_IDS[leg][j];
      int32_t raw = homeRawFor(leg, j);
      dxl_wb.goalPosition(id, (int)raw, &log);   // (int) -> unidades crudas
      Serial.print(raw);
      Serial.print(j < NUM_JOINTS - 1 ? ", " : "");
    }
    Serial.println();
    delay(400);                // escalona el arranque; baja el pico de corriente
  }

  delay(1500);                 // dar tiempo a completar el movimiento

  // 5. Leer posiciones reales para verificar ----------------------------------
  Serial.println("Posiciones alcanzadas (grados):");
  for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
    Serial.print("  Pata ");
    Serial.print(leg + 1);
    Serial.print(": ");
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
      int32_t present = 0;
      if (dxl_wb.getPresentPositionData(LEG_IDS[leg][j], &present, &log)) {
        Serial.print((present - 512) * (300.0f / 1023.0f), 1);
        Serial.print(j < NUM_JOINTS - 1 ? " / " : "");
      }
    }
    Serial.println();
  }

  Serial.println("Listo. Todas las patas en HOME.");
}

void loop()
{
}
