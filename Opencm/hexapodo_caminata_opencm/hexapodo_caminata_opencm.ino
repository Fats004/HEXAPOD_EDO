/*******************************************************************************
 * HEXÁPODO - Receptor de marcha en la OpenCM9.04
 * -----------------------------------------------------------------------------
 *  MATLAB --TCP--> ESP32 --Serial2 115200--> [OpenCM9.04] --Serial3 1Mbps--> 18 AX-12A
 *
 *  Reemplaza a hexapodo_home.ino: este sketch hace lo mismo al arrancar (lleva
 *  las 6 patas a HOME) y además acepta tramas de marcha por UART.
 *
 *  QUÉ RECIBE (una línea JSON terminada en '\n'):
 *      {"q":[q1,...,q18]}   ángulos del MODELO de MATLAB, en DECIGRADOS
 *                           índice k -> servo con ID k
 *                           q1..q3 = pata 1 [coxa,fémur,tibia], q4..q6 = pata 2, ...
 *      {"c":"home"}   {"c":"relax"}   {"c":"torque"}
 *
 *  POR QUÉ ÁNGULOS Y NO VALORES CRUDOS
 *  El código de Luis mandaba 0..1023 desde MATLAB. Eso obliga a tener la
 *  calibración duplicada en dos lados. Aquí MATLAB manda el ángulo "ideal" del
 *  modelo y la placa lo traduce con SU tabla de calibración: una sola fuente de
 *  verdad, y podés seguir ajustando las patas 4-6 sin tocar MATLAB.
 *
 *  CÓMO SE TRADUCE
 *      raw = RAW_HOME[pata][j] + SIGN[pata][j]*(ang_modelo - MODEL_HOME_DEG[j])*1023/300
 *  Es decir: se trabaja con la DESVIACIÓN respecto a HOME. Así no importa que el
 *  cero del modelo de MATLAB y el cero mecánico del servo no coincidan; lo único
 *  que importa es (a) qué valor crudo deja la pata en HOME y (b) hacia qué lado
 *  gira el servo cuando el ángulo del modelo crece.
 *
 *  DIFERENCIA CON EL Opencm.ino DE LUIS
 *  Él hacía ping + goalPosition + delay(5) servo por servo: ~90 ms por trama, o
 *  sea ~11 Hz con temblor. Aquí se usa syncWrite: los 18 servos reciben su
 *  posición en un solo paquete (<1 ms) y se mueven a la vez.
 *
 *  Sofía Cardona - UVG
 ******************************************************************************/

#include <ArduinoJson.h>
#include <DynamixelWorkbench.h>

#define DEVICE_NAME  "3"          // Serial3 = bus TTL de la OpenCM 485 EXP
#define DXL_BAUD     1000000      // baudrate de fábrica del AX-12A
#define LINK_BAUD    115200       // UART hacia el ESP32 (Serial2)

const uint8_t NUM_LEGS   = 6;
const uint8_t NUM_JOINTS = 3;
const uint8_t NUM_DXL    = NUM_LEGS * NUM_JOINTS;

// --- IDs: fila = pata, columna = [coxa, fémur, tibia] -----------------------
const uint8_t LEG_IDS[NUM_LEGS][NUM_JOINTS] = {
  {  1,  2,  3 },   // Pata 1  (lado A)
  {  4,  5,  6 },   // Pata 2  (lado A)  <- la que calibraste
  {  7,  8,  9 },   // Pata 3  (lado A)
  { 10, 11, 12 },   // Pata 4  (lado B, espejado)
  { 13, 14, 15 },   // Pata 5  (lado B, espejado)
  { 16, 17, 18 }    // Pata 6  (lado B, espejado)
};

// --- HOME del MODELO de MATLAB, en grados ------------------------------------
// = rad2deg([0  0.4  -0.3])  <- el qHome de sim.m. Si cambiás qHome en MATLAB,
//   cambialo también aquí, o el robot va a interpretar mal el cero.
const float MODEL_HOME_DEG[NUM_JOINTS] = { 0.0f, 22.918f, -17.189f };

// --- Valor CRUDO que deja físicamente cada articulación en HOME --------------
// Derivado de tu hexapodo_home.ino:  degToRaw(SIGN*HOME_DEG + OFFSET)
//   coxa : 0°     -> 512
//   fémur: -30°   -> 410
//   tibia: -100°  -> 171
// Patas 1-3 validadas. Patas 4-6 arrancan con la misma plantilla: AJUSTAR.
const int32_t RAW_HOME[NUM_LEGS][NUM_JOINTS] = {
  { 512, 410, 171 },   // Pata 1  (lado A, validado)
  { 512, 410, 171 },   // Pata 2  (lado A, validado)
  { 512, 410, 171 },   // Pata 3  (lado A, validado)
  { 512, 410, 171 },   // Pata 4  (lado B) <- CALIBRAR
  { 512, 410, 171 },   // Pata 5  (lado B) <- CALIBRAR
  { 512, 410, 171 }    // Pata 6  (lado B) <- CALIBRAR
};

// --- Sentido de giro: +1 si el servo gira igual que el modelo, -1 si invertido
// CUIDADO: en MATLAB ya hay un espejo cinemático de la coxa (vector sgn) para
// las patas 4-6. Si una pata camina al revés, invertí AQUÍ o ALLÁ, nunca en los
// dos lados a la vez: se cancelan.
const int8_t JOINT_SIGN[NUM_LEGS][NUM_JOINTS] = {
  { +1, +1, -1 },   // Pata 1  (lado A, validado)
  { +1, +1, -1 },   // Pata 2  (lado A, validado)
  { +1, +1, -1 },   // Pata 3  (lado A, validado)
  { +1, +1, -1 },   // Pata 4  (lado B) <- CALIBRAR
  { +1, +1, -1 },   // Pata 5  (lado B) <- CALIBRAR
  { +1, +1, -1 }    // Pata 6  (lado B) <- CALIBRAR
};

// --- Límites de seguridad: desviación máxima permitida respecto a HOME -------
// En unidades crudas (1 unidad = 0.293°). Si el IK o un error de red mandan un
// valor absurdo, se recorta acá antes de llegar al servo.
const int32_t RAW_DEV_MAX[NUM_JOINTS] = { 200, 250, 250 };   // ~59° / ~73° / ~73°

// --- Velocidad de las articulaciones ----------------------------------------
// Con streaming de posición, Moving Speed actúa como limitador de rapidez y
// suaviza el movimiento. 0 = MÁXIMA (peligroso). 250 ~ 166 °/s.
const int32_t MOVING_SPEED_HOME = 80;    // lento para ir a HOME
const int32_t MOVING_SPEED_WALK = 250;   // para caminar

// --- Watchdog del enlace -----------------------------------------------------
// Si se corta el WiFi a media zancada, MANTENER la última pose es más seguro
// que saltar a HOME de golpe. Poné 1 si preferís que regrese solo.
#define HOME_ON_TIMEOUT   0
const uint32_t LINK_TIMEOUT_MS = 1000;

DynamixelWorkbench dxl_wb;

uint8_t  dxl_ids[NUM_DXL];
int32_t  goal_raw[NUM_DXL];

char     lineBuf[256];
size_t   lineLen = 0;
uint32_t lastFrame = 0;
bool     streaming = false;

StaticJsonDocument<384> doc;

// ============================ Conversiones ==================================

// grados absolutos del servo -> unidades crudas (1024 pasos sobre 300°)
int32_t degToRaw(float deg)
{
  float raw = 512.0f + deg * (1023.0f / 300.0f);
  if (raw < 0.0f)    raw = 0.0f;
  if (raw > 1023.0f) raw = 1023.0f;
  return (int32_t)(raw + 0.5f);
}

// ángulo del MODELO (grados) -> unidades crudas, con calibración y recorte
int32_t modelDegToRaw(uint8_t leg, uint8_t j, float degModel)
{
  float delta = (degModel - MODEL_HOME_DEG[j]) * (float)JOINT_SIGN[leg][j];
  int32_t raw = RAW_HOME[leg][j] + (int32_t)lroundf(delta * (1023.0f / 300.0f));

  int32_t lo = RAW_HOME[leg][j] - RAW_DEV_MAX[j];
  int32_t hi = RAW_HOME[leg][j] + RAW_DEV_MAX[j];
  if (raw < lo) raw = lo;
  if (raw > hi) raw = hi;
  if (raw < 0)    raw = 0;
  if (raw > 1023) raw = 1023;
  return raw;
}

// ============================ Acciones ======================================

void escribirGoles()
{
  const char *log;
  if (dxl_wb.syncWrite(0, dxl_ids, NUM_DXL, goal_raw, 1, &log) == false) {
    Serial.print("syncWrite fallo: ");
    Serial.println(log);
  }
}

void irAHome()
{
  for (uint8_t leg = 0; leg < NUM_LEGS; leg++)
    for (uint8_t j = 0; j < NUM_JOINTS; j++)
      goal_raw[leg*NUM_JOINTS + j] = RAW_HOME[leg][j];
  escribirGoles();
  Serial.println("-> HOME");
}

void setVelocidad(int32_t vel)
{
  const char *log;
  for (uint8_t i = 0; i < NUM_DXL; i++)
    dxl_wb.jointMode(dxl_ids[i], vel, 0, &log);
}

void torque(bool on)
{
  const char *log;
  for (uint8_t i = 0; i < NUM_DXL; i++) {
    if (on) dxl_wb.torqueOn(dxl_ids[i], &log);
    else    dxl_wb.torqueOff(dxl_ids[i], &log);
  }
  Serial.println(on ? "-> torque ON" : "-> torque OFF");
}

// ============================ Parseo =========================================

void procesaLinea(const char *linea)
{
  DeserializationError err = deserializeJson(doc, linea);
  if (err) {
    Serial.print("JSON invalido: ");
    Serial.println(err.c_str());
    return;
  }

  // ---- Trama de ángulos ----------------------------------------------------
  JsonArray q = doc["q"].as<JsonArray>();
  if (!q.isNull()) {
    if (q.size() != NUM_DXL) {
      Serial.print("Trama con ");
      Serial.print(q.size());
      Serial.println(" valores; se esperaban 18. Descartada.");
      return;
    }
    for (uint8_t leg = 0; leg < NUM_LEGS; leg++) {
      for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        uint8_t idx = leg*NUM_JOINTS + j;
        float deg = q[idx].as<int>() / 10.0f;      // decigrados -> grados
        goal_raw[idx] = modelDegToRaw(leg, j, deg);
      }
    }
    escribirGoles();
    lastFrame = millis();
    if (!streaming) {
      streaming = true;
      setVelocidad(MOVING_SPEED_WALK);
      Serial.println("-> streaming iniciado");
    }
    return;
  }

  // ---- Comandos ------------------------------------------------------------
  const char *c = doc["c"];
  if (c) {
    streaming = false;
    if      (strcmp(c, "home")   == 0) { setVelocidad(MOVING_SPEED_HOME); irAHome(); }
    else if (strcmp(c, "relax")  == 0) { torque(false); }
    else if (strcmp(c, "torque") == 0) { torque(true);  }
    else { Serial.print("comando desconocido: "); Serial.println(c); }
  }
}

// ============================== Setup =======================================

void setup()
{
  Serial.begin(115200);      // monitor (USB)
  Serial2.begin(LINK_BAUD);  // enlace con el ESP32
  delay(3000);               // el CDC de la OpenCM necesita su tiempo

  const char *log;
  uint16_t model_number = 0;

  // índice lineal de IDs para el syncWrite
  for (uint8_t leg = 0; leg < NUM_LEGS; leg++)
    for (uint8_t j = 0; j < NUM_JOINTS; j++)
      dxl_ids[leg*NUM_JOINTS + j] = LEG_IDS[leg][j];

  // 1. Bus -------------------------------------------------------------------
  if (dxl_wb.init(DEVICE_NAME, DXL_BAUD, &log) == false) {
    Serial.println(log);
    Serial.println("Fallo al inicializar el bus. Abortando.");
    return;
  }
  Serial.print("Bus a ");
  Serial.println(DXL_BAUD);

  // 2. Verificar los 18 servos ----------------------------------------------
  Serial.println("Verificando servos...");
  bool todos_ok = true;
  for (uint8_t i = 0; i < NUM_DXL; i++) {
    if (dxl_wb.ping(dxl_ids[i], &model_number, &log) == false) {
      Serial.print("  NO responde ID ");
      Serial.print(dxl_ids[i]);
      Serial.print(" (pata ");
      Serial.print(i / NUM_JOINTS + 1);
      Serial.println(")");
      todos_ok = false;
    }
  }
  if (!todos_ok) {
    Serial.println("Faltan servos. Abortando por seguridad.");
    return;
  }
  Serial.println("Los 18 servos responden.");

  // 3. Modo articulación, lento para el arranque -----------------------------
  setVelocidad(MOVING_SPEED_HOME);

  // 4. syncWrite de Goal Position -------------------------------------------
  if (dxl_wb.addSyncWriteHandler(dxl_ids[0], "Goal_Position", &log) == false) {
    Serial.println(log);
    Serial.println("No se pudo registrar el syncWrite. Abortando.");
    return;
  }

  // 5. HOME ------------------------------------------------------------------
  Serial.println("Moviendo a HOME...");
  irAHome();
  delay(2000);

  lastFrame = millis();
  Serial.println("Listo. Esperando tramas del ESP32 por Serial2.");
}

// =============================== Loop =======================================

void loop()
{
  // --- Lectura no bloqueante del enlace, línea por línea --------------------
  while (Serial2.available()) {
    char c = Serial2.read();

    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen > 0) procesaLinea(lineBuf);
      lineLen = 0;
    }
    else if (c != '\r') {
      if (lineLen < sizeof(lineBuf) - 1) {
        lineBuf[lineLen++] = c;
      } else {
        lineLen = 0;            // línea corrupta: se descarta entera
        Serial.println("linea demasiado larga, descartada");
      }
    }
  }

  // --- Watchdog -------------------------------------------------------------
  if (streaming && (millis() - lastFrame > LINK_TIMEOUT_MS)) {
    streaming = false;
    Serial.println("Enlace perdido.");
#if HOME_ON_TIMEOUT
    setVelocidad(MOVING_SPEED_HOME);
    irAHome();
#else
    Serial.println("Manteniendo la ultima pose.");
#endif
  }
}
