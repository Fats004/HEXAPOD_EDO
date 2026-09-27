/*******************************************************************************
 * HEXÁPODO - Puente WiFi (Robotat) -> UART
 * -----------------------------------------------------------------------------
 *  MATLAB --TCP:80--> [ESP32] --Serial2 115200--> OpenCM9.04 --> 18x AX-12A
 *
 *  Basado en JSON_Matlab_Robotat.ino (Luis Salazar), con estos cambios clave:
 *
 *   1. LA CONEXIÓN NO SE CIERRA. El código original hacía client.stop() después
 *      de cada '}'. Eso está bien para mandar una pose, pero para caminar hay
 *      que sostener el socket abierto y transmitir a 20-30 Hz.
 *   2. NO SE PARSEA EL JSON AQUÍ. El ESP32 es solo un cable: reenvía la línea
 *      tal cual por UART. Quien interpreta es la OpenCM. Menos latencia y
 *      menos cosas que se pueden romper en medio.
 *   3. Tramas delimitadas por '\n' (no por '}'), que es lo que MATLAB manda
 *      con writeline().
 *   4. DOS MODOS DE RED, en espejo con robotat_hexapod_connect.m:
 *         laboratorio -> IP FIJA derivada del ID del agente
 *         otra red    -> DHCP, y la IP se lee del monitor serie
 *
 *  ----------------------------- MODOS DE RED ---------------------------------
 *  En el laboratorio, MATLAB deduce la dirección a partir del número de agente:
 *
 *        robotat_hexapod_connect(31)  ->  192.168.50.231
 *        robotat_hexapod_connect(36)  ->  192.168.50.236
 *
 *  Para que esa cuenta sea cierta, el ESP32 NO puede pedir la dirección por
 *  DHCP: tiene que reclamar exactamente la que le corresponde. Por eso, con
 *  RED_LABORATORIO en 1 la placa se autoasigna 192.168.50.(200 + HEXAPOD_ID)
 *  antes de conectarse, y esa dirección ya no cambia entre reinicios.
 *
 *  Fuera del laboratorio no hay forma de saber qué rango maneja el router de
 *  turno, así que con RED_LABORATORIO en 0 la placa se conecta por DHCP,
 *  imprime la IP que le tocó y esa es la que se le pasa a MATLAB como segundo
 *  argumento:
 *
 *        robotat_hexapod_connect(31, '192.168.1.6')
 *
 *  En ambos casos el ESP32 imprime al arrancar la llamada exacta que hay que
 *  escribir en MATLAB, para no tener que armarla a mano.
 *
 *  Si el bloque del OptiTrack está activo (USE_ROBOTAT en 1), OJO: ROBOTAT_ID
 *  es el número de MARCADOR dentro del sistema de captura y no tiene nada que
 *  ver con HEXAPOD_ID, que es la identidad de red del robot.
 *
 *  CABLEADO ESP32 <-> OpenCM9.04 (ambos son 3.3 V, NO necesitan level shifter):
 *      ESP32 GPIO17 (TX2)  ->  OpenCM Serial2 RX
 *      ESP32 GPIO16 (RX2)  <-  OpenCM Serial2 TX
 *      GND                 <-> GND    <-- imprescindible
 *  En la OpenCM9.04, Serial2 es el conector de 4 pines donde normalmente va el
 *  BT-210 / LN-101. Verificá TX/RX en el serigrafiado de tu placa.
 ******************************************************************************/

#include <WiFi.h>

// ----------------------------- Configuración --------------------------------
#define HEXAPOD_ID       31    // Identidad del robot: 31 a 36.
                               //   En el laboratorio fija la IP:
                               //   192.168.50.(200 + HEXAPOD_ID)
                               //   Es el mismo número que recibe
                               //   robotat_hexapod_connect() en MATLAB.

#define RED_LABORATORIO   1    // 1 = red del Robotat, con IP fija.
                               // 0 = cualquier otra red, por DHCP.

#define DEBUG_ECHO        0    // 1 = imprime cada trama en el monitor serie.
                               //     Dejalo en 0 al caminar: los prints cuestan
                               //     tiempo y meten jitter.

#define USE_ROBOTAT       0    // 1 cuando ya tengas marcador en el OptiTrack
#define ROBOTAT_ID     "105"   // <-- número de MARCADOR (no es HEXAPOD_ID)

#if (HEXAPOD_ID < 31) || (HEXAPOD_ID > 36)
  #error "HEXAPOD_ID fuera de rango: los IDs permitidos son del 31 al 36."
#endif

// --------------------------- Credenciales de red ----------------------------
#if RED_LABORATORIO
  const char* ssid     = "Robotat";
  const char* password = "iemtbmcit116";

  // Red del laboratorio: 192.168.50.0/24
  IPAddress IP_FIJA (192, 168, 50, 200 + HEXAPOD_ID);
  IPAddress GATEWAY (192, 168, 50, 1);
  IPAddress MASCARA (255, 255, 255, 0);
  IPAddress DNS_1   (192, 168, 50, 1);
#else
  const char* ssid     = "Robotat";        // <-- SSID de la red que vas a usar
  const char* password = "iemtbmcit116";   // <-- su contraseña
#endif

const uint32_t WIFI_TIMEOUT_MS = 20000;  // se rinde y avisa en vez de colgarse

// ------------------------------ Enlace UART ---------------------------------
#define LINK_BAUD   115200     // UART hacia la OpenCM
#define LINK_RX     16
#define LINK_TX     17

const size_t MAXLEN = 250;

WiFiServer server(80);
WiFiClient client;
String buf;

#if USE_ROBOTAT
  const char* host  = "192.168.50.200";
  const int   port2 = 1883;
  WiFiClient client2;
  String msg = "";
#endif

// --------------------------------- Setup ------------------------------------
void setup() {
  Serial.begin(9600);
  Serial2.begin(LINK_BAUD, SERIAL_8N1, LINK_RX, LINK_TX);
  buf.reserve(MAXLEN + 2);

  delay(300);
  Serial.println();
  Serial.println("Hexapodo - puente WiFi/UART");
  Serial.print("Agente: ");
  Serial.println(HEXAPOD_ID);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // sin esto la latencia se dispara a ~100 ms

#if RED_LABORATORIO
  // La IP se reclama ANTES de conectarse; si se hace después el router ya
  // entregó otra por DHCP y la cuenta del ID deja de cumplirse.
  Serial.print("Modo laboratorio. IP fija solicitada: ");
  Serial.println(IP_FIJA);
  if (!WiFi.config(IP_FIJA, GATEWAY, MASCARA, DNS_1)) {
    Serial.println("AVISO: no se pudo fijar la IP. Se continua por DHCP y la");
    Serial.println("       direccion puede NO coincidir con la del ID.");
  }
#else
  Serial.println("Modo otra red. La direccion la asigna el router (DHCP).");
#endif

  Serial.print("Conectando a ");
  Serial.print(ssid);
  WiFi.begin(ssid, password);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");

    if (millis() - t0 > WIFI_TIMEOUT_MS) {
      Serial.println();
      Serial.println("FALLO: no se pudo conectar a la red.");
      Serial.println("  - Revisa el SSID y la contrasena.");
#if RED_LABORATORIO
      Serial.println("  - Si no estas en el laboratorio, pone RED_LABORATORIO en 0.");
      Serial.println("  - Si otro equipo ya tiene esa IP, cambia HEXAPOD_ID.");
#else
      Serial.println("  - Si estas en el laboratorio, pone RED_LABORATORIO en 1.");
#endif
      Serial.println("Reintentando...");
      WiFi.disconnect();
      delay(500);
      WiFi.begin(ssid, password);
      t0 = millis();
      Serial.print("Conectando a ");
      Serial.print(ssid);
    }
  }
  Serial.println();

  IPAddress ip = WiFi.localIP();
  Serial.print(">>> IP del ESP32: ");
  Serial.println(ip);

#if RED_LABORATORIO
  if (!(ip == IP_FIJA)) {
    Serial.println(">>> AVISO: la IP obtenida NO es la que corresponde al ID.");
    Serial.println(">>> MATLAB no va a encontrar al robot por numero de agente.");
    Serial.print(">>> Usa la forma con IP manual:  hexa = robotat_hexapod_connect(");
    Serial.print(HEXAPOD_ID);
    Serial.print(", '");
    Serial.print(ip);
    Serial.println("');");
  } else {
    Serial.print(">>> En MATLAB:  hexa = robotat_hexapod_connect(");
    Serial.print(HEXAPOD_ID);
    Serial.println(");");
  }
#else
  Serial.print(">>> En MATLAB:  hexa = robotat_hexapod_connect(");
  Serial.print(HEXAPOD_ID);
  Serial.print(", '");
  Serial.print(ip);
  Serial.println("');");
  Serial.println(">>> Esta direccion puede cambiar al reconectar. Si MATLAB no");
  Serial.println(">>> conecta, vuelve a leerla aqui.");
#endif

  server.begin();
  server.setNoDelay(true);

#if USE_ROBOTAT
  Serial.print("Conectando al servidor OptiTrack... ");
  if (client2.connect(host, port2)) {
    Serial.println("ok");
    while (client2.available()) client2.read();
    client2.write(ROBOTAT_ID);
  } else {
    Serial.println("fallo (se sigue sin pose)");
  }
#endif
}

// --------------------------------- Loop -------------------------------------
void loop() {

  // --- Aceptar / mantener el cliente de MATLAB ------------------------------
  if (!client || !client.connected()) {
    WiFiClient nuevo = server.available();
    if (nuevo) {
      client = nuevo;
      client.setNoDelay(true);       // sin Nagle: las tramas salen ya
      buf = "";
      client.print("READY\n");
      Serial.println("[MATLAB conectado]");
    }
  } else {
    // --- Leer y reenviar línea por línea ------------------------------------
    while (client.available()) {
      char c = client.read();

      if (c == '\n') {
        Serial2.print(buf);
        Serial2.print('\n');         // la OpenCM corta por '\n'
        client.print("ok\n");        // ack -> MATLAB no adelanta tramas
#if DEBUG_ECHO
        Serial.println(buf);
#endif
        buf = "";
      }
      else if (c != '\r') {
        if (buf.length() < MAXLEN) {
          buf += c;
        } else {
          buf = "";                  // trama corrupta: se descarta entera
          Serial.println("[trama demasiado larga, descartada]");
        }
      }
    }

    if (!client.connected()) {
      client.stop();
      Serial.println("[MATLAB desconectado]");
    }
  }

  // --- Pose del OptiTrack (desactivado por ahora) ---------------------------
#if USE_ROBOTAT
  if (client2.available()) {
    msg = "";
    while (client2.available()) {
      char c2 = client2.read();
      msg += c2;
      if (c2 == '}') break;
    }
    Serial.println(msg);
    client2.write(ROBOTAT_ID);
  }
#endif
}
