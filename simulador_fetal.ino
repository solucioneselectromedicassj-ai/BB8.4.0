/*
 * SIMULADOR FETAL — Firmware ESP32
 * Control neumático de contracciones uterinas (TOCO)
 * SEM — Soluciones Electromédicas SJ
 *
 * Librerías requeridas (Library Manager):
 *   - ArduinoJson  (Benoit Blanchon)
 *   - arduinoWebSockets  (Markus Sattler)  ← solo si usás WiFi
 *
 * Conexiones hardware:
 *   GPIO 25 → MOSFET gate bomba DC
 *   GPIO 26 → Solenoide entrada (inflar)  — normalmente cerrado
 *   GPIO 27 → Solenoide escape  (desinflar)— normalmente cerrado
 *   GPIO 34 → ADC sensor presión (solo entrada, 0–3.3V)
 *   GPIO 32 → LED rojo (error)
 *   GPIO 33 → LED verde (activo)
 *
 * Sensor de presión asumido: MPX5050 / MPXV5050GP
 *   Rango: 0–50 kPa = 0–375 mmHg (suficiente para 0–100 mmHg)
 *   Vout = Vs × (0.018 × P_kPa + 0.04)   Vs = 3.3V
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WebSocketsServer.h>

// ─── CONFIGURACIÓN WiFi (dejar vacío para modo solo USB) ─────────────
const char* WIFI_SSID = "";          // Ej: "SEM-Fetal"
const char* WIFI_PASS = "";          // Contraseña
const int   WS_PORT   = 81;

// ─── PINES ───────────────────────────────────────────────────────────
#define PIN_BOMBA      25
#define PIN_VALV_IN    26
#define PIN_VALV_OUT   27
#define PIN_SENSOR_P   34
#define PIN_LED_R      32
#define PIN_LED_G      33

// ─── CONSTANTES ──────────────────────────────────────────────────────
#define MAX_MMHG         110.0f   // Límite de seguridad absoluto
#define SERIAL_BAUD      115200
#define TELEM_MS         100      // Telemetría cada 100 ms
#define PID_MS           20       // PID a 50 Hz
#define ADC_SAMPLES      16       // Promedio para estabilidad
#define MAX_PROTOCOLO    20       // Máximo de grupos en un protocolo

// Calibración MPX5050 (Vs=3.3V, ADC 12-bit=4095)
#define SENSOR_VS        3.3f
#define SENSOR_SLOPE     0.018f
#define SENSOR_OFFSET    0.04f
#define KPA_TO_MMHG      7.50062f

// ─── ENUMERACIONES ───────────────────────────────────────────────────
enum Estado : uint8_t {
  IDLE, ZEROING, CALIBRATE, RISE, PEAK, FALL, INTER, ST_ERROR
};

// ─── ESTRUCTURAS ─────────────────────────────────────────────────────
struct Contraccion {
  float target;      // mmHg
  float rise_s;      // duración ascenso
  float peak_s;      // duración pico
  float fall_s;      // duración descenso
  float interval_s;  // espera entre contracciones
  int   repeat;      // repeticiones de este grupo
};

// ─── VARIABLES GLOBALES ──────────────────────────────────────────────
Estado  estado         = IDLE;
float   presion        = 0.0f;
float   zero_offset    = 0.0f;
float   cal_target     = 0.0f;

Contraccion protocolo[MAX_PROTOCOLO];
int     proto_len      = 0;
int     cont_idx       = 0;      // grupo actual
int     rep_idx        = 0;      // repetición dentro del grupo
unsigned long t_estado = 0;

// PID
float pid_kp       = 10.0f;
float pid_ki       =  0.8f;
float pid_kd       =  0.2f;
float pid_integral =  0.0f;
float pid_prev_err =  0.0f;
unsigned long t_pid = 0;

// Temporización
unsigned long t_telem  = 0;
unsigned long t_led    = 0;
bool          led_g_st = false;

// Comunicación
WebSocketsServer ws(WS_PORT);
bool wifi_ok = false;

// ─── HARDWARE ────────────────────────────────────────────────────────
void setBomba(int pwm) {
  ledcWrite(0, constrain(pwm, 0, 255));
}

void setValvulas(bool entrada, bool salida) {
  digitalWrite(PIN_VALV_IN,  entrada ? HIGH : LOW);
  digitalWrite(PIN_VALV_OUT, salida  ? HIGH : LOW);
}

// Estado seguro: desinfla y apaga bomba
void safe() {
  setBomba(0);
  setValvulas(false, true);   // Escape abierto
  pid_integral = 0.0f;
  pid_prev_err = 0.0f;
}

float leerPresionRaw() {
  long sum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    sum += analogRead(PIN_SENSOR_P);
    delayMicroseconds(50);
  }
  float adc  = sum / (float)ADC_SAMPLES;
  float vout = adc * SENSOR_VS / 4095.0f;
  float kpa  = (vout / SENSOR_VS - SENSOR_OFFSET) / SENSOR_SLOPE;
  return kpa * KPA_TO_MMHG;
}

float leerPresion() {
  return max(0.0f, leerPresionRaw() - zero_offset);
}

// ─── PID ─────────────────────────────────────────────────────────────
void pid(float target) {
  unsigned long now = millis();
  if (now - t_pid < (unsigned long)PID_MS) return;
  float dt = (now - t_pid) / 1000.0f;
  t_pid = now;

  // Seguridad absoluta — nunca superar límite
  if (presion > MAX_MMHG) {
    safe();
    estado = ST_ERROR;
    return;
  }

  float err  = target - presion;
  pid_integral += err * dt;
  pid_integral  = constrain(pid_integral, -60.0f, 60.0f);
  float deriv   = (err - pid_prev_err) / dt;
  pid_prev_err  = err;
  float out     = pid_kp * err + pid_ki * pid_integral + pid_kd * deriv;

  if (err > 1.5f) {
    // Inflar
    setValvulas(true, false);
    setBomba((int)constrain(out, 30, 255));
  } else if (err < -1.5f) {
    // Desinflar
    setValvulas(false, true);
    setBomba(0);
    pid_integral = 0;
  } else {
    // Mantener con bomba mínima
    setValvulas(true, false);
    setBomba(max(20, (int)(out * 0.3f)));
  }
}

// ─── COMUNICACIÓN ────────────────────────────────────────────────────
void enviarStr(const String& json) {
  Serial.println(json);
  if (wifi_ok) ws.broadcastTXT(json);
}

void evento(const char* ev, const String& data = "{}") {
  enviarStr("{\"event\":\"" + String(ev) + "\",\"data\":" + data + "}");
}

void telemetria() {
  float tgt = 0.0f;
  if (estado == CALIBRATE) {
    tgt = cal_target;
  } else if (cont_idx < proto_len) {
    tgt = protocolo[cont_idx].target;
  }

  char buf[128];
  snprintf(buf, sizeof(buf),
    "{\"t\":%lu,\"p\":%.1f,\"s\":\"%s\",\"c\":%d,\"r\":%d,\"tgt\":%.1f}",
    millis(), presion, estadoStr(), cont_idx, rep_idx, tgt
  );
  enviarStr(String(buf));
}

const char* estadoStr() {
  switch (estado) {
    case IDLE:      return "IDLE";
    case ZEROING:   return "ZEROING";
    case CALIBRATE: return "CAL";
    case RISE:      return "RISE";
    case PEAK:      return "PEAK";
    case FALL:      return "FALL";
    case INTER:     return "INTER";
    case ST_ERROR:  return "ERROR";
    default:        return "?";
  }
}

// ─── PROCESAMIENTO DE COMANDOS ────────────────────────────────────────
void procesarCmd(const String& line) {
  StaticJsonDocument<2048> doc;
  if (deserializeJson(doc, line) != DeserializationError::Ok) return;

  const char* cmd = doc["cmd"] | "";

  // ZERO: calibra presión ambiente como baseline
  if (strcmp(cmd, "zero") == 0) {
    safe();
    estado  = ZEROING;
    t_estado = millis();

  // STOP: para todo y desinfla
  } else if (strcmp(cmd, "stop") == 0) {
    safe();
    estado = IDLE;
    evento("stopped");

  // CALIBRATE: mantiene presión fija para verificación
  } else if (strcmp(cmd, "calibrate") == 0) {
    cal_target = constrain((float)(doc["pressure"] | 0.0f), 0.0f, MAX_MMHG);
    estado = CALIBRATE;
    t_estado = millis();
    evento("calibrating", "{\"target\":" + String(cal_target) + "}");

  // RUN: ejecuta secuencia de contracciones
  } else if (strcmp(cmd, "run") == 0) {
    proto_len = 0;
    JsonArray arr = doc["protocol"].as<JsonArray>();
    for (JsonObject c : arr) {
      if (proto_len >= MAX_PROTOCOLO) break;
      Contraccion& ct = protocolo[proto_len++];
      ct.target     = constrain((float)(c["mmhg"]     | 30.0f), 1.0f, MAX_MMHG);
      ct.rise_s     = max(5.0f,  (float)(c["rise"]     | 20.0f));
      ct.peak_s     = max(5.0f,  (float)(c["peak"]     | 20.0f));
      ct.fall_s     = max(5.0f,  (float)(c["fall"]     | 20.0f));
      ct.interval_s = max(10.0f, (float)(c["interval"] | 60.0f));
      ct.repeat     = max(1,     (int)  (c["repeat"]   | 1));
    }
    if (proto_len == 0) return;
    cont_idx = 0;
    rep_idx  = 0;
    estado   = RISE;
    t_estado = millis();
    pid_integral = 0;
    evento("started", "{\"groups\":" + String(proto_len) + "}");

  // PID: ajuste de parámetros en tiempo real
  } else if (strcmp(cmd, "pid") == 0) {
    if (doc.containsKey("kp")) pid_kp = doc["kp"];
    if (doc.containsKey("ki")) pid_ki = doc["ki"];
    if (doc.containsKey("kd")) pid_kd = doc["kd"];
    evento("pid_ok", "{\"kp\":" + String(pid_kp) +
                     ",\"ki\":" + String(pid_ki) +
                     ",\"kd\":" + String(pid_kd) + "}");

  // STATUS: pide estado actual
  } else if (strcmp(cmd, "status") == 0) {
    evento("status", "{\"state\":\"" + String(estadoStr()) +
                     "\",\"p\":" + String(presion) +
                     ",\"wifi\":" + String(wifi_ok ? "true" : "false") + "}");
  }
}

// ─── MÁQUINA DE ESTADOS ──────────────────────────────────────────────
void maquina() {
  unsigned long elapsed = millis() - t_estado;

  switch (estado) {

    case IDLE:
      safe();
      break;

    case ST_ERROR:
      safe();
      digitalWrite(PIN_LED_R, (millis() / 200) % 2);  // Blink rápido
      break;

    case ZEROING:
      safe();
      if (elapsed > 4000) {
        // Promedio largo para offset estable
        long sum = 0;
        for (int i = 0; i < 64; i++) { sum += analogRead(PIN_SENSOR_P); delay(5); }
        float adc  = sum / 64.0f;
        float vout = adc * SENSOR_VS / 4095.0f;
        float kpa  = (vout / SENSOR_VS - SENSOR_OFFSET) / SENSOR_SLOPE;
        zero_offset = kpa * KPA_TO_MMHG;
        estado = IDLE;
        evento("zeroed", "{\"offset\":" + String(zero_offset, 2) + "}");
      }
      break;

    case CALIBRATE:
      pid(cal_target);
      // Blink lento LED verde
      if (millis() - t_led > 600) {
        led_g_st = !led_g_st;
        digitalWrite(PIN_LED_G, led_g_st);
        t_led = millis();
      }
      break;

    case RISE: {
      if (cont_idx >= proto_len) { estado = IDLE; break; }
      Contraccion& c = protocolo[cont_idx];
      float prog   = min(1.0f, elapsed / (c.rise_s * 1000.0f));
      // Curva suavizada: ease-in con seno
      float smooth = 0.5f - 0.5f * cos(prog * PI);
      pid(smooth * c.target);
      digitalWrite(PIN_LED_G, HIGH);
      if (elapsed >= (unsigned long)(c.rise_s * 1000)) {
        estado   = PEAK;
        t_estado = millis();
        pid_integral = 0;
        evento("peak", "{\"c\":" + String(cont_idx) + ",\"r\":" + String(rep_idx) + "}");
      }
      break;
    }

    case PEAK: {
      Contraccion& c = protocolo[cont_idx];
      pid(c.target);
      if (elapsed >= (unsigned long)(c.peak_s * 1000)) {
        estado   = FALL;
        t_estado = millis();
      }
      break;
    }

    case FALL: {
      Contraccion& c = protocolo[cont_idx];
      float prog   = min(1.0f, elapsed / (c.fall_s * 1000.0f));
      float smooth = 0.5f + 0.5f * cos(prog * PI);  // ease-out
      pid(smooth * c.target);
      if (elapsed >= (unsigned long)(c.fall_s * 1000)) {
        safe();
        estado   = INTER;
        t_estado = millis();
        evento("inter", "{\"c\":" + String(cont_idx) + ",\"r\":" + String(rep_idx) + "}");
      }
      break;
    }

    case INTER: {
      safe();
      Contraccion& c = protocolo[cont_idx];
      if (elapsed >= (unsigned long)(c.interval_s * 1000)) {
        rep_idx++;
        if (rep_idx >= c.repeat) {
          rep_idx = 0;
          cont_idx++;
        }
        if (cont_idx >= proto_len) {
          estado = IDLE;
          evento("done", "{\"total_contracciones\":" + String(cont_idx) + "}");
        } else {
          estado   = RISE;
          t_estado = millis();
          pid_integral = 0;
        }
      }
      break;
    }
  }
}

// ─── SETUP ───────────────────────────────────────────────────────────
void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(PIN_VALV_IN,  OUTPUT);
  pinMode(PIN_VALV_OUT, OUTPUT);
  pinMode(PIN_LED_R,    OUTPUT);
  pinMode(PIN_LED_G,    OUTPUT);

  // PWM bomba — canal 0, 20 kHz, 8 bits
  ledcSetup(0, 20000, 8);
  ledcAttachPin(PIN_BOMBA, 0);

  safe();  // Estado seguro en arranque

  // WiFi opcional
  if (strlen(WIFI_SSID) > 0) {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
      delay(500);
      digitalWrite(PIN_LED_G, i % 2);
    }
    if (WiFi.status() == WL_CONNECTED) {
      wifi_ok = true;
      ws.begin();
      ws.onEvent([](uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
        if (type == WStype_TEXT) {
          procesarCmd(String((char*)payload));
        }
      });
    }
  }

  // Parpadeo de inicio
  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_LED_G, HIGH); delay(100);
    digitalWrite(PIN_LED_G, LOW);  delay(100);
  }

  evento("ready", "{\"version\":\"1.0\",\"wifi\":" +
    String(wifi_ok ? "true" : "false") +
    (wifi_ok ? (",\"ip\":\"" + WiFi.localIP().toString() + "\"") : "") +
    "}");
}

// ─── LOOP ────────────────────────────────────────────────────────────
void loop() {
  // 1. Leer presión
  presion = leerPresion();

  // 2. Comandos Serial
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 2) procesarCmd(line);
  }

  // 3. WebSocket
  if (wifi_ok) ws.loop();

  // 4. Máquina de estados
  maquina();

  // 5. Telemetría periódica
  if (millis() - t_telem >= (unsigned long)TELEM_MS) {
    telemetria();
    t_telem = millis();
  }
}
