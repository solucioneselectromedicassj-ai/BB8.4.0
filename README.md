# Simulador Fetal — SEM

**Soluciones Electromédicas SJ**

Dispositivo portátil para verificación funcional de monitores fetales. Simula contracciones uterinas mediante un sistema neumático controlado por software, evaluando la cadena completa: transductor TOCO real → cable → monitor.

A diferencia de simuladores comerciales como el Fluke PS320 (que inyectan señal eléctrica directamente), este sistema presiona el transductor físicamente, permitiendo detectar fallas en el transductor, el cable y el monitor en una sola prueba.

---

## Arquitectura

```
PWA (celular / PC)
  │  USB Serial (WebSerial API)
  │  WiFi (WebSocket)
  ▼
ESP32
  ├── PWM → MOSFET → Bomba DC (reciclada de tensiómetro)
  ├── GPIO → Solenoide entrada (inflar)
  ├── GPIO → Solenoide escape (desinflar)
  └── ADC ← Sensor de presión MPX5050

Cámara 3D (PLA)
  └── Adaptadores intercambiables por modelo de transductor
      └── Transductor TOCO real (del hospital)
              └── Monitor fetal
```

---

## Hardware

### Componentes

| Componente | Fuente | Notas |
|---|---|---|
| ESP32 DevKit | Compra | ~$5 USD |
| Bomba DC + válvula solenoide | Tensiómetro electrónico reciclado | Bomba silenciosa, rango 0–300 mmHg |
| Sensor MPX5050 / MPXV5050GP | Compra | 0–50 kPa = 0–375 mmHg |
| MOSFET IRF520 o similar | Compra | Control PWM de la bomba |
| Diodo flyback 1N4007 | Compra | Protección en solenoides |
| Cámara + adaptadores | Impresión 3D (PLA) | Diseño universal intercambiable |

### Conexiones ESP32

```
GPIO 25  →  Gate MOSFET bomba DC
GPIO 26  →  Solenoide entrada (inflar)    — normalmente cerrado
GPIO 27  →  Solenoide escape (desinflar)  — normalmente cerrado
GPIO 34  ←  Sensor presión MPX5050 (Vout) — solo entrada ADC
GPIO 32  →  LED rojo (error / límite)
GPIO 33  →  LED verde (activo / estado)
GND          Común a todos los módulos
3.3V / 5V    Según módulo
```

### Calibración del sensor

El firmware asume **MPX5050** con alimentación a 3.3V:

```
Vout = 3.3 × (0.018 × P_kPa + 0.04)
P_mmHg = P_kPa × 7.501
```

Si usás otro sensor, ajustá `SENSOR_SLOPE` y `SENSOR_OFFSET` en el firmware.

---

## Firmware

### Dependencias (Arduino Library Manager)

- **ArduinoJson** — Benoit Blanchon
- **arduinoWebSockets** — Markus Sattler *(solo si usás WiFi)*

### Configuración WiFi (opcional)

En `simulador_fetal.ino`, líneas 17–18:

```cpp
const char* WIFI_SSID = "nombre-de-tu-red";
const char* WIFI_PASS = "contraseña";
```

Dejando los campos vacíos, el dispositivo funciona solo por USB Serial.

### Flashear

1. Abrí Arduino IDE 2.x
2. Instalá soporte para ESP32: `Boards Manager → esp32 by Espressif`
3. Seleccioná `ESP32 Dev Module`
4. Abrí `firmware/simulador_fetal.ino`
5. Instalá las dependencias desde `Library Manager`
6. Flash (Ctrl+U)

---

## Protocolo de comunicación

Toda la comunicación es **JSON por línea** a 115200 baudios (Serial) o WebSocket puerto 81 (WiFi).

### Comandos → ESP32

```json
// Calibrar zero (baseline atmosférico, esperar 4s)
{"cmd": "zero"}

// Detener y desinflar
{"cmd": "stop"}

// Mantener presión fija para calibración
{"cmd": "calibrate", "pressure": 30}

// Ejecutar protocolo de contracciones
{
  "cmd": "run",
  "protocol": [
    {"mmhg": 10, "rise": 20, "peak": 20, "fall": 20, "interval": 60, "repeat": 3},
    {"mmhg": 30, "rise": 25, "peak": 30, "fall": 25, "interval": 60, "repeat": 3},
    {"mmhg": 55, "rise": 30, "peak": 30, "fall": 30, "interval": 60, "repeat": 3}
  ]
}

// Ajustar PID en tiempo real
{"cmd": "pid", "kp": 10.0, "ki": 0.8, "kd": 0.2}
```

### Telemetría ← ESP32 (cada 100 ms)

```json
{"t": 12345, "p": 28.4, "s": "PEAK", "c": 1, "r": 0, "tgt": 30.0}
```

| Campo | Descripción |
|---|---|
| `t` | Timestamp ms (millis del ESP32) |
| `p` | Presión actual en mmHg |
| `s` | Estado: IDLE / RISE / PEAK / FALL / INTER / CAL / ZEROING / ERROR |
| `c` | Índice del grupo de contracción actual |
| `r` | Repetición actual dentro del grupo |
| `tgt` | Presión objetivo actual |

### Eventos ← ESP32

```json
{"event": "ready",       "data": {"version": "1.0", "wifi": true, "ip": "192.168.1.100"}}
{"event": "zeroed",      "data": {"offset": 2.3}}
{"event": "started",     "data": {"groups": 3}}
{"event": "peak",        "data": {"c": 0, "r": 1}}
{"event": "inter",       "data": {"c": 0, "r": 1}}
{"event": "done",        "data": {"total_contracciones": 3}}
{"event": "stopped",     "data": {}}
{"event": "calibrating", "data": {"target": 30.0}}
```

---

## Estados del firmware

```
IDLE ──zero──► ZEROING ──4s──► IDLE
IDLE ──run───► RISE ──► PEAK ──► FALL ──► INTER ──► (siguiente grupo o IDLE)
IDLE ──cal───► CALIBRATE
cualquiera ──stop──► IDLE
cualquiera ──presión > 110 mmHg──► ERROR (safe: desinfla)
```

El perfil de ascenso y descenso usa suavizado sinusoidal:

```
ascenso: target × (0.5 - 0.5 × cos(progreso × π))
descenso: target × (0.5 + 0.5 × cos(progreso × π))
```

---

## PWA

### Uso

1. Abrí la app desde Vercel en Chrome (escritorio o Android)
2. **Conectar por USB**: Chrome detecta el ESP32 vía WebSerial — necesita cable USB y `chrome://flags/#enable-experimental-web-platform-features` habilitado en Android
3. **Conectar por WiFi**: ingresá la IP del ESP32 (ver monitor serial al arrancar)
4. Instalá como app: Chrome → menú → "Instalar aplicación"

### Tabs

| Tab | Función |
|---|---|
| Protocolo | Editor de secuencia de contracciones |
| Calibración | Set points fijos o libres + registro de lecturas del monitor |
| FCF ♥ | Info del canal de frecuencia cardíaca fetal (por jack, etapa actual) |
| Eventos | Log completo de la sesión |

### Despliegue en Vercel

```bash
# 1. Clonar repo
git clone https://github.com/tu-usuario/simulador-fetal
cd simulador-fetal

# 2. Conectar a Vercel (una sola vez)
npx vercel link

# 3. Deployar
npx vercel --prod
```

El `vercel.json` ya está configurado para servir la carpeta `web/` como raíz.

---

## Estructura del repositorio

```
simulador-fetal/
├── firmware/
│   └── simulador_fetal/
│       └── simulador_fetal.ino    ← Arduino IDE requiere mismo nombre que carpeta
├── web/
│   ├── simulador_fetal.html       ← app principal
│   ├── manifest.json              ← PWA manifest
│   └── icon.svg                   ← ícono de la app
├── vercel.json                    ← configuración de despliegue
└── README.md
```

---

## Comparativa con Tesis UNC (Costamagna & Flores, 2025)

| Característica | Tesis UNC | Este proyecto |
|---|---|---|
| Modelos de transductor | 2 fijos | Universal + adaptadores 3D |
| Set points | 3 fijos (10–55 mmHg) | N puntos, configurables |
| Perfil de onda | Presión estática | Waveform sinusoidal completo |
| Interfaz | Botón + LED | PWA (celular o PC) |
| Conexión | — | USB Serial + WiFi |
| Log de resultados | No | Sí, con veredicto automático |
| Costo estimado | ~$1.700 USD | < $100 USD (partes recicladas) |
| Evaluación | Monitor | Transductor + cable + monitor |

---

## Roadmap

- [x] Firmware PID de presión con perfiles de onda
- [x] PWA con protocolo configurable y log de calibración
- [ ] Integración FCF al ESP32 (control de parlante directo)
- [ ] Sincronización FCF–TOCO (desaceleraciones tipo I, II, III)
- [ ] Diseño CAD de la cámara universal (Fusion 360 / FreeCAD)
- [ ] Adaptadores 3D para modelos Philips, GE, Mindray
- [ ] Modo autónomo (pantalla OLED + botones)

---

## Licencia

MIT — libre para uso y modificación con atribución.

**SEM — Soluciones Electromédicas SJ · Pocito, San Juan, Argentina**
