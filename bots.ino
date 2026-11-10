/*
 * BOTS PCB 2.0 — синхронное управление ногами + мотор поворота
 *
 * Что изменено относительно DifferentialControl_AP:
 *  1. Убран дифференциальный микшер (y+x / y-x). Из-за него при малейшем
 *     отклонении джойстика в сторону моторы ног получали РАЗНЫЕ значения ШИМ.
 *     Теперь оба мотора ног (M1, M2) всегда получают ОДНУ И ТУ ЖЕ команду.
 *  2. Джойстик стал вертикальным: только вперёд / назад (скорость ног).
 *  3. Кнопка RELAY заменена на кнопку TURN: удержание вращает третий мотор
 *     (клеммник M3, силовой ключ MOSFET, PIN 15) с плавной регулировкой
 *     мощности через ШИМ.
 *  4. Исправлен failsafe: веб-страница шлёт «пульс» каждые 100 мс, а прошивка
 *     останавливает все моторы, если команд нет дольше 500 мс. В оригинале
 *     таймаут 3 с срабатывал даже при удержании неподвижного джойстика.
 *  5. Добавлены калибровочные коэффициенты TRIM_A / TRIM_B для выравнивания
 *     реальных скоростей двух моторов (они всегда немного отличаются).
 *
 * Плата: ESP32 Dev Module, пакет esp32 by Espressif версии 2.0.15.
 * Библиотеки те же, что в методичке (TB6612, AsyncTCP, ESPAsyncWebServer,
 * Arduino_JSON).
 */

#include <TB6612_ESP32.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>

// ------------------------- Пины (как в оригинале) -------------------------
#define AIN1 13
#define BIN1 12
#define AIN2 14
#define BIN2 27
#define PWMA 26
#define PWMB 25
#define STBY 33

#define TURN_PIN 15   // MOSFET T1 -> клеммник M3 (мотор поворота)
#define LED_PIN   2

// ------------------------------- WiFi -------------------------------------
const char* ssid     = "bot1";       // поменяйте на botN (номер на плате)
const char* password = "12345678";

// --------------------------- Настройки ног --------------------------------
// Если ноги крутятся в разные стороны — поменяйте знак у одного из offset.
const int offsetA = 1;
const int offsetB = -1;

// Калибровка: если мотор A быстрее мотора B, уменьшите TRIM_A (например 0.94).
// Если быстрее B — уменьшите TRIM_B. Значения от 0.5 до 1.0.
const float TRIM_A = 1.00;
const float TRIM_B = 1.00;

const int LEG_DEADZONE = 20;    // зона нечувствительности джойстика (0..255)
const int LEG_MIN_PWM  = 70;    // минимальный ШИМ, при котором моторы трогаются
const int LEG_MAX_PWM  = 255;   // ограничение максимальной скорости ног
const int LEG_RAMP     = 20;    // шаг разгона за один тик (плавный старт)

// ------------------------ Настройки мотора поворота -----------------------
const int TURN_CH   = 4;        // канал LEDC (1 и 2 заняты библиотекой TB6612)
const int TURN_FREQ = 5000;
const int TURN_RES  = 8;        // 0..255

// ШИМ мотора поворота: совместимость с ядром ESP32 2.x и 3.x
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define TURN_PWM_INIT()    ledcAttach(TURN_PIN, TURN_FREQ, TURN_RES)
  #define TURN_PWM_WRITE(v)  ledcWrite(TURN_PIN, (v))
#else
  #define TURN_PWM_INIT()    do { ledcSetup(TURN_CH, TURN_FREQ, TURN_RES); ledcAttachPin(TURN_PIN, TURN_CH); } while (0)
  #define TURN_PWM_WRITE(v)  ledcWrite(TURN_CH, (v))
#endif

// ------------------------------ Отладка -----------------------------------
#define DEBUG_SERIAL   1   // 1 = печатать в Serial Monitor приходящие команды
#define MOTOR_SELFTEST 0   // 1 = при старте на 1 с включить ноги и мотор поворота
                           //     (ПОДНИМИТЕ робота, чтобы ноги не касались поверхности!)

// ----------------------------- Прочее -------------------------------------
const unsigned long COMMAND_TIMEOUT = 500;  // мс без команд -> стоп
const unsigned long UPDATE_PERIOD   = 10;   // мс, период обновления моторов

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY, 5000, 8, 1);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY, 5000, 8, 2);

// Целевые значения приходят из WebSocket (другая задача), применяются в loop()
volatile int legTarget  = 0;    // -255..255, ОДНО значение на обе ноги
volatile int turnTarget = 0;    // 0..255
volatile int ledState   = 0;
volatile unsigned long lastCommandTime = 0;

int legCurrent  = 0;
int legApplied  = 1;            // != 0, чтобы при старте выполнился brake()
int turnApplied = -1;
unsigned long lastUpdate = 0;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>Bot Controller</title>
<style>
:root{--bg:#121212;--card:#1e1e1e;--text:#fff;--btn:#2c2c2c;--accent:#bb86fc;--turn:#ffb74d;--led:#03dac6}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;
  height:100dvh;width:100vw;overflow:hidden;display:flex;justify-content:center;align-items:center;
  -webkit-user-select:none;user-select:none;touch-action:none}
.controller{display:flex;flex-direction:column;width:100%;max-width:500px;height:100%;padding:20px;gap:16px}

.stick-section{flex:1;display:flex;justify-content:center;align-items:center;min-height:0}
.stick-zone{position:relative;width:120px;height:min(45vh,360px);background:var(--card);border-radius:60px;
  border:2px solid #333;box-shadow:inset 0 0 20px rgba(0,0,0,.5);touch-action:none}
.stick-zone::after{content:'';position:absolute;left:50%;top:10%;bottom:10%;width:1px;background:#333}
.stick-zone .lbl{position:absolute;left:0;right:0;text-align:center;font-size:11px;color:#777;letter-spacing:1px}
.lbl.top{top:12px}.lbl.bot{bottom:12px}
.knob{position:absolute;width:96px;height:96px;left:50%;top:50%;transform:translate(-50%,-50%);
  background:var(--accent);border-radius:50%;box-shadow:0 4px 10px rgba(0,0,0,.5);pointer-events:none;z-index:2}
.knob.active{box-shadow:0 0 20px var(--accent)}
.readout{position:absolute;right:-70px;top:50%;transform:translateY(-50%);font-size:14px;color:#aaa;width:60px}

.actions{display:flex;gap:16px}
.btn{flex:1;height:90px;background:var(--btn);border:3px solid transparent;border-radius:20px;color:var(--text);
  font-size:18px;font-weight:bold;display:flex;flex-direction:column;justify-content:center;align-items:center;gap:8px;
  outline:none;-webkit-tap-highlight-color:transparent;transition:all .15s;touch-action:none}
.ind{width:16px;height:16px;border-radius:50%;background:#555;transition:background-color .15s}
.turn-active{border-color:var(--turn)}.turn-active .ind{background:var(--turn);box-shadow:0 0 10px var(--turn)}
.led-active{border-color:var(--led)}.led-active .ind{background:var(--led);box-shadow:0 0 10px var(--led)}

.power{display:flex;align-items:center;gap:12px;font-size:14px;color:#aaa;padding-bottom:6px}
.power input{flex:1;accent-color:var(--turn)}
.power span{width:44px;text-align:right}
</style>
</head>
<body>
<div class="controller">
  <div class="stick-section">
    <div class="stick-zone" id="zone">
      <div class="lbl top">FORWARD</div>
      <div class="lbl bot">BACKWARD</div>
      <div class="knob" id="knob"></div>
      <div class="readout" id="readout">0</div>
    </div>
  </div>

  <div class="power">
    <label for="tp">Turn power</label>
    <input type="range" id="tp" min="30" max="100" value="100">
    <span id="tpv">100%</span>
  </div>

  <div class="actions">
    <button class="btn" id="btn-turn"><div class="ind"></div>TURN</button>
    <button class="btn" id="btn-led"><div class="ind"></div>LED</button>
  </div>
</div>

<script>
const gateway = `ws://${window.location.hostname}/ws`;
let websocket;
const state = {y: 0, turn: 0};

function wsReady(){ return websocket && websocket.readyState === WebSocket.OPEN; }
function sendState(){ if (wsReady()) websocket.send(JSON.stringify({y: state.y, turn: state.turn})); }

function initWebSocket(){
  websocket = new WebSocket(gateway);
  websocket.onclose = () => setTimeout(initWebSocket, 2000);
}

// ---- Vertical joystick: forward/backward only, one speed for both legs ----
const zone = document.getElementById('zone');
const knob = document.getElementById('knob');
const readout = document.getElementById('readout');
let stickPointer = null;

function moveStick(clientY){
  const r = zone.getBoundingClientRect();
  const travel = (r.height - knob.offsetHeight) / 2;
  let dy = clientY - (r.top + r.height / 2);
  dy = Math.max(-travel, Math.min(travel, dy));
  knob.style.transform = `translate(-50%, calc(-50% + ${dy}px))`;
  state.y = Math.round((-dy / travel) * 255);
  readout.textContent = state.y;
  sendState();
}
function releaseStick(){
  stickPointer = null;
  knob.classList.remove('active');
  knob.style.transform = 'translate(-50%, -50%)';
  state.y = 0;
  readout.textContent = 0;
  sendState();
}
zone.addEventListener('pointerdown', e => {
  if (stickPointer !== null) return;
  stickPointer = e.pointerId;
  zone.setPointerCapture(e.pointerId);
  knob.classList.add('active');
  moveStick(e.clientY);
});
zone.addEventListener('pointermove', e => { if (e.pointerId === stickPointer) moveStick(e.clientY); });
['pointerup','pointercancel','lostpointercapture'].forEach(ev =>
  zone.addEventListener(ev, e => { if (e.pointerId === stickPointer) releaseStick(); }));

// ---- TURN button: spins the turn motor while held ----
const btnTurn = document.getElementById('btn-turn');
const tp = document.getElementById('tp');
const tpv = document.getElementById('tpv');
let turnPointer = null;

function turnValue(){ return Math.round(tp.value * 2.55); }
tp.addEventListener('input', () => {
  tpv.textContent = tp.value + '%';
  if (turnPointer !== null) { state.turn = turnValue(); sendState(); }
});
btnTurn.addEventListener('pointerdown', e => {
  if (turnPointer !== null) return;
  turnPointer = e.pointerId;
  btnTurn.setPointerCapture(e.pointerId);
  btnTurn.classList.add('turn-active');
  state.turn = turnValue();
  sendState();
});
function releaseTurn(e){
  if (e.pointerId !== turnPointer) return;
  turnPointer = null;
  btnTurn.classList.remove('turn-active');
  state.turn = 0;
  sendState();
}
['pointerup','pointercancel','lostpointercapture'].forEach(ev => btnTurn.addEventListener(ev, releaseTurn));

// ---- LED ----
document.getElementById('btn-led').addEventListener('click', function(){
  this.classList.toggle('led-active');
  if (wsReady()) websocket.send(JSON.stringify({led: this.classList.contains('led-active') ? 1 : 0}));
});

// ---- Heartbeat: while anything is pressed, resend the command every 100 ms (for failsafe) ----
setInterval(() => { if (state.y !== 0 || state.turn !== 0) sendState(); }, 100);

window.addEventListener('load', initWebSocket);
</script>
</body>
</html>
)rawliteral";

// ----------------------------- Моторы ног ---------------------------------

// Джойстик (-255..255) -> скорость ног с зоной нечувствительности и минимальным ШИМ
int mapLegSpeed(int v) {
  int a = abs(v);
  if (a < LEG_DEADZONE) return 0;
  int pwm = map(a, LEG_DEADZONE, 255, LEG_MIN_PWM, LEG_MAX_PWM);
  return (v < 0) ? -pwm : pwm;
}

// ГЛАВНАЯ ФУНКЦИЯ СИНХРОНИЗАЦИИ: обе ноги получают одно и то же значение,
// различается только калибровочный коэффициент мотора.
void driveLegs(int speed) {
  if (speed == 0) {
    motor1.brake();
    motor2.brake();
    return;
  }
  int a = constrain((int)lroundf(speed * TRIM_A), -255, 255);
  int b = constrain((int)lroundf(speed * TRIM_B), -255, 255);
  motor1.drive(a);
  motor2.drive(b);
}

// --------------------------- Мотор поворота -------------------------------

// Вращение мотора M3 (силовой ключ, одно направление). speed: 0..255, 0 = стоп.
void driveTurn(int speed) {
  TURN_PWM_WRITE(constrain(speed, 0, 255));
}

// ------------------------------- WebSocket --------------------------------

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (!(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)) return;

  // JSON.parse требует нуль-терминированную строку
  char buf[96];
  if (len >= sizeof(buf)) return;
  memcpy(buf, data, len);
  buf[len] = 0;

  JSONVar msg = JSON.parse(buf);
  if (JSON.typeof(msg) == "undefined") return;

  lastCommandTime = millis();
#if DEBUG_SERIAL
  Serial.printf("RX: %s\n", buf);
#endif

  if (msg.hasOwnProperty("y")) {
    legTarget = mapLegSpeed(constrain((int)msg["y"], -255, 255));
  }
  if (msg.hasOwnProperty("turn")) {
    turnTarget = constrain((int)msg["turn"], 0, 255);
  }
  if (msg.hasOwnProperty("led")) {
    ledState = (int)msg["led"];
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      legTarget = 0;      // потеряли связь — сразу стоп
      turnTarget = 0;
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

// -------------------------------- Setup / loop ----------------------------

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("Firmware: BOTS_LegSync v1.1");

  pinMode(LED_PIN, OUTPUT);
  TURN_PWM_INIT();
  TURN_PWM_WRITE(0);

#if MOTOR_SELFTEST
  Serial.println("Self-test: legs (both motors forward, 1 s)");
  driveLegs(150);
  delay(1000);
  driveLegs(0);
  delay(300);
  Serial.println("Self-test: turn motor (1 s)");
  driveTurn(200);
  delay(1000);
  driveTurn(0);
  Serial.println("Self-test done");
#endif

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html);
  });
  server.begin();
}

void loop() {
  ws.cleanupClients();
  digitalWrite(LED_PIN, ledState);

  unsigned long now = millis();

  // Failsafe: нет команд дольше COMMAND_TIMEOUT -> остановить всё
  if ((legTarget != 0 || turnTarget != 0) && (now - lastCommandTime > COMMAND_TIMEOUT)) {
    legTarget = 0;
    turnTarget = 0;
    Serial.println("Command timeout! Stopping motors.");
  }

  if (now - lastUpdate < UPDATE_PERIOD) return;
  lastUpdate = now;

  // Ноги: плавный разгон по ОДНОМУ значению, торможение сразу
  int target = legTarget;
  if (target == 0) {
    legCurrent = 0;
  } else if (legCurrent < target) {
    legCurrent = min(legCurrent + LEG_RAMP, target);
  } else if (legCurrent > target) {
    legCurrent = max(legCurrent - LEG_RAMP, target);
  }
  if (legCurrent != legApplied) {
    driveLegs(legCurrent);   // оба мотора обновляются в одном месте и в один тик
    legApplied = legCurrent;
  }

  // Мотор поворота
  int t = turnTarget;
  if (t != turnApplied) {
    driveTurn(t);
    turnApplied = t;
  }
}