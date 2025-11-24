/*
  FINAL MASTER VERSION – ESP32 MOTORCYCLE BALANCER
  - MPU6050
  - Flywheel balancing
  - Rear-drive motor
  - Steering servo (ESP32Servo)
  - Web control (AP mode)
*/

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "MPU6050.h"
#include <ESP32Servo.h>

/* ---------------- PIN MAP ---------------- */
#define SDA_PIN 21
#define SCL_PIN 22

// Flywheel (L298N)
#define ENA_PIN 33
#define IN1_PIN 26
#define IN2_PIN 25

// Rear motor (single)
#define DRV_IN1 19
#define DRV_IN2 18

// Steering servo
#define SERVO_PIN 14

/* ---------------- NETWORK ---------------- */
const char* ap_ssid = "ESP32-BIKE";
const char* ap_pass = "12345678";

WebServer server(80);
WebSocketsServer webSocket(81);
WiFiUDP Udp;
const uint16_t UDP_PORT = 4210;

/* ---------------- IMU ---------------- */
MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;

bool useAngleX = true;

float angle = 0;
float gyroRate = 0;
float targetAngle = 0;

unsigned long lastMicros = 0;

/* ---------------- PD GAINS ---------------- */
float Kp = 7.0;
float Kd = 0.55;
float Kf = 0.10;
float alpha = 0.98;

bool balancingEnabled = false;
bool inAggressive = false;

const float ANGLE_CUTOFF = 40;
const float AGG_ANGLE = 10;
const float EXIT_ANGLE = 5;
const int AGG_PWM = 230;

/* ---------------- INTERNAL STATE ---------------- */
float flywheel_est = 0;
float lastPWM = 0;

/* ---------------- SERVO ---------------- */
Servo steeringServo;
int SERVO_CENTER = 90;
int SERVO_LEFT = 50;
int SERVO_RIGHT = 130;

/* ---------------- HTML PAGE ---------------- */
const char page[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
<title>Bike Control</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
button{padding:14px;margin:5px;font-size:20px}
</style>
</head>
<body>
<h2>ESP32 Motorcycle Control</h2>

<button onclick="send('START')">BALANCE START</button>
<button onclick="send('STOP')">BALANCE STOP</button><br>

<button onclick="send('FWD')">FORWARD</button>
<button onclick="send('BACK')">BACK</button><br>

<button onclick="send('LEFT')">LEFT</button>
<button onclick="send('RIGHT')">RIGHT</button>
<button onclick="send('STRAIGHT')">STRAIGHT</button><br><br>

Flywheel PWM:
<input id="p" type="number" value="0" min="-255" max="255">
<button onclick="fly()">SET</button><br><br>

Kp:<input id="kp" value="7.0">
Kd:<input id="kd" value="0.55">
Kf:<input id="kf" value="0.1">
<button onclick="setg()">SET</button>

<p>Status: <span id="st">--</span></p>

<script>
var ws = new WebSocket("ws://" + location.hostname + ":81/");
ws.onmessage = e => document.getElementById("st").innerText = e.data;

function send(x){ ws.send(x); }
function fly(){ ws.send("FLY=" + document.getElementById('p').value); }
function setg(){
  ws.send("Kp="+document.getElementById('kp').value);
  ws.send("Kd="+document.getElementById('kd').value);
  ws.send("Kf="+document.getElementById('kf').value);
}
</script>
</body>
</html>
)=====";

/* ---------------- MOTOR FUNCTIONS ---------------- */
void rearStop(){ digitalWrite(DRV_IN1, LOW); digitalWrite(DRV_IN2, LOW); }
void rearForward(){ digitalWrite(DRV_IN1, HIGH); digitalWrite(DRV_IN2, LOW); }
void rearBackward(){ digitalWrite(DRV_IN1, LOW); digitalWrite(DRV_IN2, HIGH); }

void flywheelMotor(int pwm){
  pwm = constrain(pwm, -255, 255);

  flywheel_est += (pwm - lastPWM) * 0.05;
  flywheel_est *= 0.97;
  lastPWM = pwm;

  int d = abs(pwm);

  if (pwm > 0){
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_PIN, d);
  } else if (pwm < 0){
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    analogWrite(ENA_PIN, d);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_PIN, 0);
  }
}

/* ---------------- STEERING ---------------- */
void steerLeft(){ steeringServo.write(SERVO_LEFT); }
void steerRight(){ steeringServo.write(SERVO_RIGHT); }
void steerCenter(){ steeringServo.write(SERVO_CENTER); }

/* ---------------- ANGLE FUNCTIONS ---------------- */
float calcAngleX(){ return atan2((float)ay, (float)az) * 57.3; }
float calcAngleY(){ return atan2((float)ax, (float)az) * 57.3; }

/* ---------------- AXIS DETECTION ---------------- */
void detectAxis(){
  Serial.println("Tilt L/R for 5s...");
  unsigned long t0 = millis();
  double sx=0,sx2=0,sy=0,sy2=0; int c=0;

  while(millis() - t0 < 5000){
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    float axl = calcAngleX();
    float ayl = calcAngleY();
    sx += axl; sx2 += axl*axl;
    sy += ayl; sy2 += ayl*ayl;
    c++;
    delay(20);
  }

  double varX = (sx2 - (sx*sx)/c)/c;
  double varY = (sy2 - (sy*sy)/c)/c;

  useAngleX = (varX >= varY);
  Serial.printf("Axis = %d\n", useAngleX);
}

/* ---------------- WEBSOCKET ---------------- */
void wsEvent(uint8_t n, WStype_t t, uint8_t *p, size_t l){
  if(t != WStype_TEXT) return;

  String cmd = String((char*)p).substring(0,l);
  cmd.trim();

  if(cmd=="START") balancingEnabled=true;
  else if(cmd=="STOP"){ balancingEnabled=false; flywheelMotor(0); }
  else if(cmd=="FWD") rearForward();
  else if(cmd=="BACK") rearBackward();
  else if(cmd=="STRAIGHT") steerCenter();
  else if(cmd=="LEFT") steerLeft();
  else if(cmd=="RIGHT") steerRight();
  else if(cmd.startsWith("FLY=")) flywheelMotor(cmd.substring(4).toInt());
  else if(cmd.startsWith("Kp=")) Kp = cmd.substring(3).toFloat();
  else if(cmd.startsWith("Kd=")) Kd = cmd.substring(3).toFloat();
  else if(cmd.startsWith("Kf=")) Kf = cmd.substring(3).toFloat();
}

/* ---------------- SETUP ---------------- */
void setup(){
  Serial.begin(115200);

  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(DRV_IN1, OUTPUT);
  pinMode(DRV_IN2, OUTPUT);
  rearStop();
  analogWrite(ENA_PIN, 0);

  Wire.begin(SDA_PIN, SCL_PIN);
  delay(200);

  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU OK" : "MPU FAIL");

  steeringServo.attach(SERVO_PIN);
  steerCenter();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_pass);

  server.on("/", [](){ server.send_P(200,"text/html",page); });
  server.begin();

  webSocket.begin();
  webSocket.onEvent(wsEvent);

  detectAxis();

  lastMicros = micros();
}

/* ---------------- LOOP ---------------- */
void loop(){
  server.handleClient();
  webSocket.loop();

  unsigned long now = micros();
  float dt = (now - lastMicros) / 1e6;
  if(dt <= 0) dt = 0.001;
  if(dt > 0.05) dt = 0.05;
  lastMicros = now;

  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

  float accAngle = useAngleX ? calcAngleX() : calcAngleY();
  gyroRate = gy / 131.0;

  angle = alpha*(angle + gyroRate*dt) + (1-alpha)*accAngle;

  float err = angle - targetAngle;
  float out = Kp*err + Kd*(-gyroRate) + Kf*flywheel_est;
  out = constrain(out, -230, 230);

  int pwm_final = 0;

  if(!balancingEnabled || fabs(angle) > ANGLE_CUTOFF){
    balancingEnabled = false;
    flywheelMotor(0);
  } else {
    pwm_final = (int)out;
    flywheelMotor(pwm_final);
  }

  static unsigned long ls=0;
  if(millis() - ls > 200){
    String s = "A:" + String(angle,2) + " PWM:" + String(pwm_final);
    webSocket.broadcastTXT(s);
    Serial.println(s);
    ls = millis();
  }
}
