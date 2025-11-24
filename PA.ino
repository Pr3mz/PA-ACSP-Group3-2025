/*
   ESP32 MOTOCYCLE REACTION WHEEL BALANCER (ANALOGWRITE VERSION)
   --------------------------------------------------------------
   Features:
   - Auto axis detection (tilt L<->R for 5 seconds after boot)
   - Complementary filter (gyro + accel fusion)
   - PD control using gyro as derivative
   - Optional flywheel compensation term Kf * omega
   - Web UI + WebSocket + UDP control
   - analogWrite PWM so it compiles on your ESP32 core
*/

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "MPU6050.h"

MPU6050 mpu;

// ===== PIN DEFINITIONS =====
#define ENA_PIN 33
#define IN1_PIN 26
#define IN2_PIN 25

#define SDA_PIN 21
#define SCL_PIN 22

// ===== WIFI AP MODE =====
const char* ap_ssid = "ESP32-BIKE";
const char* ap_pass = "12345678";

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
WiFiUDP Udp;
const uint16_t UDP_PORT = 4210;

// ===== CONTROL PARAMETERS =====
float Kp = 7.0;      // proportional (for 960RPM, 6–10)
float Kd = 0.55;     // derivative (gyro based)
float Kf = 0.10;     // flywheel compensation

float targetAngle = 0.0;
bool motorsEnabled = false;

// ===== IMU FUSION VARIABLES =====
float angle = 0;        // fused angle
float gyroRate = 0;     // deg/sec
float lastAngle = 0;

unsigned long lastMicros = 0;

// complementary filter constant (0.95–0.99 recommended)
float alpha = 0.98;

// ===== AUTO AXIS DETECTION =====
bool useAngleX = true;  // true = atan2(ay,az) , false = atan2(ax,az)
bool axisDetected = false;

// ===== RAW IMU =====
int16_t ax, ay, az;
int16_t gx, gy, gz;

// ===== SAFETY =====
const float ANGLE_CUTOFF = 40.0;

// ===== MOTOR CONTROL =====
float lastPWM = 0;
float flywheel_speed = 0;

void motorDrive(int pwm) 
{
  pwm = constrain(pwm, -255, 255);

  // estimate flywheel speed
  flywheel_speed += (pwm - lastPWM) * 0.05;
  flywheel_speed *= 0.97;
  lastPWM = pwm;

  int duty = abs(pwm);

  if (pwm > 0) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else if (pwm < 0) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
  }

  analogWrite(ENA_PIN, duty);
}

// ===== ANGLE COMPUTATION =====
float calcAngleX() {
  return atan2((float)ay, (float)az) * 57.2958;
}
float calcAngleY() {
  return atan2((float)ax, (float)az) * 57.2958;
}

// ===== AUTO AXIS DETECTION =====
void detectAxis() 
{
  Serial.println("AUTO AXIS DETECTION: Tilt LEFT-RIGHT for 5 seconds...");
  unsigned long t0 = millis();
  unsigned long duration = 5000;

  double sumX=0, sumX2=0;
  double sumY=0, sumY2=0;
  int count=0;

  while (millis() - t0 < duration) {
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

    float axl = calcAngleX();
    float ayl = calcAngleY();

    sumX += axl; sumX2 += axl * axl;
    sumY += ayl; sumY2 += ayl * ayl;
    count++;

    delay(20);
  }

  double varX = (sumX2 - (sumX*sumX)/count) / count;
  double varY = (sumY2 - (sumY*sumY)/count) / count;

  Serial.print("varX="); Serial.println(varX,4);
  Serial.print("varY="); Serial.println(varY,4);

  useAngleX = (varX >= varY);

  Serial.print("USING AXIS: ");
  Serial.println(useAngleX ? "ANGLE X (atan2(ay,az))" : "ANGLE Y (atan2(ax,az))");

  axisDetected = true;
}

// ===== WEBSOCKET CALLBACK =====
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length)
{
  if(type == WStype_TEXT)
  {
    String cmd = String((char*)payload).substring(0,length);
    cmd.trim();

    if(cmd == "START"){
      motorsEnabled = true;
      webSocket.sendTXT(num,"STARTED");
    }
    else if(cmd == "STOP"){
      motorsEnabled = false;
      motorDrive(0);
      webSocket.sendTXT(num,"STOPPED");
    }
    else if(cmd == "ZERO"){
      targetAngle = 0;
    }
    else if(cmd.startsWith("Kp=")){
      Kp = cmd.substring(3).toFloat();
    }
    else if(cmd.startsWith("Kd=")){
      Kd = cmd.substring(3).toFloat();
    }
    else if(cmd.startsWith("Kf=")){
      Kf = cmd.substring(3).toFloat();
    }
  }
}

// ===== UDP COMMANDS =====
void handleUDP()
{
  int pk = Udp.parsePacket();
  if(!pk) return;

  char buf[100];
  int len = Udp.read(buf,99);
  buf[len] = 0;
  String cmd = String(buf);
  cmd.trim();

  if(cmd == "START") motorsEnabled = true;
  else if(cmd == "STOP"){ motorsEnabled = false; motorDrive(0); }
  else if(cmd == "ZERO") targetAngle = 0;
  else if(cmd.startsWith("Kp=")) Kp = cmd.substring(3).toFloat();
  else if(cmd.startsWith("Kd=")) Kd = cmd.substring(3).toFloat();
  else if(cmd.startsWith("Kf=")) Kf = cmd.substring(3).toFloat();
}

// ===== HTML PAGE =====
const char webpage[] PROGMEM = R"===(
<!DOCTYPE html><html><head>
<title>ESP32 Bike</title>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<style>
body{font-family:Arial;text-align:center;margin-top:20px}
button{padding:15px 20px;font-size:20px;margin:5px}
input{width:70px;font-size:20px}
</style></head>
<body>
<h2>ESP32 Balance Bike</h2>
<button onclick="ws.send('START')">START</button>
<button onclick="ws.send('STOP')">STOP</button>
<button onclick="ws.send('ZERO')">ZERO</button><br><br>
Kp:<input id='kp' value='7.0'>
Kd:<input id='kd' value='0.55'>
Kf:<input id='kf' value='0.1'>
<button onclick="setP()">SET</button><br><br>
<p>Status: <span id='s'>---</span></p>
<script>
var ws=new WebSocket("ws://"+location.hostname+":81/");
ws.onmessage=e=>{document.getElementById("s").innerText=e.data;}
function setP(){
 ws.send("Kp="+document.getElementById('kp').value);
 ws.send("Kd="+document.getElementById('kd').value);
 ws.send("Kf="+document.getElementById('kf').value);
}
</script>
</body></html>
)===";

// ======================================================
// SETUP
// ======================================================
void setup()
{
  Serial.begin(115200);
  delay(200);

  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  analogWrite(ENA_PIN, 0);

  // MPU
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(300);
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU OK" : "MPU FAIL");

  // Axis detection
  detectAxis();

  // WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_pass);
  server.on("/", [](){ server.send_P(200,"text/html", webpage); });
  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  Udp.begin(UDP_PORT);

  lastMicros = micros();
}

// ======================================================
// MAIN LOOP
// ======================================================
void loop()
{
  server.handleClient();
  webSocket.loop();
  handleUDP();

  unsigned long now = micros();
  float dt = (now - lastMicros)/1000000.0;
  if(dt <= 0) dt = 0.001;
  if(dt > 0.05) dt = 0.05;
  lastMicros = now;

  // IMU raw
  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

  // choose roll axis
  float accAngle = useAngleX ? calcAngleX() : calcAngleY();

  // gyro rate (deg/sec), choose correct axis - assuming Y rotation for roll
  gyroRate = (float)gy / 131.0;  // 131 LSB/°/s on default setting

  // Complementary filter fusion
  angle = alpha*(angle + gyroRate*dt) + (1.0-alpha)*accAngle;

  // PD control
  float error = angle - targetAngle;
  float out = Kp*error + Kd*(-gyroRate) + Kf*flywheel_speed;

  out = constrain(out, -230, 230);

  if(!motorsEnabled || abs(angle) > ANGLE_CUTOFF){
    motorDrive(0);
  }
  else{
    motorDrive((int)out);
  }

  // send status every 200ms
  static unsigned long lastSend=0;
  if(millis()-lastSend > 200){
    String msg = "A:"+String(angle,2)+" O:"+String(out,1);
    webSocket.broadcastTXT(msg);
    Serial.println(msg);
    lastSend = millis();
  }
}
