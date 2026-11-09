#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>
#include <MPU6500_WE.h>
#include <Adafruit_BMP3XX.h>
#include <ESP32Servo.h>

Servo leftServo;
Servo rightServo;

// ---------- WiFi ----------
const char* ssid = "Butterfly";
const char* password = "12345678";

// ---------- PWM ----------
const int leftPin = 13;
const int rightPin = 12;

const int minAngle = 40;
const int maxAngle = 140;
const int neutralAngle = 90;

// ---------- MPU6500 ----------
MPU6500_WE mpu = MPU6500_WE();
#define I2C_SDA 4
#define I2C_SCL 5

// 姿态角（融合后）
float pitch = 0.0, roll = 0.0;
bool mpuReady = false;

// 互补滤波参数
const float alpha = 0.995;          // 陀螺仪权重
unsigned long lastMPUTime = 0;     // 上次更新时间
bool filterInitialized = false;    // 滤波器是否已初始化

// 陀螺仪零偏
float gyroBiasX = 0.0, gyroBiasY = 0.0, gyroBiasZ = 0.0;

// ---------- BMP390 ----------
Adafruit_BMP3XX bmp;
bool bmpReady = false;
float groundPressure = 0.0;     // 地面基准气压 (hPa)
float filteredPressure = 0.0;   // 滤波后的气压
float relativeAltitude = 0.0;   // 滤波后的相对高度
float filteredAltitude = 0;
bool bmpCalibrated = false;     // 校准完成标志
const float bmpAlpha = 0.995;    // EMA 滤波系数

// ---------- 飞行控制 ----------

// ===== 高度保持 =====

bool altitudeHold = false;
float targetAltitude = 0.0;

float altIntegral = 0.0;
float lastAltError = 0.0;

const float altKp = 0.25;
const float altKi = 0.00;
const float altKd = 0.03;

// ===== 姿态控制 =====

const float rollKp = 2.0;
const float pitchKp = 0.03;

// ===== 失联保护 =====

unsigned long lastControlPacket = 0;
const unsigned long failsafeTimeout = 5000;
bool active = false;
float baseFreq = 2.0;
float baseAmp = 50.0;
float turnDiff = 0.0;
const float freqMin = 0.5, freqMax = 2.0;
const float ampMin = 10.0, ampMax = 50.0;

unsigned long lastWingUpdate = 0;
float phase = 0.0;

// ---------- Web 服务 ----------
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ---------- HTML 双摇杆界面 ----------
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">

<meta name="viewport"
content="width=device-width,
initial-scale=1.0,
user-scalable=no">

<title>蝴蝶控制</title>

<style>

body{
    margin:0;
    background:#1a1a2e;
    color:white;
    font-family:sans-serif;
    overflow-y:auto;
}

#container{
    display:flex;
    height:auto;
    width:100%;
    justify-content:space-around;
    align-items:flex-start;
    padding-top:10px;
}

.half{
    flex:1;
    display:flex;
    flex-direction:column;
    align-items:center;
}

.label{
    font-size:4vw;
    margin-bottom:10px;
    text-align:center;
}

.joystick-area{
    position:relative;

    width:42vw;
    height:42vw;

    min-width:150px;
    min-height:150px;

    max-width:190px;
    max-height:190px;

    background:rgba(255,255,255,0.12);

    border-radius:50%;
}

.stick{
    position:absolute;

    width:35%;
    height:35%;

    background:#f39c12;

    border-radius:50%;

    left:50%;
    top:50%;

    transform:translate(-50%,-50%);
}

#info{
    text-align:center;
    padding-bottom:20px;
}

button{
    font-size:20px;

    padding:10px 30px;

    background:#e74c3c;

    border:none;

    border-radius:10px;

    color:white;
}

button.on{
    background:#2ecc71;
}

#angles{
    font-size:15px;
}

#altitude{
    font-size:15px;
}

</style>

</head>

<body>

<div id="container">

<div class="half">

<div class="label">
左手<br>
频率/幅度
</div>

<div
class="joystick-area"
id="leftArea">

<div
class="stick"
id="leftStick">
</div>

</div>

</div>

<div class="half">

<div class="label">
右手<br>
升降/转向
</div>

<div
class="joystick-area"
id="rightArea">

<div
class="stick"
id="rightStick">
</div>

</div>

</div>

</div>

<div id="info">

<p>
连接:
<span
id="conn"
style="color:red;">
断开
</span>
</p>

<p>
频率:
<span id="freq">
2.0
</span>
Hz
|
幅度:
<span id="amp">
50
</span>
</p>

<p id="angles">
姿态:
等待数据...
</p>

<p id="altitude">
高度:
等待数据...
</p>

<button
id="toggleBtn"
onclick="toggle()">
启动
</button>

</div>

<script>

var ws;
var connected=false;

function initWS()
{
    ws =
    new WebSocket(
        'ws://' +
        location.host +
        '/ws'
    );

    ws.onopen=()=>
    {
        connected=true;

        document
        .getElementById("conn")
        .style.color="lime";

        document
        .getElementById("conn")
        .innerText="已连接";
    };

    ws.onclose=()=>
    {
        connected=false;

        document
        .getElementById("conn")
        .style.color="red";

        document
        .getElementById("conn")
        .innerText="断开";

        setTimeout(
            initWS,
            2000
        );
    };

    ws.onmessage=(e)=>
    {
        if(e.data.startsWith("ATT:"))
        {
            document
            .getElementById("angles")
            .innerText=
            "姿态: " +
            e.data.substring(4);
        }

        if(e.data.startsWith("ALT:"))
        {
            document
            .getElementById("altitude")
            .innerText=
            "高度: " +
            e.data.substring(4) +
            " m";
        }
    };
}

initWS();

let leftX=0;
let leftY=0;

let rightX=0;
let rightY=0;

let active=false;

function sendJoy()
{
    if(ws && connected)
    {
        ws.send(
            `JOY:${leftX.toFixed(2)},${leftY.toFixed(2)},${rightX.toFixed(2)},${rightY.toFixed(2)}`
        );
    }
}

function toggle()
{
    active=!active;

    document
    .getElementById("toggleBtn")
    .innerText=
    active?
    "停止":
    "启动";

    document
    .getElementById("toggleBtn")
    .className=
    active?
    "on":
    "";

    if(ws && connected)
    {
        ws.send(
        active?
        "START":
        "STOP"
        );
    }
}

function setupJoystick(
    areaId,
    stickId,
    setter
)
{
    const area =
    document.getElementById(areaId);

    const stick =
    document.getElementById(stickId);

    let activeId=null;

    function updateStick(
        cx,
        cy
    )
    {
        const rect =
        area.getBoundingClientRect();

        const radius =
        rect.width/2;

        let dx =
        cx -
        rect.left -
        radius;

        let dy =
        cy -
        rect.top -
        radius;

        let dist =
        Math.sqrt(
        dx*dx +
        dy*dy
        );

        if(dist>radius)
        {
            dx *= radius/dist;
            dy *= radius/dist;
        }

        stick.style.transform=
        `translate(
        calc(-50% + ${dx}px),
        calc(-50% + ${dy}px)
        )`;

        let nx=
        dx/radius;

        let ny=
        -dy/radius;

        setter(nx,ny);

        sendJoy();
    }

    area.addEventListener(
    "touchstart",
    e=>{
        e.preventDefault();

        for(let t of e.changedTouches)
        {
            if(activeId===null)
            {
                activeId =
                t.identifier;

                updateStick(
                t.clientX,
                t.clientY
                );
            }
        }
    });

    area.addEventListener(
    "touchmove",
    e=>{
        e.preventDefault();

        for(let t of e.changedTouches)
        {
            if(
            t.identifier
            ===
            activeId
            )
            {
                updateStick(
                t.clientX,
                t.clientY
                );
            }
        }
    });

    area.addEventListener(
    "touchend",
    e=>{
        for(let t of e.changedTouches)
        {
            if(
            t.identifier
            ===
            activeId
            )
            {
                activeId=null;

                stick.style.transform=
                "translate(-50%,-50%)";

                setter(0,0);

                sendJoy();
            }
        }
    });
}

setupJoystick(
"leftArea",
"leftStick",
(x,y)=>
{
    leftX=x;
    leftY=y;

    document
    .getElementById("freq")
    .innerText=
    (2+y*3).toFixed(1);

    document
    .getElementById("amp")
    .innerText=
    (50+x*40).toFixed(0);
});

setupJoystick(
"rightArea",
"rightStick",
(x,y)=>
{
    rightX=x;
    rightY=y;
});

</script>

</body>
</html>
)rawliteral";

// ---------- WebSocket 指令处理 ----------
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if(type == WS_EVT_CONNECT){
    Serial.println("客户端连接");
  } else if(type == WS_EVT_DISCONNECT){
    Serial.println("客户端断开");
  } else if(type == WS_EVT_DATA){
    String msg = "";
    for(size_t i=0; i<len; i++) msg += (char)data[i];
    msg.trim();
    Serial.println("收到指令: " + msg);

    if(msg == "START"){
    active = true;
    lastControlPacket=millis();
    altIntegral = 0;
    lastAltError = 0;

    if(bmpCalibrated)
    {
        targetAltitude = relativeAltitude;
        altitudeHold = true;
    }

    client->text("启动");
    }
    else if(msg == "STOP"){
    active = false;
    altitudeHold = false;

    altIntegral = 0;

    client->text("停止");
    }
    else if(msg.startsWith("JOY:")) {
      lastControlPacket = millis();
      String parts = msg.substring(4);
      float lx, ly, rx, ry;
      sscanf(parts.c_str(), "%f,%f,%f,%f", &lx, &ly, &rx, &ry);
      baseFreq = 2.0 + ly * 1.5;
      baseFreq = constrain(baseFreq, freqMin, freqMax);
      baseAmp = 50.0 + lx * 40.0;
      baseAmp = constrain(baseAmp, ampMin, ampMax);
      turnDiff = rx;
      float liftFreq = ry * 1.5;
      baseFreq += liftFreq;
      baseFreq = constrain(baseFreq, freqMin, freqMax);
    }
  }
}

// ---------- 陀螺仪零偏校准 ----------
void calibrateMPU6500() {
  Serial.println("正在校准 MPU6500 陀螺仪零偏，请保持静止...");
  const int samples = 500;
  float sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < samples; i++) {
    xyzFloat gyr = mpu.getGyrValues();
    sumX += gyr.x;
    sumY += gyr.y;
    sumZ += gyr.z;
    delay(2);
  }
  gyroBiasX = sumX / samples;
  gyroBiasY = sumY / samples;
  gyroBiasZ = sumZ / samples;
  Serial.printf("陀螺仪零偏: X=%.3f, Y=%.3f, Z=%.3f °/s\n", gyroBiasX, gyroBiasY, gyroBiasZ);
}

// ---------- 姿态更新（互补滤波 + 零偏校正）----------
void updateMPU() {
  if (!mpuReady) return;

  // 读取物理值（加速度 g，陀螺仪 °/s）
  xyzFloat acc = mpu.getGValues();
  xyzFloat gyr = mpu.getGyrValues();

  // 去除陀螺仪零偏
  float gyrX = gyr.x - gyroBiasX;
  float gyrY = gyr.y - gyroBiasY;
  float gyrZ = gyr.z - gyroBiasZ;  // Z轴暂未使用，但保留

  // 加速度计角度
  float accPitch = atan2(-acc.x, sqrt(acc.y*acc.y + acc.z*acc.z)) * 180.0 / PI;
  float accRoll  = atan2(acc.y, acc.z) * 180.0 / PI;

  // 时间差
  unsigned long now = micros();
  float dt = (lastMPUTime == 0) ? 0.0 : (now - lastMPUTime) / 1000000.0;
  lastMPUTime = now;

  if (!filterInitialized) {
    pitch = accPitch;
    roll = accRoll;
    filterInitialized = true;
  } else if (dt > 0 && dt < 0.1) {
    // 注意：MPU6500坐标系，假设X轴向前，Y轴向左
    // 俯仰角速度 = 绕Y轴旋转，横滚角速度 = 绕X轴旋转
    float pitchGyro = gyrY;
    float rollGyro  = gyrX;

    pitch = alpha * (pitch + pitchGyro * dt) + (1.0 - alpha) * accPitch;
    roll  = alpha * (roll  + rollGyro  * dt) + (1.0 - alpha) * accRoll;
  }

  // 串口输出 + WebSocket 发送
  static unsigned long lastAttPrint = 0;
  if (millis() - lastAttPrint > 200) {
    Serial.printf("姿态 -> Pitch:%.1f° Roll:%.1f° (GyrX:%.2f GyrY:%.2f)\n",
                  pitch, roll, gyrX, gyrY);
    lastAttPrint = millis();

    char attMsg[32];
    sprintf(attMsg, "ATT:%.1f,%.1f", pitch, roll);
    ws.textAll(attMsg);
  }
}

// ---------- BMP390 校准 ----------
void calibrateBMP() {
  if (!bmpReady) return;
  const int samples = 200;
  const int discard = 20;
  float sum = 0;
  int valid = 0;

  Serial.println("正在校准 BMP390，保持静止...");
  for (int i = 0; i < samples + discard; i++) {
    if (bmp.performReading()) {
      float p = bmp.pressure / 100.0; // Pa -> hPa
      if (i >= discard) {
        sum += p;
        valid++;
      }
    }
    delay(10);
  }
  if (valid > 0) {
    groundPressure = sum / valid;
    bmpCalibrated = true;
    filteredPressure = groundPressure;
    relativeAltitude = 0.0;
    Serial.printf("BMP390 校准完成！地面气压: %.2f hPa\n", groundPressure);
  } else {
    Serial.println("BMP390 校准失败！");
  }
}

// ---------- BMP390 高度更新（EMA 滤波）----------
void updateBMP() {
  if (!bmpReady || !bmpCalibrated) return;

  if (bmp.performReading()) {
    float rawPressure = bmp.pressure / 100.0; // Pa -> hPa

    filteredPressure = bmpAlpha * filteredPressure + (1.0 - bmpAlpha) * rawPressure;
    float rawAltitude =44330.0 *(1.0 -pow(filteredPressure /groundPressure,0.1903));

    filteredAltitude =0.95f* filteredAltitude+0.05f *rawAltitude;
    relativeAltitude = filteredAltitude;

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200) {
      Serial.printf("高度: %.2f m (原始:%.2f hPa 滤波:%.2f hPa)\n",
                    relativeAltitude, rawPressure, filteredPressure);
      lastPrint = millis();

      char altMsg[32];
      sprintf(altMsg, "ALT:%.2f", relativeAltitude);
      ws.textAll(altMsg);
    }
  }
}

//--------------------------------------------------
// 蝴蝶扑翼波形
//
// 上拍 70% 时间（慢）
// 下拍 30% 时间（快）
//
// 相位:
// 0° 最低点
// 180° 最高点
// 360° 最低点
//--------------------------------------------------
float butterflyWave(float phaseDeg){
  phaseDeg = fmod(phaseDeg, 360.0f);
  //----------------------------------
  // 慢上拍
  //----------------------------------
    if (phaseDeg < 234.0f){
    float t = phaseDeg / 234.0f;
    return -cos(t * PI);
    }
  //----------------------------------
  // 快下拍
  //----------------------------------
    else{
    float t =
    (phaseDeg - 234.0f)/ 126.0f;
    return cos(t * PI);
    }
}

// ---------- 翅膀运动控制 ----------
void updateWings()
{
    if(!active)
    {
        leftServo.write(neutralAngle);
        rightServo.write(neutralAngle);
        return;
    }

    static unsigned long lastUpdate = 0;
    unsigned long now = millis();

        if(lastUpdate == 0)
        {
        lastUpdate = now;
        return;
        }

        if(now - lastUpdate < 20)
        return;

    float dt =(now - lastUpdate) / 1000.0f;
    lastUpdate = now;

    //--------------------------------
    // 高度保持
    //--------------------------------

    float altitudeAdjust = 0;

    if(altitudeHold && bmpReady)
    {
        float error =
            targetAltitude -
            relativeAltitude;

        altIntegral += error * dt;

        altIntegral =constrain(altIntegral,-20,20);

        float derivative =(error - lastAltError) / dt;

        lastAltError = error;

        altitudeAdjust =
            altKp * error +
            altKi * altIntegral +
            altKd * derivative;

        altitudeAdjust =constrain(altitudeAdjust,-1.0f,1.0f);
    }

    //--------------------------------
    // Roll稳定
    //--------------------------------

    float rollFiltered = roll;
    if(fabs(rollFiltered) < 3.0f){
        rollFiltered = 0;
    }

    float rollComp =rollKp *(-rollFiltered / 90.0f);

    rollComp =constrain(rollComp,-0.8f,0.8f);

    //--------------------------------
    // Pitch配平
    //--------------------------------

    float pitchFiltered = pitch;

    if(fabs(pitchFiltered) < 5.0f){
        pitchFiltered = 0;
    }

    int center =neutralAngle + pitchFiltered * 0.15f;

    //--------------------------------
    // 转向融合
    //--------------------------------

    float combinedTurn =turnDiff +rollComp;
    combinedTurn =constrain(combinedTurn,-1.0f,1.0f);

    //--------------------------------
    // 转弯升力补偿
    //--------------------------------

    float turnLiftComp =fabs(combinedTurn) * 0.5f;

    //--------------------------------
    // 最终频率
    //--------------------------------

    float finalFreq =baseFreq +altitudeAdjust +turnLiftComp;

    finalFreq =constrain(finalFreq,freqMin,freqMax);

    //--------------------------------
    // 更新相位
    //--------------------------------

    phase +=finalFreq *360.0f *dt;

    while(phase >= 360.0f)
    {
        phase -= 360.0f;
    }

    //--------------------------------
    // 蝴蝶波形
    //--------------------------------

    float wave =butterflyWave(phase);

    //--------------------------------
    // 差动幅度
    //--------------------------------

    float leftAmp =baseAmp *(1.0f - combinedTurn * 0.6f);
    float rightAmp =baseAmp *(1.0f + combinedTurn * 0.6f);

    leftAmp =constrain(leftAmp,ampMin,ampMax);
    rightAmp =constrain(rightAmp,ampMin,ampMax);

    //--------------------------------
    // 输出角度
    //--------------------------------

    int leftAngle =center +wave * leftAmp;
    int rightAngle =center +wave * rightAmp;

    leftAngle =constrain(leftAngle, minAngle, maxAngle);

    rightAngle =constrain(rightAngle, minAngle, maxAngle);

    //--------------------------------
    // 2°死区
    //--------------------------------

    static int lastLeft = 90;
    static int lastRight = 90;

    if(abs(leftAngle - lastLeft) >= 2)
    {
        leftServo.write(leftAngle);
        lastLeft = leftAngle;
    }

    if(abs(rightAngle - lastRight) >= 2)
    {
        rightServo.write(180 - rightAngle);
        lastRight = rightAngle;
    }

    //--------------------------------
    // 调试
    //--------------------------------

    static unsigned long lastPrint = 0;

    if(millis() - lastPrint > 200)
    {
        Serial.printf(
            "H=%.2f "
            "T=%.2f "
            "P=%.1f "
            "R=%.1f "
            "F=%.2f "
            "PH=%.1f "
            "L=%d "
            "R=%d\n",

            relativeAltitude,
            targetAltitude,
            pitch,
            roll,
            finalFreq,
            phase,
            leftAngle,
            rightAngle);

        lastPrint = millis();
    }
}

// ---------- 初始化 ----------
void setup() {
  Serial.begin(115200);

  // PWM 初始化
  leftServo.attach(leftPin,500,2500);
  rightServo.attach(rightPin,500,2500);
  leftServo.write(neutralAngle);
  rightServo.write(neutralAngle);

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // MPU6500
  if (mpu.init()) {
    mpuReady = true;
    mpu.setAccRange(MPU6500_ACC_RANGE_8G);
    mpu.setGyrRange(MPU6500_GYRO_RANGE_500);
    Serial.println("MPU6500 初始化成功");
    calibrateMPU6500();   // 陀螺仪零偏校准
  } else {
    Serial.println("警告: MPU6500 未连接，姿态控制禁用");
    mpuReady = false;
  }

  // BMP390
  if (bmp.begin_I2C(0x77)) {   // 若找不到请尝试 0x76
    bmpReady = true;
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_32X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
    Serial.println("BMP390 硬件就绪");
    calibrateBMP();
  } else {
    Serial.println("BMP390 未找到");
    bmpReady = false;
  }

  // WiFi 热点
  WiFi.softAP(ssid, password);
  Serial.println("WiFi 热点已创建");
  Serial.print("IP 地址: ");
  Serial.println(WiFi.softAPIP());

  // Web 服务器
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  server.begin();
  Serial.println("HTTP 服务器已启动");
}

void loop() {
  ws.cleanupClients();
  updateMPU();
  if(active)
{
    if(millis() - lastControlPacket > failsafeTimeout)
    {
        active = false;
        altitudeHold = false;

        Serial.println("Failsafe Triggered");
    }
}
  updateBMP();
  updateWings();
  delay(5);
}