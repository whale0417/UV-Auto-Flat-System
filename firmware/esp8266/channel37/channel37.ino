#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <ESP8266WiFi.h>
#include <WiFiServer.h>
#include <WiFiClient.h>

// WiFi配置
const char* ssid = "lab-112";
const char* password = "wei67890.";
const int serverPort = 8888;

// WiFi服务器
WiFiServer server(serverPort);
WiFiClient client;
bool hasWiFiClient = false;

// ADS1015配置 - 3个芯片，地址分别为GND(0x48), VDD(0x49), SDA(0x4A)
#define ADS1015_COUNT 3
#define ADS1015_ADDR_1 0x48  // ADDR接GND
#define ADS1015_ADDR_2 0x49  // ADDR接VDD
#define ADS1015_ADDR_3 0x4A  // ADDR接SDA

// ADS1015对象数组
Adafruit_ADS1015 ads[ADS1015_COUNT];
bool adsInitialized[ADS1015_COUNT] = {false, false, false};

// DG4051控制引脚 - 根据你的描述
const int pinA = 16;  // GPIO16 控制所有DG4051的A引脚
const int pinB = 14;  // GPIO14 控制所有DG4051的B引脚  
const int pinC = 12;  // GPIO12 控制所有DG4051的C引脚

// 系统配置
const int TOTAL_SENSORS = 96;
const int SENSOR_GROUPS = 8;
const int SENSORS_PER_GROUP = 12;

// 增益设置
adsGain_t currentGain = GAIN_ONE;  // 默认增益 ±4.096V

// 串口通信
String inputString = "";
boolean stringComplete = false;

// WiFi通信
String wifiInputString = "";
boolean wifiStringComplete = false;

// 自动读取控制
bool autoMode = false;
unsigned long lastReadTime = 0;
unsigned long autoInterval = 1000;

// 通信模式
enum CommMode {
  SERIAL_ONLY,
  WIFI_ONLY,
  DUAL_MODE
};
CommMode commMode = DUAL_MODE;  // 默认双模式

// MUX通道选择模式 - 控制所有DG4051选择相同通道
// 通道编码: C B A (C=GPIO12, B=GPIO14, A=GPIO16)
const uint8_t muxChannels[8][3] = {
  // {C, B, A}
  {LOW,  LOW,  LOW},   // 通道0: 000
  {LOW,  LOW,  HIGH},  // 通道1: 001
  {LOW,  HIGH, LOW},   // 通道2: 010
  {LOW,  HIGH, HIGH},  // 通道3: 011
  {HIGH, LOW,  LOW},   // 通道4: 100
  {HIGH, LOW,  HIGH},  // 通道5: 101
  {HIGH, HIGH, LOW},   // 通道6: 110
  {HIGH, HIGH, HIGH}   // 通道7: 111
};

void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println("\n=================================");
  Serial.println("ESP8266 96通道ADC系统启动中...");
  Serial.println("=================================");
  
  // 初始化DG4051控制引脚
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  pinMode(pinC, OUTPUT);
  
  // 设置初始通道为0
  setAllMuxChannels(0);
  
  Serial.println("DG4051控制引脚初始化完成");
  Serial.print("引脚配置: A=GPIO"); Serial.print(pinA);
  Serial.print(", B=GPIO"); Serial.print(pinB);
  Serial.print(", C=GPIO"); Serial.println(pinC);
  
  // 初始化WiFi热点模式
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  
  Serial.print("WiFi热点已启动: ");
  Serial.println(ssid);
  Serial.print("IP地址: ");
  Serial.println(WiFi.softAPIP());
  
  server.begin();
  Serial.print("TCP服务器已启动，端口: ");
  Serial.println(serverPort);
  
  // 初始化I2C - 使用默认引脚(D2=GPIO4, D1=GPIO5)
  Wire.begin();
  delay(10);
  Serial.println("I2C初始化完成 (SDA=GPIO4/D2, SCL=GPIO5/D1)");
  
  // 初始化所有ADS1015
  Serial.println("开始初始化ADS1015芯片...");
  
  // 首先扫描I2C总线
  Serial.println("扫描I2C设备...");
  scanI2CBus();
  
  bool allInitialized = true;
  
  // 初始化第一个ADS1015 (地址0x48)
  Serial.printf("初始化ADS#0 (Addr:0x%02X)... ", ADS1015_ADDR_1);
  if (!ads[0].begin(ADS1015_ADDR_1)) {
    Serial.println("失败!");
    adsInitialized[0] = false;
    allInitialized = false;
  } else {
    Serial.println("成功!");
    ads[0].setGain(currentGain);
    adsInitialized[0] = true;
  }
  
  // 初始化第二个ADS1015 (地址0x49)
  Serial.printf("初始化ADS#1 (Addr:0x%02X)... ", ADS1015_ADDR_2);
  if (!ads[1].begin(ADS1015_ADDR_2)) {
    Serial.println("失败!");
    adsInitialized[1] = false;
    allInitialized = false;
  } else {
    Serial.println("成功!");
    ads[1].setGain(currentGain);
    adsInitialized[1] = true;
  }
  
  // 初始化第三个ADS1015 (地址0x4A)
  Serial.printf("初始化ADS#2 (Addr:0x%02X)... ", ADS1015_ADDR_3);
  if (!ads[2].begin(ADS1015_ADDR_3)) {
    Serial.println("失败!");
    adsInitialized[2] = false;
    allInitialized = false;
  } else {
    Serial.println("成功!");
    ads[2].setGain(currentGain);
    adsInitialized[2] = true;
  }
  
  if (!allInitialized) {
    Serial.println("警告: 部分ADS1015初始化失败! 系统将继续运行");
  } else {
    Serial.println("所有ADS1015初始化完成");
  }
  
  Serial.println("增益设置完成: " + getGainString(currentGain));
  
  String initMsg = "96通道ADC系统初始化完成\n";
  initMsg += "系统架构: 12个DG4051 × 8通道 + 3个ADS1015\n";
  initMsg += "硬件连接:\n";
  initMsg += "  - DG4051控制: A=GPIO" + String(pinA) + ", B=GPIO" + String(pinB) + ", C=GPIO" + String(pinC) + "\n";
  initMsg += "  - ADS1015地址: 0x48, 0x49, 0x4A\n";
  initMsg += "  - I2C引脚: SDA=GPIO4/D2, SCL=GPIO5/D1\n";
  initMsg += "支持串口和WiFi双模通信\n";
  initMsg += "WiFi热点: " + String(ssid) + "\n";
  initMsg += "TCP端口: " + String(serverPort) + "\n";
  initMsg += "命令格式:\n";
  initMsg += "GAIN:0 -> ±6.144V\n";
  initMsg += "GAIN:1 -> ±4.096V\n";
  initMsg += "GAIN:2 -> ±2.048V\n";
  initMsg += "GAIN:4 -> ±1.024V\n";
  initMsg += "GAIN:8 -> ±0.512V\n";
  initMsg += "GAIN:16 -> ±0.256V\n";
  initMsg += "READ -> 读取所有96个通道\n";
  initMsg += "READSINGLE -> 直接读取ADS1015\n";
  initMsg += "TESTMUX:0-7 -> 测试单个MUX通道\n";
  initMsg += "AUTO:1000 -> 每1000ms自动读取\n";
  initMsg += "STOP -> 停止自动读取\n";
  initMsg += "SCAN -> 扫描I2C设备\n";
  initMsg += "STATUS -> 显示系统状态\n";
  
  sendToAll(initMsg);
  
  Serial.println("=================================");
  Serial.println("系统初始化完成，等待命令...");
  Serial.println("发送 'READ' 命令开始读取数据");
  Serial.println("发送 'TESTMUX:0' 测试MUX通道0");
  Serial.println("=================================");
}

// 扫描I2C总线
void scanI2CBus() {
  byte error, address;
  int nDevices = 0;
  
  Serial.println("扫描I2C总线...");
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.printf("  发现设备: 0x%02X", address);
      if (address == ADS1015_ADDR_1) Serial.print(" (ADS1015 #1)");
      else if (address == ADS1015_ADDR_2) Serial.print(" (ADS1015 #2)");
      else if (address == ADS1015_ADDR_3) Serial.print(" (ADS1015 #3)");
      Serial.println();
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.println("  未发现任何I2C设备!");
  } else {
    Serial.printf("  共发现%d个I2C设备\n", nDevices);
  }
}

// 设置所有多路复用器到指定通道
void setAllMuxChannels(uint8_t channel) {
  if (channel > 7) return;
  
  // 根据通道号设置A、B、C引脚电平
  digitalWrite(pinA, muxChannels[channel][2]);  // A
  digitalWrite(pinB, muxChannels[channel][1]);  // B
  digitalWrite(pinC, muxChannels[channel][0]);  // C
  
  // 稳定时间
  delayMicroseconds(100);
  
  // 调试信息
  Serial.printf("设置MUX通道 %d: C=%d, B=%d, A=%d\n", 
                channel, 
                muxChannels[channel][0],
                muxChannels[channel][1],
                muxChannels[channel][2]);
}

void loop() {
  // 处理串口事件
  if (Serial.available()) {
    serialEvent();
  }
  
  // 处理WiFi客户端连接
  handleWiFiClients();
  
  // 处理串口命令
  if (stringComplete) {
    processCommand(inputString, true);
    inputString = "";
    stringComplete = false;
  }
  
  // 处理WiFi命令
  if (wifiStringComplete) {
    processCommand(wifiInputString, false);
    wifiInputString = "";
    wifiStringComplete = false;
  }
  
  // 自动读取模式
  if (autoMode && (millis() - lastReadTime >= autoInterval)) {
    readAllChannels();
    lastReadTime = millis();
  }
  
  delay(1);
}

void handleWiFiClients() {
  // 检查新客户端连接
  if (server.hasClient()) {
    if (!client || !client.connected()) {
      if (client) client.stop();
      client = server.available();
      hasWiFiClient = true;
      sendToAll("WiFi客户端已连接: " + client.remoteIP().toString());
    } else {
      WiFiClient newClient = server.available();
      newClient.stop();
    }
  }
  
  // 处理WiFi客户端数据
  if (client && client.connected() && client.available()) {
    while (client.available()) {
      char inChar = (char)client.read();
      if (inChar == '\n') {
        wifiStringComplete = true;
      } else if (inChar != '\r') {
        wifiInputString += inChar;
      }
    }
  }
  
  // 检查客户端连接状态
  if (hasWiFiClient && (!client || !client.connected())) {
    hasWiFiClient = false;
    sendToAll("WiFi客户端已断开");
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else if (inChar != '\r') {
      inputString += inChar;
    }
  }
}

void processCommand(String command, bool fromSerial) {
  command.trim();
  String originalCommand = command;
  command.toUpperCase();
  
  String source = fromSerial ? "串口" : "WiFi";
  
  if (command.startsWith("GAIN:")) {
    int gainValue = command.substring(5).toInt();
    adsGain_t newGain;
    
    switch (gainValue) {
      case 0: newGain = GAIN_TWOTHIRDS; break;
      case 1: newGain = GAIN_ONE; break;
      case 2: newGain = GAIN_TWO; break;
      case 4: newGain = GAIN_FOUR; break;
      case 8: newGain = GAIN_EIGHT; break;
      case 16: newGain = GAIN_SIXTEEN; break;
      default:
        sendToAll("错误: 无效的增益值");
        return;
    }
    
    setGain(newGain);
    String response = "增益已设置为: " + getGainString(newGain) + " (来自" + source + ")";
    sendToAll(response);
    
  } else if (command == "READ") {
    readAllChannels();
    
  } else if (command == "READSINGLE") {
    readADS1015Direct();
    
  } else if (command.startsWith("TESTMUX:")) {
    int channel = command.substring(8).toInt();
    if (channel >= 0 && channel <= 7) {
      testSingleMuxChannel(channel);
    } else {
      sendToAll("错误: MUX通道必须在0-7之间");
    }
    
  } else if (command.startsWith("AUTO:")) {
    int interval = command.substring(5).toInt();
    if (interval >= 300) {
      autoInterval = interval;
      autoMode = true;
      String response = "自动读取模式已启用，间隔: " + String(interval) + "ms";
      sendToAll(response);
    } else {
      sendToAll("错误: 间隔时间必须≥300ms");
    }
    
  } else if (command == "STOP") {
    autoMode = false;
    sendToAll("自动读取模式已停止");
    
  } else if (command == "SCAN") {
    scanI2CBus();
    
  } else if (command == "STATUS") {
    String status = "系统状态:\n";
    status += "ADC通道数: 96 (12×8 MUX阵列)\n";
    status += "增益: " + getGainString(currentGain) + "\n";
    status += "自动模式: " + String(autoMode ? "开启" : "关闭") + "\n";
    if (autoMode) {
      status += "读取间隔: " + String(autoInterval) + "ms\n";
    }
    status += "通信模式: ";
    switch (commMode) {
      case SERIAL_ONLY: status += "仅串口"; break;
      case WIFI_ONLY: status += "仅WiFi"; break;
      case DUAL_MODE: status += "双模式"; break;
    }
    status += "\nDG4051控制引脚: A=GPIO" + String(pinA) + 
              ", B=GPIO" + String(pinB) + 
              ", C=GPIO" + String(pinC) + "\n";
    status += "ADS1015状态: ";
    for (int i = 0; i < ADS1015_COUNT; i++) {
      status += String(i) + ":" + (adsInitialized[i] ? "正常 " : "异常 ");
    }
    status += "\nWiFi客户端: " + String(hasWiFiClient ? "已连接" : "未连接");
    sendToAll(status);
    
  } else {
    sendToAll("错误: 未知命令 '" + originalCommand + "'");
  }
}

void sendToAll(String message) {
  switch (commMode) {
    case SERIAL_ONLY:
      Serial.println(message);
      break;
    case WIFI_ONLY:
      if (hasWiFiClient && client.connected()) {
        client.println(message);
      }
      break;
    case DUAL_MODE:
      Serial.println(message);
      if (hasWiFiClient && client.connected()) {
        client.println(message);
      }
      break;
  }
}

void setGain(adsGain_t gain) {
  currentGain = gain;
  for (int i = 0; i < ADS1015_COUNT; i++) {
    if (adsInitialized[i]) {
      ads[i].setGain(gain);
    }
  }
}

String getGainString(adsGain_t gain) {
  switch (gain) {
    case GAIN_TWOTHIRDS: return "±6.144V";
    case GAIN_ONE: return "±4.096V";
    case GAIN_TWO: return "±2.048V";
    case GAIN_FOUR: return "±1.024V";
    case GAIN_EIGHT: return "±0.512V";
    case GAIN_SIXTEEN: return "±0.256V";
    default: return "未知";
  }
}

// 测试单个MUX通道
void testSingleMuxChannel(uint8_t channel) {
  sendToAll("测试MUX通道: " + String(channel));
  
  setAllMuxChannels(channel);
  delay(10);
  
  sendToAll("MUX_TEST_START");
  
  bool anySuccess = false;
  
  // 读取所有ADS1015
  for (int adsNum = 0; adsNum < ADS1015_COUNT; adsNum++) {
    if (!adsInitialized[adsNum]) {
      for (int ch = 0; ch < 4; ch++) {
        String data = "ADS" + String(adsNum) + "_CH" + String(ch) + ":未初始化,0.000000V";
        sendToAll(data);
      }
      continue;
    }
    
    for (int ch = 0; ch < 4; ch++) {
      int16_t adcValue = ads[adsNum].readADC_SingleEnded(ch);
      float voltage = ads[adsNum].computeVolts(adcValue);
      
      if (abs(adcValue) > 10) {
        anySuccess = true;
      }
      
      String data = "ADS" + String(adsNum) + "_CH" + String(ch) + ":" + 
                    String(adcValue) + "," + String(voltage, 6) + "V";
      sendToAll(data);
      
      delay(5);
    }
  }
  
  if (!anySuccess) {
    sendToAll("警告: 未读取到有效数据");
  }
  
  sendToAll("MUX_TEST_END");
}

// 直接读取ADS1015（固定MUX通道为0）
void readADS1015Direct() {
  sendToAll("ADC_DIRECT_START");
  
  // 设置MUX到通道0
  setAllMuxChannels(0);
  delay(10);
  
  bool anySuccess = false;
  
  for (int adsNum = 0; adsNum < ADS1015_COUNT; adsNum++) {
    if (!adsInitialized[adsNum]) {
      for (int ch = 0; ch < 4; ch++) {
        String data = "ADS" + String(adsNum) + "_CH" + String(ch) + ":未初始化,0.000000V";
        sendToAll(data);
      }
      continue;
    }
    
    for (int ch = 0; ch < 4; ch++) {
      int16_t adcValue = ads[adsNum].readADC_SingleEnded(ch);
      float voltage = ads[adsNum].computeVolts(adcValue);
      
      if (abs(adcValue) > 10) {
        anySuccess = true;
      }
      
      String data = "ADS" + String(adsNum) + "_CH" + String(ch) + ":" + 
                    String(adcValue) + "," + String(voltage, 6) + "V";
      sendToAll(data);
      
      delay(5);
    }
  }
  
  if (!anySuccess) {
    sendToAll("警告: 未读取到有效数据");
  }
  
  sendToAll("ADC_DIRECT_END");
}

void readAllChannels() {
  sendToAll("ADC_DATA_START");
  
  unsigned long startTime = micros();
  bool anySuccess = false;
  
  // 循环8组传感器（8个MUX通道）
  for (int group = 0; group < SENSOR_GROUPS; group++) {
    // 设置所有MUX到当前组
    setAllMuxChannels(group);
    
    // 稳定时间
    delay(5);
    
    // 读取所有ADS1015数据（12个通道）
    int16_t adcValues[12];
    
    // ADS1015 #1 - 通道0-3
    if (adsInitialized[0]) {
      adcValues[0] = ads[0].readADC_SingleEnded(0);
      adcValues[1] = ads[0].readADC_SingleEnded(1);
      adcValues[2] = ads[0].readADC_SingleEnded(2);
      adcValues[3] = ads[0].readADC_SingleEnded(3);
      if (abs(adcValues[0]) > 10 || abs(adcValues[1]) > 10 || 
          abs(adcValues[2]) > 10 || abs(adcValues[3]) > 10) {
        anySuccess = true;
      }
    } else {
      adcValues[0] = adcValues[1] = adcValues[2] = adcValues[3] = 0;
    }
    
    // ADS1015 #2 - 通道4-7
    if (adsInitialized[1]) {
      adcValues[4] = ads[1].readADC_SingleEnded(0);
      adcValues[5] = ads[1].readADC_SingleEnded(1);
      adcValues[6] = ads[1].readADC_SingleEnded(2);
      adcValues[7] = ads[1].readADC_SingleEnded(3);
      if (abs(adcValues[4]) > 10 || abs(adcValues[5]) > 10 || 
          abs(adcValues[6]) > 10 || abs(adcValues[7]) > 10) {
        anySuccess = true;
      }
    } else {
      adcValues[4] = adcValues[5] = adcValues[6] = adcValues[7] = 0;
    }
    
    // ADS1015 #3 - 通道8-11
    if (adsInitialized[2]) {
      adcValues[8] = ads[2].readADC_SingleEnded(0);
      adcValues[9] = ads[2].readADC_SingleEnded(1);
      adcValues[10] = ads[2].readADC_SingleEnded(2);
      adcValues[11] = ads[2].readADC_SingleEnded(3);
      if (abs(adcValues[8]) > 10 || abs(adcValues[9]) > 10 || 
          abs(adcValues[10]) > 10 || abs(adcValues[11]) > 10) {
        anySuccess = true;
      }
    } else {
      adcValues[8] = adcValues[9] = adcValues[10] = adcValues[11] = 0;
    }
    
    // 发送当前组的12个传感器数据
    for (int i = 0; i < 12; i++) {
      sendSensorData(group * 12 + i, adcValues[i]);
    }
  }
  
  unsigned long endTime = micros();
  
  if (!anySuccess) {
    sendToAll("警告: 未读取到有效数据");
  }
  
  sendToAll("ADC_DATA_END");
  
  String timeInfo = "采集耗时: " + String(endTime - startTime) + "μs";
  sendToAll(timeInfo);
}

// 发送单个传感器数据
void sendSensorData(int channel, int16_t adcValue) {
  float voltage = 0.0;
  
  // 使用第一个已初始化的ADS1015计算电压
  for (int i = 0; i < ADS1015_COUNT; i++) {
    if (adsInitialized[i]) {
      voltage = ads[i].computeVolts(adcValue);
      break;
    }
  }
  
  String data = "CH" + String(channel) + ":" + 
                String(adcValue) + "," + 
                String(voltage, 6) + "V";
  
  sendToAll(data);
}