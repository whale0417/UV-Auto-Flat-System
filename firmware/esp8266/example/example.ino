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

// ADS1015配置
#define ADS1015_COUNT 3
#define ADS1015_ADDR_1 0x48
#define ADS1015_ADDR_2 0x49
#define ADS1015_ADDR_3 0x4A

Adafruit_ADS1015 ads[ADS1015_COUNT];
bool adsInitialized[ADS1015_COUNT] = {false, false, false};

// DG4051控制引脚
const int pinA = 16;  // GPIO16
const int pinB = 14;  // GPIO14
const int pinC = 12;  // GPIO12

// 系统配置
const int TOTAL_SENSORS = 96;
const int SENSOR_GROUPS = 8;
const int SENSORS_PER_GROUP = 12;

// 增益设置 - 使用±6.144V量程以适应4.1V信号
adsGain_t currentGain = GAIN_TWOTHIRDS;  // ±6.144V

// 读取参数
const int MUX_SETTLE_MS = 10;      // MUX稳定时间(ms)
const int SAMPLE_COUNT = 5;        // 采样次数
const int SAMPLE_DELAY_MS = 2;     // 采样间隔(ms)

// 校准值
float calibrationOffset = 0.0;
float perChannelOffset[TOTAL_SENSORS] = {0};

// 通信缓冲区
String inputString = "";
boolean stringComplete = false;
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
CommMode commMode = DUAL_MODE;

// MUX通道选择模式
const uint8_t muxChannels[8][3] = {
  {LOW,  LOW,  LOW},   // 通道0: C=0,B=0,A=0
  {HIGH, LOW,  LOW},   // 通道1: C=0,B=0,A=1
  {LOW,  HIGH, LOW},   // 通道2: C=0,B=1,A=0
  {HIGH, HIGH, LOW},   // 通道3: C=0,B=1,A=1
  {LOW,  LOW,  HIGH},  // 通道4: C=1,B=0,A=0
  {HIGH, LOW,  HIGH},  // 通道5: C=1,B=0,A=1
  {LOW,  HIGH, HIGH},  // 通道6: C=1,B=1,A=0
  {HIGH, HIGH, HIGH}   // 通道7: C=1,B=1,A=1
};

// 调试标志
bool debugMode = false;

// 函数声明
void setMuxChannel(uint8_t channel, bool initial = false);
float readChannel(int adsNum, int channel, bool medianFilter = true);
void sendToAll(String message);
void setGain(adsGain_t gain);
String getGainString(adsGain_t gain);
void testSingleChannel(int channel);
void calibrateAllChannels();
void calibrateSingleChannel(int channel);
void readAllChannels();
void readAllChannelsSlow();
void processCommand(String command, bool fromSerial);

// 设置MUX通道
void setMuxChannel(uint8_t channel, bool initial) {
  if (channel > 7) return;
  
  digitalWrite(pinA, muxChannels[channel][2]);
  digitalWrite(pinB, muxChannels[channel][1]);
  digitalWrite(pinC, muxChannels[channel][0]);
  
  if (debugMode || initial) {
    Serial.printf("MUX通道%d: C=%d, B=%d, A=%d\n", 
                  channel,
                  muxChannels[channel][0],
                  muxChannels[channel][1],
                  muxChannels[channel][2]);
  }
  
  if (!initial) {
    delay(MUX_SETTLE_MS);
  }
}

// 读取单个通道（多次采样取平均）
float readChannel(int adsNum, int channel, bool medianFilter) {
  if (!adsInitialized[adsNum]) return 0.0;
  
  int16_t readings[SAMPLE_COUNT];
  
  // 采集多个样本
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    readings[i] = ads[adsNum].readADC_SingleEnded(channel);
    delay(SAMPLE_DELAY_MS);
  }
  
  // 排序找中值（中值滤波）
  if (medianFilter) {
    for (int i = 0; i < SAMPLE_COUNT - 1; i++) {
      for (int j = i + 1; j < SAMPLE_COUNT; j++) {
        if (readings[i] > readings[j]) {
          int16_t temp = readings[i];
          readings[i] = readings[j];
          readings[j] = temp;
        }
      }
    }
  }
  
  // 计算平均值（去掉最高和最低值）
  int32_t sum = 0;
  int count = 0;
  for (int i = 1; i < SAMPLE_COUNT - 1; i++) { // 去掉第一个和最后一个
    sum += readings[i];
    count++;
  }
  
  if (count == 0) return 0.0;
  
  int16_t average = sum / count;
  return ads[adsNum].computeVolts(average) + calibrationOffset;
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // 增加启动延迟
  
  Serial.println("\n=================================");
  Serial.println("96通道传感器系统（抗干扰优化版）");
  Serial.println("=================================");
  
  // 初始化DG4051控制引脚
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  pinMode(pinC, OUTPUT);
  
  // 设置初始通道为0
  setMuxChannel(0, true);
  
  Serial.println("DG4051控制引脚初始化完成");
  
  // 初始化WiFi
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  
  Serial.print("WiFi热点: ");
  Serial.println(ssid);
  Serial.print("IP地址: ");
  Serial.println(WiFi.softAPIP());
  
  server.begin();
  Serial.print("TCP端口: ");
  Serial.println(serverPort);
  
  // 初始化I2C - 降低时钟频率以提高稳定性
  Wire.begin();
  Wire.setClock(50000);  // 降低到50kHz以提高抗干扰能力
  delay(200);
  
  Serial.println("I2C初始化完成（50kHz）");
  
  // 初始化ADS1015
  Serial.println("\n初始化ADS1015...");
  
  bool allOK = true;
  if (ads[0].begin(ADS1015_ADDR_1, &Wire)) {
    ads[0].setGain(currentGain);
    adsInitialized[0] = true;
    Serial.println("ADS1015 #1 (0x48): 成功");
  } else {
    Serial.println("ADS1015 #1 (0x48): 失败");
    allOK = false;
  }
  
  if (ads[1].begin(ADS1015_ADDR_2, &Wire)) {
    ads[1].setGain(currentGain);
    adsInitialized[1] = true;
    Serial.println("ADS1015 #2 (0x49): 成功");
  } else {
    Serial.println("ADS1015 #2 (0x49): 失败");
    allOK = false;
  }
  
  if (ads[2].begin(ADS1015_ADDR_3, &Wire)) {
    ads[2].setGain(currentGain);
    adsInitialized[2] = true;
    Serial.println("ADS1015 #3 (0x4A): 成功");
  } else {
    Serial.println("ADS1015 #3 (0x4A): 失败");
    allOK = false;
  }
  
  // 系统信息
  Serial.println("\n系统配置:");
  Serial.println("增益: " + getGainString(currentGain));
  Serial.println("MUX稳定时间: " + String(MUX_SETTLE_MS) + "ms");
  Serial.println("采样次数: " + String(SAMPLE_COUNT));
  Serial.println("采样间隔: " + String(SAMPLE_DELAY_MS) + "ms");
  
  String initMsg = "96通道传感器系统初始化完成\n";
  initMsg += "此版本针对多传感器干扰进行了优化\n";
  initMsg += "主要改进:\n";
  initMsg += "1. 降低I2C时钟频率至50kHz\n";
  initMsg += "2. 增加MUX稳定时间\n";
  initMsg += "3. 多次采样取平均\n";
  initMsg += "4. 支持单通道校准\n";
  initMsg += "当前增益：" + getGainString(currentGain) + "\n";
  initMsg += "命令格式:\n";
  initMsg += "READ - 读取所有通道\n";
  initMsg += "READSLOW - 慢速读取（更稳定）\n";
  initMsg += "TESTCH:0-95 - 测试单个通道\n";
  initMsg += "CALIBRATE - 校准所有通道\n";
  initMsg += "CALCH:0-95 - 校准单个通道\n";
  initMsg += "SETGAIN:0-16 - 设置增益\n";
  initMsg += "STATUS - 系统状态\n";
  initMsg += "HELP - 帮助信息\n";
  
  sendToAll(initMsg);
  
  Serial.println("\n系统准备就绪");
  Serial.println("=================================");
}

void loop() {
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
    readAllChannelsSlow();
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
      sendToAll("WiFi客户端已连接");
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
  
  if (command == "HELP") {
    sendToAll("可用命令:");
    sendToAll("  READ - 读取所有通道");
    sendToAll("  READSLOW - 慢速读取（更稳定）");
    sendToAll("  TESTCH:0-95 - 测试单个通道");
    sendToAll("  CALIBRATE - 校准所有通道");
    sendToAll("  CALCH:0-95 - 校准单个通道");
    sendToAll("  SETGAIN:0-16 - 设置增益");
    sendToAll("  STATUS - 系统状态");
    sendToAll("  DEBUG:ON/OFF - 调试模式");
    
  } else if (command == "READ") {
    readAllChannels();
    
  } else if (command == "READSLOW") {
    readAllChannelsSlow();
    
  } else if (command.startsWith("TESTCH:")) {
    int channel = command.substring(7).toInt();
    if (channel >= 0 && channel < TOTAL_SENSORS) {
      testSingleChannel(channel);
    } else {
      sendToAll("错误: 通道号必须在0-95之间");
    }
    
  } else if (command == "CALIBRATE") {
    calibrateAllChannels();
    
  } else if (command.startsWith("CALCH:")) {
    int channel = command.substring(6).toInt();
    if (channel >= 0 && channel < TOTAL_SENSORS) {
      calibrateSingleChannel(channel);
    } else {
      sendToAll("错误: 通道号必须在0-95之间");
    }
    
  } else if (command.startsWith("SETGAIN:")) {
    int gainValue = command.substring(8).toInt();
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
    sendToAll("增益已设置为: " + getGainString(newGain));
    
  } else if (command.startsWith("DEBUG:")) {
    String mode = command.substring(6);
    if (mode == "ON") {
      debugMode = true;
      sendToAll("调试模式已开启");
    } else if (mode == "OFF") {
      debugMode = false;
      sendToAll("调试模式已关闭");
    }
    
  } else if (command == "STATUS") {
    String status = "系统状态:\n";
    status += "ADS1015状态: ";
    for (int i = 0; i < ADS1015_COUNT; i++) {
      status += String(i) + ":" + (adsInitialized[i] ? "正常 " : "异常 ");
    }
    status += "\n增益: " + getGainString(currentGain);
    status += "\n全局校准偏移: " + String(calibrationOffset, 3) + "V";
    status += "\nMUX稳定时间: " + String(MUX_SETTLE_MS) + "ms";
    status += "\n采样次数: " + String(SAMPLE_COUNT);
    sendToAll(status);
    
  } else {
    sendToAll("未知命令: " + originalCommand);
    sendToAll("输入HELP查看可用命令");
  }
}

void sendToAll(String message) {
  Serial.println(message);
  if (hasWiFiClient && client.connected()) {
    client.println(message);
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

// 测试单个通道
void testSingleChannel(int channel) {
  int group = channel / 12;
  int adsNum = (channel % 12) / 4;
  int ch = channel % 4;
  
  setMuxChannel(group, false);
  
  float voltage = readChannel(adsNum, ch, true);
  
  sendToAll("通道" + String(channel) + "测试:");
  sendToAll("  MUX组: " + String(group));
  sendToAll("  ADS1015: #" + String(adsNum));
  sendToAll("  通道: " + String(ch));
  sendToAll("  电压: " + String(voltage, 6) + "V");
  sendToAll("  校准偏移: " + String(perChannelOffset[channel], 6) + "V");
}

// 校准所有通道
void calibrateAllChannels() {
  sendToAll("开始校准所有通道...");
  sendToAll("请确保所有传感器未安装");
  sendToAll("正在读取基准值...");
  
  float totalVoltage = 0;
  int count = 0;
  
  // 读取所有通道的基准值
  for (int group = 0; group < 8; group++) {
    setMuxChannel(group, false);
    
    for (int adsNum = 0; adsNum < 3; adsNum++) {
      if (!adsInitialized[adsNum]) continue;
      
      for (int ch = 0; ch < 4; ch++) {
        float voltage = readChannel(adsNum, ch, false);
        int channelIndex = group * 12 + adsNum * 4 + ch;
        perChannelOffset[channelIndex] = -voltage;
        totalVoltage += voltage;
        count++;
      }
    }
  }
  
  if (count > 0) {
    calibrationOffset = -totalVoltage / count;
    sendToAll("校准完成");
    sendToAll("平均基准电压: " + String(totalVoltage / count, 6) + "V");
    sendToAll("全局校准偏移: " + String(calibrationOffset, 6) + "V");
  } else {
    sendToAll("校准失败: 无有效数据");
  }
}

// 校准单个通道
void calibrateSingleChannel(int channel) {
  int group = channel / 12;
  int adsNum = (channel % 12) / 4;
  int ch = channel % 4;
  
  setMuxChannel(group, false);
  
  sendToAll("校准通道" + String(channel) + "...");
  sendToAll("请确保该通道传感器未安装");
  
  float voltage = readChannel(adsNum, ch, false);
  perChannelOffset[channel] = -voltage;
  
  sendToAll("通道" + String(channel) + "校准完成");
  sendToAll("基准电压: " + String(voltage, 6) + "V");
  sendToAll("通道偏移: " + String(perChannelOffset[channel], 6) + "V");
}

// 快速读取所有通道
void readAllChannels() {
  sendToAll("ADC_DATA_START");
  
  unsigned long startTime = micros();
  
  for (int group = 0; group < 8; group++) {
    setMuxChannel(group, false);
    
    for (int adsNum = 0; adsNum < 3; adsNum++) {
      if (!adsInitialized[adsNum]) continue;
      
      for (int ch = 0; ch < 4; ch++) {
        int16_t adcValue = ads[adsNum].readADC_SingleEnded(ch);
        float voltage = ads[adsNum].computeVolts(adcValue) + 
                       calibrationOffset + 
                       perChannelOffset[group * 12 + adsNum * 4 + ch];
        
        int channelIndex = group * 12 + adsNum * 4 + ch;
        String data = "CH" + String(channelIndex) + ":" + 
                      String(adcValue) + "," + String(voltage, 6) + "V";
        sendToAll(data);
      }
    }
  }
  
  unsigned long endTime = micros();
  
  sendToAll("ADC_DATA_END");
  sendToAll("采集耗时: " + String(endTime - startTime) + "μs");
}

// 慢速读取所有通道（更稳定）
void readAllChannelsSlow() {
  sendToAll("ADC_DATA_START_SLOW");
  
  unsigned long startTime = micros();
  
  for (int group = 0; group < 8; group++) {
    setMuxChannel(group, false);
    
    for (int adsNum = 0; adsNum < 3; adsNum++) {
      if (!adsInitialized[adsNum]) continue;
      
      for (int ch = 0; ch < 4; ch++) {
        float voltage = readChannel(adsNum, ch, true) + 
                       perChannelOffset[group * 12 + adsNum * 4 + ch];
        
        int channelIndex = group * 12 + adsNum * 4 + ch;
        String data = "CH" + String(channelIndex) + ":" + 
                      String(voltage, 6) + "V";
        sendToAll(data);
      }
    }
  }
  
  unsigned long endTime = micros();
  
  sendToAll("ADC_DATA_END");
  sendToAll("采集耗时: " + String(endTime - startTime) + "μs");
}