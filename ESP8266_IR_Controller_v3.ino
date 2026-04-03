/*
 * ESP8266/ESP-12F 红外遥控智能控制器 (v3 - 完整功能版)
 * 功能：
 * 1. 红外学习功能 - 接收并存储红外遥控编码，支持自定义命名
 * 2. 操作链管理 - 创建自定义操作序列（发送命令 + 等待）
 * 3. 定时任务 - 每天定时触发不同的操作链
 * 4. Web 配置界面 - 全功能图形化配置
 * 5. OTA 在线更新 - 支持无线固件升级
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266mDNS.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <IRutils.h>
#include <EEPROM.h>
#include <Ticker.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <LittleFS.h>

// ============== 配置区域 ==============
// WiFi 配置 - 请修改为你的网络信息
const char* ssid = "Nuance";
const char* password = "Nuance2018";

// NTP 服务器配置
const char* ntpServer = "pool.ntp.org";
const long gmtOffsetSec = 8 * 3600;
const int daylightOffsetSec = 0;

// 引脚定义
const uint16_t IR_RECEIVE_PIN = 14;  // D5 - 红外接收
const uint16_t IR_SEND_PIN = 4;      // D2 - 红外发射

// 存储配置
const int MAX_IR_CODES = 30;          // 最大存储30个红外编码
const int EEPROM_SIZE = 1024;
const char* CODES_FILE = "/codes.json";
const char* CHAINS_FILE = "/chains.json";
const char* SCHEDULES_FILE = "/schedules.json";
// ============== 配置结束 ==============

// 红外编码结构
struct IRCode {
  char name[32];
  uint64_t code;
  uint16_t bits;
  uint16_t protocol;
  bool valid;
};

// 操作步骤结构
struct Step {
  enum Type { SEND, WAIT } type;
  char data[64];
};

// 操作链结构
struct Chain {
  int id;
  char name[64];
  Step steps[50];
  int stepCount;
  bool valid;
};

// 定时任务结构
struct Schedule {
  int id;
  int chainId;
  int hour;
  int minute;
  bool enabled;
  bool valid;
  bool triggeredToday;
};

// 全局对象
IRrecv irrecv(IR_RECEIVE_PIN);
IRsend irsend(IR_SEND_PIN);
ESP8266WebServer server(80);
ESP8266HTTPUpdateServer httpUpdater;
Ticker timeTicker;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServer, gmtOffsetSec, 60000);

// 全局变量
IRCode irCodes[MAX_IR_CODES];
Chain chains[20];
Schedule schedules[20];
int chainIdCounter = 1;
int scheduleIdCounter = 1;

// 学习模式状态
bool learningMode = false;
int learningSlot = -1;
char learningName[32] = {0};
unsigned long learningStartTime = 0;
const unsigned long LEARNING_TIMEOUT = 30000;

// 定时任务状态
int lastCheckedHour = -1;
int lastCheckedMinute = -1;
unsigned long lastNtpSync = 0;
bool isRunningChain = false;

// ============== 函数声明 ==============
void loadIRCodes();
void saveIRCodes();
void loadChains();
void saveChains();
void loadSchedules();
void saveSchedules();
void initFilesystem();
void sendIRCodeByName(const char* name);
void sendIRCode(uint64_t code, uint16_t bits, uint16_t protocol);
void runChain(int chainId);
void checkScheduledTasks();
void enterLearningMode(const char* name);
void syncNTPTime();
String getCurrentTimeString();
String protocolToString(uint16_t protocol);

// Web 处理函数
void handleRoot();
void handleLearnStart();
void handleLearnStatus();
void handleCodesList();
void handleCodesDelete();
void handleChainsList();
void handleChainsCreate();
void handleChainsUpdate();
void handleChainsDelete();
void handleChainsRun();
void handleSchedulesList();
void handleSchedulesCreate();
void handleSchedulesUpdate();
void handleSchedulesDelete();
void handleStatus();
void handleNotFound();

// ============== 设置函数 ==============
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== ESP8266 IR Controller v3 ===");

  // 初始化 EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // 初始化文件系统
  initFilesystem();

  // 加载存储的数据
  loadIRCodes();
  loadChains();
  loadSchedules();

  // 初始化红外接收和发射
  irrecv.enableIRIn();
  irsend.begin();

  // 连接 WiFi
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // 启动 NTP 客户端
    timeClient.begin();
    syncNTPTime();
  } else {
    Serial.println("\nWiFi connection failed, starting AP mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("ESP8266_IR_Controller");
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
  }

  // 设置 HTTP 路由
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/learn/start", HTTP_GET, handleLearnStart);
  server.on("/learn/status", HTTP_GET, handleLearnStatus);
  server.on("/codes", HTTP_GET, handleCodesList);
  server.on("/codes/delete", HTTP_GET, handleCodesDelete);
  server.on("/chains", HTTP_GET, handleChainsList);
  server.on("/chains/create", HTTP_POST, handleChainsCreate);
  server.on("/chains/update", HTTP_POST, handleChainsUpdate);
  server.on("/chains/delete", HTTP_GET, handleChainsDelete);
  server.on("/chains/run", HTTP_GET, handleChainsRun);
  server.on("/schedules", HTTP_GET, handleSchedulesList);
  server.on("/schedules/create", HTTP_POST, handleSchedulesCreate);
  server.on("/schedules/update", HTTP_POST, handleSchedulesUpdate);
  server.on("/schedules/delete", HTTP_GET, handleSchedulesDelete);
  server.onNotFound(handleNotFound);

  // 设置 OTA 更新
  httpUpdater.setup(&server);
  Serial.println("OTA update service started (access /update)");

  // 设置 mDNS
  if (MDNS.begin("esp8266-ir")) {
    Serial.println("mDNS started: esp8266-ir.local");
  }

  server.begin();
  Serial.println("HTTP server started");

  // 启动定时任务检查（每分钟检查一次）
  timeTicker.attach(60.0, checkScheduledTasks);

  Serial.println("System initialization complete!");
  Serial.println("===========================================\n");
}

// ============== 主循环 ==============
void loop() {
  server.handleClient();
  MDNS.update();
  timeClient.update();

  // 每小时同步一次 NTP 时间
  if (millis() - lastNtpSync > 3600000) {
    syncNTPTime();
  }

  // 处理红外学习模式
  if (learningMode) {
    if (millis() - learningStartTime > LEARNING_TIMEOUT) {
      Serial.println("Learning mode timeout!");
      learningMode = false;
      learningSlot = -1;
    } else {
      decode_results results;
      if (irrecv.decode(&results)) {
        Serial.print("Received IR signal: ");
        Serial.print("Protocol: ");
        Serial.print(typeToString(results.decode_type));
        Serial.print("  Code: 0x");
        Serial.println(results.value, HEX);

        if (learningSlot >= 0) {
          irCodes[learningSlot].code = results.value;
          irCodes[learningSlot].bits = results.bits;
          irCodes[learningSlot].protocol = results.decode_type;
          irCodes[learningSlot].valid = true;
          strncpy(irCodes[learningSlot].name, learningName, 31);
          irCodes[learningSlot].name[31] = '\0';
          saveIRCodes();

          Serial.print("Code saved to slot ");
          Serial.print(learningSlot);
          Serial.print(" Name: ");
          Serial.println(irCodes[learningSlot].name);

          learningMode = false;
          learningSlot = -1;
        }

        irrecv.resume();
      }
    }
  }

  delay(10);
}

// ============== 文件系统操作 ==============
void initFilesystem() {
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed, trying to format...");
    LittleFS.format();
    if (LittleFS.begin()) {
      Serial.println("LittleFS formatted and mounted successfully");
    } else {
      Serial.println("LittleFS mount failed!");
    }
  } else {
    Serial.println("LittleFS mounted successfully");
  }
}

// ============== 红外编码存储操作 ==============
void loadIRCodes() {
  for (int i = 0; i < MAX_IR_CODES; i++) {
    irCodes[i].valid = false;
  }

  File file = LittleFS.open(CODES_FILE, "r");
  if (!file) {
    Serial.println("IR codes file not found, loading from EEPROM");
    int addr = 0;
    for (int i = 0; i < MAX_IR_CODES; i++) {
      EEPROM.get(addr, irCodes[i]);
      addr += sizeof(IRCode);
    }
    saveIRCodes();
    return;
  }

  DynamicJsonDocument doc(4096);
  DeserializationError error = deserializeJson(doc, file);
  file.close();

  if (error) {
    Serial.println("Failed to parse IR codes file");
    return;
  }

  JsonArray arr = doc.as<JsonArray>();
  int idx = 0;
  for (JsonObject obj : arr) {
    if (idx >= MAX_IR_CODES) break;
    strncpy(irCodes[idx].name, obj["name"] | "", 31);
    irCodes[idx].code = strtoull(obj["code"] | "0", NULL, 16);
    irCodes[idx].bits = obj["bits"] | 0;
    irCodes[idx].protocol = obj["protocol"] | 0;
    irCodes[idx].valid = obj["valid"] | false;
    idx++;
  }

  Serial.print("Loaded ");
  Serial.print(idx);
  Serial.println(" IR codes");
}

void saveIRCodes() {
  DynamicJsonDocument doc(4096);
  JsonArray arr = doc.to<JsonArray>();

  for (int i = 0; i < MAX_IR_CODES; i++) {
    if (irCodes[i].valid) {
      JsonObject obj = arr.createNestedObject();
      obj["name"] = irCodes[i].name;
      char codeStr[32];
      sprintf(codeStr, "0x%llx", irCodes[i].code);
      obj["code"] = codeStr;
      obj["bits"] = irCodes[i].bits;
      obj["protocol"] = irCodes[i].protocol;
      obj["valid"] = true;
    }
  }

  File file = LittleFS.open(CODES_FILE, "w");
  if (!file) {
    Serial.println("Failed to write IR codes file");
    return;
  }

  serializeJson(doc, file);
  file.close();
  Serial.println("IR codes saved");
}

// ============== 操作链存储操作 ==============
void loadChains() {
  for (int i = 0; i < 20; i++) {
    chains[i].valid = false;
  }

  File file = LittleFS.open(CHAINS_FILE, "r");
  if (!file) {
    Serial.println("Chains file not found");
    return;
  }

  DynamicJsonDocument doc(8192);
  DeserializationError error = deserializeJson(doc, file);
  file.close();

  if (error) {
    Serial.println("Failed to parse chains file");
    return;
  }

  JsonArray arr = doc.as<JsonArray>();
  int idx = 0;
  for (JsonObject obj : arr) {
    if (idx >= 20) break;
    chains[idx].id = obj["id"] | 0;
    strncpy(chains[idx].name, obj["name"] | "", 63);
    chains[idx].stepCount = 0;

    if (obj.containsKey("steps")) {
      JsonArray stepsArr = obj["steps"];
      for (JsonObject stepObj : stepsArr) {
        if (chains[idx].stepCount >= 50) break;
        const char* typeStr = stepObj["type"] | "send";
        if (strcmp(typeStr, "wait") == 0) {
          chains[idx].steps[chains[idx].stepCount].type = Step::WAIT;
        } else {
          chains[idx].steps[chains[idx].stepCount].type = Step::SEND;
        }
        strncpy(chains[idx].steps[chains[idx].stepCount].data, stepObj["data"] | "", 63);
        chains[idx].stepCount++;
      }
    }

    chains[idx].valid = true;
    if (chains[idx].id >= chainIdCounter) {
      chainIdCounter = chains[idx].id + 1;
    }
    idx++;
  }

  Serial.print("Loaded ");
  Serial.print(idx);
  Serial.println(" chains");
}

void saveChains() {
  DynamicJsonDocument doc(8192);
  JsonArray arr = doc.to<JsonArray>();

  for (int i = 0; i < 20; i++) {
    if (chains[i].valid) {
      JsonObject obj = arr.createNestedObject();
      obj["id"] = chains[i].id;
      obj["name"] = chains[i].name;
      JsonArray stepsArr = obj.createNestedArray("steps");
      for (int j = 0; j < chains[i].stepCount; j++) {
        JsonObject stepObj = stepsArr.createNestedObject();
        stepObj["type"] = (chains[i].steps[j].type == Step::SEND) ? "send" : "wait";
        stepObj["data"] = chains[i].steps[j].data;
      }
    }
  }

  File file = LittleFS.open(CHAINS_FILE, "w");
  if (!file) {
    Serial.println("Failed to write chains file");
    return;
  }

  serializeJson(doc, file);
  file.close();
  Serial.println("Chains saved");
}

// ============== 定时任务存储操作 ==============
void loadSchedules() {
  for (int i = 0; i < 20; i++) {
    schedules[i].valid = false;
    schedules[i].triggeredToday = false;
  }

  File file = LittleFS.open(SCHEDULES_FILE, "r");
  if (!file) {
    Serial.println("Schedules file not found");
    return;
  }

  DynamicJsonDocument doc(4096);
  DeserializationError error = deserializeJson(doc, file);
  file.close();

  if (error) {
    Serial.println("Failed to parse schedules file");
    return;
  }

  JsonArray arr = doc.as<JsonArray>();
  int idx = 0;
  for (JsonObject obj : arr) {
    if (idx >= 20) break;
    schedules[idx].id = obj["id"] | 0;
    schedules[idx].chainId = obj["chainId"] | 0;
    schedules[idx].hour = obj["hour"] | 0;
    schedules[idx].minute = obj["minute"] | 0;
    schedules[idx].enabled = obj["enabled"] | true;
    schedules[idx].valid = true;
    schedules[idx].triggeredToday = false;
    if (schedules[idx].id >= scheduleIdCounter) {
      scheduleIdCounter = schedules[idx].id + 1;
    }
    idx++;
  }

  Serial.print("Loaded ");
  Serial.print(idx);
  Serial.println(" schedules");
}

void saveSchedules() {
  DynamicJsonDocument doc(4096);
  JsonArray arr = doc.to<JsonArray>();

  for (int i = 0; i < 20; i++) {
    if (schedules[i].valid) {
      JsonObject obj = arr.createNestedObject();
      obj["id"] = schedules[i].id;
      obj["chainId"] = schedules[i].chainId;
      obj["hour"] = schedules[i].hour;
      obj["minute"] = schedules[i].minute;
      obj["enabled"] = schedules[i].enabled;
    }
  }

  File file = LittleFS.open(SCHEDULES_FILE, "w");
  if (!file) {
    Serial.println("Failed to write schedules file");
    return;
  }

  serializeJson(doc, file);
  file.close();
  Serial.println("Schedules saved");
}

// ============== 红外操作函数 ==============
void sendIRCodeByName(const char* name) {
  for (int i = 0; i < MAX_IR_CODES; i++) {
    if (irCodes[i].valid && strcmp(irCodes[i].name, name) == 0) {
      sendIRCode(irCodes[i].code, irCodes[i].bits, irCodes[i].protocol);
      Serial.print("Send IR command: ");
      Serial.print(name);
      Serial.print(" Code: 0x");
      Serial.println(irCodes[i].code, HEX);
      return;
    }
  }
  Serial.print("Command not found: ");
  Serial.println(name);
}

void sendIRCode(uint64_t code, uint16_t bits, uint16_t protocol) {
  if (protocol == NEC) {
    irsend.sendNEC(code, bits);
  } else {
    irsend.sendNEC(code, bits);
  }
  delay(100);
}

void enterLearningMode(const char* name) {
  int slot = -1;
  for (int i = 0; i < MAX_IR_CODES; i++) {
    if (irCodes[i].valid && strcmp(irCodes[i].name, name) == 0) {
      slot = i;
      break;
    }
  }
  if (slot == -1) {
    for (int i = 0; i < MAX_IR_CODES; i++) {
      if (!irCodes[i].valid) {
        slot = i;
        break;
      }
    }
  }
  if (slot == -1) {
    Serial.println("Storage full");
    return;
  }

  strncpy(learningName, name, 31);
  learningName[31] = '\0';
  learningMode = true;
  learningSlot = slot;
  learningStartTime = millis();
  Serial.print("Enter learning mode, slot: ");
  Serial.println(slot);
}

// ============== 操作链执行 ==============
void runChain(int chainId) {
  if (isRunningChain) {
    Serial.println("Another chain is already running");
    return;
  }

  int chainIdx = -1;
  for (int i = 0; i < 20; i++) {
    if (chains[i].valid && chains[i].id == chainId) {
      chainIdx = i;
      break;
    }
  }

  if (chainIdx == -1) {
    Serial.print("Chain not found, ID: ");
    Serial.println(chainId);
    return;
  }

  isRunningChain = true;
  Serial.println("========================================");
  Serial.print("Start executing chain: ");
  Serial.println(chains[chainIdx].name);
  Serial.println("========================================");

  for (int i = 0; i < chains[chainIdx].stepCount; i++) {
    Step* step = &chains[chainIdx].steps[i];
    if (step->type == Step::WAIT) {
      int waitMs = atoi(step->data);
      Serial.print("[Wait] ");
      Serial.print(waitMs);
      Serial.println(" ms");
      delay(waitMs);
    } else {
      Serial.print("[Send] ");
      Serial.println(step->data);
      sendIRCodeByName(step->data);
      delay(500);
    }
  }

  Serial.println("========================================");
  Serial.println("Chain execution complete!");
  Serial.println("========================================");
  isRunningChain = false;
}

// ============== 定时任务 ==============
void syncNTPTime() {
  Serial.println("Syncing NTP time...");
  timeClient.update();
  Serial.print("Current time: ");
  Serial.println(timeClient.getFormattedTime());
  lastNtpSync = millis();
}

String getCurrentTimeString() {
  return timeClient.getFormattedTime();
}

void checkScheduledTasks() {
  if (!timeClient.isTimeSet()) {
    Serial.println("Waiting for time sync...");
    return;
  }

  int currentHour = timeClient.getHours();
  int currentMinute = timeClient.getMinutes();

  if (currentMinute == lastCheckedMinute) {
    return;
  }
  lastCheckedMinute = currentMinute;

  if (currentHour == 0 && currentMinute == 0) {
    for (int i = 0; i < 20; i++) {
      schedules[i].triggeredToday = false;
    }
    Serial.println("New day, reset trigger status");
  }

  Serial.print("Current time: ");
  Serial.print(currentHour);
  Serial.print(":");
  Serial.println(currentMinute);

  for (int i = 0; i < 20; i++) {
    if (schedules[i].valid && schedules[i].enabled && !schedules[i].triggeredToday) {
      if (schedules[i].hour == currentHour && schedules[i].minute == currentMinute) {
        Serial.print("Trigger schedule ID: ");
        Serial.println(schedules[i].id);
        schedules[i].triggeredToday = true;
        runChain(schedules[i].chainId);
      }
    }
  }
}

String protocolToString(uint16_t protocol) {
  return typeToString((decode_type_t)protocol);
}

// ============== Web 处理函数 ==============

void handleRoot() {
  String html = R"(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ESP8266 IR Controller v3</title>
  <style>
    * { box-sizing: border-box; }
    body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif; margin: 0; padding: 0; background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%); min-height: 100vh; color: #eee; }
    .container { max-width: 900px; margin: 0 auto; padding: 20px; }
    h1 { text-align: center; margin-bottom: 10px; font-size: 28px; }
    .subtitle { text-align: center; color: #888; margin-bottom: 30px; }
    .tabs { display: flex; gap: 5px; margin-bottom: 20px; flex-wrap: wrap; }
    .tab { flex: 1; min-width: 100px; padding: 12px 16px; background: #2a2a4a; border: none; color: #aaa; cursor: pointer; border-radius: 8px; font-size: 14px; transition: all 0.3s; }
    .tab:hover { background: #3a3a5a; }
    .tab.active { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; font-weight: bold; }
    .card { background: #2a2a4a; border-radius: 12px; padding: 20px; margin-bottom: 20px; display: none; }
    .card.active { display: block; }
    h3 { margin-top: 0; color: #667eea; border-bottom: 1px solid #444; padding-bottom: 10px; }
    .btn { padding: 10px 20px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; border: none; border-radius: 8px; cursor: pointer; font-size: 14px; transition: transform 0.2s; }
    .btn:hover { transform: translateY(-2px); }
    .btn-danger { background: linear-gradient(135deg, #f5576c 0%, #f093fb 100%); }
    .btn-success { background: linear-gradient(135deg, #11998e 0%, #38ef7d 100%); }
    .btn-warning { background: linear-gradient(135deg, #f093fb 0%, #f5576c 100%); }
    input, select { padding: 10px; border: 2px solid #444; border-radius: 8px; background: #1a1a2e; color: white; font-size: 14px; width: 100%; }
    input:focus, select:focus { outline: none; border-color: #667eea; }
    .form-group { margin-bottom: 15px; }
    label { display: block; margin-bottom: 5px; color: #aaa; font-size: 14px; }
    .row { display: flex; gap: 10px; flex-wrap: wrap; }
    .col { flex: 1; min-width: 150px; }
    .status-bar { background: #1a1a2e; padding: 15px; border-radius: 8px; margin-bottom: 20px; font-family: monospace; font-size: 13px; }
    .status-item { display: flex; justify-content: space-between; padding: 5px 0; }
    .list-item { background: #1a1a2e; padding: 12px; border-radius: 8px; margin-bottom: 10px; display: flex; justify-content: space-between; align-items: center; }
    .list-item .name { font-weight: bold; }
    .list-item .meta { color: #888; font-size: 12px; margin-top: 4px; }
    .list-item .actions { display: flex; gap: 5px; }
    .btn-sm { padding: 6px 12px; font-size: 12px; }
    .step-item { background: #1a1a2e; padding: 10px; border-radius: 6px; margin-bottom: 8px; display: flex; gap: 10px; align-items: center; }
    .step-item .handle { cursor: grab; color: #666; }
    .step-item .content { flex: 1; }
    .step-badge { display: inline-block; padding: 2px 8px; border-radius: 10px; font-size: 11px; margin-right: 8px; }
    .badge-send { background: #667eea; }
    .badge-wait { background: #f5576c; }
    .modal { display: none; position: fixed; top: 0; left: 0; right: 0; bottom: 0; background: rgba(0,0,0,0.7); z-index: 1000; align-items: center; justify-content: center; }
    .modal.active { display: flex; }
    .modal-content { background: #2a2a4a; padding: 25px; border-radius: 12px; max-width: 500px; width: 90%; max-height: 80vh; overflow-y: auto; }
    .modal-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 20px; }
    .modal-header h3 { border: none; padding: 0; margin: 0; }
    .close-btn { background: none; border: none; color: #888; font-size: 24px; cursor: pointer; }
    .learning-indicator { padding: 15px; border-radius: 8px; text-align: center; display: none; }
    .learning-indicator.active { display: block; }
    .learning-indicator.waiting { background: #2a1a1a; border: 2px dashed #f5576c; }
    .learning-indicator.success { background: #1a2a1a; border: 2px solid #38ef7d; }
    .pulse { animation: pulse 1.5s infinite; }
    @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.5; } }
    .empty-state { text-align: center; padding: 40px; color: #666; }
    .toggle { display: inline-flex; align-items: center; cursor: pointer; }
    .toggle input { display: none; }
    .toggle-slider { width: 44px; height: 24px; background: #444; border-radius: 12px; position: relative; transition: background 0.3s; margin-right: 10px; }
    .toggle-slider::after { content: ''; position: absolute; width: 20px; height: 20px; background: white; border-radius: 50%; top: 2px; left: 2px; transition: transform 0.3s; }
    .toggle input:checked + .toggle-slider { background: #38ef7d; }
    .toggle input:checked + .toggle-slider::after { transform: translateX(20px); }
  </style>
</head>
<body>
  <div class="container">
    <h1>ESP8266 IR Controller v3</h1>
    <p class="subtitle">Smart IR Control | Chains | Schedules</p>

    <div class="status-bar" id="statusBar">
      <div class="status-item"><span>WiFi</span><span id="wifiStatus">Connecting...</span></div>
      <div class="status-item"><span>IP</span><span id="ipStatus">--</span></div>
      <div class="status-item"><span>Time</span><span id="timeStatus">--</span></div>
      <div class="status-item"><span>Codes</span><span id="codesCount">0</span></div>
      <div class="status-item" style="margin-top:10px;padding-top:10px;border-top:1px solid #444;">
        <span style="color:#888;">Firmware Update:</span>
        <a href="/update" style="color:#667eea;text-decoration:none;" target="_blank">Go to OTA update page</a>
      </div>
    </div>

    <div class="tabs">
      <button class="tab active" data-tab="learn">Learn IR</button>
      <button class="tab" data-tab="codes">Codes</button>
      <button class="tab" data-tab="chains">Chains</button>
      <button class="tab" data-tab="schedules">Schedules</button>
    </div>

    <div class="card active" id="tab-learn">
      <h3>Learn New Button</h3>
      <div class="form-group">
        <label>Button name (e.g. power, vol+, channel_up)</label>
        <div class="row">
          <div class="col" style="flex: 3;">
            <input type="text" id="learnName" placeholder="Enter button name" maxlength="30">
          </div>
          <div class="col">
            <button class="btn" id="learnBtn" onclick="startLearning()">Start Learning</button>
          </div>
        </div>
      </div>

      <div class="learning-indicator waiting" id="learningWaiting">
        <div class="pulse"><strong>Waiting for IR signal...</strong></div>
        <p style="margin-top:10px;color:#888;">Press remote button within 30 seconds</p>
      </div>

      <div class="learning-indicator success" id="learningSuccess">
        <strong>Learning successful!</strong>
      </div>

      <div style="margin-top:20px;">
        <h4>Tips</h4>
        <ul style="color:#888;font-size:14px;line-height:1.8;">
          <li>Point remote at ESP8266 IR receiver</li>
          <li>Recommended distance: 5-30 cm</li>
          <li>Same name will overwrite existing code</li>
        </ul>
      </div>
    </div>

    <div class="card" id="tab-codes">
      <h3>Learned Codes</h3>
      <div id="codesList"></div>
    </div>

    <div class="card" id="tab-chains">
      <h3>Operation Chains</h3>
      <div style="margin-bottom:15px;">
        <button class="btn btn-success" onclick="openChainModal()">+ New Chain</button>
      </div>
      <div id="chainsList"></div>
    </div>

    <div class="card" id="tab-schedules">
      <h3>Scheduled Tasks</h3>
      <div style="margin-bottom:15px;">
        <button class="btn btn-success" onclick="openScheduleModal()">+ New Schedule</button>
      </div>
      <div id="schedulesList"></div>
    </div>
  </div>

  <div class="modal" id="chainModal">
    <div class="modal-content">
      <div class="modal-header">
        <h3 id="chainModalTitle">New Chain</h3>
        <button class="close-btn" onclick="closeChainModal()">&times;</button>
      </div>
      <div class="form-group">
        <label>Chain Name</label>
        <input type="text" id="chainName" placeholder="e.g. Morning Power On">
      </div>
      <div style="margin-bottom:15px;">
        <label>Steps</label>
        <div id="stepsList"></div>
        <div class="row" style="margin-top:10px;">
          <div class="col"><select id="stepType"><option value="send">Send Command</option><option value="wait">Wait</option></select></div>
          <div class="col" id="stepSendCol"><select id="stepSendCode"></select></div>
          <div class="col" id="stepWaitCol" style="display:none;"><input type="number" id="stepWaitMs" placeholder="milliseconds" value="1000"></div>
          <div class="col" style="flex:none;"><button class="btn btn-sm" onclick="addStep()">+ Add</button></div>
        </div>
      </div>
      <div style="text-align:right;">
        <button class="btn" onclick="saveChain()">Save</button>
      </div>
    </div>
  </div>

  <div class="modal" id="scheduleModal">
    <div class="modal-content">
      <div class="modal-header">
        <h3 id="scheduleModalTitle">New Schedule</h3>
        <button class="close-btn" onclick="closeScheduleModal()">&times;</button>
      </div>
      <div class="form-group">
        <label>Trigger Time</label>
        <div class="row">
          <div class="col"><input type="number" id="scheduleHour" placeholder="hour" min="0" max="23"></div>
          <div class="col"><input type="number" id="scheduleMinute" placeholder="minute" min="0" max="59"></div>
        </div>
      </div>
      <div class="form-group">
        <label>Execute Chain</label>
        <select id="scheduleChain"></select>
      </div>
      <div style="text-align:right;">
        <button class="btn" onclick="saveSchedule()">Save</button>
      </div>
    </div>
  </div>

  <script>
    let currentEditChainId = null;
    let currentEditScheduleId = null;
    let tempSteps = [];

    function updateStatus() {
      fetch('/status')
        .then(r => r.json())
        .then(data => {
          document.getElementById('wifiStatus').textContent = data.wifi;
          document.getElementById('ipStatus').textContent = data.ip;
          document.getElementById('timeStatus').textContent = data.time;
          document.getElementById('codesCount').textContent = data.codes;
        });
    }

    document.querySelectorAll('.tab').forEach(tab => {
      tab.addEventListener('click', () => {
        document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
        document.querySelectorAll('.card').forEach(c => c.classList.remove('active'));
        tab.classList.add('active');
        document.getElementById('tab-' + tab.dataset.tab).classList.add('active');
        if (tab.dataset.tab === 'codes') loadCodes();
        if (tab.dataset.tab === 'chains') loadChains();
        if (tab.dataset.tab === 'schedules') loadSchedules();
      });
    });

    function startLearning() {
      const name = document.getElementById('learnName').value.trim();
      if (!name) { alert('Please enter button name'); return; }
      document.getElementById('learningWaiting').classList.add('active');
      document.getElementById('learningSuccess').classList.remove('active');
      fetch('/learn/start?name=' + encodeURIComponent(name))
        .then(r => r.text())
        .then(() => pollLearnStatus());
    }

    function pollLearnStatus() {
      const interval = setInterval(() => {
        fetch('/learn/status')
          .then(r => r.json())
          .then(data => {
            if (!data.learning) {
              clearInterval(interval);
              document.getElementById('learningWaiting').classList.remove('active');
              if (data.success) {
                document.getElementById('learningSuccess').classList.add('active');
                document.getElementById('learnName').value = '';
                setTimeout(() => {
                  document.getElementById('learningSuccess').classList.remove('active');
                }, 3000);
              }
              updateStatus();
            }
          });
      }, 500);
    }

    function loadCodes() {
      fetch('/codes')
        .then(r => r.json())
        .then(data => {
          const html = data.codes.length ? data.codes.map(c => `
            <div class="list-item">
              <div>
                <div class="name">${c.name}</div>
                <div class="meta">${c.code} . ${c.bits} bits</div>
              </div>
              <div class="actions">
                <button class="btn btn-sm" onclick="testCode('${c.name}')">Test</button>
                <button class="btn btn-sm btn-danger" onclick="deleteCode('${c.name}')">Delete</button>
              </div>
            </div>
          `).join('') : '<div class="empty-state">No codes learned yet. Go to "Learn IR" page to add some!</div>';
          document.getElementById('codesList').innerHTML = html;
        });
    }

    function testCode(name) {
      fetch('/send?name=' + encodeURIComponent(name));
    }

    function deleteCode(name) {
      if (!confirm('Delete "' + name + '"?')) return;
      fetch('/codes/delete?name=' + encodeURIComponent(name))
        .then(() => loadCodes());
    }

    function loadChains() {
      fetch('/chains')
        .then(r => r.json())
        .then(data => {
          const html = data.chains.length ? data.chains.map(c => `
            <div class="list-item">
              <div>
                <div class="name">${c.name}</div>
                <div class="meta">${c.stepCount} steps</div>
              </div>
              <div class="actions">
                <button class="btn btn-sm btn-success" onclick="runChain(${c.id})">Run</button>
                <button class="btn btn-sm" onclick="editChain(${c.id})">Edit</button>
                <button class="btn btn-sm btn-danger" onclick="deleteChain(${c.id})">Delete</button>
              </div>
            </div>
          `).join('') : '<div class="empty-state">No chains yet. Click button above to create one!</div>';
          document.getElementById('chainsList').innerHTML = html;
        });
    }

    function openChainModal(id = null) {
      currentEditChainId = id;
      tempSteps = [];
      document.getElementById('chainModalTitle').textContent = id ? 'Edit Chain' : 'New Chain';
      document.getElementById('chainName').value = '';
      document.getElementById('stepsList').innerHTML = '';

      fetch('/codes').then(r => r.json()).then(data => {
        const sel = document.getElementById('stepSendCode');
        sel.innerHTML = data.codes.map(c => '<option value="' + c.name + '">' + c.name + '</option>').join('');
      });

      if (id) {
        fetch('/chains')
          .then(r => r.json())
          .then(data => {
            const chain = data.chains.find(c => c.id === id);
            if (chain) {
              document.getElementById('chainName').value = chain.name;
              tempSteps = chain.steps || [];
              renderSteps();
            }
          });
      }

      document.getElementById('chainModal').classList.add('active');
    }

    function closeChainModal() {
      document.getElementById('chainModal').classList.remove('active');
    }

    function editChain(id) {
      openChainModal(id);
    }

    function deleteChain(id) {
      if (!confirm('Delete this chain?')) return;
      fetch('/chains/delete?id=' + id)
        .then(() => loadChains());
    }

    function runChain(id) {
      fetch('/chains/run?id=' + id);
    }

    document.getElementById('stepType').addEventListener('change', function() {
      document.getElementById('stepSendCol').style.display = this.value === 'send' ? 'block' : 'none';
      document.getElementById('stepWaitCol').style.display = this.value === 'wait' ? 'block' : 'none';
    });

    function addStep() {
      const type = document.getElementById('stepType').value;
      const data = type === 'send' ? document.getElementById('stepSendCode').value : document.getElementById('stepWaitMs').value;
      tempSteps.push({ type, data });
      renderSteps();
    }

    function renderSteps() {
      document.getElementById('stepsList').innerHTML = tempSteps.length ? tempSteps.map((s, i) => `
        <div class="step-item">
          <span class="handle">::</span>
          <span class="content">
            <span class="step-badge ${s.type === 'send' ? 'badge-send' : 'badge-wait'}">${s.type === 'send' ? 'Send' : 'Wait'}</span>
            ${s.type === 'send' ? s.data : s.data + 'ms'}
          </span>
          <button class="btn btn-sm btn-danger" onclick="removeStep(${i})">X</button>
        </div>
      `).join('') : '<div style="color:#666;text-align:center;padding:20px;">No steps yet. Add some below</div>';
    }

    function removeStep(i) {
      tempSteps.splice(i, 1);
      renderSteps();
    }

    function saveChain() {
      const name = document.getElementById('chainName').value.trim();
      if (!name) { alert('Please enter chain name'); return; }
      if (!tempSteps.length) { alert('Please add at least one step'); return; }

      const payload = { name, steps: tempSteps };
      if (currentEditChainId) payload.id = currentEditChainId;

      fetch(currentEditChainId ? '/chains/update' : '/chains/create', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      }).then(() => {
        closeChainModal();
        loadChains();
      });
    }

    function loadSchedules() {
      Promise.all([fetch('/schedules'), fetch('/chains')])
        .then(([r1, r2]) => Promise.all([r1.json(), r2.json()]))
        .then(([data, chainsData]) => {
          const html = data.schedules.length ? data.schedules.map(s => {
            const chain = chainsData.chains.find(c => c.id === s.chainId);
            const chainName = chain ? chain.name : 'Unknown';
            const time = String(s.hour).padStart(2, '0') + ':' + String(s.minute).padStart(2, '0');
            return `
              <div class="list-item">
                <div>
                  <div class="name">${time} . ${chainName}</div>
                  <div class="meta">${s.enabled ? 'Enabled' : 'Disabled'}</div>
                </div>
                <div class="actions">
                  <label class="toggle">
                    <input type="checkbox" ${s.enabled ? 'checked' : ''} onchange="toggleSchedule(${s.id}, this.checked)">
                    <span class="toggle-slider"></span>
                  </label>
                  <button class="btn btn-sm" onclick="editSchedule(${s.id})">Edit</button>
                  <button class="btn btn-sm btn-danger" onclick="deleteSchedule(${s.id})">Delete</button>
                </div>
              </div>
            `;
          }).join('') : '<div class="empty-state">No schedules yet. Click button above to create one!</div>';
          document.getElementById('schedulesList').innerHTML = html;
        });
    }

    function openScheduleModal(id = null) {
      currentEditScheduleId = id;
      document.getElementById('scheduleModalTitle').textContent = id ? 'Edit Schedule' : 'New Schedule';
      document.getElementById('scheduleHour').value = '';
      document.getElementById('scheduleMinute').value = '';

      fetch('/chains').then(r => r.json()).then(data => {
        const sel = document.getElementById('scheduleChain');
        sel.innerHTML = data.chains.map(c => '<option value="' + c.id + '">' + c.name + '</option>').join('');
      });

      if (id) {
        fetch('/schedules')
          .then(r => r.json())
          .then(data => {
            const sched = data.schedules.find(s => s.id === id);
            if (sched) {
              document.getElementById('scheduleHour').value = sched.hour;
              document.getElementById('scheduleMinute').value = sched.minute;
              document.getElementById('scheduleChain').value = sched.chainId;
            }
          });
      }

      document.getElementById('scheduleModal').classList.add('active');
    }

    function closeScheduleModal() {
      document.getElementById('scheduleModal').classList.remove('active');
    }

    function editSchedule(id) {
      openScheduleModal(id);
    }

    function deleteSchedule(id) {
      if (!confirm('Delete this schedule?')) return;
      fetch('/schedules/delete?id=' + id)
        .then(() => loadSchedules());
    }

    function toggleSchedule(id, enabled) {
      fetch('/schedules')
        .then(r => r.json())
        .then(data => {
          const sched = data.schedules.find(s => s.id === id);
          if (sched) {
            sched.enabled = enabled;
            fetch('/schedules/update', {
              method: 'POST',
              headers: { 'Content-Type': 'application/json' },
              body: JSON.stringify(sched)
            });
          }
        });
    }

    function saveSchedule() {
      const hour = parseInt(document.getElementById('scheduleHour').value);
      const minute = parseInt(document.getElementById('scheduleMinute').value);
      const chainId = parseInt(document.getElementById('scheduleChain').value);

      if (isNaN(hour) || isNaN(minute)) { alert('Please enter valid time'); return; }

      const payload = { hour, minute, chainId, enabled: true };
      if (currentEditScheduleId) payload.id = currentEditScheduleId;

      fetch(currentEditScheduleId ? '/schedules/update' : '/schedules/create', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      }).then(() => {
        closeScheduleModal();
        loadSchedules();
      });
    }

    setInterval(updateStatus, 3000);
    updateStatus();
  </script>
</body>
</html>
  )";
  server.send(200, "text/html", html);
}

void handleStatus() {
  String json = "{";
  json += "\"wifi\":\"" + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected") + "\",";
  json += "\"ip\":\"" + String(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "--") + "\",";
  int validCodes = 0;
  for (int i = 0; i < MAX_IR_CODES; i++) {
    if (irCodes[i].valid) validCodes++;
  }
  json += "\"codes\":" + String(validCodes) + ",";
  json += "\"learning\":" + String(learningMode ? "true" : "false") + ",";
  json += "\"time\":\"" + getCurrentTimeString() + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleLearnStart() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "Missing name parameter");
    return;
  }
  String name = server.arg("name");
  enterLearningMode(name.c_str());
  server.send(200, "text/plain", "ok");
}

void handleLearnStatus() {
  String json = "{";
  json += "\"learning\":" + String(learningMode ? "true" : "false") + ",";
  json += "\"success\":" + String(!learningMode && learningSlot == -1 ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void handleCodesList() {
  DynamicJsonDocument doc(4096);
  JsonArray arr = doc.createNestedArray("codes");
  for (int i = 0; i < MAX_IR_CODES; i++) {
    if (irCodes[i].valid) {
      JsonObject obj = arr.createNestedObject();
      obj["name"] = irCodes[i].name;
      char codeStr[32];
      sprintf(codeStr, "0x%llx", irCodes[i].code);
      obj["code"] = codeStr;
      obj["bits"] = irCodes[i].bits;
      obj["protocol"] = protocolToString(irCodes[i].protocol);
    }
  }
  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}

void handleCodesDelete() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "Missing name parameter");
    return;
  }
  String name = server.arg("name");
  for (int i = 0; i < MAX_IR_CODES; i++) {
    if (irCodes[i].valid && strcmp(irCodes[i].name, name.c_str()) == 0) {
      irCodes[i].valid = false;
      saveIRCodes();
      break;
    }
  }
  server.send(200, "text/plain", "ok");
}

void handleChainsList() {
  DynamicJsonDocument doc(8192);
  JsonArray arr = doc.createNestedArray("chains");
  for (int i = 0; i < 20; i++) {
    if (chains[i].valid) {
      JsonObject obj = arr.createNestedObject();
      obj["id"] = chains[i].id;
      obj["name"] = chains[i].name;
      obj["stepCount"] = chains[i].stepCount;
      JsonArray stepsArr = obj.createNestedArray("steps");
      for (int j = 0; j < chains[i].stepCount; j++) {
        JsonObject stepObj = stepsArr.createNestedObject();
        stepObj["type"] = (chains[i].steps[j].type == Step::SEND) ? "send" : "wait";
        stepObj["data"] = chains[i].steps[j].data;
      }
    }
  }
  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}

void handleChainsCreate() {
  DynamicJsonDocument doc(2048);
  deserializeJson(doc, server.arg("plain"));

  int idx = -1;
  for (int i = 0; i < 20; i++) {
    if (!chains[i].valid) {
      idx = i;
      break;
    }
  }
  if (idx == -1) {
    server.send(500, "text/plain", "Too many chains");
    return;
  }

  chains[idx].id = chainIdCounter++;
  strncpy(chains[idx].name, doc["name"] | "", 63);
  chains[idx].stepCount = 0;

  if (doc.containsKey("steps")) {
    JsonArray stepsArr = doc["steps"];
    for (JsonObject stepObj : stepsArr) {
      if (chains[idx].stepCount >= 50) break;
      const char* typeStr = stepObj["type"] | "send";
      chains[idx].steps[chains[idx].stepCount].type = (strcmp(typeStr, "wait") == 0) ? Step::WAIT : Step::SEND;
      strncpy(chains[idx].steps[chains[idx].stepCount].data, stepObj["data"] | "", 63);
      chains[idx].stepCount++;
    }
  }

  chains[idx].valid = true;
  saveChains();
  server.send(200, "text/plain", "ok");
}

void handleChainsUpdate() {
  DynamicJsonDocument doc(2048);
  deserializeJson(doc, server.arg("plain"));

  int id = doc["id"] | 0;
  int idx = -1;
  for (int i = 0; i < 20; i++) {
    if (chains[i].valid && chains[i].id == id) {
      idx = i;
      break;
    }
  }
  if (idx == -1) {
    server.send(404, "text/plain", "Chain not found");
    return;
  }

  strncpy(chains[idx].name, doc["name"] | "", 63);
  chains[idx].stepCount = 0;

  if (doc.containsKey("steps")) {
    JsonArray stepsArr = doc["steps"];
    for (JsonObject stepObj : stepsArr) {
      if (chains[idx].stepCount >= 50) break;
      const char* typeStr = stepObj["type"] | "send";
      chains[idx].steps[chains[idx].stepCount].type = (strcmp(typeStr, "wait") == 0) ? Step::WAIT : Step::SEND;
      strncpy(chains[idx].steps[chains[idx].stepCount].data, stepObj["data"] | "", 63);
      chains[idx].stepCount++;
    }
  }

  saveChains();
  server.send(200, "text/plain", "ok");
}

void handleChainsDelete() {
  if (!server.hasArg("id")) {
    server.send(400, "text/plain", "Missing id parameter");
    return;
  }
  int id = server.arg("id").toInt();
  for (int i = 0; i < 20; i++) {
    if (chains[i].valid && chains[i].id == id) {
      chains[i].valid = false;
      saveChains();
      break;
    }
  }
  server.send(200, "text/plain", "ok");
}

void handleChainsRun() {
  if (!server.hasArg("id")) {
    server.send(400, "text/plain", "Missing id parameter");
    return;
  }
  int id = server.arg("id").toInt();
  server.send(200, "text/plain", "ok");
  delay(100);
  runChain(id);
}

void handleSchedulesList() {
  DynamicJsonDocument doc(4096);
  JsonArray arr = doc.createNestedArray("schedules");
  for (int i = 0; i < 20; i++) {
    if (schedules[i].valid) {
      JsonObject obj = arr.createNestedObject();
      obj["id"] = schedules[i].id;
      obj["chainId"] = schedules[i].chainId;
      obj["hour"] = schedules[i].hour;
      obj["minute"] = schedules[i].minute;
      obj["enabled"] = schedules[i].enabled;
    }
  }
  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}

void handleSchedulesCreate() {
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, server.arg("plain"));

  int idx = -1;
  for (int i = 0; i < 20; i++) {
    if (!schedules[i].valid) {
      idx = i;
      break;
    }
  }
  if (idx == -1) {
    server.send(500, "text/plain", "Too many schedules");
    return;
  }

  schedules[idx].id = scheduleIdCounter++;
  schedules[idx].chainId = doc["chainId"] | 0;
  schedules[idx].hour = doc["hour"] | 0;
  schedules[idx].minute = doc["minute"] | 0;
  schedules[idx].enabled = doc["enabled"] | true;
  schedules[idx].valid = true;
  schedules[idx].triggeredToday = false;

  saveSchedules();
  server.send(200, "text/plain", "ok");
}

void handleSchedulesUpdate() {
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, server.arg("plain"));

  int id = doc["id"] | 0;
  int idx = -1;
  for (int i = 0; i < 20; i++) {
    if (schedules[i].valid && schedules[i].id == id) {
      idx = i;
      break;
    }
  }
  if (idx == -1) {
    server.send(404, "text/plain", "Schedule not found");
    return;
  }

  schedules[idx].chainId = doc["chainId"] | schedules[idx].chainId;
  schedules[idx].hour = doc.containsKey("hour") ? (doc["hour"] | 0) : schedules[idx].hour;
  schedules[idx].minute = doc.containsKey("minute") ? (doc["minute"] | 0) : schedules[idx].minute;
  schedules[idx].enabled = doc.containsKey("enabled") ? (doc["enabled"] | true) : schedules[idx].enabled;

  saveSchedules();
  server.send(200, "text/plain", "ok");
}

void handleSchedulesDelete() {
  if (!server.hasArg("id")) {
    server.send(400, "text/plain", "Missing id parameter");
    return;
  }
  int id = server.arg("id").toInt();
  for (int i = 0; i < 20; i++) {
    if (schedules[i].valid && schedules[i].id == id) {
      schedules[i].valid = false;
      saveSchedules();
      break;
    }
  }
  server.send(200, "text/plain", "ok");
}

void handleNotFound() {
  if (server.method() == HTTP_GET && server.uri() == "/send") {
    if (server.hasArg("name")) {
      sendIRCodeByName(server.arg("name").c_str());
      server.send(200, "text/plain", "ok");
      return;
    }
  }
  server.send(404, "text/plain", "Not Found");
}
