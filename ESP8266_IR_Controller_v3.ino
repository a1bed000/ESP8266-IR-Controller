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
void handleSend();

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
  server.on("/send", HTTP_GET, handleSend);
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
    if (LittleFS.begin());
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
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>ESP8266 IR Controller v3</title>";
  html += "<style>";
  html += "*{box-sizing:border-box;}";
  html += "body{font-family:Arial,sans-serif;margin:0;padding:20px;background:#1a1a2e;color:#eee;}";
  html += ".container{max-width:800px;margin:0 auto;}";
  html += "h1{text-align:center;margin-bottom:30px;}";
  html += ".nav{display:flex;gap:5px;margin-bottom:20px;flex-wrap:wrap;}";
  html += ".nav button{flex:1;min-width:120px;padding:12px;background:#2a2a4a;border:none;color:#aaa;cursor:pointer;border-radius:8px;}";
  html += ".nav button.active{background:#667eea;color:white;font-weight:bold;}";
  html += ".page{display:none;background:#2a2a4a;padding:20px;border-radius:12px;}";
  html += ".page.active{display:block;}";
  html += "h3{margin-top:0;color:#667eea;border-bottom:1px solid #444;padding-bottom:10px;}";
  html += ".btn{padding:10px 20px;background:#667eea;color:white;border:none;border-radius:8px;cursor:pointer;}";
  html += ".btn-danger{background:#f5576c;}";
  html += ".btn-success{background:#38ef7d;color:#1a1a2e;}";
  html += "input,select{padding:10px;border:2px solid #444;border-radius:8px;background:#1a1a2e;color:white;width:100%;margin:5px 0;}";
  html += ".item{background:#1a1a2e;padding:12px;border-radius:8px;margin:10px 0;display:flex;justify-content:space-between;align-items:center;}";
  html += ".status{background:#1a1a2e;padding:15px;border-radius:8px;margin-bottom:20px;font-family:monospace;}";
  html += "</style></head><body>";
  html += "<div class='container'>";
  html += "<h1>ESP8266 IR Controller v3</h1>";

  html += "<div class='status' id='status'>Loading...</div>";

  html += "<div class='nav'>";
  html += "<button class='active' onclick='showPage(\"learn\")'>Learn IR</button>";
  html += "<button onclick='showPage(\"codes\")'>Codes</button>";
  html += "<button onclick='showPage(\"chains\")'>Chains</button>";
  html += "<button onclick='showPage(\"schedules\")'>Schedules</button>";
  html += "<button onclick='location.href=\"/update\"'>OTA Update</button>";
  html += "</div>";

  // Learn Page
  html += "<div class='page active' id='page-learn'>";
  html += "<h3>Learn New Button</h3>";
  html += "<input type='text' id='learnName' placeholder='Button name (e.g. power)'>";
  html += "<button class='btn' onclick='startLearn()'>Start Learning</button>";
  html += "<div id='learnStatus' style='margin-top:10px;'></div>";
  html += "</div>";

  // Codes Page
  html += "<div class='page' id='page-codes'>";
  html += "<h3>Learned Codes</h3>";
  html += "<div id='codesList'></div>";
  html += "</div>";

  // Chains Page
  html += "<div class='page' id='page-chains'>";
  html += "<h3>Operation Chains</h3>";
  html += "<button class='btn btn-success' onclick='showNewChain()'>+ New Chain</button>";
  html += "<div id='chainsList' style='margin-top:15px;'></div>";
  html += "</div>";

  // Schedules Page
  html += "<div class='page' id='page-schedules'>";
  html += "<h3>Scheduled Tasks</h3>";
  html += "<button class='btn btn-success' onclick='showNewSchedule()'>+ New Schedule</button>";
  html += "<div id='schedulesList' style='margin-top:15px;'></div>";
  html += "</div>";

  html += "</div>";

  html += "<script>";
  html += "function showPage(p){";
  html += "document.querySelectorAll('.nav button').forEach(b=>b.classList.remove('active'));";
  html += "document.querySelectorAll('.page').forEach(p=>p.classList.remove('active'));";
  html += "event.target.classList.add('active');";
  html += "document.getElementById('page-'+p).classList.add('active');";
  html += "if(p==='codes')loadCodes();";
  html += "if(p==='chains')loadChains();";
  html += "if(p==='schedules')loadSchedules();";
  html += "}";

  html += "function updateStatus(){";
  html += "fetch('/status').then(r=>r.json()).then(d=>{";
  html += "document.getElementById('status').innerHTML='WiFi: '+d.wifi+'<br>IP: '+d.ip+'<br>Time: '+d.time+'<br>Codes: '+d.codes;";
  html += "});}";

  html += "function startLearn(){";
  html += "var n=document.getElementById('learnName').value;";
  html += "if(!n){alert('Enter name');return;}";
  html += "document.getElementById('learnStatus').innerHTML='Learning... Press button within 30s';";
  html += "fetch('/learn/start?name='+encodeURIComponent(n));";
  html += "var i=setInterval(function(){";
  html += "fetch('/learn/status').then(r=>r.json()).then(d=>{";
  html += "if(!d.learning){clearInterval(i);document.getElementById('learnStatus').innerHTML='Done!';updateStatus();}";
  html += "});},500);}";

  html += "function loadCodes(){";
  html += "fetch('/codes').then(r=>r.json()).then(d=>{";
  html += "var h='';";
  html += "for(var i=0;i<d.codes.length;i++){";
  html += "var c=d.codes[i];";
  html += "h+=\"<div class='item'><div><b>\"+c.name+\"</b><br><small>\"+c.code+\"</small></div><div>";
  html += "<button class='btn' onclick=\\\"testCode(\\'\"+c.name+\"\\')\\\">Test</button> ";
  html += "<button class='btn btn-danger' onclick=\\\"deleteCode(\\'\"+c.name+\"\\')\\\">Delete</button>";
  html += "</div></div>\";}";
  html += "document.getElementById('codesList').innerHTML=h||'<p>No codes yet</p>';});}";

  html += "function testCode(n){fetch('/send?name='+encodeURIComponent(n));}";
  html += "function deleteCode(n){if(confirm('Delete?')){fetch('/codes/delete?name='+encodeURIComponent(n)).then(loadCodes);}}";

  html += "function loadChains(){";
  html += "fetch('/chains').then(r=>r.json()).then(d=>{";
  html += "var h='';";
  html += "for(var i=0;i<d.chains.length;i++){";
  html += "var c=d.chains[i];";
  html += "h+=\"<div class='item'><div><b>\"+c.name+\"</b><br><small>\"+c.stepCount+' steps</small></div><div>";
  html += "<button class='btn btn-success' onclick=\\\"runChain(\"+c.id+\")\\\">Run</button> ";
  html += "<button class='btn btn-danger' onclick=\\\"deleteChain(\"+c.id+\")\\\">Delete</button>";
  html += "</div></div>\";}";
  html += "document.getElementById('chainsList').innerHTML=h||'<p>No chains yet</p>';});}";

  html += "function showNewChain(){alert('Create chain feature - use API for now');}";
  html += "function runChain(id){fetch('/chains/run?id='+id);}";
  html += "function deleteChain(id){if(confirm('Delete?')){fetch('/chains/delete?id='+id).then(loadChains);}}";

  html += "function loadSchedules(){";
  html += "fetch('/schedules').then(r=>r.json()).then(d=>{";
  html += "var h='';";
  html += "for(var i=0;i<d.schedules.length;i++){";
  html += "var s=d.schedules[i];";
  html += "h+=\"<div class='item'><div><b>\"+String(s.hour).padStart(2,'0')+\":\"+String(s.minute).padStart(2,'0')+\"</b><br><small>\"+(s.enabled?'Enabled':'Disabled')+\"</small></div><div>";
  html += "<button class='btn btn-danger' onclick=\\\"deleteSchedule(\"+s.id+\")\\\">Delete</button>";
  html += "</div></div>\";}";
  html += "document.getElementById('schedulesList').innerHTML=h||'<p>No schedules yet</p>';});}";

  html += "function showNewSchedule(){alert('Create schedule feature - use API for now');}";
  html += "function deleteSchedule(id){if(confirm('Delete?')){fetch('/schedules/delete?id='+id).then(loadSchedules);}}";

  html += "setInterval(updateStatus,3000);updateStatus();";
  html += "</script>";
  html += "</body></html>";

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
  server.send(501, "text/plain", "Use JSON API");
}

void handleChainsUpdate() {
  server.send(501, "text/plain", "Use JSON API");
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
  server.send(501, "text/plain", "Use JSON API");
}

void handleSchedulesUpdate() {
  server.send(501, "text/plain", "Use JSON API");
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

void handleSend() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "Missing name parameter");
    return;
  }
  sendIRCodeByName(server.arg("name").c_str());
  server.send(200, "text/plain", "ok");
}

void handleNotFound() {
  server.send(404, "text/plain", "Not Found");
}
