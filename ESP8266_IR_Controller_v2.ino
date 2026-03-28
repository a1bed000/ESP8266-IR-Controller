/*
 * ESP8266/ESP-12F 红外遥控智能控制器 (v2 - 带NTP时间同步)
 * 功能：
 * 1. 红外学习功能 - 接收并存储红外遥控编码
 * 2. WiFi透传控制 - 通过网络发送红外指令
 * 3. NTP时间同步 - 从网络获取准确时间
 * 4. 定时任务 - 每天8点自动开机播放，19点关机
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <IRutils.h>
#include <EEPROM.h>
#include <Ticker.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// ============== 配置区域 ==============
// WiFi 配置 - 请修改为你的网络信息
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// NTP 服务器配置
const char* ntpServer = "pool.ntp.org";  // NTP服务器
const long gmtOffsetSec = 8 * 3600;        // 北京时间 UTC+8
const int daylightOffsetSec = 0;           // 不启用夏令时

// 定时任务时间配置（24小时制）
const int AUTO_ON_HOUR = 8;     // 早上8点自动开机
const int AUTO_OFF_HOUR = 19;   // 晚上7点关机
const int AUTO_ON_MINUTE = 0;   // 自动开机分钟
const int AUTO_OFF_MINUTE = 0;  // 自动关机分钟

// 自动播放序列延迟（毫秒）
const int DELAY_BETWEEN_COMMANDS = 2000;

// 时区偏移（小时）
const int TIMEZONE_OFFSET = 8;
// ============== 配置结束 ==============

// 引脚定义
const uint16_t IR_RECEIVE_PIN = 14;  // D5 - 红外接收
const uint16_t IR_SEND_PIN = 4;      // D2 - 红外发射

// 存储的红外编码结构
struct IRCode {
  uint32_t code;        // NEC编码值
  uint8_t bits;         // 编码位数
  char name[16];        // 编码名称
  bool valid;           // 是否有效
};

// 预设的命令槽位
const int MAX_CODES = 20;
IRCode storedCodes[MAX_CODES];

// 电视机自动播放的操作序列
const char* AUTO_SEQUENCE[] = {
  "power",      // 电源键
  "wait3",      // 等待3秒
  "source",     // 信号源
  "ok",         // 确认
  "down",       // 向下
  "down",       // 向下
  "ok"          // 确认选择
};
const int SEQ_LENGTH = 7;

// 全局对象
IRrecv irrecv(IR_RECEIVE_PIN);
IRsend irsend(IR_SEND_PIN);
ESP8266WebServer server(80);
Ticker timeTicker;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServer, gmtOffsetSec, 60000);

// 学习模式状态
bool learningMode = false;
int learningSlot = -1;
unsigned long learningStartTime = 0;
const unsigned long LEARNING_TIMEOUT = 30000;  // 30秒超时

// 定时任务状态
bool autoControlEnabled = true;
int lastCheckedHour = -1;
int lastCheckedMinute = -1;
bool autoOnTriggered = false;
bool autoOffTriggered = false;
unsigned long lastNtpSync = 0;

// ============== 函数声明 ==============
void loadCodesFromEEPROM();
void saveCodesToEEPROM();
void sendIRCommand(const char* cmdName);
void sendNECCode(uint32_t code, uint8_t bits);
void handleLearn();
void handleSend();
void handleList();
void handleAutoSequence();
void handleRoot();
void handleStatus();
void checkScheduledTasks();
void executeAutoSequence();
void enterLearningMode(int slot, const char* name);
void syncNTPTime();
String getCurrentTimeString();

// ============== 设置函数 ==============
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== ESP8266 红外智能控制器 v2 ===");

  // 初始化EEPROM
  EEPROM.begin(512);

  // 从EEPROM加载存储的红外编码
  loadCodesFromEEPROM();

  // 初始化红外接收和发射
  irrecv.enableIRIn();
  irsend.begin();

  // 连接WiFi
  Serial.print("正在连接WiFi: ");
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
    Serial.println("\nWiFi已连接!");
    Serial.print("IP地址: ");
    Serial.println(WiFi.localIP());

    // 启动NTP客户端
    timeClient.begin();
    syncNTPTime();
  } else {
    Serial.println("\nWiFi连接失败，启动AP模式");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("ESP8266_IR_Controller");
    Serial.print("AP IP地址: ");
    Serial.println(WiFi.softAPIP());
  }

  // 设置HTTP路由
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/learn", HTTP_GET, handleLearn);
  server.on("/send", HTTP_GET, handleSend);
  server.on("/list", HTTP_GET, handleList);
  server.on("/auto", HTTP_GET, handleAutoSequence);

  server.begin();
  Serial.println("HTTP服务器已启动");

  // 启动定时任务检查（每10秒检查一次）
  timeTicker.attach(10.0, checkScheduledTasks);

  Serial.println("系统初始化完成!");
  Serial.println("===========================================\n");
}

// ============== 主循环 ==============
void loop() {
  server.handleClient();
  timeClient.update();

  // 每小时同步一次NTP时间
  if (millis() - lastNtpSync > 3600000) {
    syncNTPTime();
  }

  // 处理红外学习模式
  if (learningMode) {
    if (millis() - learningStartTime > LEARNING_TIMEOUT) {
      Serial.println("学习模式超时!");
      learningMode = false;
      learningSlot = -1;
    } else {
      decode_results results;
      if (irrecv.decode(&results)) {
        Serial.print("接收到红外信号: ");
        Serial.print("协议: ");
        Serial.print(typeToString(results.decode_type));
        Serial.print(" 编码: 0x");
        Serial.println(results.value, HEX);

        if (results.decode_type == NEC && learningSlot >= 0) {
          storedCodes[learningSlot].code = results.value;
          storedCodes[learningSlot].bits = results.bits;
          storedCodes[learningSlot].valid = true;
          saveCodesToEEPROM();

          Serial.print("编码已保存到槽位 ");
          Serial.print(learningSlot);
          Serial.print(" 名称: ");
          Serial.println(storedCodes[learningSlot].name);

          learningMode = false;
          learningSlot = -1;
        } else if (results.decode_type != NEC) {
          Serial.println("警告: 只支持NEC协议编码!");
        }

        irrecv.resume();
      }
    }
  }

  delay(10);
}

// ============== 网络请求处理 ==============

void handleRoot() {
  // 内联HTML界面（与之前相同，为了节省空间省略）
  // 实际使用时，应该使用PROGMEM或外部文件存储
  server.send(200, "text/html", "<a href='/status'>状态</a> | <a href='/list'>编码列表</a>");
}

void handleStatus() {
  String json = "{";
  json += "\"wifi\":\"" + String(WiFi.status() == WL_CONNECTED ? "已连接" : "未连接") + "\",";
  json += "\"ip\":\"" + String(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "--") + "\",";

  int validCodes = 0;
  for (int i = 0; i < MAX_CODES; i++) {
    if (storedCodes[i].valid) validCodes++;
  }
  json += "\"codes\":" + String(validCodes) + ",";
  json += "\"learning\":" + String(learningMode ? "true" : "false") + ",";
  json += "\"time\":\"" + getCurrentTimeString() + "\",";
  json += "\"nextOn\":\"" + String(AUTO_ON_HOUR) + ":" + String(AUTO_ON_MINUTE) + "\",";
  json += "\"nextOff\":\"" + String(AUTO_OFF_HOUR) + ":" + String(AUTO_OFF_MINUTE) + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleLearn() {
  if (!server.hasArg("slot")) {
    server.send(400, "text/plain", "缺少slot参数");
    return;
  }

  String slotName = server.arg("slot");
  int slot = -1;
  for (int i = 0; i < MAX_CODES; i++) {
    if (storedCodes[i].valid && strcmp(storedCodes[i].name, slotName.c_str()) == 0) {
      slot = i;
      break;
    }
    if (slot == -1 && !storedCodes[i].valid) {
      slot = i;
    }
  }

  if (slot == -1) {
    server.send(500, "text/plain", "存储空间已满");
    return;
  }

  strncpy(storedCodes[slot].name, slotName.c_str(), 15);
  storedCodes[slot].name[15] = '\0';
  enterLearningMode(slot, storedCodes[slot].name);

  server.send(200, "text/plain", "学习模式已启动，请在30秒内按遥控器");
}

void handleSend() {
  if (!server.hasArg("cmd")) {
    server.send(400, "text/plain", "缺少cmd参数");
    return;
  }
  String cmd = server.arg("cmd");
  sendIRCommand(cmd.c_str());
  server.send(200, "text/plain", "已发送: " + cmd);
}

void handleList() {
  String json = "{";
  json += "\"codes\":[";
  bool first = true;
  for (int i = 0; i < MAX_CODES; i++) {
    if (storedCodes[i].valid) {
      if (!first) json += ",";
      first = false;
      json += "{";
      json += "\"name\":\"" + String(storedCodes[i].name) + "\",";
      json += "\"code\":\"0x" + String(storedCodes[i].code, HEX) + "\",";
      json += "\"bits\":" + String(storedCodes[i].bits);
      json += "}";
    }
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleAutoSequence() {
  server.send(200, "text/plain", "自动播放序列已启动");
  executeAutoSequence();
}

// ============== 红外操作函数 ==============

void sendIRCommand(const char* cmdName) {
  for (int i = 0; i < MAX_CODES; i++) {
    if (storedCodes[i].valid && strcmp(storedCodes[i].name, cmdName) == 0) {
      sendNECCode(storedCodes[i].code, storedCodes[i].bits);
      Serial.print("发送红外命令: ");
      Serial.print(cmdName);
      Serial.print(" 编码: 0x");
      Serial.println(storedCodes[i].code, HEX);
      return;
    }
  }

  // 预设命令（常见电视遥控器编码）
  if (strcmp(cmdName, "power") == 0) {
    sendNECCode(0x20DF10EF, 32);
  } else if (strcmp(cmdName, "vol+") == 0) {
    sendNECCode(0x20DF40BF, 32);
  } else if (strcmp(cmdName, "vol-") == 0) {
    sendNECCode(0x20DFC03F, 32);
  } else if (strcmp(cmdName, "source") == 0) {
    sendNECCode(0x20DFD02F, 32);
  } else if (strcmp(cmdName, "up") == 0) {
    sendNECCode(0x20DF02FD, 32);
  } else if (strcmp(cmdName, "down") == 0) {
    sendNECCode(0x20DF827D, 32);
  } else if (strcmp(cmdName, "left") == 0) {
    sendNECCode(0x20DFE01F, 32);
  } else if (strcmp(cmdName, "right") == 0) {
    sendNECCode(0x20DF609F, 32);
  } else if (strcmp(cmdName, "ok") == 0) {
    sendNECCode(0x20DF22DD, 32);
  } else if (strcmp(cmdName, "menu") == 0) {
    sendNECCode(0x20DFC23D, 32);
  } else if (strcmp(cmdName, "back") == 0) {
    sendNECCode(0x20DF14EB, 32);
  } else if (strcmp(cmdName, "home") == 0) {
    sendNECCode(0x20DF3EC1, 32);
  } else {
    Serial.print("未知命令: ");
    Serial.println(cmdName);
  }
}

void sendNECCode(uint32_t code, uint8_t bits) {
  irsend.sendNEC(code, bits);
  delay(100);
}

void enterLearningMode(int slot, const char* name) {
  learningMode = true;
  learningSlot = slot;
  learningStartTime = millis();
  Serial.print("进入学习模式，槽位: ");
  Serial.println(slot);
}

// ============== 定时任务 ==============

void syncNTPTime() {
  Serial.println("正在同步NTP时间...");
  timeClient.update();
  Serial.print("当前时间: ");
  Serial.println(timeClient.getFormattedTime());
  lastNtpSync = millis();
}

String getCurrentTimeString() {
  return timeClient.getFormattedTime();
}

void checkScheduledTasks() {
  // 确保NTP时间已同步
  if (!timeClient.isTimeSet()) {
    Serial.println("等待时间同步...");
    return;
  }

  int currentHour = timeClient.getHours();
  int currentMinute = timeClient.getMinutes();

  // 只在分钟变化时检查
  if (currentMinute == lastCheckedMinute) {
    return;
  }
  lastCheckedMinute = currentMinute;

  Serial.print("当前时间: ");
  Serial.print(currentHour);
  Serial.print(":");
  Serial.println(currentMinute);

  // 早上自动开机
  if (currentHour == AUTO_ON_HOUR && currentMinute == AUTO_ON_MINUTE && !autoOnTriggered) {
    Serial.println("执行自动开机任务!");
    executeAutoSequence();
    autoOnTriggered = true;
    autoOffTriggered = false;
  }

  // 晚上自动关机
  if (currentHour == AUTO_OFF_HOUR && currentMinute == AUTO_OFF_MINUTE && !autoOffTriggered) {
    Serial.println("执行自动关机任务!");
    sendIRCommand("power");
    autoOffTriggered = true;
    autoOnTriggered = false;
  }

  // 重置触发状态（新的一天）
  if (currentHour == 0 && currentMinute == 0) {
    autoOnTriggered = false;
    autoOffTriggered = false;
  }
}

void executeAutoSequence() {
  Serial.println("========================================");
  Serial.println("开始执行自动播放序列...");
  Serial.println("========================================");

  for (int i = 0; i < SEQ_LENGTH; i++) {
    const char* cmd = AUTO_SEQUENCE[i];

    if (strncmp(cmd, "wait", 4) == 0) {
      int waitSec = atoi(cmd + 4);
      Serial.print("[等待] ");
      Serial.print(waitSec);
      Serial.println(" 秒...");
      delay(waitSec * 1000);
    } else {
      Serial.print("[发送] ");
      Serial.println(cmd);
      sendIRCommand(cmd);
      delay(DELAY_BETWEEN_COMMANDS);
    }
  }

  Serial.println("========================================");
  Serial.println("自动播放序列执行完成!");
  Serial.println("========================================");
}

// ============== EEPROM 操作 ==============

void loadCodesFromEEPROM() {
  int addr = 0;
  for (int i = 0; i < MAX_CODES; i++) {
    EEPROM.get(addr, storedCodes[i]);
    addr += sizeof(IRCode);

    if (storedCodes[i].valid && storedCodes[i].code != 0) {
      Serial.print("加载编码: ");
      Serial.print(storedCodes[i].name);
      Serial.print(" = 0x");
      Serial.println(storedCodes[i].code, HEX);
    }
  }
}

void saveCodesToEEPROM() {
  int addr = 0;
  for (int i = 0; i < MAX_CODES; i++) {
    EEPROM.put(addr, storedCodes[i]);
    addr += sizeof(IRCode);
  }
  EEPROM.commit();
  Serial.println("编码已保存到EEPROM");
}

// ============== HTTP 处理函数 ==============

void handleRoot() {
  String html = R"(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ESP8266 红外控制器</title>
  <style>
    body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif; max-width: 600px; margin: 0 auto; padding: 20px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); min-height: 100vh; }
    h1 { color: white; text-align: center; text-shadow: 2px 2px 4px rgba(0,0,0,0.3); margin-bottom: 10px; }
    .subtitle { color: rgba(255,255,255,0.8); text-align: center; margin-bottom: 20px; }
    .card { background: white; border-radius: 12px; padding: 20px; margin: 15px 0; box-shadow: 0 4px 15px rgba(0,0,0,0.1); }
    .btn { display: inline-block; padding: 12px 24px; margin: 5px; background: #667eea; color: white; text-decoration: none; border-radius: 25px; border: none; cursor: pointer; font-size: 14px; transition: all 0.3s; }
    .btn:hover { transform: translateY(-2px); box-shadow: 0 4px 12px rgba(102,126,234,0.4); }
    .btn-green { background: linear-gradient(135deg, #11998e 0%, #38ef7d 100%); }
    .btn-green:hover { box-shadow: 0 4px 12px rgba(17,153,142,0.4); }
    .btn-orange { background: linear-gradient(135deg, #f093fb 0%, #f5576c 100%); }
    .btn-orange:hover { box-shadow: 0 4px 12px rgba(240,147,251,0.4); }
    .status { padding: 15px; border-radius: 8px; background: linear-gradient(135deg, #f5f7fa 0%, #e4e8ec 100%); color: #333; font-family: monospace; }
    input[type="text"] { padding: 12px; border: 2px solid #e0e0e0; border-radius: 25px; width: 150px; font-size: 14px; transition: border-color 0.3s; }
    input[type="text"]:focus { outline: none; border-color: #667eea; }
    .schedule { color: #666; font-size: 14px; display: flex; align-items: center; gap: 10px; margin: 8px 0; }
    .clock-icon { font-size: 20px; }
    .result { margin-top: 10px; padding: 10px; border-radius: 5px; background: #e8f5e9; color: #2e7d32; }
    h3 { color: #333; margin-top: 0; border-bottom: 2px solid #667eea; padding-bottom: 10px; }
  </style>
</head>
<body>
  <h1>📺 ESP8266 红外控制器</h1>
  <p class="subtitle">智能遥控 · 定时任务 · WiFi控制</p>

  <div class="card">
    <h3>🖥️ 系统状态</h3>
    <div class="status" id="status">正在连接...⏳</div>
  </div>

  <div class="card">
    <h3>🎓 红外学习</h3>
    <p style="color:#666;font-size:14px;">输入命令名称，点击学习，然后在30秒内按遥控器按键</p>
    <input type="text" id="cmdName" placeholder="如: power, vol+" maxlength="15">
    <button class="btn btn-orange" onclick="learn()">🎓 开始学习</button>
    <div id="learnResult"></div>
  </div>

  <div class="card">
    <h3>🎮 手动控制</h3>
    <input type="text" id="sendCmd" placeholder="命令名称" maxlength="15">
    <button class="btn" onclick="send()">📡 发送指令</button>
    <div id="sendResult"></div>
    <p style="color:#999;font-size:12px;margin-top:10px;">常用: power, vol+, vol-, source, up, down, left, right, ok, menu, back, home</p>
  </div>

  <div class="card">
    <h3>▶️ 自动播放</h3>
    <p style="color:#666;font-size:14px;">立即执行自动播放序列（开机 → 选择信号源 → 播放）</p>
    <button class="btn btn-green" onclick="autoPlay()">▶️ 立即执行</button>
    <div id="autoResult"></div>
  </div>

  <div class="card">
    <h3>⏰ 定时任务</h3>
    <div class="schedule"><span class="clock-icon">🌅</span> 每天早上 08:00 自动开机并播放视频</div>
    <div class="schedule"><span class="clock-icon">🌙</span> 每天晚上 19:00 自动关机</div>
  </div>

  <script>
    function updateStatus() {
      fetch('/status')
        .then(r => r.json())
        .then(data => {
          document.getElementById('status').innerHTML =
            '📶 WiFi: ' + data.wifi + '<br>' +
            '🌐 IP: ' + data.ip + '<br>' +
            '💾 存储命令: ' + data.codes + ' 个<br>' +
            '🕐 当前时间: ' + data.time + '<br>' +
            '🎓 学习模式: ' + (data.learning ? '<span style=\"color:orange\">进行中⏳</span>' : '<span style=\"color:green\">空闲✓</span>');
        })
        .catch(() => {
          document.getElementById('status').innerHTML = '⚠️ 连接失败，请刷新页面';
        });
    }

    function learn() {
      const name = document.getElementById('cmdName').value.trim();
      if (!name) { alert('请输入命令名称'); return; }
      document.getElementById('learnResult').innerHTML = '<div class="result">🎓 学习模式已启动...请在30秒内按遥控器按键</div>';
      fetch('/learn?slot=' + encodeURIComponent(name))
        .then(r => r.text())
        .then(t => {
          document.getElementById('learnResult').innerHTML = '<div class="result">✅ ' + t + '</div>';
          updateStatus();
        })
        .catch(e => {
          document.getElementById('learnResult').innerHTML = '<div style="background:#ffebee;color:#c62828;padding:10px;border-radius:5px;">❌ 学习失败: ' + e + '</div>';
        });
    }

    function send() {
      const cmd = document.getElementById('sendCmd').value.trim();
      if (!cmd) { alert('请输入命令名称'); return; }
      fetch('/send?cmd=' + encodeURIComponent(cmd))
        .then(r => r.text())
        .then(t => document.getElementById('sendResult').innerHTML = '<div class="result">✅ ' + t + '</div>');
    }

    function autoPlay() {
      document.getElementById('autoResult').innerHTML = '<div class="result">▶️ 正在执行自动播放序列，请稍候...</div>';
      fetch('/auto')
        .then(r => r.text())
        .then(t => document.getElementById('autoResult').innerHTML = '<div class="result">✅ ' + t + '</div>');
    }

    setInterval(updateStatus, 3000);
    updateStatus();
  </script>
</body>
</html>
  )";
  server.send(200, "text/html", html);
}

// ============== 其他函数实现 ==============

void loadCodesFromEEPROM() {
  int addr = 0;
  for (int i = 0; i < MAX_CODES; i++) {
    EEPROM.get(addr, storedCodes[i]);
    addr += sizeof(IRCode);

    if (storedCodes[i].valid && storedCodes[i].code != 0) {
      Serial.print("加载编码: ");
      Serial.print(storedCodes[i].name);
      Serial.print(" = 0x");
      Serial.println(storedCodes[i].code, HEX);
    }
  }
}

void saveCodesToEEPROM() {
  int addr = 0;
  for (int i = 0; i < MAX_CODES; i++) {
    EEPROM.put(addr, storedCodes[i]);
    addr += sizeof(IRCode);
  }
  EEPROM.commit();
  Serial.println("编码已保存到EEPROM");
}

void enterLearningMode(int slot, const char* name) {
  learningMode = true;
  learningSlot = slot;
  learningStartTime = millis();
}

void syncNTPTime() {
  Serial.println("正在同步NTP时间...");
  timeClient.update();
  Serial.print("当前时间: ");
  Serial.println(timeClient.getFormattedTime());
  lastNtpSync = millis();
}

String getCurrentTimeString() {
  return timeClient.getFormattedTime();
}

void executeAutoSequence() {
  Serial.println("开始执行自动播放序列...");

  for (int i = 0; i < SEQ_LENGTH; i++) {
    const char* cmd = AUTO_SEQUENCE[i];

    if (strncmp(cmd, "wait", 4) == 0) {
      int waitSec = atoi(cmd + 4);
      Serial.print("等待 ");
      Serial.print(waitSec);
      Serial.println(" 秒...");
      delay(waitSec * 1000);
    } else {
      Serial.print("发送: ");
      Serial.println(cmd);
      sendIRCommand(cmd);
      delay(DELAY_BETWEEN_COMMANDS);
    }
  }

  Serial.println("序列执行完成!");
}

void checkScheduledTasks() {
  if (!timeClient.isTimeSet()) {
    return;
  }

  int currentHour = timeClient.getHours();
  int currentMinute = timeClient.getMinutes();

  if (currentMinute == lastCheckedMinute) {
    return;
  }
  lastCheckedMinute = currentMinute;

  // 自动开机
  if (currentHour == AUTO_ON_HOUR && currentMinute == AUTO_ON_MINUTE && !autoOnTriggered) {
    Serial.println("执行自动开机任务!");
    executeAutoSequence();
    autoOnTriggered = true;
    autoOffTriggered = false;
  }

  // 自动关机
  if (currentHour == AUTO_OFF_HOUR && currentMinute == AUTO_OFF_MINUTE && !autoOffTriggered) {
    Serial.println("执行自动关机任务!");
    sendIRCommand("power");
    autoOffTriggered = true;
    autoOnTriggered = false;
  }

  // 重置触发状态
  if (currentHour == 0 && currentMinute == 0) {
    autoOnTriggered = false;
    autoOffTriggered = false;
  }
}
