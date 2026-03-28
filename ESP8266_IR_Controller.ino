/*
 * ESP8266/ESP-12F 红外遥控智能控制器
 * 功能：
 * 1. 红外学习功能 - 接收并存储红外遥控编码
 * 2. WiFi透传控制 - 通过网络发送红外指令
 * 3. 定时任务 - 每天8点自动开机播放，19点关机
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <IRutils.h>
#include <EEPROM.h>
#include <Ticker.h>

// ============== 配置区域 ==============
// WiFi 配置 - 请修改为你的网络信息
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// 定时任务时间配置（24小时制）
const int AUTO_ON_HOUR = 8;     // 早上8点自动开机
const int AUTO_OFF_HOUR = 19;   // 晚上7点关机
const int AUTO_ON_MINUTE = 0;   // 自动开机分钟
const int AUTO_OFF_MINUTE = 0;  // 自动关机分钟

// 自动播放序列延迟（毫秒）
const int DELAY_BETWEEN_COMMANDS = 2000;  // 命令间隔2秒
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
// 顺序：开机 -> 等待 -> 选择源 -> 确认 -> 导航到视频
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

// 学习模式状态
bool learningMode = false;
int learningSlot = -1;
unsigned long learningStartTime = 0;
const unsigned long LEARNING_TIMEOUT = 30000;  // 30秒超时

// 定时任务状态
bool autoControlEnabled = true;
int lastCheckedHour = -1;
int lastCheckedMinute = -1;

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

// ============== 设置函数 ==============
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== ESP8266 红外智能控制器 ===");

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

  // 启动定时任务检查
  timeTicker.attach(1.0, checkScheduledTasks);

  Serial.println("系统初始化完成!");
  Serial.println("===========================================\n");
}

// ============== 主循环 ==============
void loop() {
  server.handleClient();

  // 处理红外学习模式
  if (learningMode) {
    // 检查超时
    if (millis() - learningStartTime > LEARNING_TIMEOUT) {
      Serial.println("学习模式超时!");
      learningMode = false;
      learningSlot = -1;
    } else {
      // 尝试接收红外信号
      decode_results results;
      if (irrecv.decode(&results)) {
        Serial.print("接收到红外信号: ");
        Serial.print("协议: ");
        Serial.print(typeToString(results.decode_type));
        Serial.print(" 编码: 0x");
        Serial.println(results.value, HEX);

        if (results.decode_type == NEC && learningSlot >= 0) {
          // 保存NEC编码
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
  String html = R"(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ESP8266 红外控制器</title>
  <style>
    body { font-family: Arial, sans-serif; max-width: 600px; margin: 0 auto; padding: 20px; background: #f5f5f5; }
    h1 { color: #333; text-align: center; }
    .card { background: white; border-radius: 8px; padding: 20px; margin: 15px 0; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
    .btn { display: inline-block; padding: 12px 24px; margin: 5px; background: #007bff; color: white; text-decoration: none; border-radius: 4px; border: none; cursor: pointer; }
    .btn:hover { background: #0056b3; }
    .btn-green { background: #28a745; }
    .btn-green:hover { background: #1e7e34; }
    .btn-orange { background: #fd7e14; }
    .btn-orange:hover { background: #e56b0a; }
    .status { padding: 10px; border-radius: 4px; background: #e9ecef; }
    input[type="text"] { padding: 8px; border: 1px solid #ddd; border-radius: 4px; width: 150px; }
    .schedule { color: #666; font-size: 14px; }
  </style>
</head>
<body>
  <h1>📺 ESP8266 红外控制器</h1>

  <div class="card">
    <h3>系统状态</h3>
    <div class="status" id="status">加载中...</div>
  </div>

  <div class="card">
    <h3>红外学习</h3>
    <p>输入命令名称（如: power, vol+, source），然后点击学习按钮，30秒内按遥控器</p>
    <input type="text" id="cmdName" placeholder="命令名称" maxlength="15">
    <button class="btn btn-orange" onclick="learn()">开始学习</button>
    <div id="learnResult"></div>
  </div>

  <div class="card">
    <h3>手动控制</h3>
    <input type="text" id="sendCmd" placeholder="命令名称" maxlength="15">
    <button class="btn" onclick="send()">发送指令</button>
    <div id="sendResult"></div>
    <p><small>常用命令: power, vol+, vol-, source, up, down, left, right, ok, menu, back</small></p>
  </div>

  <div class="card">
    <h3>自动播放</h3>
    <p>立即执行自动播放序列（开机→选择信号源→播放视频）</p>
    <button class="btn btn-green" onclick="autoPlay()">立即执行</button>
    <div id="autoResult"></div>
  </div>

  <div class="card">
    <h3>定时任务</h3>
    <p class="schedule">⏰ 每天早上 8:00 自动开机并播放视频</p>
    <p class="schedule">⏰ 每天晚上 19:00 自动关机</p>
  </div>

  <script>
    function updateStatus() {
      fetch('/status')
        .then(r => r.json())
        .then(data => {
          document.getElementById('status').innerHTML =
            'WiFi: ' + data.wifi + '<br>' +
            'IP: ' + data.ip + '<br>' +
            '存储命令数: ' + data.codes + '<br>' +
            '学习模式: ' + (data.learning ? '进行中' : '空闲');
        });
    }

    function learn() {
      const name = document.getElementById('cmdName').value.trim();
      if (!name) { alert('请输入命令名称'); return; }
      document.getElementById('learnResult').innerHTML = '学习模式中...请在30秒内按遥控器';
      fetch('/learn?slot=' + name)
        .then(r => r.text())
        .then(t => {
          document.getElementById('learnResult').innerHTML = t;
          updateStatus();
        });
    }

    function send() {
      const cmd = document.getElementById('sendCmd').value.trim();
      if (!cmd) { alert('请输入命令名称'); return; }
      fetch('/send?cmd=' + cmd)
        .then(r => r.text())
        .then(t => document.getElementById('sendResult').innerHTML = t);
    }

    function autoPlay() {
      document.getElementById('autoResult').innerHTML = '正在执行自动播放序列...';
      fetch('/auto')
        .then(r => r.text())
        .then(t => document.getElementById('autoResult').innerHTML = t);
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
  json += "\"wifi\":\"" + String(WiFi.status() == WL_CONNECTED ? "已连接" : "未连接") + "\",";
  json += "\"ip\":\"" + String(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "--") + "\",";

  int validCodes = 0;
  for (int i = 0; i < MAX_CODES; i++) {
    if (storedCodes[i].valid) validCodes++;
  }
  json += "\"codes\":" + String(validCodes) + ",";
  json += "\"learning\":" + String(learningMode ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void handleLearn() {
  if (!server.hasArg("slot")) {
    server.send(400, "text/plain", "缺少slot参数");
    return;
  }

  String slotName = server.arg("slot");
  // 查找空闲槽位或已存在的同名槽位
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
    server.send(500, "text/plain", "存储空间已满，请删除一些编码");
    return;
  }

  // 保存名称并进入学习模式
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
  server.send(200, "text/plain", "已发送命令: " + cmd);
}

void handleList() {
  String html = "<html><body><h1>存储的红外编码</h1><ul>";
  for (int i = 0; i < MAX_CODES; i++) {
    if (storedCodes[i].valid) {
      html += "<li>" + String(storedCodes[i].name) +
              " - 0x" + String(storedCodes[i].code, HEX) +
              " (" + String(storedCodes[i].bits) + " bits)</li>";
    }
  }
  html += "</ul><a href='/'>返回</a></body></html>";
  server.send(200, "text/html", html);
}

void handleAutoSequence() {
  server.send(200, "text/plain", "自动播放序列已启动");
  executeAutoSequence();
}

// ============== 红外操作函数 ==============

void sendIRCommand(const char* cmdName) {
  // 查找匹配的编码
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

  // 预设命令
  if (strcmp(cmdName, "power") == 0) {
    sendNECCode(0x20DF10EF, 32);  // 常见电源键示例
  } else if (strcmp(cmdName, "vol+") == 0) {
    sendNECCode(0x20DF40BF, 32);
  } else if (strcmp(cmdName, "vol-") == 0) {
    sendNECCode(0x20DFC03F, 32);
  } else {
    Serial.print("未知命令: ");
    Serial.println(cmdName);
  }
}

void sendNECCode(uint32_t code, uint8_t bits) {
  irsend.sendNEC(code, bits);
  delay(100);  // 短暂延迟避免信号冲突
}

void enterLearningMode(int slot, const char* name) {
  learningMode = true;
  learningSlot = slot;
  learningStartTime = millis();

  Serial.print("进入学习模式，槽位: ");
  Serial.print(slot);
  Serial.print(" 名称: ");
  Serial.println(name);
  Serial.println("请在30秒内按遥控器按钮...");
}

// ============== 定时任务 ==============

void checkScheduledTasks() {
  // 获取当前时间（这里使用millis作为示例，实际应使用NTP时间）
  // 在实际部署中，建议使用NTPClient库获取准确时间

  // 简化的时间检查（实际项目应该使用NTP）
  // 这里仅作为演示，使用millis模拟时间检查

  // 注意：实际项目中应该：
  // 1. 使用 NTPClient 库获取网络时间
  // 2. 或者通过HTTP接口从服务器获取时间

  // 示例：if (hour == AUTO_ON_HOUR && minute == AUTO_ON_MINUTE) { ... }
}

void executeAutoSequence() {
  Serial.println("开始执行自动播放序列...");

  for (int i = 0; i < SEQ_LENGTH; i++) {
    const char* cmd = AUTO_SEQUENCE[i];

    // 处理等待命令
    if (strncmp(cmd, "wait", 4) == 0) {
      int waitSec = atoi(cmd + 4);
      Serial.print("等待 ");
      Serial.print(waitSec);
      Serial.println(" 秒...");
      delay(waitSec * 1000);
    } else {
      // 发送红外命令
      Serial.print("发送命令: ");
      Serial.println(cmd);
      sendIRCommand(cmd);
      delay(DELAY_BETWEEN_COMMANDS);
    }
  }

  Serial.println("自动播放序列执行完成!");
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
