/*
 * ESP8266/ESP-12F 红外遥控智能控制器 (v3 - 内存优化版)
 * 功能：
 * 1. 红外学习功能 - 接收并存储红外遥控编码
 * 2. 操作链管理 - 简单操作链支持
 * 3. 定时任务 - 每天定时触发
 * 4. OTA 在线更新
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <IRutils.h>
#include <EEPROM.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <LittleFS.h>

// ============== 配置区域 ==============
const char* ssid = "Nuance";
const char* password = "Nuance2018";
const char* ntpServer = "pool.ntp.org";
const long gmtOffsetSec = 8 * 3600;

const uint16_t IR_RECEIVE_PIN = 14;
const uint16_t IR_SEND_PIN = 4;
const int MAX_CODES = 20;
const int EEPROM_SIZE = 512;
// ============== 配置结束 ==============

struct IRCode {
  char name[20];
  uint64_t code;
  uint16_t bits;
  bool valid;
};

struct Schedule {
  int hour;
  int minute;
  int codeIndex;  // -1 = power, 0-19 = 已学编码
  bool enabled;
  bool valid;
  bool triggered;
};

IRRecv irrecv(IR_RECEIVE_PIN);
IRsend irsend(IR_SEND_PIN);
ESP8266WebServer server(80);
ESP8266HTTPUpdateServer httpUpdater;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServer, gmtOffsetSec, 60000);

IRCode irCodes[MAX_CODES];
Schedule schedules[5];
bool learningMode = false;
int learningSlot = -1;
char learningName[20];
unsigned long learnStart = 0;
int lastMinute = -1;

void loadCodes();
void saveCodes();
void loadSchedules();
void saveSchedules();
void sendIR(int idx);
void handleRoot();
void handleLearn();
void handleStatus();
void handleList();
void handleSend();
void handleDelete();

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== IR Controller v3 ===");

  EEPROM.begin(EEPROM_SIZE);
  LittleFS.begin();
  loadCodes();
  loadSchedules();

  irrecv.enableIRIn();
  irsend.begin();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 30) {
    delay(500);
    Serial.print(".");
    tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    timeClient.begin();
    timeClient.update();
  }

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/learn", handleLearn);
  server.on("/list", handleList);
  server.on("/send", handleSend);
  server.on("/delete", handleDelete);
  httpUpdater.setup(&server);
  server.begin();

  Serial.println("Ready!");
}

void loop() {
  server.handleClient();
  timeClient.update();

  if (learningMode) {
    if (millis() - learnStart > 30000) {
      learningMode = false;
    } else {
      decode_results r;
      if (irrecv.decode(&r)) {
        if (learningSlot >= 0 && r.decode_type == NEC) {
          irCodes[learningSlot].code = r.value;
          irCodes[learningSlot].bits = r.bits;
          irCodes[learningSlot].valid = true;
          saveCodes();
          Serial.print("Saved: ");
          Serial.println(irCodes[learningSlot].name);
        }
        learningMode = false;
        irrecv.resume();
      }
    }
  }

  int m = timeClient.getMinutes();
  if (m != lastMinute) {
    lastMinute = m;
    int h = timeClient.getHours();
    Serial.print("Time: ");
    Serial.print(h);
    Serial.print(":");
    Serial.println(m);

    if (h == 0 && m == 0) {
      for (int i = 0; i < 5; i++) schedules[i].triggered = false;
    }

    for (int i = 0; i < 5; i++) {
      if (schedules[i].valid && schedules[i].enabled && !schedules[i].triggered) {
        if (schedules[i].hour == h && schedules[i].minute == m) {
          Serial.print("Trigger schedule ");
          Serial.println(i);
          schedules[i].triggered = true;
          if (schedules[i].codeIndex < 0) {
            irsend.sendNEC(0x20DF10EF, 32);
          } else if (schedules[i].codeIndex < MAX_CODES && irCodes[schedules[i].codeIndex].valid) {
            irsend.sendNEC(irCodes[schedules[i].codeIndex].code, irCodes[schedules[i].codeIndex].bits);
          }
        }
      }
    }
  }

  delay(10);
}

void loadCodes() {
  for (int i = 0; i < MAX_CODES; i++) irCodes[i].valid = false;
  File f = LittleFS.open("/codes.json", "r");
  if (!f) return;
  DynamicJsonDocument doc(2048);
  deserializeJson(doc, f);
  f.close();
  JsonArray arr = doc.as<JsonArray>();
  int idx = 0;
  for (JsonObject obj : arr) {
    if (idx >= MAX_CODES) break;
    strncpy(irCodes[idx].name, obj["name"] | "", 19);
    irCodes[idx].code = strtoull(obj["code"] | "0", NULL, 16);
    irCodes[idx].bits = obj["bits"] | 32;
    irCodes[idx].valid = obj["valid"] | false;
    idx++;
  }
}

void saveCodes() {
  DynamicJsonDocument doc(2048);
  JsonArray arr = doc.to<JsonArray>();
  for (int i = 0; i < MAX_CODES; i++) {
    if (irCodes[i].valid) {
      JsonObject obj = arr.createNestedObject();
      obj["name"] = irCodes[i].name;
      char s[32];
      sprintf(s, "0x%llx", irCodes[i].code);
      obj["code"] = s;
      obj["bits"] = irCodes[i].bits;
      obj["valid"] = true;
    }
  }
  File f = LittleFS.open("/codes.json", "w");
  if (f) { serializeJson(doc, f); f.close(); }
}

void loadSchedules() {
  for (int i = 0; i < 5; i++) {
    schedules[i].valid = false;
    schedules[i].triggered = false;
  }
  File f = LittleFS.open("/sched.json", "r");
  if (!f) return;
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, f);
  f.close();
  JsonArray arr = doc.as<JsonArray>();
  int idx = 0;
  for (JsonObject obj : arr) {
    if (idx >= 5) break;
    schedules[idx].hour = obj["h"] | 8;
    schedules[idx].minute = obj["m"] | 0;
    schedules[idx].codeIndex = obj["c"] | -1;
    schedules[idx].enabled = obj["e"] | true;
    schedules[idx].valid = true;
    idx++;
  }
}

void saveSchedules() {
  DynamicJsonDocument doc(1024);
  JsonArray arr = doc.to<JsonArray>();
  for (int i = 0; i < 5; i++) {
    if (schedules[i].valid) {
      JsonObject obj = arr.createNestedObject();
      obj["h"] = schedules[i].hour;
      obj["m"] = schedules[i].minute;
      obj["c"] = schedules[i].codeIndex;
      obj["e"] = schedules[i].enabled;
    }
  }
  File f = LittleFS.open("/sched.json", "w");
  if (f) { serializeJson(doc, f); f.close(); }
}

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width'>";
  html += "<title>IR Controller</title><style>";
  html += "body{font-family:Arial;padding:20px;background:#1a1a2e;color:#eee;max-width:600px;margin:0 auto;}";
  html += "h1{text-align:center;margin-bottom:20px;}";
  html += ".card{background:#2a2a4a;padding:15px;border-radius:10px;margin:10px 0;}";
  html += "input,button{padding:10px;margin:5px;border-radius:6px;border:none;}";
  html += "input{background:#1a1a2e;color:white;border:1px solid #444;width:150px;}";
  html += "button{background:#667eea;color:white;cursor:pointer;}";
  html += ".btn-danger{background:#f5576c;}";
  html += ".item{background:#1a1a2e;padding:10px;border-radius:6px;margin:5px 0;display:flex;justify-content:space-between;}";
  html += "</style></head><body>";

  html += "<h1>IR Controller v3</h1>";

  html += "<div class='card'>";
  html += "<div id='status'>Loading...</div>";
  html += "<p><a href='/update' style='color:#667eea;'>OTA Update</a></p>";
  html += "</div>";

  html += "<div class='card'><h3>Learn IR</h3>";
  html += "<input id='name' placeholder='Button name'>";
  html += "<button onclick='learn()'>Learn</button>";
  html += "<div id='learnStatus'></div></div>";

  html += "<div class='card'><h3>Codes</h3><div id='list'></div></div>";

  html += "<div class='card'><h3>Schedule</h3><div id='sched'></div></div>";

  html += "<script>";
  html += "function s(){fetch('/status').then(r=>r.json()).then(d=>{";
  html += "document.getElementById('status').innerHTML='WiFi: '+d.wifi+'<br>IP: '+d.ip+'<br>Time: '+d.t+'<br>Learning: '+d.l;";
  html += "})}";
  html += "function l(){fetch('/list').then(r=>r.json()).then(d=>{";
  html += "var h='';for(var i=0;i<d.codes.length;i++){";
  html += "h+=\"<div class='item'><span>\"+d.codes[i].name+\"</span><div>\";";
  html += "h+=\"<button onclick=\\\"send(\\'\"+d.codes[i].name+\"\\')\\\">Send</button> \";";
  html += "h+=\"<button class='btn-danger' onclick=\\\"del(\\'\"+d.codes[i].name+\"\\')\\\">Del</button>\";";
  html += "h+=\"</div></div>\";}document.getElementById('list').innerHTML=h||'<p>No codes</p>';";
  html += "h='';for(var i=0;i<d.sched.length;i++){";
  html += "var s=d.sched[i];h+=\"<div class='item'><span>\"+s.h+\":\"+String(s.m).padStart(2,'0')+\"</span>\";";
  html += "h+=\"<span>\"+(s.e?'On':'Off')+\"</span></div>\";}";
  html += "document.getElementById('sched').innerHTML=h||'<p>No schedules</p>';";
  html += "})}";
  html += "function learn(){var n=document.getElementById('name').value;if(!n)return;";
  html += "document.getElementById('learnStatus').innerHTML='Learning...';";
  html += "fetch('/learn?name='+encodeURIComponent(n));}";
  html += "function send(n){fetch('/send?name='+encodeURIComponent(n));}";
  html += "function del(n){if(confirm('Delete?')){fetch('/delete?name='+encodeURIComponent(n)).then(l);}}";
  html += "setInterval(s,3000);s();l();";
  html += "</script>";

  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleStatus() {
  String json = "{";
  json += "\"wifi\":\"" + String(WiFi.status() == WL_CONNECTED ? "OK" : "No") + "\",";
  json += "\"ip\":\"" + String(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "-") + "\",";
  json += "\"t\":\"" + timeClient.getFormattedTime() + "\",";
  json += "\"l\":" + String(learningMode ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void handleLearn() {
  if (!server.hasArg("name")) { server.send(400, "text/plain", "ERR"); return; }
  String name = server.arg("name");
  int slot = -1;
  for (int i = 0; i < MAX_CODES; i++) {
    if (irCodes[i].valid && strcmp(irCodes[i].name, name.c_str()) == 0) { slot = i; break; }
  }
  if (slot == -1) {
    for (int i = 0; i < MAX_CODES; i++) {
      if (!irCodes[i].valid) { slot = i; break; }
    }
  }
  if (slot == -1) { server.send(500, "text/plain", "FULL"); return; }
  strncpy(irCodes[slot].name, name.c_str(), 19);
  irCodes[slot].name[19] = '\0';
  learningSlot = slot;
  learningMode = true;
  learnStart = millis();
  server.send(200, "text/plain", "OK");
}

void handleList() {
  DynamicJsonDocument doc(2048);
  JsonArray codes = doc.createNestedArray("codes");
  JsonArray sched = doc.createNestedArray("sched");
  for (int i = 0; i < MAX_CODES; i++) {
    if (irCodes[i].valid) {
      JsonObject o = codes.createNestedObject();
      o["name"] = irCodes[i].name;
    }
  }
  for (int i = 0; i < 5; i++) {
    if (schedules[i].valid) {
      JsonObject o = sched.createNestedObject();
      o["h"] = schedules[i].hour;
      o["m"] = schedules[i].minute;
      o["e"] = schedules[i].enabled;
    }
  }
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleSend() {
  if (!server.hasArg("name")) { server.send(400, "text/plain", "ERR"); return; }
  String name = server.arg("name");
  for (int i = 0; i < MAX_CODES; i++) {
    if (irCodes[i].valid && strcmp(irCodes[i].name, name.c_str()) == 0) {
      irsend.sendNEC(irCodes[i].code, irCodes[i].bits);
      break;
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleDelete() {
  if (!server.hasArg("name")) { server.send(400, "text/plain", "ERR"); return; }
  String name = server.arg("name");
  for (int i = 0; i < MAX_CODES; i++) {
    if (irCodes[i].valid && strcmp(irCodes[i].name, name.c_str()) == 0) {
      irCodes[i].valid = false;
      saveCodes();
      break;
    }
  }
  server.send(200, "text/plain", "OK");
}
