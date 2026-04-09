#!/bin/bash

# إعداد المتغيرات (قم بتغيير الرابط أدناه إذا كان اسم المستخدم مختلفاً)
GITHUB_REPO_URL="https://github.com/rryfjkr/smart-incubator.git"

echo "🚀 بدء الإنشاء الشامل..."

# 1. إنشاء المجلد والملفات
mkdir -p smart-incubator && cd smart-incubator

# كتابة كود C++
cat > incubator_core.cpp << 'EOF'
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sstream>
#include <mutex>
#include <wiringPi.h>
#define DHT_PIN 4
#define HEATER_PIN 17
#define FAN_PIN 23
#define MOTOR_PIN 22
const char* SOCKET_PATH = "/tmp/incubator_socket";
std::mutex data_mutex;
double shared_temp = 0.0, shared_hum = 0.0, shared_pid = 0.0;
bool shared_heater = false, shared_fan = false, shared_motor = false;
bool readDHT22(float &t, float &h) {
    int bits[5] = {0};
    pinMode(DHT_PIN, OUTPUT); digitalWrite(DHT_PIN, LOW); delay(18);
    digitalWrite(DHT_PIN, HIGH); delayMicroseconds(40); pinMode(DHT_PIN, INPUT);
    if (digitalRead(DHT_PIN) == HIGH) return false;
    while (digitalRead(DHT_PIN) == LOW); while (digitalRead(DHT_PIN) == HIGH);
    for (int i = 0; i < 40; i++) {
        while (digitalRead(DHT_PIN) == LOW); unsigned long s = micros();
        while (digitalRead(DHT_PIN) == HIGH); if ((micros() - s) > 40) bits[i/8] |= (1 << (7-(i%8)));
    }
    if (bits[4] != ((bits[0]+bits[1]+bits[2]+bits[3]) & 0xFF)) return false;
    h = (float)((bits[0]*256+bits[1])/10.0); t = (float)(((bits[2]&0x7F)*256+bits[3])/10.0);
    if (bits[2]&0x80) t *= -1; return true;
}
class PID {
    double Kp=5.0, Ki=0.1, Kd=2.0, sp=37.8, prev_e=0, int_e=0;
public:
    double compute(double in) {
        double e = sp - in; int_e += e; if(int_e>100) int_e=100; if(int_e<-100) int_e=-100;
        double d = e - prev_e; double out = (Kp*e) + (Ki*int_e) + (Kd*d);        if(out>100) out=100; if(out<0) out=0; prev_e=e; return out;
    }
    void setSP(double v) { sp=v; int_e=0; }
} pid;
bool heater_st = false;
void control_loop() {
    if(wiringPiSetupGpio()<0){ std::cerr<<"GPIO Error\n"; exit(1); }
    pinMode(DHT_PIN, INPUT); pullUpDnControl(DHT_PIN, PUD_UP);
    pinMode(HEATER_PIN, OUTPUT); pinMode(FAN_PIN, OUTPUT); pinMode(MOTOR_PIN, OUTPUT);
    digitalWrite(HEATER_PIN, LOW); digitalWrite(FAN_PIN, LOW); digitalWrite(MOTOR_PIN, LOW);
    auto last_fan = std::chrono::steady_clock::now();
    while(true) {
        float t, h;
        if(readDHT22(t,h)) {
            double p = pid.compute(t);
            bool new_h = (p > 50.0);
            if(new_h != heater_st) { heater_st = new_h; digitalWrite(HEATER_PIN, heater_st?HIGH:LOW); }
            auto now = std::chrono::steady_clock::now();
            if(!shared_fan && std::chrono::duration_cast<std::chrono::seconds>(now-last_fan).count()>900) {
                shared_fan=true; digitalWrite(FAN_PIN, HIGH); last_fan=now;
            } else if(shared_fan && std::chrono::duration_cast<std::chrono::seconds>(now-last_fan).count()>30) {
                shared_fan=false; digitalWrite(FAN_PIN, LOW);
            }
            std::lock_guard<std::mutex> lock(data_mutex);
            shared_temp=t; shared_hum=h; shared_pid=p; shared_heater=heater_st;
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}
void socket_server() {
    int s_sock, c_sock; struct sockaddr_un addr; unlink(SOCKET_PATH);
    s_sock = socket(AF_UNIX, SOCK_STREAM, 0); memset(&addr,0,sizeof(addr));
    addr.sun_family = AF_UNIX; strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path)-1);
    bind(s_sock, (struct sockaddr*)&addr, sizeof(addr)); listen(s_sock, 1);
    while(true) {
        c_sock = accept(s_sock, NULL, NULL); if(c_sock<0) continue;
        char buf[256];
        while(true) {
            double t,h,p; bool ht,fn,mt;
            { std::lock_guard<std::mutex> lock(data_mutex); t=shared_temp; h=shared_hum; p=shared_pid; ht=shared_heater; fn=shared_fan; mt=shared_motor; }
            std::ostringstream oss; oss.precision(2); oss<<std::fixed<<t<<","<<h<<","<<p<<","<<(ht?1:0)<<","<<(fn?1:0)<<","<<(mt?1:0)<<"\n";
            std::string msg = oss.str(); if(send(c_sock, msg.c_str(), msg.length(), 0)<0) break;
            fd_set rfds; FD_ZERO(&rfds); FD_SET(c_sock, &rfds); struct timeval tv={0,100000};
            if(select(c_sock+1, &rfds, NULL, NULL, &tv)>0) {
                int n=recv(c_sock, buf, sizeof(buf)-1, 0); if(n<=0) break; buf[n]='\0';
                std::string cmd(buf); if(cmd.find("SET_TEMP:")!=std::string::npos) {
                    try{ pid.setSetpoint(std::stod(cmd.substr(9))); }catch(...){}
                }
            }
        } close(c_sock);    }
}
int main() { std::thread t1(control_loop); std::thread t2(socket_server); t1.join(); t2.join(); return 0; }
EOF

# كتابة كود Python
cat > web_server.py << 'EOF'
import socket, threading, time
from flask import Flask, render_template_string, jsonify, request
SOCKET_PATH = '/tmp/incubator_socket'
app = Flask(__name__)
data = {"temp":0.0, "hum":0.0, "pid_val":0.0, "heater":False, "fan":False, "motor":False, "target_temp":37.8, "connected":False}
lock = threading.Lock()
def connect_cpp():
    while True:
        client = None
        try:
            client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            client.connect(SOCKET_PATH)
            print("✅ Connected to C++")
            with lock: data["connected"] = True
            buf = ""
            while True:
                chunk = client.recv(1024).decode('utf-8')
                if not chunk: raise ConnectionError("Disconnected")
                buf += chunk
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    if not line: continue
                    parts = line.split(',')
                    if len(parts)>=6:
                        try:
                            with lock:
                                data["temp"]=float(parts[0]); data["hum"]=float(parts[1]); data["pid_val"]=float(parts[2])
                                data["heater"]=bool(int(parts[3])); data["fan"]=bool(int(parts[4])); data["motor"]=bool(int(parts[5]))
                        except: pass
                time.sleep(0.1)
        except Exception as e:
            print(f"❌ Error: {e}. Retrying...")
            with lock: data["connected"] = False
            time.sleep(5)
        finally:
            if client: 
                try: client.close()
                except: pass
@app.route('/')
def index():
    return render_template_string("""
    <!DOCTYPE html><html lang="ar" dir="rtl"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Smart Incubator</title>    <style>body{font-family:sans-serif;background:#f4f6f7;margin:0;padding:20px;color:#333}.container{max-width:600px;margin:0 auto;background:white;padding:20px;border-radius:12px;box-shadow:0 4px 12px rgba(0,0,0,0.1)}h1{text-align:center;color:#2c3e50}.status{text-align:center;margin-bottom:20px;font-size:0.9em}.on{color:#27ae60;font-weight:bold}.off{color:#e74c3c;font-weight:bold}.grid{display:grid;grid-template-columns:1fr 1fr;gap:15px}.card{background:#ecf0f1;padding:15px;border-radius:8px;text-align:center}.val{font-size:2em;font-weight:bold;color:#2980b9}.lbl{font-size:0.85em;color:#7f8c8d}.ind{display:inline-block;width:12px;height:12px;border-radius:50%;margin-right:5px}.active{background:#27ae60;box-shadow:0 0 5px #27ae60}.inactive{background:#bdc3c7}.controls{margin-top:20px;padding-top:15px;border-top:1px solid #ddd;text-align:center}input{padding:8px;width:60px;text-align:center;border-radius:4px;border:1px solid #ccc}button{padding:8px 16px;background:#3498db;color:white;border:none;border-radius:4px;cursor:pointer}button:hover{background:#2980b9}</style></head>
    <body><div class="container"><h1>🥚 Smart Incubator</h1><div class="status">C++ Core: <span id="conn" class="off">Connecting...</span></div>
    <div class="grid"><div class="card"><div class="lbl">Temp (°C)</div><div id="temp" class="val">--</div><div><span id="h-ind" class="ind inactive"></span>Heater</div></div>
    <div class="card"><div class="lbl">Humidity (%)</div><div id="hum" class="val">--</div><div><span id="f-ind" class="ind inactive"></span>Fan</div></div>
    <div class="card" style="grid-column: span 2;"><div class="lbl">PID Output (%)</div><div id="pid" class="val" style="font-size:1.5em">--</div></div></div>
    <div class="controls"><label>Target Temp:</label><input type="number" id="target" step="0.1" value="37.8"><button onclick="updateTarget()">Update</button></div></div>
    <script>function fetch_data(){fetch('/api/data').then(r=>r.json()).then(d=>{document.getElementById('temp').innerText=d.temp.toFixed(2);document.getElementById('hum').innerText=d.hum.toFixed(2);document.getElementById('pid').innerText=d.pid_val.toFixed(1);const c=document.getElementById('conn');c.innerText=d.connected?"Connected":"Disconnected";c.className=d.connected?"on":"off";document.getElementById('h-ind').className="ind "+(d.heater?"active":"inactive");document.getElementById('f-ind').className="ind "+(d.fan?"active":"inactive");if(document.activeElement.tagName!=="INPUT")document.getElementById('target').value=d.target_temp});}function updateTarget(){const v=document.getElementById('target').value;fetch('/api/set_temp',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({temp:v})}).then(()=>alert("Sent")); }setInterval(fetch_data,1000);fetch_data();</script></body></html>
    """)
@app.route('/api/data')
def get_data():
    with lock: return jsonify(data)
@app.route('/api/set_temp', methods=['POST'])
def set_temp():
    req = request.json
    val = req.get('temp')
    if val is None: return jsonify({"status":"error"}), 400
    try:
        client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        client.settimeout(2)
        client.connect(SOCKET_PATH)
        client.send(f"SET_TEMP:{val}".encode())
        client.close()
        with lock: data["target_temp"] = float(val)
        return jsonify({"status":"success"})
    except Exception as e:
        return jsonify({"status":"error", "msg":str(e)}), 500
if __name__ == '__main__':
    threading.Thread(target=connect_cpp, daemon=True).start()
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
EOF

# كتابة Makefile
printf "CC = g++\nCFLAGS = -std=c++11 -pthread -O2\nLDFLAGS = -lwiringPi\nTARGET = incubator_core\n\nall: \$(TARGET)\n\n\$(TARGET): incubator_core.cpp\n\t\$(CC) \$(CFLAGS) -o \$(TARGET) incubator_core.cpp \$(LDFLAGS)\n\nclean:\n\trm -f \$(TARGET)\n" > Makefile

# التثبيت والتجميع
echo "📦 تثبيت المكتبات..."
sudo apt update && sudo apt install wiringpi python3-pip git make g++ -y
pip3 install flask
echo "🔨 تجميع الكود..."
make

# رفع Git
echo " رفع المشروع إلى GitHub..."
git init
git add .
git commit -m "Initial commit: Smart Incubator System"
git branch -M main
git remote add origin $GITHUB_REPO_URL

echo "⚠️ الآن سيتم طلب كلمة المرور (استخدم Personal Access Token)."git push -u origin main

echo "✅ تم الانتهاء! المشروع مرفوع على حسابك."
