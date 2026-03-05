#include "controls.h"

RemoteTuner::RemoteTuner(PIDController<float>& pitch, PIDController<float>& velocity, 
                         float* livePitch, float* liveTarget, float* liveOutput,
                         float* legExt, float* leftPct, float* rightPct, 
                         float* leftAng, float* rightAng) 
    : server(80), _pitchPID(pitch), _velPID(velocity), 
      _livePitch(livePitch), _liveTarget(liveTarget), _liveOutput(liveOutput),
      _legExt(legExt), _leftPct(leftPct), _rightPct(rightPct),
      _leftAng(leftAng), _rightAng(rightAng) {}

void RemoteTuner::begin(const char* ssid, const char* password) {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\nIP: " + WiFi.localIP().toString());

    server.on("/", [this]() { handleRoot(); });
    server.on("/update", [this]() { handleUpdate(); });
    server.on("/data", [this]() { handleData(); }); 
    server.on("/move", [this]() { handleMove(); });

    server.begin();
}

void RemoteTuner::handle() {
    server.handleClient();
}

void RemoteTuner::handleData() {
    String json = "{";
    json += "\"pitch\":" + String(*_livePitch, 3) + ",";
    json += "\"target\":" + String(*_liveTarget, 3) + ",";
    json += "\"output\":" + String(*_liveOutput, 3) + ",";
    json += "\"legExt\":" + String(*_legExt) + ",";
    json += "\"lPct\":" + String(*_leftPct) + ",";
    json += "\"rPct\":" + String(*_rightPct) + ",";
    json += "\"lAng\":" + String(*_leftAng) + ",";
    json += "\"rAng\":" + String(*_rightAng);
    json += "}";
    server.send(200, "application/json", json);
}

void RemoteTuner::handleUpdate() {
    // PID Updates
    if (server.hasArg("p_p")) _pitchPID.setP(server.arg("p_p").toFloat());
    if (server.hasArg("p_i")) _pitchPID.setI(server.arg("p_i").toFloat());
    if (server.hasArg("p_d")) _pitchPID.setD(server.arg("p_d").toFloat());
    if (server.hasArg("v_p")) _velPID.setP(server.arg("v_p").toFloat());
    if (server.hasArg("v_i")) _velPID.setI(server.arg("v_i").toFloat());
    if (server.hasArg("v_d")) _velPID.setD(server.arg("v_d").toFloat());

    // Leg Extension Update
    if (server.hasArg("leg_ext")) *_legExt = (uint8_t)server.arg("leg_ext").toInt();

    server.sendHeader("Location", "/");
    server.send(303);
}

void RemoteTuner::handleMove() {
    if (server.hasArg("x")) _targetTurn = server.arg("x").toFloat();     // -1.0 to 1.0
    if (server.hasArg("y")) _targetMove = server.arg("y").toFloat();     // -1.0 to 1.0
    server.send(200, "text/plain", "OK");
}

void RemoteTuner::handleRoot() {
    server.send(200, "text/html", getHTML());
}

String RemoteTuner::getHTML() {
    String html = "<html><head><script src='https://cdn.jsdelivr.net/npm/chart.js'></script>";
    html += "<style>body{font-family:sans-serif;margin:20px;background:#f4f4f4;} .card{background:white;padding:15px;border-radius:10px;margin-bottom:20px;box-shadow:0 2px 5px rgba(0,0,0,0.1);} ";
    html += "input[type=range]{width:100%;} .val-label{font-weight:bold;color:#2c3e50;}</style></head><body>";
    
    html += "<h2>Biped Tuner & Actuator Control</h2>";

    // PID & Leg Extension Form
    html += "<div class='card'><form action='/update'>";
    html += "<h3>PID Tuning</h3>";
    html += "Pitch P/I/D: <input name='p_p' step='0.001' type='number' size='5' value='" + String(_pitchPID.getP(), 3) + "'> ";
    html += "<input name='p_i' step='0.001' type='number' size='5' value='" + String(_pitchPID.getI(), 3) + "'> ";
    html += "<input name='p_d' step='0.001' type='number' size='5' value='" + String(_pitchPID.getD(), 3) + "'><br><br>";
    
    html += "Velocity P/I/D: <input name='v_p' step='0.001' type='number' size='5' value='" + String(_velPID.getP(), 3) + "'> ";
    html += "<input name='v_i' step='0.001' type='number' size='5' value='" + String(_velPID.getI(), 3) + "'> ";
    html += "<input name='v_d' step='0.001' type='number' size='5' value='" + String(_velPID.getD(), 3) + "'>";

    html += "<h3>Leg Extension Control</h3>";
    html += "Extension: <input type='range' name='leg_ext' min='0' max='100' value='" + String(*_legExt) + "' oninput='this.nextElementSibling.value = this.value'> ";
    html += "<output class='val-label'>" + String(*_legExt) + "</output>%<br><br>";
    
    html += "<input type='submit' value='Apply Changes' style='width:100%;padding:10px;background:#2ecc71;color:white;border:none;border-radius:5px;'>";
    html += "</form></div>";

    // Read-Only Servo Status
    html += "<div class='card'><h3>Servo Feedback (Read-Only)</h3>";
    html += "Left Extension: <input type='range' id='lPctSlid' disabled> <span id='lPctVal' class='val-label'>0</span>%<br>";
    html += "Right Extension: <input type='range' id='rPctSlid' disabled> <span id='rPctVal' class='val-label'>0</span>%<br>";
    html += "Left Angle: <input type='range' id='lAngSlid' min='0' max='180' disabled> <span id='lAngVal' class='val-label'>0</span>&deg;<br>";
    html += "Right Angle: <input type='range' id='rAngSlid' min='0' max='180' disabled> <span id='rAngVal' class='val-label'>0</span>&deg;</div>";

    // Graph
    html += "<div class='card'><canvas id='liveChart' width='400' height='200'></canvas></div>";

    html += "<script>";
    html += "const ctx = document.getElementById('liveChart').getContext('2d');";
    html += "const chart = new Chart(ctx, {type:'line', data:{labels:[], datasets:[";
    html += "{label:'Pitch', borderColor:'red', data:[], fill:false},";
    html += "{label:'Target', borderColor:'blue', data:[], fill:false},";
    html += "{label:'Motor', borderColor:'green', data:[], fill:false}";
    html += "]}, options:{animation:false, scales:{y:{suggestedMin:-20, suggestedMax:20}}}});";
    
    html += "setInterval(() => {";
    html += "  fetch('/data').then(response => response.json()).then(data => {";
    html += "    const now = new Date().toLocaleTimeString();";
    html += "    if(chart.data.labels.length > 50) { chart.data.labels.shift(); chart.data.datasets.forEach(d => d.data.shift()); }";
    html += "    chart.data.labels.push(now);";
    html += "    chart.data.datasets[0].data.push(data.pitch);";
    html += "    chart.data.datasets[1].data.push(data.target);";
    html += "    chart.data.datasets[2].data.push(data.output);";
    html += "    chart.update();";
    // Update Read-Only UI
    html += "    document.getElementById('lPctSlid').value = data.lPct; document.getElementById('lPctVal').innerText = data.lPct;";
    html += "    document.getElementById('rPctSlid').value = data.rPct; document.getElementById('rPctVal').innerText = data.rPct;";
    html += "    document.getElementById('lAngSlid').value = data.lAng; document.getElementById('lAngVal').innerText = data.lAng;";
    html += "    document.getElementById('rAngSlid').value = data.rAng; document.getElementById('rAngVal').innerText = data.rAng;";
    html += "  });";
    html += "}, 200);"; 
    html += "document.addEventListener('keydown', (e) => {";
    html += "    let x = 0, y = 0;";
    html += "    if (e.key === 'ArrowUp') y = 0.8; ";
    html += "    if (e.key === 'ArrowDown') y = -0.8;";
    html += "    if (e.key === 'ArrowLeft') x = -0.5;";
    html += "    if (e.key === 'ArrowRight') x = 0.5;";
    html += "    if(x !== 0 || y !== 0) fetch(`/move?x=${x}&y=${y}`);";
    html += "});";
    html += "document.addEventListener('keyup', () => {";
    html += "    fetch(`/move?x=0&y=0`);";
    html += "});";
    html += "</script></body></html>";
    
    return html;
}