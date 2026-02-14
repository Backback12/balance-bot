#include "controls.h"

RemoteTuner::RemoteTuner(PIDController<float>& pitch, PIDController<float>& velocity, 
                        float* livePitch, float* liveTarget, float* liveOutput) 
    : server(80), _pitchPID(pitch), _velPID(velocity), 
      _livePitch(livePitch), _liveTarget(liveTarget), _liveOutput(liveOutput) {}

void RemoteTuner::begin(const char* ssid, const char* password) {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\nIP: " + WiFi.localIP().toString());

    server.on("/", [this]() { handleRoot(); });
    server.on("/update", [this]() { handleUpdate(); });
    server.on("/data", [this]() { handleData(); }); // The "Live Feed"
    server.begin();
}

void RemoteTuner::handle() {
    server.handleClient();
}

void RemoteTuner::handleData() {
    // Return live values as JSON
    String json = "{";
    json += "\"pitch\":" + String(*_livePitch, 3) + ",";
    json += "\"target\":" + String(*_liveTarget, 3) + ",";
    json += "\"output\":" + String(*_liveOutput, 3);
    json += "}";
    server.send(200, "application/json", json);
}

void RemoteTuner::handleUpdate() {
    if (server.hasArg("p_p")) _pitchPID.setP(server.arg("p_p").toFloat());
    if (server.hasArg("p_i")) _pitchPID.setI(server.arg("p_i").toFloat());
    if (server.hasArg("p_d")) _pitchPID.setD(server.arg("p_d").toFloat());
    
    if (server.hasArg("v_p")) _velPID.setP(server.arg("v_p").toFloat());
    if (server.hasArg("v_i")) _velPID.setI(server.arg("v_i").toFloat());
    if (server.hasArg("v_d")) _velPID.setD(server.arg("v_d").toFloat());

    server.sendHeader("Location", "/");
    server.send(303);
}

void RemoteTuner::handleRoot() {
    server.send(200, "text/html", getHTML());
}

String RemoteTuner::getHTML() {
    String html = "<html><head><script src='https://cdn.jsdelivr.net/npm/chart.js'></script>";
    html += "<style>body{font-family:sans-serif;margin:20px;background:#f4f4f4;} .card{background:white;padding:15px;border-radius:10px;margin-bottom:20px;box-shadow:0 2px 5px rgba(0,0,0,0.1);}</style></head><body>";
    
    html += "<h2>Biped Tuner (3-Dec Precision)</h2>";

    // PID Tuning Forms
    html += "<div class='card'><form action='/update'>";
    html += "<h3>Pitch PID</h3>";
    html += "P: <input name='p_p' step='0.001' type='number' value='" + String(_pitchPID.getP(), 3) + "'> ";
    html += "I: <input name='p_i' step='0.001' type='number' value='" + String(_pitchPID.getI(), 3) + "'> ";
    html += "D: <input name='p_d' step='0.001' type='number' value='" + String(_pitchPID.getD(), 3) + "'><br>";
    
    html += "<h3>Velocity PID</h3>";
    html += "P: <input name='v_p' step='0.001' type='number' value='" + String(_velPID.getP(), 3) + "'> ";
    html += "I: <input name='v_i' step='0.001' type='number' value='" + String(_velPID.getI(), 3) + "'> ";
    html += "D: <input name='v_d' step='0.001' type='number' value='" + String(_velPID.getD(), 3) + "'><br><br>";
    html += "<input type='submit' value='Update PID' style='width:100%;padding:10px;background:#2ecc71;color:white;border:none;border-radius:5px;'>";
    html += "</form></div>";

    // Graph Container
    html += "<div class='card'><canvas id='liveChart' width='400' height='200'></canvas></div>";

    // JavaScript for Graphing
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
    html += "  });";
    html += "}, 200);"; // Update every 200ms
    html += "</script></body></html>";
    
    return html;
}