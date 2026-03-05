#ifndef CONTROLS_H
#define CONTROLS_H

#include <WiFi.h>
#include <WebServer.h>
#include "PID.h"

class RemoteTuner {
public:
    RemoteTuner(PIDController<float>& pitch, PIDController<float>& velocity, 
                float* livePitch, float* liveTarget, float* liveOutput,
                float* legExt, float* leftPct, float* rightPct, 
                float* leftAng, float* rightAng);

    void begin(const char* ssid, const char* password);
    void handle();

    float _targetMove;
    float _targetTurn;

private:
    WebServer server; // Use WebServer for ESP32
    PIDController<float>& _pitchPID;
    PIDController<float>& _velPID;
    
    // Live Telemetry Pointers
    float* _livePitch;
    float* _liveTarget;
    float* _liveOutput;

    // Servo/Leg Pointers
    float* _legExt;    // Read/Write
    float* _leftPct;   // Read Only
    float* _rightPct;  // Read Only
    float* _leftAng;   // Read Only
    float* _rightAng;  // Read Only


    void handleRoot();
    void handleUpdate();
    void handleData();
    void handleMove();
    String getHTML();
};

#endif