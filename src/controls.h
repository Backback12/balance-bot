#ifndef CONTROLS_H
#define CONTROLS_H

#include <WiFi.h>
#include <WebServer.h>
#include "PID.h"

class RemoteTuner {
public:
    // Added pointers to the live data we want to graph
    RemoteTuner(PIDController<float>& pitch, PIDController<float>& velocity, 
                float* livePitch, float* liveTarget, float* liveOutput);
    
    void begin(const char* ssid, const char* password);
    void handle();

private:
    WebServer server;
    PIDController<float>& _pitchPID;
    PIDController<float>& _velPID;
    
    // Pointers to Core 0 variables
    float* _livePitch;
    float* _liveTarget;
    float* _liveOutput;

    void handleRoot();
    void handleUpdate();
    void handleData(); // New endpoint for AJAX data
    String getHTML();
};

#endif