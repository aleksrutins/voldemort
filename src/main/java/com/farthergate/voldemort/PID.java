package com.farthergate.voldemort;

public class PID {
    public float kp;
    public float ki;
    public float kd;
    public float t;

    public float limMaxInt;
    public float limMinInt;

    public float integrator = 0f;
    public float prevError = 0f;
    public float differentiator = 0f;
    public float prevMeasurement = 0f;
    public float out = 0;

    public void update(float setpoint, float measurement) {
        float error = setpoint - measurement;

        float proportional = kp * error;

        integrator = integrator + 0.5f * ki * t * (error + prevError);

        if(integrator > limMaxInt) integrator = limMaxInt;
        else if(integrator < limMinInt) integrator = limMinInt;
    }
}
