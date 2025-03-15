package com.farthergate.voldemort;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class PIDTest {

    /* Controller parameters */
    static final float PID_KP = 2.0f;
    static final float PID_KI = 0.5f;
    static final float PID_KD = 0.25f;

    static final float PID_TAU =0.02f;

    static final float PID_LIM_MIN =-10.0f;
    static final float PID_LIM_MAX = 10.0f;

    static final float PID_LIM_MIN_INT =-5.0f;
    static final float PID_LIM_MAX_INT = 5.0f;

    static final float SAMPLE_TIME_S = 0.01f;

    /* Maximum run-time of simulation */
    static final float SIMULATION_TIME_MAX = 4.0f;
    private class TestSystem {
        float output = 0.0f;
        float alpha = 0.02f;
        private float update(float inp) { 

            output = (SAMPLE_TIME_S * inp + output) / (1.0f + alpha * SAMPLE_TIME_S);

            return output;
        }
    }

    @Test
    void testPID() {
        PID pid = new PID(PID_KP, PID_KI, PID_KD, PID_TAU, PID_LIM_MIN, PID_LIM_MAX, PID_LIM_MIN_INT, PID_LIM_MAX_INT, SAMPLE_TIME_S);
        TestSystem system = new TestSystem();

        float setpoint = 1f;

        float acceptableError = 0.1f;
        float measurement = 0f;

        System.out.println("Time (s)\\tSystem Output\\tControllerOutput");

        for (float t = 0.0f; t <= SIMULATION_TIME_MAX; t += SAMPLE_TIME_S) {

            /* Get measurement from system */
            measurement = system.update(pid.out);
    
            /* Compute new control signal */
            pid.update(setpoint, measurement);
    
            System.out.printf("%f\t%f\t%f\n", t, measurement, pid.out);
    
        }

        assertEquals(Math.abs(measurement - setpoint) > acceptableError, false);
    }
}
