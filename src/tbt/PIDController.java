package tbt;

public class PIDController {

    private final float kp;
    private final float kd;
    private final float ki;
    private float desiredValue;
    private float windupvalue;
    private float derivative;
    private float integral;
    private float previousError;

    public PIDController(float kp, float kd, float ki, float desiredValue) {
        this.kp = kp;
        this.kd = kd;
        this.ki = ki;
        this.desiredValue = desiredValue;
        derivative = 0f;
        integral = 0f;
        previousError = 0f;
        windupvalue = 0.005f;
    }

    public float calculate(float measuredValue) {
        float error = desiredValue - measuredValue;

        if (Math.abs(error) < windupvalue) {//zero the integral if windup needed
            integral = 0;
        }

        integral = (integral * (2f / 3f)) + error;
        derivative = error - previousError;

        return kp * error + kd * derivative + ki * integral;
    }

    public void reset(){
        derivative = 0f;
        integral = 0f;
        previousError = 0f;
    }
}
