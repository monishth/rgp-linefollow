package tbt;

public class PIDController {

    private final float kp;
    private final float kd;
    private final float ki;
    private float desiredValue;
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
    }

    public float calculate(float measuredValue) {
        float error = desiredValue - measuredValue;

        float current = kp * error + kd * derivative + ki * integral;

        integral = (integral * (2f / 3f)) + error; //Integral windup. Each time the integral is calculated the previous integral has a smaller effect
        derivative = error - previousError;

        return current;
    }

    public void reset() {
        derivative = 0f;
        integral = 0f;
        previousError = 0f;
    }
}
