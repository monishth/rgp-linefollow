package tbt;

import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LineFollowBot {

    public static final float BLACK = 0.07f;
    public static final float MID = 0.15f;
    public static final float WHITE = 0.24f;
    private static float kp = 1700f;
    private static float kd = 0f;
    private static float ki = 0f;
    private RegulatedMotor motorRight;
    private RegulatedMotor motorLeft;
    private RegulatedMotor ultrasoundMotor;
    private SensorMode colorSensor;
    private SampleProvider ultrasoundSensor;
    private float[] colorSample;
    private float[] ultrasoundSample;
    private int sw;
    private int sh;
    private GraphicsLCD g;

    public LineFollowBot() {
        motorRight = new EV3LargeRegulatedMotor(MotorPort.A);
        motorLeft = new EV3LargeRegulatedMotor(MotorPort.B);
        ultrasoundMotor = new EV3MediumRegulatedMotor(MotorPort.C);
        colorSensor = new EV3ColorSensor(SensorPort.S1).getRedMode();
        ultrasoundSensor = new EV3UltrasonicSensor(SensorPort.S2).getDistanceMode();
        g = BrickFinder.getDefault().getGraphicsLCD();
        sw = g.getWidth();
        sh = g.getHeight();
        colorSample = new float[colorSensor.sampleSize()];
        ultrasoundSample = new float[ultrasoundSensor.sampleSize()];
        colorSensor.fetchSample(colorSample, 0);
        ultrasoundSensor.fetchSample(ultrasoundSample, 0);
    }

    public static void main(String[] args) {
        LineFollowBot robot = new LineFollowBot();
        robot.lineFollow();
    }

    public void lineFollow() {

        float derivative = 0f;
        float integral = 0f;
        float previousError = 0f;


        while (!Button.LEFT.isDown()) {
            if (ultrasoundSample[0] > 0.1f) {
                g.clear();

                colorSensor.fetchSample(colorSample, 0);
                ultrasoundSensor.fetchSample(ultrasoundSample, 0);
                if (colorSample[0] < 0.4) {
                    float ultrasoundDistance = ultrasoundSample[0];
                    g.drawString(colorSample[0] + " : " + ultrasoundDistance + " m ", sw / 2, sh / 2, GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
                    g.refresh();

                    float error = MID - colorSample[0];

                    if (Math.abs(error) < 0.005f) {//zero the integral windup
                        integral = 0;
                    }

                    integral = (integral * (2f / 3f)) + error; //dampen the integral
                    derivative = error - previousError;

                    pidSpeed(error, derivative, integral, kp, kd, ki);
                }

            } else {
                avoidObstacle();
            }

            Delay.msDelay(10);

        }
    }

    private void avoidObstacle() {

        Sound.twoBeeps();
        turnRight();
        ultrasoundMotor.rotateTo(-90);
        float derivative = 0f;
        float integral = 0f;
        float previousError = 0f;

        colorSensor.fetchSample(colorSample, 0);
        ultrasoundSensor.fetchSample(ultrasoundSample, 0);

        while (Math.abs(colorSample[0]) >= 0.12f) {
            colorSensor.fetchSample(colorSample, 0);
            ultrasoundSensor.fetchSample(ultrasoundSample, 0);
            float error = 0.07f - ultrasoundSample[0];

            if (Math.abs(error) < 0.005f) {//zero the integral windup
                integral = 0;
            }

            integral = (integral * (2f / 3f)) + error;
            derivative = error - previousError;

            pidSpeed(error, derivative, integral, 2000, 0, 0);

            Delay.msDelay(10);
        }

        turnRight();

        Delay.msDelay(300);
        ultrasoundMotor.rotateTo(0);

    }

    private void turnRight() {
        motorRight.setSpeed(180);
        motorLeft.setSpeed(180);
        motorRight.backward();
        motorLeft.forward();
        Delay.msDelay(500);

        motorLeft.stop();
        motorRight.stop();
    }

    private void pidSpeed(float proportional, float derivative, float integral, float kp, float kd, float ki) {
        float turn = kp * proportional + kd * derivative + ki * integral;
        motorLeft.setSpeed((int) (180 - turn));
        motorRight.setSpeed((int) (180 + turn));

        motorRight.forward();
        motorLeft.forward();
    }
}
