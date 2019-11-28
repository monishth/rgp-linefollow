package tbt;

import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.Color;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LineFollow {

    public static final float BLACK = 0.07f;
    public static final float MID = 0.15f;
    public static final float WHITE = 0.24f;
    private static float kp = 1400f;
    private static float kd = 0f;
    private static float ki = 0f;
    private static RegulatedMotor motorRight;
    private static RegulatedMotor motorLeft;
    private static RegulatedMotor ultrasoundMotor;
    private static SensorMode colorSensor;
    private static SampleProvider ultrasoundSensor;
    private static float[] sample;
    private static float[] ultrasoundSample;

    public static void main(String[] args) {
        motorRight = new EV3LargeRegulatedMotor(MotorPort.A);
        motorLeft = new EV3LargeRegulatedMotor(MotorPort.B);
        ultrasoundMotor = new EV3MediumRegulatedMotor(MotorPort.C);
        colorSensor = new EV3ColorSensor(SensorPort.S1).getRedMode();
        ultrasoundSensor = new EV3UltrasonicSensor(SensorPort.S2).getDistanceMode();
        //motorLeft.synchronizeWith(new RegulatedMotor[]{motorRight});
        GraphicsLCD g = BrickFinder.getDefault().getGraphicsLCD();
        final int SW =  g.getWidth();
        final int SH = g.getHeight();
        /*colorSensor.setCurrentMode("Red");
        colorSensor.setFloodlight(Color.RED);
        colorSensor.setFloodlight(true);*/
        sample = new float[colorSensor.sampleSize()];
        ultrasoundSample = new float[ultrasoundSensor.sampleSize()];
        colorSensor.fetchSample(sample, 0);
        ultrasoundSensor.fetchSample(ultrasoundSample, 0);

        float derivative = 0f;
        float integral = 0f;
        float previousError = 0f;


        while (true) {
            while(ultrasoundSample[0] > 0.1f){
                g.clear();

                colorSensor.fetchSample(sample, 0);
                ultrasoundSensor.fetchSample(ultrasoundSample, 0);
                float lightLevel = sample[0];
                float ultrasoundDistance = ultrasoundSample[0];
                g.drawString(lightLevel +" : " + ultrasoundDistance + " m ", SW/2, SH/2, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
                g.refresh();

                float error = MID - lightLevel;
                integral += error;
                derivative = error - previousError;

                pidSpeed(error, derivative, integral);

                Delay.msDelay(10);

            }

            avoidObstacle();
        }

    }

    private static void avoidObstacle() {
        ultrasoundMotor.rotateTo(-90);

        float derivative = 0f;
        float integral = 0f;
        float previousError = 0f;

        colorSensor.fetchSample(sample, 0);
        ultrasoundSensor.fetchSample(ultrasoundSample, 0);

        while(Math.abs(sample[0]) < 0.18f){
            float error = 0.05f - ultrasoundSample[0];
            integral += error;
            derivative = error - previousError;

            pidSpeed(error, derivative, integral);
            colorSensor.fetchSample(sample, 0);
            ultrasoundSensor.fetchSample(ultrasoundSample, 0);

        }

        Delay.msDelay(1000);
        ultrasoundMotor.rotateTo(0);

    }

    private static void pidSpeed(float proportional, float derivative, float integral){
        float speedOffset = kp * proportional + kd * derivative + ki * integral;
        motorRight.setSpeed((int) (180-speedOffset));
        motorLeft.setSpeed((int) (180+speedOffset));

        motorRight.forward();
        motorLeft.forward();
    }
}
