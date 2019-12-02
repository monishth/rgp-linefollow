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
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

import java.io.File;

public class LineFollowBot {

    public static final float BLACK = 0.07f;
    public static final float MID = 0.15f;
    public static final float WHITE = 0.24f;
    private static float kp = 1600f;
    private static float kd = 0f;
    private static float ki = 7.5f;
    private RegulatedMotor motorRight;
    private RegulatedMotor motorLeft;
    private RegulatedMotor ultrasoundMotor;
    private EV3ColorSensor colorSensor;
    private SampleProvider ultrasoundSensor;
    private float[] colorSample;
    private float[] ultrasoundSample;
    private float[] redSample;
    private int sw;
    private int sh;
    private GraphicsLCD g;
    private final PIDController lineFollowController;
    private final PIDController obstacleAvoidController;

    public LineFollowBot() {
        //Setup Motors
        motorRight = new EV3LargeRegulatedMotor(MotorPort.A);
        motorLeft = new EV3LargeRegulatedMotor(MotorPort.B);
        ultrasoundMotor = new EV3MediumRegulatedMotor(MotorPort.C);

        //Setup Sensors
        colorSensor = new EV3ColorSensor(SensorPort.S1);
        ultrasoundSensor = new EV3UltrasonicSensor(SensorPort.S2).getDistanceMode();

        //Setup LCD display
        g = BrickFinder.getDefault().getGraphicsLCD();
        sw = g.getWidth();
        sh = g.getHeight();

        //Create sample variables
        colorSample = new float[colorSensor.sampleSize()];
        ultrasoundSample = new float[ultrasoundSensor.sampleSize()];
        redSample = new float[colorSensor.sampleSize()];

        //Initial samples
        colorSensor.fetchSample(colorSample, 0);
        ultrasoundSensor.fetchSample(ultrasoundSample, 0);

        //Initialise speed controllers
        lineFollowController = new PIDController(kp, kd, ki, MID);
        obstacleAvoidController = new PIDController(2000, 0,0, 0.095f);
    }

    public static void main(String[] args) {
        LineFollowBot robot = new LineFollowBot();
        robot.lineFollow();
    }

    public void lineFollow() {

        int colorCheckCounter = 0; //Counter to check for the red stop line

        while (!Button.LEFT.isDown()) {

            colorCheckCounter++;
            if (colorCheckCounter > 10) { //Checks every 10 cycles for red
                colorSensor.setCurrentMode("ColorID"); //Switches color sensor to give colors instead of light intensity
                colorSensor.fetchSample(redSample, 0);
                Delay.msDelay(20);
                colorCheckCounter = 0;
                colorSensor.setCurrentMode("Red"); //Switches back to light intensity
            }

            if (redSample[0] != 0) { //If the color sensed is not red then continue following the line
                if (ultrasoundSample[0] > 0.1f) { //As long as there is no obstacle within 10cm of the robot continue
                    g.clear();
                    colorSensor.fetchSample(colorSample, 0);
                    ultrasoundSensor.fetchSample(ultrasoundSample, 0);
                    if (colorSample[0] < 0.4) {
                        float ultrasoundDistance = ultrasoundSample[0];
                        g.drawString(colorSample[0] + " : " + ultrasoundDistance + " m ", sw / 2, sh / 2, GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
                        g.refresh();

                        //set new speed using the newly collected data
                        setSpeed(lineFollowController.calculate(colorSample[0]), 220);

                        /*float error = MID - colorSample[0];
                        if (Math.abs(error) < 0.005f) {//zero the integral windup
                            integral = 0;
                        }

                        integral = (integral * (2f / 3f)) + error; //dampen the integral
                        derivative = error - previousError;

                        pidSpeed(error, derivative, integral, kp, kd, ki);
                    */}

                } else{//If an obstacle is closer than 0.1f then avoid it
                    avoidObstacle();
                }
            }else{//Stop the motors when red is sensed
                motorLeft.stop(true);
                motorRight.stop(true);
                //Delay.msDelay(1000);
                //Sound.playSample(new File("despacito.wav"));
            }
            Delay.msDelay(10); //keeps a 10sec delay as every cycle is not necessary
        }
        System.exit(0);
    }

    private void avoidObstacle() {
        Sound.twoBeeps();
        //Robot backs up
        g.clear();
        g.drawString("Obstacle Found\nBacking up", sw / 2, sh / 2, GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
        g.refresh();
        backUp();
        //Robot turns right
        g.clear();
        g.drawString("Obstacle Found\nTurn Right", sw / 2, sh / 2, GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
        g.refresh();
        turnRight();
        //Turns head left to follow obstacle
        g.clear();
        g.drawString("Obstacle Found\nTurn head", sw / 2, sh / 2, GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
        g.refresh();

        ultrasoundMotor.rotateTo(-90);

        colorSensor.fetchSample(colorSample, 0);
        ultrasoundSensor.fetchSample(ultrasoundSample, 0);

        while (Math.abs(colorSample[0]) >= 0.12f) { //Avoid obstacle until back on line
            colorSensor.fetchSample(colorSample, 0);
            ultrasoundSensor.fetchSample(ultrasoundSample, 0);
            g.clear();
            g.drawString("Obstacle Found\nTurning Around Obstacle\nDistance from obstacle" + ultrasoundSample[0], sw / 2, sh / 2, GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
            g.refresh();

            setSpeed(obstacleAvoidController.calculate(ultrasoundSample[0]), 220);

            /*float error = 0.095f - ultrasoundSample[0];
            if (error < -1) {
                error = -0.03f;
            }
            if (Math.abs(error) < 0.0005f) {//zero the integral windup
                integral = 0;
            }

            integral = (integral * (2f / 3f)) + error;
            derivative = error - previousError;

            pidSpeed(error, derivative, integral, 2000, 0, 0);
*/
            Delay.msDelay(10);
        }

        obstacleAvoidController.reset(); //Resets controller as the previous errors should not affect the next obstacle avoidance
        g.clear();
        g.drawString("Obstacle Found\nFound Line\nTurning head back", sw / 2, sh / 2, GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
        g.refresh();
        turnRight();

        Delay.msDelay(300);
        ultrasoundMotor.rotateTo(0);

    }

    private void backUp() {
        motorRight.setSpeed(150);
        motorLeft.setSpeed(150);
        motorRight.backward();
        motorLeft.backward();

        Delay.msDelay(400);

        motorRight.stop(true);
        motorLeft.stop(true);
    }

    private void turnRight() {
        motorRight.setSpeed(180);
        motorLeft.setSpeed(180);
        motorRight.backward();
        motorLeft.forward();

        Delay.msDelay(700);

        motorLeft.stop(true);
        motorRight.stop();
    }

    /*private void pidSpeed(float proportional, float derivative, float integral, float kp, float kd, float ki) {
        float turn = kp * proportional + kd * derivative + ki * integral;
        motorRight.setSpeed((int) (220 - turn));
        motorLeft.setSpeed((int) (220 + turn));

        motorRight.forward();
        motorLeft.forward();
    }*/

    private void setSpeed(float turn, float speed) {
        //limit speeds to > 0
        float right = speed - turn > 0 ? speed - turn : 0;
        float left = speed + turn > 0 ? speed + turn : 0;

        right = (speed - turn) > 2*speed ? 2*speed : (speed-turn);
        left = (speed + turn) > 2*speed ? 2*speed : (speed+turn);
        motorRight.setSpeed((int) right);
        motorLeft.setSpeed((int) left);

        motorRight.forward();
        motorLeft.forward();
    }
}
