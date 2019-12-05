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

public class LineFollowBot {

    public static final float BLACK = 0.07f;
    public static final float MID = 0.155f;
    public static final float WHITE = 0.24f;
    public static final int INTERVAL = 20;
    private static float kp = 1500f;
    private static float kd = 0f;
    private static float ki = 20f;
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
        obstacleAvoidController = new PIDController(2000, 0, 0, 0.095f);
    }

    public static void main(String[] args) {
        LineFollowBot robot = new LineFollowBot();
        robot.lineFollow();
    }

    public void lineFollow() {

        boolean isStopped = false;
        int colorCheckCounter = 0; //Counter to check for the red stop line
        long lastTime = -1;
        while (!Button.LEFT.isDown()) {

            if (System.currentTimeMillis()-lastTime >= INTERVAL) {
                lastTime = System.currentTimeMillis();
                colorCheckCounter++;
                if (colorCheckCounter > 10) { //Checks every 10 cycles for red
                    colorSensor.setCurrentMode("ColorID"); //Switches color sensor to give colors instead of light intensity
                    colorSensor.fetchSample(redSample, 0);
                    Delay.msDelay(20);
                    colorCheckCounter = 0;
                    colorSensor.setCurrentMode("Red"); //Switches back to light intensity
                }

                if (redSample[0] != 0) { //If the color sensed is not red then continue following the line
                    isStopped = false;
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
                        }
                    } else {//If an obstacle is closer than 0.1f then avoid it
                        avoidObstacle();
                    }
                } else {//Stop the motors when red is sensed
                    motorRight.stop(true);
                    motorLeft.stop(true);
                    if (!isStopped) {
                        motorRight.setSpeed(150);
                        motorRight.backward();
                        Delay.msDelay(200);
                        motorRight.stop();
                        isStopped = true;
                    }
                }

                //Delay.msDelay(10); //keeps a 10sec delay as every cycle is not necessary
            }
        }
        System.exit(0);
    }

    private void avoidObstacle() {
        Sound.twoBeeps();
        //Robot backs up
        g.clear();
        g.drawString("Obstacle Found\nBacking up", sw / 2, sh / 2, GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
        g.refresh();
        backUp(400);
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

        while (Math.abs(colorSample[0]) >= 0.13f) { //Avoid obstacle until back on line
            colorSensor.fetchSample(colorSample, 0);
            ultrasoundSensor.fetchSample(ultrasoundSample, 0);
            g.clear();
            g.drawString(String.valueOf(ultrasoundSample[0]), sw / 2, sh / 2, GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
            g.refresh();
            if (ultrasoundSample[0] >= 1){ ultrasoundSample[0] = 0.12f;}
            setSpeed(obstacleAvoidController.calculate(ultrasoundSample[0]), 220);

        }

        obstacleAvoidController.reset(); //Resets controller as the previous errors should not affect the next obstacle avoidance
        g.clear();
        g.drawString("Obstacle Found\nFound Line\nTurning head back", sw / 2, sh / 2, GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
        g.refresh();
        //backUp(600);
        turnRight();
        backUp(100);
        Delay.msDelay(300);
        ultrasoundMotor.rotateTo(0);

        //Follow the line slower so it catches the line without going over it
        PIDController lineFinder = new PIDController(1000, 0, 0, MID);
        boolean lineFound = false;
        int counter = 0;
        while (counter < 250) {
            colorSensor.fetchSample(colorSample, 0);
            ultrasoundSensor.fetchSample(ultrasoundSample, 0);

            setSpeed(lineFinder.calculate(colorSample[0]), 50);

            if (colorSample[0] < 0.12f) lineFound = true;
            if (lineFound) counter++;
            Delay.msDelay(10);
        }

    }

    private void backUp(int period) {
        motorRight.setSpeed(150);
        motorLeft.setSpeed(150);
        motorRight.backward();
        motorLeft.backward();

        Delay.msDelay(period);

        motorRight.stop(true);
        motorLeft.stop();
    }

    private void turnRight() {
        motorRight.setSpeed(150);
        motorLeft.setSpeed(150);
        motorRight.backward();
        motorLeft.forward();

        Delay.msDelay(840);

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

        right = (speed - turn) > 2 * speed ? 2 * speed : (speed - turn);
        left = (speed + turn) > 2 * speed ? 2 * speed : (speed + turn);
        motorRight.setSpeed((int) right);
        motorLeft.setSpeed((int) left);

        motorRight.forward();
        motorLeft.forward();
    }
}
