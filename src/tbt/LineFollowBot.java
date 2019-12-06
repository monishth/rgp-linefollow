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

    public static final float BLACK = 0.10f;
    public static final float MID = 0.14f;
    public static final float WHITE = 1.00f;
    public static final int INTERVAL = 20;
    private static float kp = 1200f;
    private static float kd = 20f;
    private static float ki = 100f;
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
    private PIDController lineFollowController;
    private final PIDController obstacleAvoidController;

    public LineFollowBot() {
        //Setup Motors
        motorRight = new EV3LargeRegulatedMotor(MotorPort.A);
        motorLeft = new EV3LargeRegulatedMotor(MotorPort.B);
        ultrasoundMotor = new EV3MediumRegulatedMotor(MotorPort.C);

        //Setup Sensors
        colorSensor = new EV3ColorSensor(SensorPort.S3);
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
        int changingConstant = 0;
        while (!Button.LEFT.isDown()) {

            if(Button.RIGHT.isDown()){
                changingConstant = (changingConstant+1)%3;
            }
            if(Button.UP.isDown()){
                switch (changingConstant) {
                    case 0:
                        kp += 100;
                        lineFollowController = new PIDController(kp, kd, ki, MID);
                        Delay.msDelay(100);
                        break;
                    case 1:
                        ki += 1;
                        lineFollowController = new PIDController(kp, kd, ki, MID);
                        Delay.msDelay(100);
                        break;
                    case 2:
                        kd += 1;
                        lineFollowController = new PIDController(kp, kd, ki, MID);
                        Delay.msDelay(100);
                        break;
                }
            }

            if(Button.DOWN.isDown()){
                switch (changingConstant) {
                    case 0:
                        kp -= 100;
                        lineFollowController = new PIDController(kp, kd, ki, MID);
                        Delay.msDelay(100);
                        break;
                    case 1:
                        ki -= 1;
                        lineFollowController = new PIDController(kp, kd, ki, MID);
                        Delay.msDelay(100);
                        break;
                    case 2:
                        kd -= 1;
                        lineFollowController = new PIDController(kp, kd, ki, MID);
                        Delay.msDelay(100);
                        break;
                }
            }

            g.clear();
            switch (changingConstant) {
                case 0:
                    g.drawString("kp = " + kp+"\n"+colorSample[0], sw / 2, sh / 2, GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
                    break;
                case 1:
                    g.drawString("ki = " + ki+"\n"+colorSample[0], sw / 2, sh / 2, GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
                    break;
                case 2:
                    g.drawString("kd = " + kd+"\n"+colorSample[0], sw / 2, sh / 2, GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
                    break;
            }
            g.refresh();

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

                        colorSensor.fetchSample(colorSample, 0);
                        ultrasoundSensor.fetchSample(ultrasoundSample, 0);

                        if (colorSample[0] < 0.4) {
                            float ultrasoundDistance = ultrasoundSample[0];


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
        turnRight(840);
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
            if (ultrasoundSample[0] >= 0.3){ ultrasoundSample[0] = 0.3f;}
            setSpeed(obstacleAvoidController.calculate(ultrasoundSample[0]), 220);

        }

        obstacleAvoidController.reset(); //Resets controller as the previous errors should not affect the next obstacle avoidance
        g.clear();
        g.drawString("Obstacle Found\nFound Line\nTurning head back", sw / 2, sh / 2, GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
        g.refresh();
        //backUp(600);
        turnRight(1340);
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

            setSpeed(lineFinder.calculate(colorSample[0]), 100);

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

    private void turnRight(int period) {
        motorRight.setSpeed(150);
        motorLeft.setSpeed(150);
        motorRight.backward();
        motorLeft.forward();

        Delay.msDelay(period);

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
