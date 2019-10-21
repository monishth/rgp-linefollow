package tbt;

import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;
import lejos.robotics.RegulatedMotor;

public class LineFollow {

    public static void main(String[] args) {
        RegulatedMotor motorRight = new EV3LargeRegulatedMotor(MotorPort.A);
        RegulatedMotor motorLeft = new EV3LargeRegulatedMotor(MotorPort.B);
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);

        GraphicsLCD g = BrickFinder.getDefault().getGraphicsLCD();
        final int SW =  g.getWidth();
        final int SH = g.getHeight();
        colorSensor.setCurrentMode("Red");
        colorSensor.setFloodlight(Color.RED);
        colorSensor.setFloodlight(true);
        float[] sample = new float[colorSensor.sampleSize()];


        while(!Button.LEFT.isDown()){
            g.clear();

            colorSensor.fetchSample(sample, 0);
            float lightLevel = sample[0];
            g.drawString(String.valueOf(sample[0]), SW/2, SH/2, GraphicsLCD.BASELINE|GraphicsLCD.HCENTER);
            g.refresh();

            if (lightLevel > 0.17){
                motorRight.stop(true);
                motorLeft.stop(true);
            }else if(lightLevel < 0.07){
                motorRight.setSpeed(360);
                motorLeft.setSpeed(50);
                motorLeft.forward();
                motorRight.forward();

            }else if(lightLevel >= 0.15){
                motorRight.setSpeed(50);
                motorLeft.setSpeed(360);
                motorLeft.forward();
                motorRight.forward();
            }

        }

        System.exit(0);

    }
}
