package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//Last Edited 10/27/2022 6:59PM MST

public class BotHardwareNew {
    //constantsNew constants = new constantsNew();

    /* Public OpMode members. */
    //MOTOR NULE DECLARATION
    public DcMotor fr = null;
    public DcMotor br = null;
    public DcMotor fl = null;
    public DcMotor bl = null;
    public DcMotor liftA = null;
    public DcMotor liftB = null;
    public DcMotor intake = null;
    //SENSORS

    //HARVESTER SERVOS
    public Servo L1;
    public Servo L2;
    public Servo planeS = null;
    public Servo wristL = null;
    public Servo wristR = null;
    public Servo heightS = null;

    //Blinkin//
    public RevBlinkinLedDriver lights;
    /* local OpMode members. */
    //DECLARING HARDWARE MAP AND A TIME SYSTEM
    HardwareMap hwMap = null;


    /* Constructor */
    public BotHardwareNew() {

    }


    /* Initialize standard Hardware interfaces */
    //SAYS IT CAN BE ACCESSED or transported
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
//        revBlinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, LED)
        // Define and Initialize Motors
        ElapsedTime waitTime = new ElapsedTime();
        fr = hwMap.get(DcMotor.class, "fr");
        br = hwMap.get(DcMotor.class, "br");
        fl = hwMap.get(DcMotor.class, "fl");
        bl = hwMap.get(DcMotor.class, "bl");
        liftA = hwMap.get(DcMotor.class, "liftA");
        liftB = hwMap.get(DcMotor.class, "liftB");
        intake = hwMap.get(DcMotor.class, "intake");
        //SERVOS
        wristL = hwMap.get(Servo.class, "wristL");
        wristR = hwMap.get(Servo.class, "wristR");
        planeS = hwMap.get(Servo.class, "plane");
        heightS = hwMap.get(Servo.class, "heightS");

        //SENSORS

        //Blinkin Driver
        lights = hwMap.get(RevBlinkinLedDriver.class, "lights");

        //SETING MOTOR DIRECTIONS
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        liftA.setDirection(DcMotor.Direction.FORWARD);
        liftB.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

//        liftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set all MOTOR zero power
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
        liftA.setPower(0);
        liftB.setPower(0);
        intake.setPower(0);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //HARVESTER SERVOS
        L1 = hwMap.get(Servo.class, "LIntake");
        L2 = hwMap.get(Servo.class, "RIntake");

        //wrist.setPosition(0);
        L1.setPosition(.735);
        L2.setPosition(.66);
        wristR.setPosition(.406);
        wristL.setPosition(.406);
        heightS.setPosition(.425);
    }


    //////////LIFT VALUES/////////////////
    public static double LIFT_POWER = 0.50;


    public static double lift = 1;

    ////////Height Positions///////////
    public static final double heightSHigh = .425;
    public static final double firstPixel = 0.60;
    public static final double secondPixel = .62;
    public static final double thirdPixel = 0;
    public static final double fourthPixel = 0;
    public static final double heightSLow = .68;

    //////////CLAW VALUES//////////////////
    public static final double OUTTAKEA_OPEN = 0.735;
    public static final double OUTTAKEB_OPEN = 0.66;
    public static final double OUTTAKEA_CLOSE = 0.193;
    public static final double OUTTAKEB_CLOSE = 0.1724;

    //////////INTAKE VALUES////////////////
    public static final double INTAKE_IN = -1.00;
    public static final double INTAKE_OUT = 1.00;

    /////////WRIST VALUES//////////////////
    public static final double WRIST_UP = 0.600;
    public static final double WRIST_DOWN = 0.406;
    public static final double LIFTENCODERTRIGGER = 1500;

    ////////DRIVE VALUE/////////////////////
    public static double x = 1.0;


    //SET MOTOR SPEEDS AND DIRECTIONS USUALLY WITH MATH ABS
    public void forward(double speed) {
        fr.setPower(Math.abs(speed));
        fl.setPower(Math.abs(speed));
        br.setPower(Math.abs(speed));
        bl.setPower(Math.abs(speed));
    }

    public void backward(double speed) {
        fr.setPower(-Math.abs(speed));
        fl.setPower(-Math.abs(speed));
        br.setPower(-Math.abs(speed));
        bl.setPower(-Math.abs(speed));
    }

    public void left(double speed) {
        fr.setPower(-Math.abs(speed));
        fl.setPower(Math.abs(speed));
        br.setPower(Math.abs(speed));
        bl.setPower(-Math.abs(speed));
    }

    public void right(double speed) {
        fr.setPower(Math.abs(speed));
        fl.setPower(-Math.abs(speed));
        br.setPower(-Math.abs(speed));
        bl.setPower(Math.abs(speed));
    }

    public void turnLeft(double speed) {
        fr.setPower(Math.abs(speed));
        fl.setPower(-Math.abs(speed));
        br.setPower(Math.abs(speed));
        bl.setPower(-Math.abs(speed));
    }

    public void turnRight(double speed) {
        fr.setPower(-Math.abs(speed));
        fl.setPower(Math.abs(speed));
        br.setPower(-Math.abs(speed));
        bl.setPower(Math.abs(speed));
    }

    public void BackwardLeft(double speed) {
        fr.setPower(-Math.abs(speed));
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(-Math.abs(speed));
    }

    public void stop() {
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }

    public void wristUp() {
        wristL.setPosition(WRIST_UP);
        wristR.setPosition(WRIST_UP);
    }

    public void wristDown() {
        wristL.setPosition(WRIST_DOWN);
        wristR.setPosition(WRIST_DOWN);
    }

    public void servo(boolean close, int servoNumber, boolean servoSideLeft) {
        if (servoNumber == 1) {
            if (!servoSideLeft) {
                if (close) {
                    L2.setPosition(OUTTAKEB_CLOSE);
                } else {
                    L2.setPosition(OUTTAKEB_OPEN);
                }
            }
            if (servoSideLeft) {
                if (close) {
                    L1.setPosition(OUTTAKEA_CLOSE);
                } else {
                    L1.setPosition(OUTTAKEA_OPEN);
                }
            }
        } else if (servoNumber == 2) {
            if (close) {
                L1.setPosition(OUTTAKEA_CLOSE);
                L2.setPosition(OUTTAKEB_CLOSE);
            } else {
                L1.setPosition(OUTTAKEA_OPEN);
                L2.setPosition(OUTTAKEB_OPEN);
            }
        }
    }
    public void servoOpenLightsBoth()
    {
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        L1.setPosition(OUTTAKEA_OPEN);
        L2.setPosition(OUTTAKEB_OPEN);
    }
    public void servoOpenLightsL1()
    {
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
        L1.setPosition(OUTTAKEA_OPEN);
    }
    public void servoOpenLightsL2()
    {
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
        L2.setPosition(OUTTAKEB_OPEN);
    }
    public void servoCloseLights()
    {
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        L1.setPosition(OUTTAKEA_CLOSE);
        L2.setPosition(OUTTAKEB_CLOSE);
    }
    ///////Height Control///////////
    public void high() {
        heightS.setPosition(heightSHigh);
    }
    public void firstPixel(){heightS.setPosition(firstPixel);}
    public void secondPixel(){heightS.setPosition(secondPixel);}
    public void thirdPixel(){heightS.setPosition(thirdPixel);}
    public void fourthPixel(){heightS.setPosition(fourthPixel);}
    public void low() {
        heightS.setPosition(heightSLow);
    }


}

