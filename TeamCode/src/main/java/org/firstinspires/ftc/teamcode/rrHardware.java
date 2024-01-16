package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//Last Edited 10/27/2022 6:59PM MST

public class rrHardware
{
    //constantsNew constants = new constantsNew();

    /* Public OpMode members. */
    //MOTOR NULE DECLARATION
    public DcMotor liftA = null;
    public DcMotor liftB = null;
    public DcMotor intake = null;
    public DistanceSensor ds = null;

    //HARVESTER SERVOS
    public Servo    L1;
    public Servo   L2;
    public Servo planeS = null;
    public Servo wristL;
    public Servo wristR;


    /* local OpMode members. */
    //DECLARING HARDWARE MAP AND A TIME SYSTEM
    HardwareMap hwMap           =  null;



    /* Constructor */
    public rrHardware(){

    }


    /* Initialize standard Hardware interfaces */
    //SAYS IT CAN BE ACCESSED or transported
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
//        revBlinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, LED)
        // Define and Initialize Motors
        ElapsedTime waitTime = new ElapsedTime();
        liftA = hwMap.get(DcMotor.class, "liftA");
        liftB = hwMap.get(DcMotor.class, "liftB");
//        planeS = hwMap.get(Servo.class, "plane");
        intake = hwMap.get(DcMotor.class, "intake");
        wristL = hwMap.get(Servo.class, "wristL");
        wristR = hwMap.get(Servo.class, "wristR");
        ds = hwMap.get(DistanceSensor.class, "ds");
        planeS = hwMap.get(Servo.class, "plane");
        //SETING MOTOR DIRECTIONS


        liftA.setDirection(DcMotor.Direction.FORWARD);
        liftB.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
//        liftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftA.setPower(0);
        liftB.setPower(0);
        intake.setPower(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //HARVESTER SERVOS
        L1  = hwMap.get(Servo.class, "LIntake");
        L2  = hwMap.get(Servo.class, "RIntake");

        //wrist.setPosition(0);
        L1.setPosition(.735);
        L2.setPosition(.66);
        wristR.setPosition(.403);
        wristL.setPosition(.403);
    }
    ///////////AUTON VALUES////////////////


    //////////LIFT VALUES/////////////////
    public static double LIFT_POWER = 0.50;
    public static final int LIFTA_LOW = 2000;
    public static final int LIFTA_MED = 3000;
    public static final int LIFTA_HIGH = 4500;

    public static double lift = 1;

    //////////CLAW VALUES//////////////////
    public static final double OUTTAKEA_OPEN = 0.735; //UPDATE THIS
    public static final double OUTTAKEB_OPEN = 0.66; //UPDATE THIS
    public static final double OUTTAKEA_CLOSE = 0.193; //UPDATE THIS
    public static final double OUTTAKEB_CLOSE = 0.1724; //UPDATE THIS

    //////////INTAKE VALUES////////////////
    public static final double INTAKE_IN = 1.00; //UPDATE THIS
    public static final double INTAKE_OUT = -1.00; //UPDATE THIS

    /////////WRIST VALUES//////////////////
    public static final double WRIST_UP = 0.600;
    public static final double WRIST_DOWN = 0.40;
    public static final double LIFTENCODERTRIGGER = 1000;

    ////////DRIVE VALUE/////////////////////
    public static double x= 1.0;

    public enum MotorStateA {
        LVLA,
        RESETA
    }
    public enum MotorStateB {
        LVLB,
        RESETB
    }
    boolean GoPosA = true;
    boolean GoPosB = true;
    MotorStateB motorStateB = MotorStateB.LVLB;
    MotorStateA motorStateA = MotorStateA.LVLA;
    public void UpdateA(int target) {
//        drive.lift.switchToLevel(target);
//        telemetry.update();
        liftA.setPower(LIFT_POWER);
        liftB.setPower(LIFT_POWER);
        StateUpdateA(true, target);
    }

    public void SetTargetPositionA(int poz) {
        liftA.setTargetPosition(poz);
        liftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void ResetA() {
        liftA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isBusyA() {
        return liftA.isBusy();
    }

    public void StateUpdateA(boolean IsAuto, int target) {
        switch (motorStateA) {
            case LVLA:
                Position_LvlA(target);
                break;
            case RESETA:
                if (IsAuto)
                    liftA.setPower(0);
                liftB.setPower(0);
                ResetA();

                break;
        }
        if (!GoPosA) {
            motorStateA = MotorStateA.RESETA;
        }
    }

    public void Position_LvlA(int poz2) {
        SetTargetPositionA(poz2);
        if (!isBusyA()) {
            GoPosA = false;
        }
    }

    // LIFT POSITIONS B


    public void UpdateLifts(int target1) {
        UpdateA(target1);
    }

    //SET MOTOR SPEEDS AND DIRECTIONS USUALLY WITH MATH ABS
    public void wristUp(){
        wristL.setPosition(WRIST_UP);
        wristR.setPosition(WRIST_UP);
    }
    public void wristDown(){
        wristL.setPosition(WRIST_DOWN);
        wristR.setPosition(WRIST_DOWN);
    }
    public void servo(boolean close, int servoNumber, boolean servoSideLeft) {
        if(servoNumber == 1) {
            if(!servoSideLeft) {
                if (close) {
                    L2.setPosition(OUTTAKEB_CLOSE);
                } else {
                    L2.setPosition(OUTTAKEB_OPEN);
                }
            }
            if(servoSideLeft) {
                if (close) {
                    L1.setPosition(OUTTAKEA_CLOSE);
                } else {
                    L1.setPosition(OUTTAKEA_OPEN);
                }
            }
        }
        else if(servoNumber == 2){
            if(close){
                L1.setPosition(OUTTAKEA_CLOSE);
                L2.setPosition(OUTTAKEB_CLOSE);
            }
            else{
                L1.setPosition(OUTTAKEA_CLOSE);
                L2.setPosition(OUTTAKEB_CLOSE);
            }
        }
    }


}