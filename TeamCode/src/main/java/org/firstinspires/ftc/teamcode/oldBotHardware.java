package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Encoder;

//Last Edited 10/27/2022 6:59PM MST
@Disabled
public class oldBotHardware
{


    /* Public OpMode members. */
    //MOTOR NULE DECLARATION
    public DcMotor  fr   = null;
    public DcMotor  br   = null;
    public DcMotor  fl   = null;
    public DcMotor  bl   = null;
    public DcMotor  leftEncoder   = null;
    public DcMotor  rightEncoder   = null;
    public DcMotor  frontEncoder   = null;
     public DcMotor Harvest  = null;
     public DcMotor duckie = null;
    public DcMotor lift = null;

    //public Encoder leftEncoder, rightEncoder, frontEncoder;

    //    BNO055IMU imu;
    // public DcMotor    magnetArm = null;
//    public DistanceSensor distanceTower = null;
    //public DistanceSensor distancePole = null;
    //SENSOR DECLARATION
    // public DistanceSensor sensor_range = null;
    // public DistanceSensor Locator = null;


    //SERVO DECLARATION
    // public Servo    bucket  = null;
    //public Servo    Break = null;
    // public Servo    donkey  = null;
    public Servo    wrist = null;
    public Servo    wrist2 = null;
    public Servo    distancePoleServo = null;
    public Servo    encoderServo = null;
       public Servo    hand  = null;

    //HARVESTER SERVOS
    public CRServoImplEx    L1  = null;
    //    public CRServo    L2  = null;
    public CRServoImplEx R1  = null;
    public CRServo    R2  = null;


    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.98; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    private List<Integer> lastEncPositions;

    /* local OpMode members. */
    //DECLARING HARDWARE MAP AND A TIME SYSTEM
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    /* Constructor */
    public oldBotHardware(){

    }


    /* Initialize standard Hardware interfaces */
    //SAYS IT CAN BE ACCESSED or transported
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
//        revBlinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, LED)
        // Define and Initialize Motors
        fr  = hwMap.get(DcMotor.class, "fr");
        br = hwMap.get(DcMotor.class, "br");
        fl    = hwMap.get(DcMotor.class, "fl");
        bl  = hwMap.get(DcMotor.class, "bl");
        // Harvest = hwMap.get(DcMotor.class, "Harvest");
        // duckie = hwMap.get(DcMotor.class, "duckie");
       lift = hwMap.get(DcMotor.class, "lift");
        // magnetArm = hwMap.get(DcMotor.class,"magnetArm");
        //distancePole = hwMap.get(DistanceSensor.class, "distancePole");
//        distanceTower = hwMap.get(DistanceSensor.class, "ds");

        //SETING MOTOR DIRECTIONS
        fl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        // Harvest.setDirection(DcMotor.Direction.FORWARD);
        // duckie.setDirection(DcMotor.Direction.FORWARD);
//        lift.setDirection(DcMotor.Direction.FORWARD);
        //magnetArm.setDirection(CRServo.Direction.FORWARD);

//        leftEncoder = ahwMap.get(DcMotorEx.class, "leftEncoder");
//        rightEncoder = ahwMap.get(DcMotorEx.class, "rightEncoder");
//        frontEncoder = ahwMap.get(DcMotorEx.class, "frontEncoder");
//        rightEncoder.setDirection(DcMotor.Direction.REVERSE);

//        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set all MOTOR zero power
        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
//        lift.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Harvest.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // duckie.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //       lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //magnetArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        imu = hwMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);


        // // Define and initialize ALL installed servos.
        wrist  = hwMap.get(Servo.class, "wrist");
        distancePoleServo = hwMap.get(ServoImplEx.class, "dsServo");
        encoderServo  = hwMap.get(Servo.class, "encoderServo");

        //HARVESTER SERVOS
        L1  = hwMap.get(CRServoImplEx.class, "L1");
        R1  = hwMap.get(CRServoImplEx.class, "R1");

        wrist.setPosition(0);
        L1.setPower(0);
        R1.setPower(0);
        encoderServo.setPosition(0.41);
        wrist.setPosition(0.5);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

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
    public void BackwardLeft (double speed) {
        fr.setPower(-Math.abs(speed));
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(-Math.abs(speed));
    }
    public void stop(){
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }
//    ElapsedTime waitTime = new ElapsedTime();
//    public void driveForSeconds( double left, double right, int maxMS) {
//        int frontLeftcounts = 0;
//
//        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        fl.setPower(left);
//        bl.setPower(left);
//        fr.setPower(right);
//        br.setPower(right);
//
//        waitTime.reset();
//
//        while ((waitTime.milliseconds() < maxMS)) {
//            frontLeftcounts = fl.getCurrentPosition();
//
//        }
//
//        stop();
//
//
//    }
//    public void driveForDs(int maxMS, double left, double right, int distance) {
//        int frontLeftcounts = 0;
//
//        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        fl.setPower(left);
//        bl.setPower(left);
//        fr.setPower(right);
//        br.setPower(right);
//
//        waitTime.reset();
//
//        while ((waitTime.milliseconds() < maxMS) && distance>ds.getDistance(DistanceUnit.INCH)) {
//            frontLeftcounts = fl.getCurrentPosition();
//
//        }
//
//        stop();
//
//
//    }
//    public void driveForDsStrafe(int maxMS, double left, double right, int distance) {
//        int frontLeftcounts = 0;
//
//        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        left(left);
//
//        waitTime.reset();
//
//        while ((waitTime.milliseconds() < maxMS) && distance>ds.getDistance(DistanceUnit.INCH)) {
//            frontLeftcounts = fl.getCurrentPosition();
//
//        }
//
//        stop();
//
//
//    }


}