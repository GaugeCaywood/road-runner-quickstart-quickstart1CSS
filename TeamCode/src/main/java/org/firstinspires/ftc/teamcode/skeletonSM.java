//package org.firstinspires.ftc.teamcode;
//import android.widget.FrameLayout;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
////import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
//import java.util.List;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.vuforia.Vuforia;
//
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//@Disabled
//@Autonomous(name="skeletonSM", group="Auton")
//public class skeletonSM extends LinearOpMode {
//    BotHardware robot = new BotHardware();
//
//
//    public enum MotorStateA {
//        LVLA,
//        RESETA
//    }
//
//    ;
//
//    public enum MotorStateB {
//        LVLB,
//        RESETB
//    }
//    //POWERS FOR LIFT CONSTANT & SERVO
//    private static final double LIFT_POWER = 0.50;
//    private static final double SERVO_POWER = 0.50;
//    //VARIABLES FOR PURPLE PIXEL SCORE ON LINe
//    private static final double MOTOR_POWER = 0.50;
//    private static final long PIXEL_PLACEMENT_FORWARD = 1000;  // Time in milliseconds
//    private static final long PIXEL_PLACEMENT_LEFT = 250;
//    private static final long PIXEL_PLACEMENT_RIGHT = 250;
//    private static final long PIXEL_PLACE = 100;
//    private static final double LEFT_PIXEL_PLACEMENT_SPEED = 0.25;
//    private static final double PIXEL_PLACEMENT_RIGHT_SPEED = 0.25;
//    //VARIABLES FOR YELLOW PIXEL SCORE ON BOARD
//    private static final double LEFT_PIXEL_SCORE_SPEED = 0.25;
//    private static final long PIXEL_SCORE_FORWARD = 2000;
//    private static final long LIFTENGAGE = 1000;
//    private static final long PIXEL_SCORE_LEFT = 250;
//    private static final double RIGHT_PIXEL_SCORE_SPEED = 0.25;
//    private static final long PIXEL_SCORE_RIGHT = 250;
//    private static final long PIXEL_SCORE = 250;
//    private static final long PIXEL_SCORE_TURNLEFT = 250;
//    private static final double TURN_LEFT_SPEED = 0.25;
//    private int detectedLine = -1;
//    //MODEL FILE NAME CHANGE FOR OTHER MODELS
//
//    // Function to detect the blue block and determine the line during init
//
//    void detectBlueBlockDuringInit() {
//
//
//        List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
//        if (updatedRecognitions != null) {
//            for (Recognition recognition : updatedRecognitions) {
//                if (recognition.getLabel().equals("blueCube")) {
//                    float blockX = recognition.getLeft();
//
//                    //tune
//                    if (blockX < 4/*replace with tuned number*/) {
//                        detectedLine = 1;
//                    } else if (blockX < 6 && blockX > 4/*replace with tuned number*/) {
//                        detectedLine = 2;
//                    } else {
//                        detectedLine = 3;
//                    }
//                }
//            }
//        }
//    }
//
//    ;
//
//    boolean GoPosA = true;
//    boolean GoPosB = true;
//    MotorStateB motorStateB = MotorStateB.LVLB;
//    MotorStateA motorStateA = MotorStateA.LVLA;
//    double targetA = 0;
//    double targetB = 0;
//    //TOWER ARRAY
//
//
//    ElapsedTime liftTimer = new ElapsedTime();
//
//    //Sensor variables
//
//
//    //Lift variables
//
//
//    // LIFT POSITIONS
//    public void UpdateA(int target) {
////        drive.lift.switchToLevel(target);
//        telemetry.update();
//        robot.liftA.setPower(LIFT_POWER);
//        StateUpdateA(true, target);
//    }
//
//    public void SetTargetPositionA(int poz) {
//        robot.liftA.setTargetPosition(poz);
//        robot.liftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//
//    public void ResetA() {
//        robot.liftA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    public boolean isBusyA() {
//        return robot.liftA.isBusy();
//    }
//
//    public void StateUpdateA(boolean IsAuto, int target) {
//        switch (motorStateA) {
//            case LVLA:
//                Position_LvlA(target);
//                break;
//            case RESETA:
//                if (IsAuto) robot.liftA.setPower(0);
//                ResetA();
//                break;
//        }
//        if (!GoPosA) {
//            motorStateA = MotorStateA.RESETA;
//        }
//    }
//
//    public void Position_LvlA(int poz2) {
//        SetTargetPositionA(poz2);
//        if (!isBusyA()) {
//            GoPosA = false;
//        }
//    }
//
//    // LIFT POSITIONS
//    public void UpdateB(int target) {
////        drive.lift.switchToLevel(target);
//        telemetry.update();
//        robot.liftB.setPower(LIFT_POWER);
//        StateUpdateB(true, target);
//    }
//
//    public void SetTargetPositionB(int poz) {
//        robot.liftB.setTargetPosition(poz);
//        robot.liftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//
//    public void ResetB() {
//        robot.liftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    public boolean isBusyB() {
//        return robot.liftB.isBusy();
//    }
//
//    public void StateUpdateB(boolean IsAuto, int target) {
//        switch (motorStateB) {
//            case LVLB:
//                Position_LvlB(target);
//                break;
//            case RESETB:
//                if (IsAuto) robot.liftB.setPower(0);
//                ResetB();
//                break;
//        }
//        if (!GoPosB) {
//            motorStateB = MotorStateB.RESETB;
//        }
//    }
//
//    public void Position_LvlB(int poz2) {
//        SetTargetPositionB(poz2);
//        if (!isBusyB()) {
//            GoPosB = false;
//        }
//    }
//
//    public void UpdateLifts(int target) {
//        UpdateA(target);
//        UpdateB(target);
//    }
//
//    private void moveForwardAndPlacePixel(int line) {
//
//        robot.forward(MOTOR_POWER);
//
//
//        ElapsedTime timer = new ElapsedTime();
//
//        while (opModeIsActive() && timer.milliseconds() < PIXEL_PLACEMENT_FORWARD) {
//
//            idle();
//
//
//        }
//        robot.stop();
//        if(detectedLine == 1){
//            timer.reset();
//            robot.left(LEFT_PIXEL_PLACEMENT_SPEED);
//            while (opModeIsActive() && timer.milliseconds() < PIXEL_PLACEMENT_LEFT) {
//
//                idle();
//
//
//            }
//            robot.stop();
//        }
//        else if(detectedLine == 3){
//            robot.right(PIXEL_PLACEMENT_RIGHT_SPEED);
//        }
//
//        robot.wrist.setPosition(.178);
//        timer.reset();
//
//        while (opModeIsActive() && timer.milliseconds() < PIXEL_PLACE) {
//            // Allow the program to run other tasks during the waiting period
//            idle();
//
//        }
//        robot.claw.setPosition(.5);
//        robot.wrist.setPosition(.373);
//
//        while (opModeIsActive() && timer.milliseconds() < PIXEL_PLACE) {
//            // Allow the program to run other tasks during the waiting period
//            idle();
//
//
//        }
//
//    }
//    public void turnMoveForwardPlacePixel(int line){
//        ElapsedTime timer = new ElapsedTime();
//        robot.turnLeft(TURN_LEFT_SPEED);
//        resetRuntime();
//        while(opModeIsActive() && timer.milliseconds() < PIXEL_SCORE_TURNLEFT){
//            idle();
//
//        }
//        robot.stop();
//        resetRuntime();
//        robot.forward(MOTOR_POWER);
//
//        // Track the start time
//
//
//        while (opModeIsActive() && timer.milliseconds() < PIXEL_SCORE_FORWARD) {
//            // Allow the program to run other tasks during the waiting period
//            idle();
//
//
//        }
//        robot.stop();
//        UpdateLifts(-200);
//        timer.reset();
//        while(opModeIsActive() && timer.milliseconds() < LIFTENGAGE && (robot.liftB.getCurrentPosition() + robot.liftA.getCurrentPosition() < 175)){
//            idle();
//            UpdateLifts(-200);
//        }
//        if(detectedLine == 1){
//            timer.reset();
//
//            robot.left(LEFT_PIXEL_SCORE_SPEED);
//            timer.reset();
//            while (opModeIsActive() && timer.milliseconds() < PIXEL_SCORE_LEFT) {
//                // Allow the program to run other tasks during the waiting period
//                idle();
//
//            }
//            robot.stop();
//        }
//        else if(detectedLine == 3){
//            robot.right(RIGHT_PIXEL_SCORE_SPEED);
//            timer.reset();
//            while (opModeIsActive() && timer.milliseconds() < PIXEL_SCORE_RIGHT){
//                // Allow the program to run other tasks during the waiting period
//                idle();
//
//            }
//            robot.stop();
//        }
//
//        robot.L2.setPosition(0.827);
//        robot.L1.setPosition(0.821);
//        timer.reset();
//
//        while (opModeIsActive() && timer.milliseconds() < PIXEL_SCORE) {
//            // Allow the program to run other tasks during the waiting period
//            idle();
//
//        }
//
//        robot.L2.setPosition(0.449);
//        robot.L1.setPosition(0.449);
//    }
//    @Override
//    public void runOpMode() {
//
//        UpdateLifts(-200);
//
//
//    }
//}