package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.tseSaturationProcessor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.vision.VisionPortal;

import android.util.Size;
@Disabled
@Autonomous(name="redHumanPlayerSide", group="Auton")
public class redHPSide extends LinearOpMode {
    BotHardware robot = new BotHardware();
    Constants constant = new Constants();
    private VisionPortal visionPortal;
    private tseSaturationProcessor visionProcessor;
    tseSaturationProcessor.Selected m_tse = tseSaturationProcessor.Selected.NONE;

    // you may also want to take a look at some of the examples for instructions on
    // how to have a switchable camera (switch back and forth between two cameras)
    // or how to manually edit the exposure and gain, to account for different lighting conditions
    // these may be extra features for you to work on to ensure that your robot performs
    // consistently, even in different environments


    //POWERS FOR LIFT CONSTANT & SERVO

    private int detectedLine = -1;
    boolean searchTSE = false;
    //MODEL FILE NAME CHANGE FOR OTHER MODELS

    // Function to detect the blue block and determine the line during init

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
//                    if (blockX < 200/*replace with tuned number*/) {
//                        detectedLine = 1;
//                    } else if (blockX < 200 && blockX > 400/*replace with tuned number*/) {
//                        detectedLine = 2;
//                    } else {
//                        detectedLine = 3;
//                    }
//                }
//            }
//        }


    ;

    //TOWER ARRAY


    ElapsedTime liftTimer = new ElapsedTime();

    //Sensor variables


    //Lift variables


    // LIFT POSITIONS


    private void moveForwardAndPlacePixeleft(int line) {
        robot.wrist.setPosition(constant.WRIST_MEDIUM);
        forward(constant.MOTOR_POWER, constant.PIXEL_PLACEMENT_FORWARD, 0);


        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.milliseconds() < constant.PIXEL_PLACEMENT_FORWARD) {




        }
        timer.reset();
        turnLeft(constant.LEFT_PIXEL_PLACEMENT_SPEED, constant.PIXEL_PLACEMENT_LEFT, 0);
        while (opModeIsActive() && timer.milliseconds() < constant.PIXEL_PLACEMENT_LEFT) {




        }


        robot.stop();
        robot.claw.setPosition(constant.CLAW_OPEN);
        robot.wrist.setPosition(constant.WRIST_DOWN);
        timer.reset();
        while(timer.milliseconds()< 1000){
            idle();
        }

    }
//    private void moveForwardAndPlacePixelRight(int line) {
//        robot.wrist.setPosition(constant.WRIST_MEDIUM);
//        forward(constant.MOTOR_POWER, constant.PIXEL_PLACEMENT_FORWARD, 0);
//
//
//        ElapsedTime timer = new ElapsedTime();
//
//        while (opModeIsActive() && timer.milliseconds() < constant.PIXEL_PLACEMENT_FORWARD) {
//
//            idle();
//
//
//        }
//        timer.reset();
//        turnLeft(constant.LEFT_PIXEL_PLACEMENT_SPEED, constant.PIXEL_PLACEMENT_LEFT, 0);
//        while (opModeIsActive() && timer.milliseconds() < constant.PIXEL_PLACEMENT_LEFT) {
//
//            idle();
//
//
//        }
//
//
//        robot.stop();
//        robot.claw.setPosition(.6349);
//        robot.wrist.setPosition(0.178);
//        timer.reset();
//        while(timer.milliseconds()< 1000){
//            idle();
//        }
//
//    }
//
//
//
//    private void moveForwardAndPlacePixelMiddle(int line) {
//        //robot.wrist.setPosition(.373);
//        robot.wrist.setPosition(.7);
//        forward(constant.MOTOR_POWER, constant.PIXEL_PLACEMENT_FORWARD, 0);
//
//
//        ElapsedTime timer = new ElapsedTime();
//
//
//        robot.stop();
//
//        robot.stop();
//        while (opModeIsActive() && timer.milliseconds() < constant.PIXEL_PLACEMENT_LEFT) {
//
//            idle();
//
//
//        }
//
//
//        timer.reset();
//
//        while (opModeIsActive() && timer.milliseconds() < constant.PIXEL_PLACE) {
//            // Allow the program to run other tasks during the waiting period
//            idle();
//
//        }
//        robot.claw.setPosition(.5);
//        while (opModeIsActive() && timer.milliseconds() < constant.PIXEL_PLACE) {
//            // Allow the program to run other tasks during the waiting period
//            idle();
//
//        }
//
//
//        while (opModeIsActive() && timer.milliseconds() < constant.PIXEL_PLACE) {
//            // Allow the program to run other tasks during the waiting period
//            idle();
//
//
//        }
//
//    }
//    public void turnMoveForwardPlacePixelLeft(int line){
//        ElapsedTime timer = new ElapsedTime();
//
//        resetRuntime();
//        while(opModeIsActive() && timer.milliseconds() < constant.PIXEL_SCORE_TURNLEFT){
//            idle();
//
//        }
//        robot.stop();
//        resetRuntime();
//        robot.forward(constant.MOTOR_POWER);
//
//        // Track the start time
//
//
//        while (opModeIsActive() && timer.milliseconds() < constant.PIXEL_SCORE_FORWARD) {
//            // Allow the program to run other tasks during the waiting period
//            idle();
//
//
//        }
//        robot.stop();
//        UpdateLifts(-200);
//        timer.reset();
//        while(opModeIsActive() && timer.milliseconds() < constant.LIFTENGAGE && (robot.liftB.getCurrentPosition() + robot.liftA.getCurrentPosition() < 175)){
//            idle();
//            UpdateLifts(-200);
//        }
//
//        timer.reset();
//
//        robot.left(constant.LEFT_PIXEL_SCORE_SPEED);
//        timer.reset();
//        while (opModeIsActive() && timer.milliseconds() < constant.PIXEL_SCORE_LEFT) {
//            // Allow the program to run other tasks during the waiting period
//            idle();
//
//        }
//        robot.stop();
//
//
//
//        robot.L2.setPosition(0.827);
//        robot.L1.setPosition(0.821);
//        timer.reset();
//
//        while (opModeIsActive() && timer.milliseconds() < constant.PIXEL_SCORE) {
//            // Allow the program to run other tasks during the waiting period
//            idle();
//
//        }
//
//        robot.L2.setPosition(0.449);
//        robot.L1.setPosition(0.449);
//    }
//    public void turnMoveForwardPlacePixelMiddle(int line){
//        ElapsedTime timer = new ElapsedTime();
//
//        resetRuntime();
//        while(opModeIsActive() && timer.milliseconds() < constant.PIXEL_SCORE_TURNLEFT){
//            idle();
//
//        }
//        robot.stop();
//        resetRuntime();
//        robot.forward(constant.MOTOR_POWER);
//
//        // Track the start time
//
//
//        while (opModeIsActive() && timer.milliseconds() < constant.PIXEL_SCORE_FORWARD) {
//            // Allow the program to run other tasks during the waiting period
//            idle();
//
//
//        }
//        robot.stop();
//        UpdateLifts(-200);
//        timer.reset();
//        while(opModeIsActive() && timer.milliseconds() < constant.LIFTENGAGE && (robot.liftB.getCurrentPosition() + robot.liftA.getCurrentPosition() < 175)){
//            idle();
//            UpdateLifts(-200);
//        }
//
//
//        robot.L2.setPosition(0.827);
//        robot.L1.setPosition(0.821);
//        timer.reset();
//
//        while (opModeIsActive() && timer.milliseconds() < constant.PIXEL_SCORE) {
//            // Allow the program to run other tasks during the waiting period
//            idle();
//
//        }
//
//        robot.L2.setPosition(0.449);
//        robot.L1.setPosition(0.449);
//    }
//    public void turnMoveForwardPlacePixelRight(int line){
//        ElapsedTime timer = new ElapsedTime();
//        resetRuntime();
//        while(opModeIsActive() && timer.milliseconds() < constant.PIXEL_SCORE_TURNLEFT){
//            idle();
//
//        }
//        robot.stop();
//        resetRuntime();
//        robot.forward(constant.MOTOR_POWER);
//
//        // Track the start time
//
//
//        while (opModeIsActive() && timer.milliseconds() < constant.PIXEL_SCORE_FORWARD) {
//            // Allow the program to run other tasks during the waiting period
//            idle();
//
//
//        }
//        robot.stop();
//        UpdateLifts(-200);
//        timer.reset();
//        while(opModeIsActive() && timer.milliseconds() < constant.LIFTENGAGE && (robot.liftB.getCurrentPosition() + robot.liftA.getCurrentPosition() < 175)){
//            idle();
//            UpdateLifts(-200);
//        }
//
//        robot.right(constant.RIGHT_PIXEL_SCORE_SPEED);
//        timer.reset();
//        while (opModeIsActive() && timer.milliseconds() < constant.PIXEL_SCORE_RIGHT){
//            // Allow the program to run other tasks during the waiting period
//            idle();
//
//        }
//        robot.stop();
//
//
//        robot.L2.setPosition(0.827);
//        robot.L1.setPosition(0.821);
//        timer.reset();
//
//        while (opModeIsActive() && timer.milliseconds() < constant.PIXEL_SCORE) {
//            // Allow the program to run other tasks during the waiting period
//            idle();
//
//        }
//
//        robot.L2.setPosition(0.449);
//        robot.L1.setPosition(0.449);
//    }
    public void starts() {
        // shuts down the camera once the match starts, we dont need to look any more

        switch (m_tse) {
            case LEFT:
                moveForwardAndPlacePixeleft(1);
                break;
            case RIGHT:
//                moveForwardAndPlacePixelRight(1);

                break;
            case MIDDLE:
//                moveForwardAndPlacePixelMiddle(1);
                break;
            case NONE:
//                moveForwardAndPlacePixelMiddle(1);
                break;
        }
    }
    public void stops() {
        // this closes down the portal when we stop the code, its good practice!

        visionPortal.close();
        telemetry.addData("Done Pokie", "Have A Great Day");
        telemetry.update();
    }
    public void inits(){
        visionProcessor = new tseSaturationProcessor();
        try {
//            visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(visionProcessor)
                    .setCameraResolution(new Size(800, 600))
                    .enableLiveView(true)
                    .setAutoStopLiveView(true)
                    .build();
        } catch (Exception e) {
        }
    }
    public void getLine(){
        visionPortal.resumeStreaming();
        //The TSE processor continuously tries to set the selected window
        searchTSE = true;
        m_tse = visionProcessor.getSelection();
        if(m_tse != tseSaturationProcessor.Selected.NONE) { //Unknown TSE location, try to find it
            telemetry.addLine("Vision Processor identified TSE");
        }
        telemetry.update();
    }
    public void strafeleft(double speed, long time, int lifts){
        robot.left(speed);
        while(getRuntime() < time){

            constant.UpdateLifts(lifts);
        }
        robot.stop();
    }
    public void straferight(double speed, long time, int lifts){
        robot.right(speed);
        while(getRuntime() < time){
            constant.UpdateLifts(lifts);
        }
        robot.stop();
    }
    public void forward(double speed, long time, int lifts){
        robot.forward(speed);
        while(getRuntime() < time){
            idle();
            constant.UpdateLifts(lifts);
        }
        robot.stop();
    }
    public void backward(double speed, long time, int lifts){
        robot.backward(speed);
        while(getRuntime() < time){
            idle();
            constant.UpdateLifts(lifts);
        }
        robot.stop();
    }
    public void turnLeft(double speed, long time, int lifts){
        robot.turnLeft(speed);
        while(getRuntime() < time){
            idle();
            constant.UpdateLifts(lifts);
        }
        robot.stop();
    }


    @Override
    public void runOpMode() {


        robot.init(hardwareMap);
        inits();
        while(opModeIsActive() && !isStarted()) {

            getLine();
        }
        searchTSE = false;
        visionPortal.stopStreaming();
        telemetry.update();
        telemetry.update();




        starts();
        stops();
    }
}