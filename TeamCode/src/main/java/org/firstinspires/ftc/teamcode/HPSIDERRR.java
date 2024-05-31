//package org.firstinspires.ftc.teamcode;
//
//import android.util.Size;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.processors.tseSaturationProcessor;
//
//import org.firstinspires.ftc.vision.VisionPortal;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import java.lang.Math;
//import java.util.ArrayList;
//
//@Disabled
//@Autonomous(name="Human Player Side Red", group="Auton")
//public class HPSIDERRR extends LinearOpMode {
//    int detectedLine = -1;
//    private VisionPortal visionPortal;
//    private tseSaturationProcessor visionProcessor;
//    tseSaturationProcessor.Selected m_tse = tseSaturationProcessor.Selected.NONE;
//    boolean searchTSE = false;
//    BotHardwareNew robot = new BotHardwareNew();
//    constantsNew constants = new constantsNew();
//    public ElapsedTime runtime = new ElapsedTime();
//    public boolean SIDE = true;
//    public int slide = 1;
//    public boolean position = true;
//    public enum robotState{
//        FORWARDTOPIXEL,
//        PLACEONBACKDROP,
//        IDLE
//    }
//robotState currentState = robotState.IDLE;
//
//    @Override
//    public void runOpMode() {
//        robot.init(hardwareMap);
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        while(opModeInInit()){
//            if(gamepad1.dpad_right && slide == 1 ){
//                if(SIDE = true) {
//                    SIDE = false;
//                    telemetry();
//                }
//                if(SIDE = false){
//                    SIDE = true;
//                    telemetry();
//                }
//            }
//            if(gamepad1.dpad_down && slide == 1){
//                slide++;
//                telemetry();
//            }
//            if(gamepad1.dpad_up && slide == 2){
//                slide--;
//                telemetry();
//            }
//            if(gamepad1.dpad_right && slide == 2){
//                if(position = true){
//                    position = false;
//                }
//                if(position = false){
//                    position = true;
//                }
//            }
//
//        }
//
//        ///// TRAJECTORIES /////
//        //FORWARD
//
//
//        //////START STATE MACHINE//////
//
//        //start moving
//
//        waitForStart();
//        while (opModeIsActive() && !isStopRequested()) {
//            starts();
//
//        }
//        runs();
//
//    }
//    public void getLine(){
//        visionPortal.resumeStreaming();
//        //The TSE processor continuously tries to set the selected window
//        searchTSE = true;
//        m_tse = visionProcessor.getSelection();
//        if(m_tse != tseSaturationProcessor.Selected.NONE) { //Unknown TSE location, try to find it
//            telemetry.addLine("Vision Processor identified TSE");
//        }
//        telemetry.update();
//    }
//    public void starts() {
//        // shuts down the camera once the match starts, we dont need to look any more
//
//        switch (m_tse) {
//            case LEFT:
//                detectedLine = 1;
//
//                break;
//            case RIGHT:
//                detectedLine = 3;
//
//                break;
//            case MIDDLE:
//                detectedLine = 2;
//                break;
//            case NONE:
//                detectedLine = 3;
//                break;
//        }
//    }
//    public void stops() {
//        // this closes down the portal when we stop the code, its good practice!
//
//        visionPortal.close();
//        telemetry.addData("Done Pokie", "Have A Great Day");
//        telemetry.update();
//    }
//    public void inits(){
//        visionProcessor = new tseSaturationProcessor();
//        try {
////            visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);
//            visionPortal = new VisionPortal.Builder()
//                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                    .addProcessor(visionProcessor)
//                    .setCameraResolution(new Size(800, 600))
//                    .enableLiveView(true)
//                    .setAutoStopLiveView(true)
//                    .build();
//        } catch (Exception e) {
//            e.printStackTrace();
//        }
//
//    }
//    public void runs(){
//        switch(currentState){
//            case FORWARDTOPIXEL:
//                constants.UpdateLifts(200);
//                if(detectedLine == 1){
//                    //traj
//                }
//                else if(detectedLine == 2){
//                    //traj
//                }
//                else if (detectedLine == 3) {
//
//
//                    //traj
//                }
//                break;
//            case PLACEONBACKDROP:
//                constants.UpdateLifts(0);
//                //traj2
//                if(detectedLine == 1){
//                    //traj
//                }
//                else if(detectedLine == 2){
//                    //traj
//                }
//                else if (detectedLine == 3) {
//
//
//                    //traj
//                }
//            case IDLE:
//        }
//
//    }
//    public void telemetry(){
//        if(slide == 1 && opModeInInit()) {
//            telemetry.addData("Side is red = ", SIDE);
//            telemetry.update();
//        }
//        if(slide == 2 && opModeInInit()) {
//            telemetry.addData("Are you by the human player area = ", position);
//            telemetry.update();
//        }
//    }
//}