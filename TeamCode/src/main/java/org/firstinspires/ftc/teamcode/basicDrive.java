package org.firstinspires.ftc.teamcode;

//IMPORTING FUCTIONS
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
//Last Edited 2/19/2021 10:10PM AE
// NAMING IT ON SCREEN
@TeleOp(name="basic drive", group="Pushbot")

public class basicDrive extends LinearOpMode {
    /* Declare OpMode members. */
    // NOT NEEDED IN THIS PROGRAM BECAUSE IT IMPORTS INFORMATION
//    BotHardware robot = new BotHardware();
    //TIMER
    ElapsedTime runtime = new ElapsedTime();
    // private DistanceSensor sensor_range;
    //DECLARING THAT THESE VARIABLES ARE MOTORS AND ARE YET TO HAVE VALUES
    public DcMotor  fr   = null;
    public DcMotor  br   = null;
    public DcMotor  fl   = null;
    public DcMotor  bl   = null;
    public DcMotor liftA = null;
    public DcMotor liftB = null;
    @Override
    public void runOpMode() {
        //YOU NEED TO ASSIGN NAMES TO YOUR MOTORS ON THE DRIVER HUB AND THIS LOOKS FOR THE NAMES
        fr  = hardwareMap.get(DcMotor.class, "fr");
        br = hardwareMap.get(DcMotor.class, "br");
        fl    = hardwareMap.get(DcMotor.class, "fl");
        bl  = hardwareMap.get(DcMotor.class, "bl");
        // Harvest = hwMap.get(DcMotor.class, "Harvest");
        // duckie = hwMap.get(DcMotor.class, "duckie");
        liftA = hardwareMap.get(DcMotor.class, "liftA");
        liftB = hardwareMap.get(DcMotor.class, "liftB");
        // magnetArm = hwMap.get(DcMotor.class,"magnetArm");
        //distancePole = hwMap.get(DistanceSensor.class, "distancePole");
//        distanceTower = hwMap.get(DistanceSensor.class, "ds");
        //THESE NEED TO BE REVERSED FOR MECCANUMS
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        liftA.setDirection(DcMotor.Direction.FORWARD);
        liftB.setDirection(DcMotorSimple.Direction.REVERSE);
        // RUN WITHOUT CHECKS BY THE ENCODER VALUES
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        USED FOR CHANGING TOWAR POSITIONS LAST SEASON IT USED ENCODERS
//        int tower = 5;
//        boolean changeTower = false;


        //SETTING A SERVOS POSITION (OUTSIDE EXTENSION)
//        robot.encoderServo.setPosition(1);

// Send telemetry message to signify robot waiting;
        telemetry.update();
        telemetry.addData("Say", "Drive program 4"); //
        telemetry.update();
//      SPEED CONTROLS
        double x= 1.0;
        double lift = .5;
//      WAIT FOR PLAY BUTTON
        waitForStart();


//      RESETING TIMER
        runtime.reset();

        while (opModeIsActive()) {


            /*ENCODER TELEMETRY*/
            //UPDATING THE SCREEN
            telemetry.update();
//            telemetry.addData("Lift Motor: ", robot.lift.getCurrentPosition() );
//            telemetry.addData("Auto Postion: ", autoPosition);
//            telemetry.addData("Lift", "%.3f", robot.lift.getPower());
//            telemetry.addData("Tower Position: ", tower);
//            telemetry.addData(" ", " ");
            // ADDING CURRENT TIME RUNTIME IS A EXTENSION OF ELAPSED TIME
            telemetry.addData("Current time: ", runtime.seconds());
            telemetry.addData(" ", " ");

//            forwardMovement = (robot.leftEncoder.getCurrentPosition() + robot.rightEncoder.getCurrentPosition()) / 2;

//            telemetry.addData("Left Encoder: ", robot.encoderTicksToInches(robot.leftEncoder.getCurrentPosition()));
//            telemetry.addData("Right Encoder: ", robot.encoderTicksToInches(robot.rightEncoder.getCurrentPosition()));
//            telemetry.addData("Forward Movement: ", robot.encoderTicksToInches(forwardMovement));
//            telemetry.addData("Back Encoder: ", robot.encoderTicksToInches(robot.frontEncoder.getCurrentPosition()));

            //UPDATING SCREEN
            telemetry.update();

            // IF ZERO POWER STOP CAN ALSO BE FLOAT AKA KEEP MOVING WHILE SLOWING DOWN
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            /*DRIVE PROGRAMING*/
            // Y IS THE Y ON YOUR GAMEPAD STICK AKA UP AND DOWN
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            //GAMEPAD STICK X AKA RIGHT AND LEFT CONTROLS STRAFING
            double lx = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            //GAMEPAD STICK X2 AKA RIGHT AND LEFT CONTROLS TURNING
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            //MATH THAT IS LONG AND CONFUSING TO EXPLAIN
            double denominator = Math.max(Math.abs(y) + Math.abs(lx) + Math.abs(rx), 1);
            double frontLeftPower = (y + lx + rx) / denominator;
            double backLeftPower = (y - lx + rx) / denominator;
            double frontRightPower = (y - lx - rx) / denominator;
            double backRightPower = (y + lx - rx) / denominator;
            //SETS THE POWER TO THE MATH BUT ALSO CONTROLS SPEED
            fl.setPower(frontLeftPower * x);
            bl.setPower(backLeftPower * x);
            fr.setPower(frontRightPower * x);
            br.setPower(backRightPower * x);



            /*SPEED VARIABLES FOR DRIVE*/
            //CONTROLS SPEED IF GAMEPAD 1Y OR RIGHT BUMPER THEN SET SPEED TO THIS PERCENTAGE OF POWER
            if(gamepad1.y || gamepad1.right_bumper) {
                x = 1;
            }
            else if(gamepad1.a) {
                x = 0.5;
            }
            else if (gamepad1.x){
                x = 0.75;
            }
            else if (gamepad1.b || gamepad1.left_bumper){
                x = 0.25;
            }


            //LIFTS PROGRAMMING
            //IF DPAD IS PRESSED UP GO UP AND MULTIPLY IT BY SPEED VARIABLE FOR LIFT
            if(gamepad2.dpad_up){
                liftA.setPower(1 * lift);
                liftB.setPower(1 * lift);
            }

            else if(gamepad2.dpad_down){
                liftA.setPower(-1 * lift);
                liftB.setPower(-1 * lift);
            }
            else{
                liftA.setPower(0);
                liftB.setPower(0);
            }
            //LIFT SPEED CONTROL
            if(gamepad2.y) {
                lift = 1;
            }
            else if(gamepad2.a) {
                lift = 0.5;
            }
            else if (gamepad2.x){
                lift = 0.75;
            }
            else if (gamepad2.b){
                lift = 0.25;
            }

            //WRIST PROGRAMING
            //old wrist
            //WRIST DOWN MOVEMENT
//            if (gamepad2.a){
//                robot.wrist.setPosition(1);
//            }
//            else if (gamepad2.y){
//                robot.wrist.setPosition(0);
//            }
//            else if (gamepad2.b){
//                robot.wrist.setPosition(0.5);
//            }
//
//            if (gamepad2.left_bumper){
//                robot.R1.setPower(1.0);
//                robot.L1.setPower(1.0);
//            }
//            else if (gamepad2.right_bumper){
//                robot.R1.setPower(-1.0);
//                robot.L1.setPower(-1.0);
//            }
//            else{
//                robot.R1.setPower(0);
//                robot.L1.setPower(0);
//            }
//
//            // LIFT PROGRAMMING
//             LIFT UP TO SPECIFIC POSITION BASED ON ENCODERS
//
//            if (gamepad2.dpad_down){
//                robot.lift.getCurrentPosition();
//                robot.lift.setTargetPosition(-11);
//                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.lift.setPower(-1);
//
//            }
//            else if (robot.lift.getCurrentPosition() < 36 && robot.lift.getCurrentPosition() > -13){
//                robot.lift.setPower(0);
//            }
//            if (gamepad2.dpad_left){
//                robot.lift.getCurrentPosition();
//                robot.lift.setTargetPosition(-1145);
//                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                robot.lift.setPower(-1);
//            }
//
//            if (gamepad2.dpad_right){
//                robot.lift.getCurrentPosition();
//                robot.lift.setTargetPosition(-2072);
//                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                robot.lift.setPower(-1);
//
//            }
//            if (gamepad2.dpad_up){
//                robot.lift.getCurrentPosition();
//                robot.lift.setTargetPosition(-2856);
//                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                robot.lift.setPower(-1);
//
//            }
//            if (gamepad2.left_trigger > 0) {
//                robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            }
//            telemetry.update();
//            if (gamepad2.right_trigger > 0){
//
//
//                if (gamepad2.right_stick_y > 0) {
//                    robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    robot.lift.setPower(.5);
//                }
//                else if (gamepad2.right_stick_y< 0) {
//                    robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    robot.lift.setPower(-.4);
//                }
//                else{
//                    robot.lift.setPower(0);
//                }
//            }
//
//            //////////////////////Tower Code//////////////////////
            //SETS OUR LIFT TO EACH OF THE STACK POSITIONS
            //CONE FIVE
//            if(gamepad2.x && (tower == 5)){
//                changeTower = true;
//             //GETS THE CURRENT POSITION NOT NEEDED BUT SUGGESTED
//                robot.lift.getCurrentPosition();
            //SETS THE POSITION TO THE TARGE OF -470
//                robot.lift.setTargetPosition(-470);
            //SETS THE LIFT TO GO TO THE POSITION
//                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
            //SETS THE POWER
//                robot.lift.setPower(-0.5);
//            }
//
//            if(gamepad2.x && (tower == 4)){
//                changeTower = true;
//
//                robot.lift.getCurrentPosition();
//                robot.lift.setTargetPosition(-342);
//                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                robot.lift.setPower(-0.5);
//            }
//
//            if(gamepad2.x && (tower == 3)){
//                changeTower = true;
//
//                robot.lift.getCurrentPosition();
//                robot.lift.setTargetPosition(-247);
//                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                robot.lift.setPower(-0.5);
//            }
//
//            if(gamepad2.x && (tower == 2)){
//                changeTower = true;
//
//
//                robot.lift.getCurrentPosition();
//                robot.lift.setTargetPosition(-90);
//                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                robot.lift.setPower(-0.5);
//            }
//
//            if(gamepad2.x && (tower == 1)){
//                changeTower = true;
//
//                robot.lift.getCurrentPosition();
//                robot.lift.setTargetPosition(33);
//                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                robot.lift.setPower(-0.5);
//            }
//
//            //reset tower sequence
//            if(gamepad2.guide){
//                changeTower = true;
//
//                tower = 5;
//            }
//            //RESETS POSITION TO TOP IF MISTAKE WAS MADE
//            if(gamepad2.touchpad && changeTower){
//                changeTower = false;
//                if(tower == 1){
//                    tower = 5;
//                }
//                SET THE TOWER TO THE NEXT POSITION DOWN
//                else{
//                    tower = tower-1;
//                }
        }

        //////////////////////RUMBLE CODE/////////////////////////////
        //SHAKES GAME CONTROLLER ONLY AVAIBLE ON REV CONTROLLER
        if((runtime.seconds() > 85) && (runtime.seconds() < 86) && !gamepad1.isRumbling()){
            gamepad1.rumbleBlips(5);
            gamepad2.rumbleBlips(5);
        }

        if((runtime.seconds() > 90) && (runtime.seconds() < 91) && !gamepad1.isRumbling()){
            gamepad1.rumble(1000);
            gamepad2.rumble(1000);
        }


    }
}

