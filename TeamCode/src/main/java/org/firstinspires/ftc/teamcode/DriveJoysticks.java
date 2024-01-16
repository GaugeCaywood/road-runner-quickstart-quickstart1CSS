package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

@TeleOp(name="1 - Joystick Drive", group="Pushbot")

public class DriveJoysticks extends LinearOpMode {
    public double autoPosition = 0;

    /* Declare OpMode members. */
    BotHardware robot = new BotHardware();
    ElapsedTime runtime = new ElapsedTime();
    // private DistanceSensor sensor_range;
    Constants constants = new Constants();
    @Override
    public void runOpMode() {





        robot.init(hardwareMap);
//        robot.encoderServo.setPosition(1);

// Send telemetry message to signify robot waiting;
        telemetry.update();
        telemetry.addData("Say", "Drive program 4"); //
        telemetry.update();
        robot.planeS.setPosition(1);
        double x= 1.0;
        double lift = 1;
        double forwardMovement = 0;

        waitForStart();



        runtime.reset();

        while (opModeIsActive()) {


            /*ENCODER TELEMETRY*/
            telemetry.update();

            telemetry.addData("Current time: ", runtime.seconds());
            telemetry.addData(" ", " ");
            telemetry.addData("LiftA", robot.liftA.getCurrentPosition());
            telemetry.addData("LiftB", robot.liftB.getCurrentPosition());



            telemetry.update();


            robot.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            /*DRIVE PROGRAMING*/
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double lx = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(lx) + Math.abs(rx), 1);
            double frontLeftPower = (y + lx + rx) / denominator;
            double backLeftPower = (y - lx + rx) / denominator;
            double frontRightPower = (y - lx - rx) / denominator;
            double backRightPower = (y + lx - rx) / denominator;

            robot.fl.setPower(frontLeftPower * x);
            robot.bl.setPower(backLeftPower * x);
            robot.fr.setPower(frontRightPower * x);
            robot.br.setPower(backRightPower * x);



            /*SPEED VARIABLES FOR DRIVE*/
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


            //LIFTS PROGRAMMING SLAY!



             if(gamepad2.a) {
                lift = 0.5;
            }
            else if (gamepad2.x){
                lift = 0.75;
            }
            if(gamepad2.y){
                robot.liftsUp(lift);
            }
            else if(gamepad2.b){
                robot.liftsDown(1);
            }
            else{
                robot.liftsStop();
            }

            if (gamepad2.left_bumper){
                robot.L2.setPosition(constants.OUTTAKEB_OUT);
                robot.L1.setPosition(constants.OUTTAKEA_OUT);
            }
            else if (gamepad2.right_bumper){
                robot.L2.setPosition(constants.OUTTAKEA_IN);
                robot.L1.setPosition(constants.OUTTAKEB_IN);
            }

            if (gamepad2.dpad_right){
                robot.wrist.setPosition(constants.WRIST_UP);

            }
            else if (gamepad2.dpad_left){
                robot.wrist.setPosition(constants.WRIST_DOWN);
            }
            else if (gamepad2.dpad_up){
                robot.wrist.setPosition(constants.WRIST_MEDIUM);
            }

            if (gamepad2.right_trigger > .1){
                robot.claw.setPosition(constants.CLAW_OPEN);

            }
            else if (gamepad2.left_trigger > .1){
                robot.claw.setPosition(constants.CLAW_CLOSE);
            }
            if(gamepad1.right_trigger > .1){
                robot.planeS.setPosition(constants.PLANEPOSITION);
            }

            }

            //////////////////////RUMBLE CODE/////////////////////////////
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

