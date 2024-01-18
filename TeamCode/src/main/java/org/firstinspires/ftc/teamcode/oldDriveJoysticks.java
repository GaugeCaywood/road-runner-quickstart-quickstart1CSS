package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
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
@Disabled
@TeleOp(name="1 - Joystick Driveold", group="Pushbot")

public class oldDriveJoysticks extends LinearOpMode {
    public double autoPosition = 0;

    /* Declare OpMode members. */
    oldBotHardware robot = new oldBotHardware();
    ElapsedTime runtime = new ElapsedTime();
    // private DistanceSensor sensor_range;

    @Override
    public void runOpMode() {

        int tower = 5;
        boolean changeTower = false;


        robot.init(hardwareMap);
//        robot.encoderServo.setPosition(1);

// Send telemetry message to signify robot waiting;
        telemetry.update();
        telemetry.addData("Say", "Drive program 4"); //
        telemetry.update();

        double x = 1.0;
        double forwardMovement = 0;

        waitForStart();


        runtime.reset();

        while (opModeIsActive()) {


            /*ENCODER TELEMETRY*/
            telemetry.update();
//            telemetry.addData("Lift Motor: ", robot.lift.getCurrentPosition() );
//            telemetry.addData("Auto Postion: ", autoPosition);
//            telemetry.addData("Lift", "%.3f", robot.lift.getPower());
//            telemetry.addData("Tower Position: ", tower);
//            telemetry.addData(" ", " ");

            telemetry.addData("Current time: ", runtime.seconds());
            telemetry.addData(" ", " ");

//            forwardMovement = (robot.leftEncoder.getCurrentPosition() + robot.rightEncoder.getCurrentPosition()) / 2;

//            telemetry.addData("Left Encoder: ", robot.encoderTicksToInches(robot.leftEncoder.getCurrentPosition()));
//            telemetry.addData("Right Encoder: ", robot.encoderTicksToInches(robot.rightEncoder.getCurrentPosition()));
//            telemetry.addData("Forward Movement: ", robot.encoderTicksToInches(forwardMovement));
//            telemetry.addData("Back Encoder: ", robot.encoderTicksToInches(robot.frontEncoder.getCurrentPosition()));


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
            if (gamepad1.y || gamepad1.right_bumper) {
                x = 1;
            } else if (gamepad1.a) {
                x = 0.5;
            } else if (gamepad1.x) {
                x = 0.75;
            } else if (gamepad1.b || gamepad1.left_bumper) {
                x = 0.25;
            }


            //WRIST PROGRAMING
            //old wrist
            if (gamepad2.a) {
                robot.wrist.setPosition(1);
            } else if (gamepad2.y) {
                robot.wrist.setPosition(0);
            } else if (gamepad2.b) {
                robot.wrist.setPosition(0.5);
            }

            if (gamepad2.left_bumper) {
                robot.R1.setPower(1.0);
                robot.L1.setPower(1.0);
            } else if (gamepad2.right_bumper) {
                robot.R1.setPower(-1.0);
                robot.L1.setPower(-1.0);
            } else {
                robot.R1.setPower(0);
                robot.L1.setPower(0);
            }

            // LIFT PROGRAMMING


            if (gamepad2.dpad_down) {
                robot.lift.getCurrentPosition();
                robot.lift.setTargetPosition(-11);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(-1);

            } else if (robot.lift.getCurrentPosition() < 36 && robot.lift.getCurrentPosition() > -13) {
                robot.lift.setPower(0);
            }
            if (gamepad2.dpad_left) {
                robot.lift.getCurrentPosition();
                robot.lift.setTargetPosition(-1145);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.lift.setPower(-1);
            }

            if (gamepad2.dpad_right) {
                robot.lift.getCurrentPosition();
                robot.lift.setTargetPosition(-2072);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.lift.setPower(-1);

            }
            if (gamepad2.dpad_up) {
                robot.lift.getCurrentPosition();
                robot.lift.setTargetPosition(-2856);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.lift.setPower(-1);

            }
            if (gamepad2.left_trigger > 0) {
                robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            telemetry.update();
            if (gamepad2.right_trigger > 0) {


                if (gamepad2.right_stick_y > 0) {
                    robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.lift.setPower(.5);
                } else if (gamepad2.right_stick_y < 0) {
                    robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.lift.setPower(-.4);
                } else {
                    robot.lift.setPower(0);
                }
            }

            //////////////////////Tower Code//////////////////////
            if (gamepad2.x && (tower == 5)) {
                changeTower = true;

                robot.lift.getCurrentPosition();
                robot.lift.setTargetPosition(-470);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.lift.setPower(-0.5);
            }

            if (gamepad2.x && (tower == 4)) {
                changeTower = true;

                robot.lift.getCurrentPosition();
                robot.lift.setTargetPosition(-342);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.lift.setPower(-0.5);
            }

            if (gamepad2.x && (tower == 3)) {
                changeTower = true;

                robot.lift.getCurrentPosition();
                robot.lift.setTargetPosition(-247);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.lift.setPower(-0.5);
            }

            if (gamepad2.x && (tower == 2)) {
                changeTower = true;


                robot.lift.getCurrentPosition();
                robot.lift.setTargetPosition(-90);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.lift.setPower(-0.5);
            }

            if (gamepad2.x && (tower == 1)) {
                changeTower = true;

                robot.lift.getCurrentPosition();
                robot.lift.setTargetPosition(33);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.lift.setPower(-0.5);
            }

            //reset tower sequence
            if (gamepad2.guide) {
                changeTower = true;

                tower = 5;
            }

            if (gamepad2.touchpad && changeTower) {
                changeTower = false;
                if (tower == 1) {
                    tower = 5;
                } else {
                    tower = tower - 1;
                }
            }

            //////////////////////RUMBLE CODE/////////////////////////////
            if ((runtime.seconds() > 85) && (runtime.seconds() < 86) && !gamepad1.isRumbling()) {
                gamepad1.rumbleBlips(5);
                gamepad2.rumbleBlips(5);
            }

            if ((runtime.seconds() > 90) && (runtime.seconds() < 91) && !gamepad1.isRumbling()) {
                gamepad1.rumble(1000);
                gamepad2.rumble(1000);
            }
            telemetry.addData("Lift Value ", robot.lift.getCurrentPosition() );

        }
    }
}

