package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
//Last Edited 2/19/2021 10:10PM AE

@Config
@TeleOp(name="New Drive", group="Pushbot")

public class newDrive extends LinearOpMode {
    /* Declare OpMode members. */
    BotHardwareNew robot = new BotHardwareNew();
    ElapsedTime runtime = new ElapsedTime();
    int time = 0;
    PIDController controller;
    public static double p = 0.007, i = 0, d = 0.000;
    public static double f = 0.001;
    public static int target = -15;
    public static double downpos = .65;
    public static int manual = 150;
    private final double ticks_in_degree = 751.8 / 180;
    public double first=0;
    public double end =0;
    public double loopTime=0;

    public enum heightControl {
        High,
        Medium,
        low
    }

    heightControl Height = heightControl.High;

    @Override
    public void runOpMode() {


        robot.init(hardwareMap);

        robot.planeS.setPosition(.945);
        waitForStart();
        robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        runtime.reset();
        while (opModeIsActive()) {
            first =runtime.milliseconds();
            controller = new PIDController(p, i, d);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            int armPoz = robot.liftA.getCurrentPosition();
            double pid = controller.calculate(armPoz, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            double power = pid + ff;
            /*ENCODER TELEMETRY*/



            /*DRIVE PROGRAMING*/
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double lx = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(lx) + Math.abs(rx), 1);
            double frontLeftPower = (y + lx + rx) / denominator;
            double backLeftPower = (y - lx + rx) / denominator;
            double frontRightPower = (y - lx - rx) / denominator;
            double backRightPower = (y + lx - rx) / denominator;

            robot.fl.setPower(frontLeftPower * robot.x);
            robot.bl.setPower(backLeftPower * robot.x);
            robot.fr.setPower(frontRightPower * robot.x);
            robot.br.setPower(backRightPower * robot.x);



            /*SPEED VARIABLES FOR DRIVE*/
            if (gamepad1.y || gamepad1.right_bumper) {
                robot.x = 1;
            } else if (gamepad1.a) {
                robot.x = 0.5;
            } else if (gamepad1.x) {
                robot.x = 0.75;
            } else if (gamepad1.b || gamepad1.left_bumper) {
                robot.x = 0.25;
            }
            //LIFTS PROGRAMMING SLAY!

            robot.liftA.setPower(power);
            robot.liftB.setPower(power);
            if (gamepad2.dpad_down) {
                target = -20;
            } else if (gamepad2.dpad_left) {
                target = 2000;
            } else if (gamepad2.dpad_right) {
                target = 3000;
            } else if (gamepad2.dpad_up) {
                target = 4500;
            } else if (gamepad2.left_bumper && target >= 100) {
                target -= 150;
            } else if (gamepad2.right_bumper && target <= 5000) {
                target += manual;
            } else if (gamepad2.left_stick_x > .1) {
                target -= 50;
            } else if (gamepad1.dpad_up) {
                target = 2650;
            } else if (gamepad2.touchpad) {
                robot.liftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.liftA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            ///////////9.11 programming/////////////////
            if (gamepad1.right_trigger > .1) {

                robot.planeS.setPosition(.846);


            }
            /////////////INTAKE HEIGHT CONTROL////////////////

            /////////////INTAKE PROGRAMMING///////////////////
            if (gamepad2.right_trigger > .3) {
                robot.intake.setPower(robot.INTAKE_OUT);
            } else if (gamepad2.left_trigger > .3) {
                robot.intake.setPower(robot.INTAKE_IN);
            } else {
                robot.intake.setPower(0);
            }
            if(gamepad2.right_trigger> .3){
                robot.heightS.setPosition(downpos);
            }
            else{
                robot.high();
            }
            //////////////CLAW CODE////////////////
            if (gamepad2.y) {
                robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                robot.L1.setPosition(robot.OUTTAKEA_CLOSE);
                robot.L2.setPosition(robot.OUTTAKEB_CLOSE);
                gamepad1.rumble(200);
            } else if (gamepad2.x) {
                robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
                robot.L1.setPosition(robot.OUTTAKEA_OPEN);
            } else if (gamepad2.a) {
                robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                robot.L1.setPosition(robot.OUTTAKEA_OPEN);
                robot.L2.setPosition(robot.OUTTAKEB_OPEN);
            } else if (gamepad2.b) {
                robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
                robot.L2.setPosition(robot.OUTTAKEB_OPEN);
            }

            //////////////WRIST CODE//////////////////
            if (robot.liftA.getCurrentPosition() > robot.LIFTENCODERTRIGGER) {
                robot.wristUp();
            } else if (robot.liftA.getCurrentPosition() < robot.LIFTENCODERTRIGGER) {
                robot.wristDown();
            }
            //////////////////////RUMBLE CODE/////////////////////////////
            if ((runtime.seconds() > 85) && (runtime.seconds() < 86) && !gamepad1.isRumbling()) {
                gamepad1.rumbleBlips(5);
                gamepad2.rumbleBlips(5);
            } else if ((runtime.seconds() > 90) && (runtime.seconds() < 91) && !gamepad1.isRumbling()) {
                gamepad1.rumble(1000);
                gamepad2.rumble(1000);
            }
            telemetry.addData("Target: ", target);
            telemetry.addData("LiftA: ", robot.liftA.getCurrentPosition());
            end = runtime.milliseconds();
            loopTime= end -first;
            telemetry.addData("looptime: ", loopTime);
            telemetry.update();


            //////////////Height Control////////

        }
    }
}
