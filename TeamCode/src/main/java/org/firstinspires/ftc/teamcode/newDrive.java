package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.arcrobotics.ftclib.controller.PIDController;
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

@TeleOp(name="New Drive", group="Pushbot")

public class newDrive extends LinearOpMode {
    /* Declare OpMode members. */
    BotHardwareNew robot = new BotHardwareNew();
    ElapsedTime runtime = new ElapsedTime();
    private PIDController controller;

    public static double p = 0.015, i = 0, d = 0;
    public static double f = 0.05;
    public static int target = 0;
    int liftPos = 0;
    private final double ticks_in_degree = 751.8 / 180;


    @Override
    public void runOpMode() {

        controller = new PIDController(p, i, d);
        robot.init(hardwareMap);
        robot.planeS.setPosition(.61);
        waitForStart();

        runtime.reset();
        while (opModeIsActive()) {
            controller.setPID(p, i, d);
            int armPoz = robot.liftA.getCurrentPosition();
            double pid = controller.calculate(armPoz, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            double power = pid + ff;

            /*ENCODER TELEMETRY*/

            telemetry.addData("Current time: ", runtime.seconds());
            telemetry.addData(" ", " ");
            telemetry.addData("LiftA", robot.liftA.getCurrentPosition());
            telemetry.addData("LiftB", robot.liftB.getCurrentPosition());
            telemetry.addData("Left Encoder: ", robot.bl.getCurrentPosition());
            telemetry.addData("Right Encoder: ", robot.fr.getCurrentPosition());
            telemetry.addData("Front Encoder: ", robot.fl.getCurrentPosition());


            robot.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            /*DRIVE PROGRAMING*/
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double lx = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
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
            if(gamepad2.dpad_down){
                target = 0;
            }
            else if(gamepad2.dpad_right){
                target = 2000;
            }
            else if(gamepad2.dpad_left){
                target = 3000;
            }
            else if(gamepad2.dpad_up){
                target = 4500;
            }
            if(gamepad2.right_bumper){
                target -= 3;
            }
            else if(gamepad2.left_bumper){
                target += 10;
            }

            /////////////INTAKE PROGRAMMING///////////////////
            if(gamepad2.right_trigger > .1){
                robot.intake.setPower(robot.INTAKE_OUT);
            }
            else if(gamepad2.left_trigger > .1){
                robot.intake.setPower(robot.INTAKE_IN);
            }
            else{
                robot.intake.setPower(0);
            }
            //////////////CLAW CODE////////////////
            if(gamepad2.x){
                robot.L1.setPosition(robot.OUTTAKEA_OPEN);
            }
            else if(gamepad2.a){
                robot.L1.setPosition(robot.OUTTAKEA_OPEN);
                robot.L2.setPosition(robot.OUTTAKEB_OPEN);
            }
            else if(gamepad2.b){
                robot.L2.setPosition(robot.OUTTAKEB_OPEN);
            }
            if(gamepad2.y){
                robot.L1.setPosition(robot.OUTTAKEA_CLOSE);
                robot.L2.setPosition(robot.OUTTAKEB_CLOSE);
                if(!gamepad1.isRumbling()) {
                    gamepad1.rumble(750);
                }
            }

            //////////////WRIST CODE//////////////////
            if(robot.liftA.getCurrentPosition() > robot.LIFTENCODERTRIGGER){
                robot.wristUp();
            }
            if(robot.liftA.getCurrentPosition() < robot.LIFTENCODERTRIGGER){
                robot.wristDown();
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
            telemetry.addData("LIFTA IS ", robot.liftA.getCurrentPosition());
            telemetry.addData("LIFTB IS ", robot.liftB.getCurrentPosition());
            telemetry.addData("Target ", target);
            telemetry.update();
        }
    }
}

