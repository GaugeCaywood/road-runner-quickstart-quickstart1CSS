package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.robotcore.external.Const;
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
import com.qualcomm.robotcore.hardware.ColorSensor;
//Last Edited 2/19/2021 10:10PM AE

@Config
@TeleOp(name="JUDGING DRIVE DO NOT USE DURING MATCH", group="judging")

public class judgingDrive extends LinearOpMode {
    /* Declare OpMode members. */
    BotHardwareNew robot = new BotHardwareNew();
    ElapsedTime runtime = new ElapsedTime();
    int time = 0;
    PIDController controller;
    public static double p = 0.007, i = 0, d = 0.000;
    public static double f = 0.001;
    public static int target = -15;

    private final double ticks_in_degree = 751.8 / 180;
 enum judging{
        wait,
        liftUp1,
        liftUp2,
        liftUp3,
        wristUp,
        claw
    }
    judging Judging = judging.wait;
    @Override
    public void runOpMode() {



        robot.init(hardwareMap);
        robot.planeS.setPosition(.61);

        waitForStart();

        runtime.reset();
        while (opModeIsActive()) {

            controller = new PIDController(p, i, d);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            int armPoz = robot.liftA.getCurrentPosition();
            double pid = controller.calculate(armPoz, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            double power = pid + ff;


            //LIFTS PROGRAMMING SLAY!

            robot.liftA.setPower(power);
            robot.liftB.setPower(power);
            if(gamepad2.dpad_down){
                target = -20;
            }
            else if(gamepad2.dpad_left){
                target = 2000;
            }
            else if(gamepad2.dpad_right){
                target = 3000;
            }
            else if(gamepad2.dpad_up){
                target = 4500;
            }
            else if(gamepad2.left_bumper && target >=    100){
                target -=  50;
            }
            else if (gamepad2.right_bumper&& target <= 4500) {
                target += 50;
            }
            else if(gamepad2.left_stick_x > .1){
                target -= 3;
            }
            else if(gamepad1.dpad_up){
                target =2650;
            }
            else if (gamepad2.touchpad) {
                robot.liftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.liftA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            ///////////9.11 programming/////////////////
            if(gamepad1.right_trigger > .1){

                robot.planeS.setPosition(0);


            }
            /////////////INTAKE HEIGHT CONTROL////////////////

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
            if(gamepad2.y){
                robot.L1.setPosition(robot.OUTTAKEA_CLOSE);
                robot.L2.setPosition(robot.OUTTAKEB_CLOSE);
            }
            else if(gamepad2.x){
                robot.L1.setPosition(robot.OUTTAKEA_OPEN);
            }
            else if(gamepad2.a){
                robot.L1.setPosition(robot.OUTTAKEA_OPEN);
                robot.L2.setPosition(robot.OUTTAKEB_OPEN);
            }
            else if(gamepad2.b){
                robot.L2.setPosition(robot.OUTTAKEB_OPEN);
            }

            if(gamepad1.dpad_up){
                robot.wristUp();
            }
            if(gamepad1.dpad_down){
                robot.wristDown();
            }
            //////////////////////RUMBLE CODE/////////////////////////////
            if ((runtime.seconds() > 85) && (runtime.seconds() < 86) && !gamepad1.isRumbling()) {
                gamepad1.rumbleBlips(5);
                gamepad2.rumbleBlips(5);
            }

            else if ((runtime.seconds() > 90) && (runtime.seconds() < 91) && !gamepad1.isRumbling()) {
                gamepad1.rumble(1000);
                gamepad2.rumble(1000);
            }
            telemetry.addData("Target: ", target);
            telemetry.addData("LiftA: ", robot.liftA.getCurrentPosition());
            telemetry.update();
            switch(Judging){
                case wait:
                    if(gamepad1.a){
                        Judging = judging.liftUp1;
                    }
                    break;
                case liftUp1:
                    target = 2000;
                    if(gamepad1.a){
                        Judging = judging.liftUp2;
                    }
                    break;
                case liftUp2:
                    target = 3000;
                    if(gamepad1.a){
                        Judging = judging.liftUp3;
                    }
                    break;
                case liftUp3:
                    target =4000;
                    if(gamepad1.a){
                        Judging = judging.claw;
                    }
                    break;
                case claw:
                    robot.servo(false, 2,true);
                    if(gamepad1.a){
                        Judging = judging.wristUp;
                    }
                    break;
                case wristUp:
                    robot.wristUp();
                    if(gamepad1.a){
                        Judging = judging.wait;
                    }
            }
        }
    }
}

