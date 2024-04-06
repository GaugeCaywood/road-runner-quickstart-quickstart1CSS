package org.firstinspires.ftc.teamcode;
import android.annotation.SuppressLint;
import android.hardware.TriggerEvent;
import android.hardware.TriggerEventListener;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TestDrive")
public class commandOpMode extends CommandOpMode{
    //LIFT PIDF
    PIDController controller;
    public static double p = 0.007, i = 0, d = 0.000; public static double f = 0.001;
    private final double ticks_in_degree = 751.8 / 180;
    //START TARGET
    public static int target = -15;
    //MANUAL CONTROL VARIABLE
    public static int manual = 150;
    //GAMEPADS INIT
    GamepadEx secondaryGamepad = null;
    GamepadEx firstGamepad = null;
    // SPEED VARIABLE
    public static double x = 1;
    //HARDWAREMAP
    BotHardwareNew robot = new BotHardwareNew();
    //TIMER FOR RUMBLE
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void initialize() {


        secondaryGamepad = new GamepadEx(gamepad2);
        firstGamepad = new GamepadEx(gamepad1);
        //Gamepad 1
        //SPEED CONTROLS
        new GamepadButton(firstGamepad, GamepadKeys.Button.A).whenPressed(()-> x = .5);
        new GamepadButton(firstGamepad,GamepadKeys.Button.X).whenPressed(() -> x =.75);
        new GamepadButton(firstGamepad,GamepadKeys.Button.B).whenPressed(() -> x =.25);
        new GamepadButton(firstGamepad,GamepadKeys.Button.Y).whenPressed(() -> x =1);
        //HANG
        new GamepadButton(firstGamepad, GamepadKeys.Button.DPAD_UP).whenPressed(()-> target = 2650);
        //PLANE
        new Trigger(() -> gamepad1.right_trigger> .1).whenActive(() ->robot.planeS.setPosition(0));
        //WRIST TRIGGERS

        //Gamepad 2
        //LIFT CONTROL
        new GamepadButton(secondaryGamepad,GamepadKeys.Button.DPAD_DOWN).whenPressed(()-> target = -20);
        new GamepadButton(secondaryGamepad,GamepadKeys.Button.DPAD_UP).whenPressed(()-> target = 4500);
        new GamepadButton(secondaryGamepad, GamepadKeys.Button.DPAD_LEFT).whenPressed(()-> target = 2000);
        new GamepadButton(secondaryGamepad, GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> target = 3000);
        new GamepadButton(secondaryGamepad, GamepadKeys.Button.RIGHT_BUMPER).and(new Trigger(()->target >= 100)) .whenActive(()->target -= 150 );
        new Trigger(()-> gamepad2.left_stick_x > .1).whenActive(()-> target -=50);
        new GamepadButton(secondaryGamepad, GamepadKeys.Button.LEFT_BUMPER).and(new Trigger(()->target <= 5000)) .whenActive(()->target += manual );
        //CLAW CONTROLS GOES TO HARDWAREMAP
        new GamepadButton(secondaryGamepad, GamepadKeys.Button.A).whenPressed(() -> robot.servoOpenLightsBoth());
        new GamepadButton(secondaryGamepad, GamepadKeys.Button.B).whenPressed(() -> robot.servoOpenLightsL2());
        new GamepadButton(secondaryGamepad, GamepadKeys.Button.Y).whenPressed(() -> robot.servoCloseLights());
        new GamepadButton(secondaryGamepad, GamepadKeys.Button.X).whenPressed(() -> robot.servoOpenLightsL1());
        //INTAKE CONTROLS
        new Trigger(() -> gamepad2.right_trigger> .1).whenActive(() ->robot.intake.setPower(robot.INTAKE_OUT));
        new Trigger(()-> gamepad2.left_trigger>.1).whenActive(() -> robot.intake.setPower(robot.INTAKE_IN));
        new Trigger(()-> gamepad2.left_trigger< .1).and(new Trigger(()-> gamepad2.right_trigger<.1)) .whenActive(()-> robot.intake.setPower(0));
        //Events
        new Trigger(()-> runtime.seconds()>85).and(new Trigger(()-> runtime.seconds()<86)).and(new Trigger(()-> !gamepad1.isRumbling())).whenActive(() -> gamepad1.rumbleBlips(5)).whenActive(()-> gamepad2.rumbleBlips(5));
        new Trigger(()-> runtime.seconds()>90).and(new Trigger(()-> runtime.seconds()<91)).and(new Trigger(()-> !gamepad1.isRumbling())).whenActive(() -> gamepad1.rumble(1000)).whenActive(()-> gamepad2.rumble(1000));
    }
    @Override
    public  void run() {
        //PLANESCLOSE

        //START HARDWAREMAP
        robot.init(hardwareMap);

        new Trigger(() -> robot.liftA.getCurrentPosition() > robot.LIFTENCODERTRIGGER).whenActive(()-> robot.wristUp());
        new Trigger(()-> robot.liftA.getCurrentPosition() < robot.LIFTENCODERTRIGGER).whenActive(()-> robot.wristDown());
        //WAIT FOR START AKA PLAY BUTTON
        waitForStart();
        while (opModeIsActive()) {
            //IDK WHICH ONE TO USE
            super.run();
            run();
            //LIFT PROGRAMMING
            if (gamepad2.touchpad) {
                robot.liftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.liftA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            controller = new PIDController(p, i, d);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            int armPoz = robot.liftA.getCurrentPosition();
            double pid = controller.calculate(armPoz, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            double power = pid + ff;
            robot.liftA.setPower(power);
            robot.liftB.setPower(power);



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

            telemetry.addData("Target: ", target);
            telemetry.addData("LiftA: ", robot.liftA.getCurrentPosition());
            telemetry.update();

        }
    }



}
