package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.RumbleCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoSubsystem;
import org.firstinspires.ftc.teamcode.triggers.TimeTrigger;

@Config
@TeleOp(name="New Drive FTC LIB", group="Pushbot")
public class newDriveFTCLib extends LinearOpMode {
    /* Declare OpMode members. */
    private DriveSubsystem driveSubsystem;
    private LiftSubsystem liftSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private ServoSubsystem servoSubsystem;
    private ElapsedTime runtime = new ElapsedTime();
    public static double p = 0.007, i = 0, d = 0.000, f = 0.001;
    public static int target = -15;
    public static double downpos = .65;
    public static int manual = 150;
    private final double ticks_in_degree = 751.8 / 180;
    private PIDController controller;
    public Pose2d endPos1;
    public int allianceColor = -1;

public double first=0;
public double end =0;
public double loopTime=0;
    // GamepadEx and ButtonReaders
    private GamepadEx gamepadEx1, gamepadEx2;
    private ButtonReader yButton1, aButton1, xButton1, bButton1, rightBumper1, leftBumper1;
    private ButtonReader dpadUp1, dpadUp2, dpadDown2, dpadLeft2, dpadRight2;
    private ButtonReader leftBumper2, rightBumper2, touchpad2, yButton2, xButton2, aButton2, bButton2;

    @Override
    public void runOpMode() {

        driveSubsystem = new DriveSubsystem(hardwareMap);
        liftSubsystem = new LiftSubsystem(hardwareMap, p, i, d, f, ticks_in_degree);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        servoSubsystem = new ServoSubsystem(hardwareMap);
        controller = new PIDController(p, i, d);
        controller.setSetPoint(target); // Ensure setpoint is updated in update method
        int armPoz = liftSubsystem.liftA.getCurrentPosition();
        double pid = controller.calculate(armPoz);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double power = pid + ff;
        while(opModeInInit()){
            liftSubsystem = new LiftSubsystem(hardwareMap, p, i, d, f, ticks_in_degree);
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        float endPositionX = prefs.getFloat("endPositionX", Float.NaN);
        float endPositionY = prefs.getFloat("endPositionY", Float.NaN);
        float endPosition_Heading = prefs.getFloat("endPosition_Heading",Float.NaN);
        float color = prefs.getFloat("color", Float.NaN);
        if(Float.isNaN(endPositionX) || Float.isNaN(endPositionY) || Float.isNaN(endPosition_Heading)){
            telemetry.addData("Status ", "Autonomous not run, using the default POSITION");
            Pose2d endPos1 = new Pose2d(0,0,0);
            driveSubsystem.driveUpdate(endPos1);
        }
        else{
            telemetry.addData("Status ", "Autonomous run, using the correct POSITION");
            Pose2d endPos1 = new Pose2d(endPositionX,endPositionY,endPosition_Heading);
            telemetry.addData("End Position X", endPositionX);
            driveSubsystem.driveUpdate(endPos1);
        }
        if(color == -1){
            telemetry.addData("Status ", "Autonomous  run, using the default ALLIANCE");
            allianceColor = 0;

        }
        else {
            allianceColor = (int) color;
            telemetry.addData("Status ", "Autonomous run, using the given ALLIANCE");

        }
        telemetry.addData("End Position X", endPositionX);
        telemetry.addData("End Position Y", endPositionY);
        telemetry.addData("End Position Heading", endPositionX);
        telemetry.addData("End Position Color", color);


        // Initialize GamepadEx and ButtonReaders
        telemetry.addData("Buttons ", "Initializing Buttons");
        telemetry.update();
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        yButton1 = new ButtonReader(gamepadEx1, GamepadKeys.Button.Y);
        aButton1 = new ButtonReader(gamepadEx1, GamepadKeys.Button.A);
        xButton1 = new ButtonReader(gamepadEx1, GamepadKeys.Button.X);
        bButton1 = new ButtonReader(gamepadEx1, GamepadKeys.Button.B);
        rightBumper1 = new ButtonReader(gamepadEx1, GamepadKeys.Button.RIGHT_BUMPER);
        leftBumper1 = new ButtonReader(gamepadEx1, GamepadKeys.Button.LEFT_BUMPER);

        dpadUp1 = new ButtonReader(gamepadEx1, GamepadKeys.Button.DPAD_UP);
        dpadUp2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.DPAD_UP);
        dpadDown2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.DPAD_DOWN);
        dpadLeft2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.DPAD_LEFT);
        dpadRight2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.DPAD_RIGHT);

        leftBumper2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.LEFT_BUMPER);
        rightBumper2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.RIGHT_BUMPER);

        yButton2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.Y);
        xButton2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.X);
        aButton2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.A);
        bButton2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.B);
        touchpad2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.START);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Buttons ", "Initialized");
        telemetry.addData("End Position X", endPositionX);
        telemetry.addData("End Position Y", endPositionY);
        telemetry.addData("End Position Heading", endPositionX);
        telemetry.addData("End Position Color", color);
        telemetry.update();}

        waitForStart();
        runtime.reset();
        Trigger rumble85Trigger = new TimeTrigger(runtime, 85, 86);
        Trigger rumble90Trigger = new TimeTrigger(runtime, 90, 91);

        // Bind triggers to rumble commands
        rumble85Trigger.whenActive(new RumbleCommand(gamepad1, gamepad2, 5));
        rumble90Trigger.whenActive(new RumbleCommand(gamepad1, gamepad2, 1000));
        liftSubsystem.update();
        while (opModeIsActive()) {
            first = runtime.milliseconds();
            // Update gamepad buttons
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double lx = -gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

                driveSubsystem.drive(y, lx, rx);


            // Speed control
            if (yButton1.isDown()) {
                driveSubsystem.setSpeedModifier(1);
            } else if (aButton1.isDown()) {
                driveSubsystem.setSpeedModifier(0.5);
            } else if (xButton1.isDown()) {
                driveSubsystem.setSpeedModifier(0.75);
            } else if (bButton1.isDown() || leftBumper1.isDown()) {
                driveSubsystem.setSpeedModifier(0.25);
            }
            // Turn 90 degrees when left bumper on gamepad1 is pressed
//            if (leftBumper1.isDown()) {
//                telemetry.addData("Gamepad1", "left_bumper pressed");
//                driveSubsystem.turn90Degrees();
//            }
//            else if(rightBumper1.isDown()){
//                telemetry.addData("Gamepad1", "right_bumper pressed");
//                driveSubsystem.turnneg90Degrees();
//            } // Move to a new position when right bumper on gamepad1 is pressed
            if (leftBumper1.isDown()) {
                telemetry.addData("SYSTEM OVERRIDE DO NOT TOUCH ", "I REPEAT DO NOT DRIVE");
                driveSubsystem.goToPlaceRR(allianceColor);
            }
            else if(rightBumper1.isDown()){
                telemetry.addData("SYSTEM OVERRIDE DO NOT TOUCH ", "I REPEAT DO NOT DRIVE");
                driveSubsystem.toCollect(allianceColor);
            }
            // Lift control
            if (dpadDown2.isDown()) {
                telemetry.addData("Gamepad2", "dpad_down pressed");
                liftSubsystem.setTarget(-20);
            } else if (dpadLeft2.isDown()) {
                telemetry.addData("Gamepad2", "dpad_left pressed");
                liftSubsystem.setTarget(2000);
            } else if (dpadRight2.isDown()) {
                telemetry.addData("Gamepad2", "dpad_right pressed");
                liftSubsystem.setTarget(3000);
            } else if (dpadUp2.isDown()) {
                telemetry.addData("Gamepad2", "dpad_up pressed");
                liftSubsystem.setTarget(4500);
            } else if (leftBumper2.isDown()) {
                telemetry.addData("Gamepad2", "left_bumper pressed");
                liftSubsystem.setTarget(liftSubsystem.target - 150);
            } else if (rightBumper2.isDown()) {
                telemetry.addData("Gamepad2", "right_bumper pressed");
                liftSubsystem.setTarget(liftSubsystem.target + manual);
            } else if (gamepad2.left_stick_x > .1) {
                telemetry.addData("Gamepad2", "left_stick_x > 0.1");
                liftSubsystem.setTarget(liftSubsystem.target - 50);
            } else if (dpadUp1.isDown()) {
                telemetry.addData("Gamepad1", "dpad_up pressed");
                liftSubsystem.setTarget(2650);
            } else if (touchpad2.isDown()) {
                telemetry.addData("Gamepad2", "touchpad pressed");
                liftSubsystem.resetEncoder();
            }
            liftSubsystem.update();


            // Intake control
            if (gamepad2.right_trigger > .3) {
                telemetry.addData("Gamepad2", "right_trigger > 0.3");
                intakeSubsystem.intakeOut();
            } else if (gamepad2.left_trigger > .3) {
                telemetry.addData("Gamepad2", "left_trigger > 0.3");
                intakeSubsystem.intakeIn();
            } else {
                intakeSubsystem.stop();
            }

            if (gamepad2.right_trigger > .3) {
                servoSubsystem.setHeightSPosition(downpos);
            } else {
                servoSubsystem.setHeightSPosition(Constants.heightSHigh);
            }

            // Claw control
            if (yButton2.isDown()) {
                telemetry.addData("Gamepad2", "y pressed");
                servoSubsystem.closeClaw();
                CommandScheduler.getInstance().schedule(new RumbleCommand(gamepad1, gamepad2, 200));
            } else if (xButton2.isDown()) {
                telemetry.addData("Gamepad2", "x pressed");
                servoSubsystem.openClaw();
            } else if (aButton2.isDown()) {
                telemetry.addData("Gamepad2", "a pressed");
                servoSubsystem.openClaw();
            } else if (bButton2.isDown()) {
                telemetry.addData("Gamepad2", "b pressed");
                servoSubsystem.openClaw();
            }

            // Wrist control
            if (liftSubsystem.getLiftPosition() > liftSubsystem.LIFT_ENCODER_TRIGGER) {
                servoSubsystem.wristUp();
            } else {
                servoSubsystem.wristDown();
            }



            telemetry.addData("Target: ", liftSubsystem.target);
            telemetry.addData("LiftA: ", liftSubsystem.getLiftPosition());
            end = runtime.milliseconds();
            loopTime=end - first ;
            telemetry.addData("looptime: ", loopTime);
            telemetry.addData("target: ", liftSubsystem.target);
            telemetry.addData("current Position: ", driveSubsystem.getCurrentPoz());
            telemetry.addData("trajectory ", driveSubsystem.trajectory);
            telemetry.addData("running ", driveSubsystem.running);

            telemetry.update();

            CommandScheduler.getInstance().run();
        }
    }

}
