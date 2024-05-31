
        package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ButtonReader;

@TeleOp(name="Gamepad2 Test", group="Test")
public class Gamepad2Test extends LinearOpMode {

    private GamepadEx gamepadEx2;
    private ButtonReader yButton2, aButton2, xButton2, bButton2;
    private ButtonReader dpadUp2, dpadDown2, dpadLeft2, dpadRight2;
    private ButtonReader leftBumper2, rightBumper2;

    @Override
    public void runOpMode() {
        gamepadEx2 = new GamepadEx(gamepad2);

        yButton2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.Y);
        aButton2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.A);
        xButton2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.X);
        bButton2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.B);
        dpadUp2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.DPAD_UP);
        dpadDown2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.DPAD_DOWN);
        dpadLeft2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.DPAD_LEFT);
        dpadRight2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.DPAD_RIGHT);
        leftBumper2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.LEFT_BUMPER);
        rightBumper2 = new ButtonReader(gamepadEx2, GamepadKeys.Button.RIGHT_BUMPER);


        telemetry.addData("Initialization", "Gamepad2 initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            gamepadEx2.readButtons();

            if (yButton2.wasJustPressed()) {
                telemetry.addData("Gamepad2", "y pressed");
            }
            if (aButton2.wasJustPressed()) {
                telemetry.addData("Gamepad2", "a pressed");
            }
            if (xButton2.wasJustPressed()) {
                telemetry.addData("Gamepad2", "x pressed");
            }
            if (bButton2.wasJustPressed()) {
                telemetry.addData("Gamepad2", "b pressed");
            }
            if (dpadUp2.wasJustPressed()) {
                telemetry.addData("Gamepad2", "dpad_up pressed");
            }
            if (dpadDown2.wasJustPressed()) {
                telemetry.addData("Gamepad2", "dpad_down pressed");
            }
            if (dpadLeft2.wasJustPressed()) {
                telemetry.addData("Gamepad2", "dpad_left pressed");
            }
            if (dpadRight2.wasJustPressed()) {
                telemetry.addData("Gamepad2", "dpad_right pressed");
            }
            if (leftBumper2.wasJustPressed()) {
                telemetry.addData("Gamepad2", "left_bumper pressed");
            }
            if (rightBumper2.wasJustPressed()) {
                telemetry.addData("Gamepad2", "right_bumper pressed");
            }


            telemetry.update();
        }
    }
}
