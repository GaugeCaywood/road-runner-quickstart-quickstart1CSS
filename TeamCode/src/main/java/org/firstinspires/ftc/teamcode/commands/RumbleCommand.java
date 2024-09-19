package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

public class RumbleCommand extends CommandBase {
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final int duration;

    public RumbleCommand(Gamepad gamepad1, Gamepad gamepad2, int duration) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.duration = duration;
    }

    @Override
    public void initialize() {
        gamepad1.rumble(duration);
        gamepad2.rumble(duration);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
