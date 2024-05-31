package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {
    private DcMotor intake;
    private static final double INTAKE_IN = -1.0;
    private static final double INTAKE_OUT = 1.0;

    public IntakeSubsystem(HardwareMap hwMap) {
        intake = hwMap.get(DcMotor.class, "intake");
    }

    public void setPower(double power) {
        intake.setPower(power);
    }

    public void intakeIn() {
        setPower(INTAKE_IN);
    }

    public void intakeOut() {
        setPower(INTAKE_OUT);
    }

    public void stop() {
        setPower(0);
    }
}
