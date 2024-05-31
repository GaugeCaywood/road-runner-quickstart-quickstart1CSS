package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem {
    private DcMotor fr, br, fl, bl;
    private double speedModifier = 1.0;

    public DriveSubsystem(HardwareMap hwMap) {
        fr = hwMap.get(DcMotor.class, "fr");
        br = hwMap.get(DcMotor.class, "br");
        fl = hwMap.get(DcMotor.class, "fl");
        bl = hwMap.get(DcMotor.class, "bl");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setSpeedModifier(double modifier) {
        this.speedModifier = modifier;
    }

    public void drive(double y, double lx, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(lx) + Math.abs(rx), 1);
        double frontLeftPower = (y + lx + rx) / denominator;
        double backLeftPower = (y - lx + rx) / denominator;
        double frontRightPower = (y - lx - rx) / denominator;
        double backRightPower = (y + lx - rx) / denominator;

        fr.setPower(frontRightPower * speedModifier);
        br.setPower(backRightPower * speedModifier);
        fl.setPower(frontLeftPower * speedModifier);
        bl.setPower(backLeftPower * speedModifier);
    }
}
