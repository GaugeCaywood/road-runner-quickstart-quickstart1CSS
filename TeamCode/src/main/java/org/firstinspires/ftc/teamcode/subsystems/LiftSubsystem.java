package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;

public class LiftSubsystem {
    public DcMotor liftA, liftB;
    private PIDController controller;
    private double p, i, d, f;
    private double ticksInDegree;
    public int target;
    public final int LIFT_ENCODER_TRIGGER = 1500;
    public LiftSubsystem(HardwareMap hwMap, double p, double i, double d, double f, double ticksInDegree) {
        liftA = hwMap.get(DcMotor.class, "liftA");
        liftB = hwMap.get(DcMotor.class, "liftB");

        liftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftA.setDirection(DcMotor.Direction.FORWARD);
        liftB.setDirection(DcMotor.Direction.REVERSE);
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.ticksInDegree = ticksInDegree;
        controller = new PIDController(p, i, d);
    }

    public void setTarget(int target) {
        this.target = target;
    }

    public void update() {
        controller.setSetPoint(target); // Ensure setpoint is updated in update method
        int armPoz = liftA.getCurrentPosition();
        double pid = controller.calculate(armPoz);
        double ff = Math.cos(Math.toRadians(target / ticksInDegree)) * f;
        double power = pid + ff;

        liftA.setPower(power);
        liftB.setPower(power);
    }

    public void resetEncoder() {
        liftA.setMode(DcMotor.RunMode.RESET_ENCODERS);
    }

    public double getLiftPosition() {
        return liftA.getCurrentPosition();
    }
}
