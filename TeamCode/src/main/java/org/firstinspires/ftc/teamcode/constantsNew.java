package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class constantsNew {
    BotHardware robot = new BotHardware();
    public static final double LIFT_POWER = 0.50;
    public static final double OUTTAKEA_OPEN = 0.00; //UPDATE THIS
    public static final double OUTTAKEB_OPEN = 0.00; //UPDATE THIS
    public static final double OUTTAKEA_CLOSE = 0.00; //UPDATE THIS
    public static final double OUTTAKEB_CLOSE = 0.00; //UPDATE THIS
    public static final double INTAKE_IN = 1.00; //UPDATE THIS
    public static final double INTAKE_OUT = -1.00; //UPDATE THIS
    public static final double WRIST_UP = 1.00;
    public static final double WRIST_DOWN = 0.00;
    public static final double LIFTENCODERTRIGGER = 200;
    public static double x= 1.0;
    public static double lift = 1;
    public enum MotorStateA {
        LVLA,
        RESETA
    }
    public enum MotorStateB {
        LVLB,
        RESETB
    }
    boolean GoPosA = true;
    boolean GoPosB = true;
    MotorStateB motorStateB = MotorStateB.LVLB;
    MotorStateA motorStateA = MotorStateA.LVLA;
    public void UpdateA(int target) {
//        drive.lift.switchToLevel(target);
//        telemetry.update();
        robot.liftA.setPower(LIFT_POWER);
        StateUpdateA(true, target);
    }

    public void SetTargetPositionA(int poz) {
        robot.liftA.setTargetPosition(poz);
        robot.liftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void ResetA() {
        robot.liftA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isBusyA() {
        return robot.liftA.isBusy();
    }

    public void StateUpdateA(boolean IsAuto, int target) {
        switch (motorStateA) {
            case LVLA:
                Position_LvlA(target);
                break;
            case RESETA:
                if (IsAuto)
                    robot.liftA.setPower(0);
                    ResetA();

                break;
        }
        if (!GoPosA) {
            motorStateA = MotorStateA.RESETA;
        }
    }

    public void Position_LvlA(int poz2) {
        SetTargetPositionA(poz2);
        if (!isBusyA()) {
            GoPosA = false;
        }
    }

    // LIFT POSITIONS B
    public void UpdateB(int target) {
//        drive.lift.switchToLevel(target);
        //telemetry.update();
        robot.liftB.setPower(LIFT_POWER);
        StateUpdateB(true, target);
    }

    public void SetTargetPositionB(int poz) {
        robot.liftB.setTargetPosition(poz);
        robot.liftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void ResetB() {
        robot.liftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isBusyB() {
        return robot.liftB.isBusy();
    }

    public void StateUpdateB(boolean IsAuto, int target) {
        switch (motorStateB) {
            case LVLB:
                Position_LvlB(target);
                break;
            case RESETB:
                if (IsAuto) robot.liftB.setPower(0);
                ResetB();
                break;
        }
        if (!GoPosB) {
            motorStateB = MotorStateB.RESETB;
        }
    }

    public void Position_LvlB(int poz2) {
        SetTargetPositionB(poz2);
        if (!isBusyB()) {
            GoPosB = false;
        }
    }

    public void UpdateLifts(int target) {
        UpdateA(target);
        UpdateB(target);
    }
}
