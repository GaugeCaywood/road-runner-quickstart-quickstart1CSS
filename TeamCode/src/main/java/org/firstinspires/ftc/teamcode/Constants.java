package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Constants {
    BotHardware robot = new BotHardware();
    public static final double LIFT_POWER = 0.50;
    public static final double SERVO_POWER = 0.30;
    //VARIABLES FOR PURPLE PIXEL SCORE ON LINe
    public static final double MOTOR_POWER = 0.50;
    public static final long PIXEL_PLACEMENT_FORWARD = 675;  // Time in milliseconds
    public static final long PIXEL_PLACEMENT_LEFT = 500;
    public static final long PIXEL_PLACEMENT_RIGHT = 300;
    public static final long PIXEL_PLACE = 100;
    public static final double LEFT_PIXEL_PLACEMENT_SPEED = 0.25;
    public static final double PIXEL_PLACEMENT_RIGHT_SPEED = 0.25;
    //VARIABLES FOR YELLOW PIXEL SCORE ON BOARD
    public static final double LEFT_PIXEL_SCORE_SPEED = 0.25;
    public static final long PIXEL_SCORE_FORWARD = 2000;
    public static final long LIFTENGAGE = 1000;
    public static final long PIXEL_SCORE_LEFT = 1000;
    public static final double RIGHT_PIXEL_SCORE_SPEED = 0.25;
    public static final long PIXEL_SCORE_RIGHT = 350;
    public static final long PIXEL_SCORE = 250;
    public static final long PIXEL_SCORE_TURNLEFT = 250;
    public static final double TURN_LEFT_SPEED = 0.25;
    public static final double CLAW_CLOSE = .487;
    public static final double CLAW_OPEN = .6349;
    public static final int PLANEPOSITION = 0;
    public static final double WRIST_MEDIUM = .373;
    public static final double WRIST_DOWN = .178;
    public static final double WRIST_UP = .7;
    public static final double OUTTAKEB_IN = 0.827;
    public static final double OUTTAKEA_IN = 0.821;
    public static final double OUTTAKEB_OUT = 0.449;
    public static final double OUTTAKEA_OUT = 0.449;
    public enum MotorStateA {
        LVLA,
        RESETA
    }

    ;

    public enum MotorStateB {
        LVLB,
        RESETB
    }
    boolean GoPosA = true;
    boolean GoPosB = true;
    MotorStateB motorStateB = MotorStateB.LVLB;
    MotorStateA motorStateA = MotorStateA.LVLA;
    double targetA = 0;
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
                if (IsAuto) robot.liftA.setPower(0);
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

    // LIFT POSITIONS
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
