package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Constants;

public class ServoSubsystem {
    private Servo L1, L2, planeS, wristL, wristR, heightS;

    public ServoSubsystem(HardwareMap hwMap) {
        L1 = hwMap.get(Servo.class, "LIntake");
        L2 = hwMap.get(Servo.class, "RIntake");
        planeS = hwMap.get(Servo.class, "plane");
        wristL = hwMap.get(Servo.class, "wristL");
        wristR = hwMap.get(Servo.class, "wristR");
        heightS = hwMap.get(Servo.class, "heightS");
    }

    public void setL1Position(double position) {
        L1.setPosition(position);
    }

    public void setL2Position(double position) {
        L2.setPosition(position);
    }

    public void setPlanePosition(double position) {
        planeS.setPosition(position);
    }

    public void setWristLPosition(double position) {
        wristL.setPosition(position);
    }

    public void setWristRPosition(double position) {
        wristR.setPosition(position);
    }

    public void setHeightSPosition(double position) {
        heightS.setPosition(position);
    }

    public void wristUp() {
        wristL.setPosition(Constants.WRIST_UP);
        wristR.setPosition(Constants.WRIST_UP);
    }

    public void wristDown() {
        wristL.setPosition(Constants.WRIST_DOWN);
        wristR.setPosition(Constants.WRIST_DOWN);
    }

    public void setHeightPositionHigh() {
        heightS.setPosition(Constants.heightSHigh);
    }

    public void setHeightPositionLow() {
        heightS.setPosition(Constants.heightSLow);
    }

    public void openClaw() {
        L1.setPosition(Constants.OUTTAKE_A_OPEN);
        L2.setPosition(Constants.OUTTAKE_B_OPEN);
    }

    public void closeClaw() {
        L1.setPosition(Constants.OUTTAKE_A_CLOSE);
        L2.setPosition(Constants.OUTTAKE_B_CLOSE);
    }
}
