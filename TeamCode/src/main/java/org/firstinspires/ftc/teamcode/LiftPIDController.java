package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Disabled
public class LiftPIDController {
    // Motors
    BotHardwareNew robot = new BotHardwareNew();


    // PID coefficients
    private double P;
    private double I;
    private double D;

    // PID control variables
    private double integral;
    private double previousError;
    private double targetHeight;

    // Encoder conversion factor
    private double encoderCountsToHeight; // Define this based on your mechanism
    // Constructor
    public LiftPIDController( double p, double i, double d) {
        P = p;
        I = i;
        D = d;
        integral = 0;
        previousError = 0;
        targetHeight = 0;

    }

    // Set target height
    public void setTargetHeight(double height) {
        targetHeight = height;
    }

    // Update motors based on PID control
    public void update() {
        double currentHeight = getCurrentHeight();
        double error = targetHeight - currentHeight;
        integral += error * 0.02;
        double derivative = (error - previousError) / 0.02;
        double output = P * error + I * integral + D * derivative;

        robot.liftA.setPower(output);
        robot.liftB.setPower(output);

        previousError = error;
    }

    // Get current height of the lift using encoder counts
    private double getCurrentHeight() {
        int averageEncoderCount = (robot.liftA.getCurrentPosition() + robot.liftB.getCurrentPosition()) / 2;
        double distancePerPulse = 120.0 / 751.8; // Conversion factor
        return averageEncoderCount * distancePerPulse;
    }
}
