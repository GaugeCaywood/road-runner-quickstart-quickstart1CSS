package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config

@TeleOp
public class liftPIDF extends OpMode{

    private PIDController controller;

    public static double p = 0.015, i = 0, d = 0.0001;
    public static double f = 0.03;
    public static int target = 0;

    private final double ticks_in_degree = 751.8 / 180;
    public DcMotorEx liftA;
    public DcMotorEx liftB;
    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        liftA = hardwareMap.get(DcMotorEx.class, "liftA");
        liftB = hardwareMap.get(DcMotorEx.class, "liftB");
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPoz = liftA.getCurrentPosition();
        double pid = controller.calculate(armPoz, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double power = pid + ff;

        liftA.setPower(power);
        liftB.setPower(-power);
        telemetry.addData(" Poz ", armPoz);
        telemetry.addData("target ", target );
        telemetry.update();
    }
}
