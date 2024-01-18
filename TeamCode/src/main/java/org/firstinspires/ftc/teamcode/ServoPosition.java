package org.firstinspires.ftc.teamcode;
//+
// .
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// @Config
@Disabled
@TeleOp
public class ServoPosition extends LinearOpMode {
//    public Servo    L1  = null;
//    public Servo   L2  = null;
    public static double servoPosition = 0.5;
    public static double servo2Position =0.5;
BotHardwareNew robot = new BotHardwareNew();
    @Override
    public void runOpMode() throws InterruptedException {
    robot.init(hardwareMap);
//        L1  = hardwareMap.get(Servo.class, "LIntake");
//        L2  = hardwareMap.get(Servo.class, "RIntake");

        servoPosition = 0.5;
        servo2Position = 0.5;

        waitForStart();


        while (!isStopRequested()) {
            if (gamepad1.dpad_up) {
                servoPosition += 0.0003;
                servo2Position += 0.0003;
            } else if (gamepad1.dpad_down) {
                servoPosition -= 0.0003;
                servo2Position -= 0.0003;
            }

            if (gamepad1.dpad_left) {
                servo2Position += 0.0003;
            } else if (gamepad1.dpad_right) {
                servo2Position -= 0.0003;
            }

            robot.planeS.setPosition(servoPosition);

            telemetry.addData("position", servoPosition);
            telemetry.addData("position2", servo2Position);

            telemetry.update();
        }

    }
}