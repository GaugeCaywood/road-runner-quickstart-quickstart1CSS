package org.firstinspires.ftc.teamcode;
//+
// .
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config

@TeleOp
public class ServoPosition extends LinearOpMode {
//    public Servo    L1  = null;
//    public Servo   L2  = null;
    public Servo wristL = null;
    public Servo wristR = null;
    public DcMotor intake = null;
    public static double servoPosition = 0.5;
    public static double servo2Position =0.5;
    public Servo heightS;
    @Override
    public void runOpMode() throws InterruptedException {

//        L1  = hardwareMap.get(Servo.class, "LIntake");
//        L2  = hardwareMap.get(Servo.class, "RIntake");
            heightS = hardwareMap.get(Servo.class, "heightS");
            intake = hardwareMap.get(DcMotor.class,"intake");
//        wristL = hardwareMap.get(Servo.class, "wristL");
//        wristR = hardwareMap.get(Servo.class, "wristR");
        servoPosition = 0.5;
        servo2Position = 0.5;

        waitForStart();


        while (!isStopRequested()) {
            if(gamepad1.right_bumper){
                intake.setPower(1);
            }
            else if(gamepad1.left_bumper){
                intake.setPower(-1);
            }
            else{
                intake.setPower(0);
            }
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

            heightS.setPosition(servoPosition);
//            wristR.setPosition(servo2Position);
            telemetry.addData("position", servoPosition);
            telemetry.addData("position2", servo2Position);

            telemetry.update();
        }

    }
}