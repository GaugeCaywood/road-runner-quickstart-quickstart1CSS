//package org.firstinspires.ftc.teamcode;
//
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
//
//import org.firstinspires.ftc.robotcore.external.Const;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.hardware.Servo;
////import com.qualcomm.robotcore.util.Hardware;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
////Last Edited 2/19/2021 10:10PM AE
//@Disabled
//@TeleOp(name="Lift Tuner", group="tuners")
//
//public class liftTuner extends LinearOpMode {
//    public double autoPosition = 0;
//
//    /* Declare OpMode members. */
//    BotHardwareNew robot = new BotHardwareNew();
//
//    // private DistanceSensor sensor_range;
//    @Override
//    public void runOpMode() {
//
//
//        robot.init(hardwareMap);
////        robot.encoderServo.setPosition(1);
//
//// Send telemetry message to signify robot waiting;
//        telemetry.update();
//        telemetry.addData("Say", "Drive program 4"); //
//        telemetry.update();
//        double x = 1.0;
//        double lift = 1;
//        double forwardMovement = 0;
//        int liftPosition = 0;
//        waitForStart();
//
//
//        while (opModeIsActive()) {
//
//
//            /*ENCODER TELEMETRY*/
//
//
//            telemetry.addData("LiftA", robot.liftA.getCurrentPosition());
//            telemetry.addData("LiftB", robot.liftB.getCurrentPosition());
//
//
//            telemetry.update();
//            //LIFTS PROGRAMMING SLAY!
//            if (gamepad2.y) {
//                robot.liftsUp(.25);
//            } else if (gamepad2.b) {
//                robot.liftsDown(.25);
//            } else {
//                robot.liftsStop();
//
//            }
//            if (gamepad2.right_trigger > .1){
//                liftPosition = robot.liftA.getCurrentPosition();
//            }
//            if (gamepad2.x){
//                robot.UpdateLifts(liftPosition);
//                telemetry.addData("Current Position is: ", liftPosition);
//                telemetry.update();
//
//                }
//            }
//        }
//    }
//
//
//
//
//
//
