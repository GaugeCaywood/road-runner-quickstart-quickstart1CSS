//package org.firstinspires.ftc.teamcode;
//
//
//import com.acmerobotics.dashboard.FtcDashboard;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
//import com.arcrobotics.ftclib.controller.PIDController;
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
//import com.qualcomm.robotcore.hardware.ColorSensor;
////Last Edited 2/19/2021 10:10PM AE
//
//@Config
//@TeleOp(name="Color Sensor Tuning", group="Pushbot")
//
//public class colorSensorTuning extends LinearOpMode {
//    /* Declare OpMode members. */
//    BotHardwareNew robot = new BotHardwareNew();
//    enum Claw{
//        open,
//        servo1,
//        servo2
//    }
//    @Override
//    public void runOpMode() {
//
//
//        robot.init(hardwareMap);
//
//        waitForStart();
//
//        Claw claw = Claw.open;
//
//        while (opModeIsActive()) {
//
//            telemetry.addData("Distance Sensor", robot.backDS.getDistance(DistanceUnit.MM));
//            telemetry.update();
//            switch(claw){
//                case open:
//                    telemetry.addData("IN: ", newDrive.Claw.values());
//                    if(gamepad2.x){
//                        robot.L1.setPosition(robot.OUTTAKEA_OPEN);
//                    }
//                    else if(gamepad2.a){
//                        robot.L1.setPosition(robot.OUTTAKEA_OPEN);
//                        robot.L2.setPosition(robot.OUTTAKEB_OPEN);
//                    }
//                    else if(gamepad2.b){
//                        robot.L2.setPosition(robot.OUTTAKEB_OPEN);
//                    }
//                    else if(robot.backDS.getDistance(DistanceUnit.MM) < 33 && robot.liftA.getCurrentPosition() < 300){
//                        claw = Claw.servo1;
//                    }
//                    break;
//                case servo1:
//
//                    telemetry.addData("BS: ", robot.backDS.getDistance(DistanceUnit.MM));
//                    telemetry.addData("FS: ", robot.frontColorSensor.alpha());
//                    if(robot.backDS.getDistance(DistanceUnit.MM) < 32 && robot.backDS.getDistance(DistanceUnit.MM) > 29){
//                        robot.L1.setPosition(robot.OUTTAKEB_CLOSE);
//                    }
//                    if(robot.liftA.getCurrentPosition() > 300){
//                        claw = Claw.open;
//                    }
//                    break;
//                case servo2:
//                    telemetry.addData("IN: ", newDrive.Claw.values());
//                    if(robot.frontColorSensor.alpha() > 600 && robot.liftA.getCurrentPosition() <300){
//                        robot.L2.setPosition(robot.OUTTAKEA_CLOSE);
//                    }
//                    if(robot.liftA.getCurrentPosition() > 300){
//                        claw = Claw.open;
//                    }
//                    break;
//            }
//            telemetry.addData("FS: ", robot.frontColorSensor.alpha());
//        }
//    }
//}
//
