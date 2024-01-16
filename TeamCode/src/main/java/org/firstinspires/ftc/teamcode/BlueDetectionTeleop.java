//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//
//import java.util.List;
//import android.widget.FrameLayout;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
////import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
//import java.util.List;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.hardware.DcMotor;
////import com.vuforia.Vuforia;
//
//@TeleOp(name="BlockDetectionTeleOp", group="TeleOp")
//public class BlueDetectionTeleop extends OpMode {
//        private final BotHardware robot = new BotHardware();
//        private int detectedLine = -1;
//        private int X1 = 10;
//        private int X2 = 20;
//        private int X3 = 30;
//        int control = 1;
//        double cubeX;
//        @Override
//        public void init() {
//            robot.init(hardwareMap);
//            robot.initVuforia();
//            robot.initTfod();
//            robot.tfod.activate();
//        }
//        @Override
//
//        public void loop() {
//
//            if (gamepad1.a) {
//                detectBlueBlock();
//                telemetry.addData("Detected line is: ", detectedLine);
//                telemetry.addData("BlockX is: ", cubeX);
//                telemetry.update();
//            }
//            if (gamepad1.dpad_right && control == 1){
//                X1++;
//            }
//            else if (gamepad1.dpad_right && control == 2){
//                X2++;
//            }
//            else if (gamepad1.dpad_right && control == 3){
//                X3++;
//            }
//            if (gamepad1.dpad_left && control == 1){
//                X1--;
//            }
//            else if (gamepad1.dpad_left && control == 2){
//                X2--;
//            }
//            else if (gamepad1.dpad_left && control == 3){
//                X3--;
//            }
//            if(gamepad1.right_bumper &&control != 3 ){
//                control++;
//                telemetry.addData("Controlled is: X", control);
//                telemetry.update();
//            }
//            if(gamepad1.right_bumper &&control == 3 ){
//                control = 1;
//                telemetry.addData("Controlled is: X", control);
//                telemetry.update();
//            }
//            if(gamepad1.right_bumper &&control != 1 ){
//                control--;
//                telemetry.addData("Controlled is: X", control);
//                telemetry.update();
//            }
//            if(gamepad1.right_bumper && control == 1 ){
//                control = 3;
//                telemetry.addData("Controlled is: X", control);
//                telemetry.update();
//            }
//            // Your other TeleOp code goes here
//            // ...
//
//            telemetry.update();
//        }
//
//        void detectBlueBlock() {
//
//
//            List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
//            if (updatedRecognitions != null) {
//                for (Recognition recognition : updatedRecognitions) {
//                    if (recognition.getLabel().equals("blueCube")) {
//                        float blockX = recognition.getLeft();
//                        cubeX = recognition.getLeft();
//                        //tune
//                        if (blockX < X1/*replace with tuned number*/) {
//                            detectedLine = 1;
//                        } else if (blockX < X2 && blockX > X3/*replace with tuned number*/) {
//                            detectedLine = 2;
//                        } else {
//                            detectedLine = 3;
//                        }
//                    }
//                }
//            }
//        }
//}