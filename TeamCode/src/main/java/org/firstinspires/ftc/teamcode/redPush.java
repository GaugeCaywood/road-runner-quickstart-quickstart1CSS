package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.teamcode.Drive;
//import org.firstinspires.ftc.teamcode.Duck;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import java.lang.Math;
import java.util.ArrayList;

@Autonomous(name="forwardRed", group="Auton")
public class redPush extends LinearOpMode {
    BotHardware robot = new BotHardware();
    public ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode () {
        robot.init(hardwareMap);
        waitForStart();
        sleep(200);
        robot.left(.25);
        sleep(250);
        robot.forward(.5);
        sleep(2200); // Sleep for 1 second

        robot.stop();

        // Move backward for 0.1 seconds
        robot.wrist.setPosition(.7);
        sleep(100);
        robot.stop();
        robot.claw.setPosition(.487);
    }

}

