package org.firstinspires.ftc.teamcode;

//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import com.vuforia.Vuforia;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
        import com.qualcomm.robotcore.hardware.HardwareMap;

//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

        import java.util.List;

        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.Servo;

//Last Edited 10/27/2022 6:59PM MST

public class BotHardware
{
    Constants constants = new Constants();
//
//    private static final String TFOD_MODEL_ASSET = "blueteamprop.tflite";
//    private TFObjectDetector.Parameters tfodParameters;
//    public VuforiaLocalizer vuforia;
//    public TFObjectDetector tfod;

    /* Public OpMode members. */
    //MOTOR NULE DECLARATION
    public DcMotor  fr   = null;
    public DcMotor  br   = null;
    public DcMotor  fl   = null;
    public DcMotor  bl   = null;
//    public DcMotor  leftEncoder   = null;
//    public DcMotor  rightEncoder   = null;
//    public DcMotor  frontEncoder   = null;
    // public DcMotor Harvest  = null;
    // public DcMotor duckie = null;
    public DcMotor liftA = null;
    public DcMotor liftB = null;


    //HARVESTER SERVOS
   public Servo    L1  = null;
   public Servo   L2  = null;
   public   Servo  wrist = null;
   public Servo claw = null;
   public Servo planeS = null;
//    public CRServoImplEx R1  = null;
//    public CRServo    R2  = null;


    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.98; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    private List<Integer> lastEncPositions;

    /* local OpMode members. */
    //DECLARING HARDWARE MAP AND A TIME SYSTEM
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    /* Constructor */
    public BotHardware(){

    }


    /* Initialize standard Hardware interfaces */
    //SAYS IT CAN BE ACCESSED or transported
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
//        revBlinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, LED)
        // Define and Initialize Motors

        fr  = hwMap.get(DcMotor.class, "fr");
        br = hwMap.get(DcMotor.class, "br");
        fl    = hwMap.get(DcMotor.class, "fl");
        bl  = hwMap.get(DcMotor.class, "bl");
        // Harvest = hwMap.get(DcMotor.class, "Harvest");
        // duckie = hwMap.get(DcMotor.class, "duckie");
        liftA = hwMap.get(DcMotor.class, "liftA");
        liftB = hwMap.get(DcMotor.class, "liftB");
        planeS = hwMap.get(Servo.class, "plane");
        // magnetArm = hwMap.get(DcMotor.class,"magnetArm");
        //distancePole = hwMap.get(DistanceSensor.class, "distancePole");
//        distanceTower = hwMap.get(DistanceSensor.class, "ds");

        //SETING MOTOR DIRECTIONS
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        liftA.setDirection(DcMotor.Direction.FORWARD);
        liftB.setDirection(DcMotorSimple.Direction.REVERSE);

        // Harvest.setDirection(DcMotor.Direction.FORWARD);
        // duckie.setDirection(DcMotor.Direction.FORWARD);
//        lift.setDirection(DcMotor.Direction.FORWARD);
        //magnetArm.setDirection(CRServo.Direction.FORWARD);

//        leftEncoder = ahwMap.get(DcMotorEx.class, "leftEncoder");
//        rightEncoder = ahwMap.get(DcMotorEx.class, "rightEncoder");
//        frontEncoder = ahwMap.get(DcMotorEx.class, "frontEncoder");
//        rightEncoder.setDirection(DcMotor.Direction.REVERSE);

//        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set all MOTOR zero power
        fr.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
        liftA.setPower(0);
        liftB.setPower(0);
//        lift.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Harvest.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // duckie.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 //       lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //magnetArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        imu = hwMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);


        // // Define and initialize ALL installed servos.
//        wrist  = hwMap.get(Servo.class, "wrist");
//        distancePoleServo = hwMap.get(ServoImplEx.class, "dsServo");
//        encoderServo  = hwMap.get(Servo.class, "encoderServo");

        //HARVESTER SERVOS
        L1  = hwMap.get(Servo.class, "L1");
        L2  = hwMap.get(Servo.class, "R1");
        claw  = hwMap.get(Servo.class, "claw");
        wrist  = hwMap.get(Servo.class, "wrist");
        //wrist.setPosition(0);
        L1.setPosition(constants.OUTTAKEA_IN);
       L2.setPosition(constants.OUTTAKEB_IN);
       claw.setPosition(constants.CLAW_CLOSE);
       liftA.setPower(0);
       liftB.setPower(0);
        wrist.setPosition(constants.WRIST_UP);

//        encoderServo.setPosition(0.41);

    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    //SET MOTOR SPEEDS AND DIRECTIONS USUALLY WITH MATH ABS
    public void forward(double speed) {
        fr.setPower(Math.abs(speed));
        fl.setPower(Math.abs(speed));
        br.setPower(Math.abs(speed));
        bl.setPower(Math.abs(speed));
    }

    public void backward(double speed) {
        fr.setPower(-Math.abs(speed));
        fl.setPower(-Math.abs(speed));
        br.setPower(-Math.abs(speed));
        bl.setPower(-Math.abs(speed));
    }

    public void left(double speed) {
        fr.setPower(-Math.abs(speed));
        fl.setPower(Math.abs(speed));
        br.setPower(Math.abs(speed));
        bl.setPower(-Math.abs(speed));
    }

    public void right(double speed) {
        fr.setPower(Math.abs(speed));
        fl.setPower(-Math.abs(speed));
        br.setPower(-Math.abs(speed));
        bl.setPower(Math.abs(speed));
    }

    public void turnLeft(double speed) {
        fr.setPower(Math.abs(speed));
        fl.setPower(-Math.abs(speed));
        br.setPower(Math.abs(speed));
        bl.setPower(-Math.abs(speed));
    }

    public void turnRight(double speed) {
        fr.setPower(-Math.abs(speed));
        fl.setPower(Math.abs(speed));
        br.setPower(-Math.abs(speed));
        bl.setPower(Math.abs(speed));
    }
    public void BackwardLeft (double speed) {
        fr.setPower(-Math.abs(speed));
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(-Math.abs(speed));
    }
    public void stop(){
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }
    public void liftsUp(double power){
        liftA.setPower(power);
        liftB.setPower(power);
    }
    public void liftsDown(double power){
        liftA.setPower(-power);
        liftB.setPower(-power);
    }
    public void liftsStop(){
        liftA.setPower(0);
        liftB.setPower(0);
    }
    private static final String VUFORIA_KEY = "ARqAbkD/////AAABmYi1HwDMnkU3t4Ggmxc2sPZEB8shz803H5JcicPgUZzk3nVoKyjpce5D1DkDsL3r6W0mt+bA7Cm3D9fZLQunIeI+UUMdhJ/4kdSq5P6YsbmySAyR5ob6Kc2P8YDTP4KZfCx0khbaJ3lBe6pbDABHBbgW1QIOo3XwBrkSokI5v7avLY6o4CoKwWaTyK1nAJjc4HHe/nzSY2YNvQw+gMT8U/RDOND5qOn/XJd4BbiLIHF5ggkNQNZZ1XZFsoLklIVywpDKN8VUrZcbulGvtsdyEMYKght023avFsjaI62OUQVxXXtWZ08uHQLOmnrcyGSxzDcjAHIXRcPwi65aTvemw1DUYd3MEu/VEMJuKxDOySwf";
    private static final String TFOD_MODEL_FILE = "blueteamprop";
    private static final boolean USE_WEBCAM = true;
    // Function to detect the blue block and determine the line during init
//    public void initVuforia() {
//        /*
//         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//         */
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam");
//
//        //  Instantiate the Vuforia engine
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
//    }
//
//    /**
//     * Initialize the Tensor Flow Object Detection engine.
//     */
//
//
//    private static final String LABEL_BLUECUBE = "blueCube";
//    public void initTfod() {
//        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_BLUECUBE);
//    }

}