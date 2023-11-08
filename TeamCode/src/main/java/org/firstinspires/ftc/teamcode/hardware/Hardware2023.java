package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;
import android.util.Size;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * This is the Robot class for 2022-2023 FTC Season
 *
 */
public class Hardware2023 {

    public HardwareMap hwMap;
    private AprilTagProcessor aprilTagProc;
    private VisionPortal visionPortal;
    private TfodProcessor tfod;
    private static final String TFOD_MODEL_ASSET = "9010.tflite";
    private static final String[] LABELS = {
            "BlueTP","RedTP"
    };

    //servos
    public Servo boxGate = null;


    //motors
    public DcMotorEx wheelFrontRight = null;
    public DcMotorEx wheelFrontLeft = null;
    public DcMotorEx wheelBackRight = null;
    public DcMotorEx wheelBackLeft = null;
    public DcMotorEx yEncoder = null;
    public DcMotorEx xEncoder = null;
    public DcMotorEx vSlideM = null;
    public DcMotorEx vSlideS = null;

    //IMU
    public IMU imu = null;

    //This is max wheel and slide motor velocity.
    static public double ANGULAR_RATE = 2000.0;

    private final double xAxisCoeff = 216.5 ;  // How many degrees encoder to turn to run an inch in X Axis
    private final double yAxisCoeff = 216.5 ;  // How many degrees encoder to turn to run an inch in Y Axis

    private boolean debug = true;
    private Telemetry telemetry;

    //PID control parameter for turning.
    private double turnKP = 0.15;
    private double turnKI = 0.1;
    private double turnKD = 0.005;
    private double turnKF = 0.0;

    private double lnKP = 0.15;
    private double lnKI = 0.20;
    private double lnKD = 0.032;
    private double lnKF = 0.0;

    private double slideKP = 1.27;
    private double slideKI = 0.03;
    private double slideKD = 0.001;

    public double getLnKF() {   return lnKF;    }

    public double getTurnKP() {
        return turnKP;
    }

    public void setTurnKP(double turnKP) {
        this.turnKP = turnKP;
    }

    public double getTurnKI() {
        return turnKI;
    }

    public void setTurnKI(double turnKI) {
        this.turnKI = turnKI;
    }

    public double getTurnKD() {
        return turnKD;
    }

    public void setTurnKD(double turnKD) {
        this.turnKD = turnKD;
    }

    public double getTurnKF() {
        return turnKF;
    }

    public void setTurnKF(double turnKF) {
        this.turnKF = turnKF;
    }

    public void setLnKF(double lnKF) {
        this.lnKF = lnKF;
    }

    public double getLnKD() {
        return lnKD;
    }

    public void setLnKD(double lnKD) {
        this.lnKD = lnKD;
    }

    public double getLnKI() {
        return lnKI;
    }

    public void setLnKI(double lnKI) {
        this.lnKI = lnKI;
    }

    public double getLnKP() {
        return lnKP;
    }

    public void setLnKP(double lnKP) {
        this.lnKP = lnKP;
    }

    /**
     * Constructor
     * @param m This is the HardwareMap, which is configured on the driver station.
     * @param tm  The Telemetry object, used for debug purpose.
     */
    public Hardware2023(HardwareMap m, Telemetry tm )
    {
        hwMap = m;
        telemetry = tm;
    }


    /**
     * Initialize hardware.
     */
    public void createHardware() {

        wheelFrontRight = hwMap.get(DcMotorEx.class, "rfWheel");
        wheelFrontLeft = hwMap.get(DcMotorEx.class, "lfWheel");
        wheelBackRight = hwMap.get(DcMotorEx.class, "rrWheel");
        wheelBackLeft = hwMap.get(DcMotorEx.class, "lrWheel");

        xEncoder = hwMap.get(DcMotorEx.class, "dwX");
        yEncoder = hwMap.get(DcMotorEx.class, "dwY");


        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wheelFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        wheelBackLeft.setDirection(DcMotor.Direction.FORWARD);
        wheelFrontRight.setDirection(DcMotor.Direction.REVERSE);
        wheelBackRight.setDirection(DcMotor.Direction.REVERSE);

        wheelFrontRight.setVelocity(0);
        wheelBackRight.setVelocity(0);
        wheelFrontLeft.setVelocity(0);
        wheelBackLeft.setVelocity(0);

        //Initialize Slide
        vSlideM = hwMap.get(DcMotorEx.class, "vSlideM");
        vSlideS = hwMap.get(DcMotorEx.class, "vSlideS");
        vSlideM.setDirection(DcMotorSimple.Direction.FORWARD);
        vSlideM.setDirection(DcMotorSimple.Direction.REVERSE);

        vSlideM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlideM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        vSlideS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlideS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Get IMU.
        imu = hwMap.get(IMU.class, "imu");

        //TODO: Update accordingly for the orientation of Control Hub.
        //Our robot mount Control hub Logo face backward, and USB port is facing Up.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

        // end method initTfod()

    }
    public void closeVisionPortal (){
        visionPortal.close();
    }

    public  void initAprilTag() {

        // Create the AprilTag processor.
        aprilTagProc = new AprilTagProcessor.Builder()
                .setLensIntrinsics(1516.76, 1516.76, 950.833, 533.379)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder()
                .setCameraResolution(new Size(1920,1080));

        // Set the camera (webcam vs. built-in RC phone camera).

        visionPortalBuilder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        visionPortalBuilder.addProcessor(aprilTagProc);

        // Build the Vision Portal, using the above settings.
        visionPortal = visionPortalBuilder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()



    /**
     * This operation move robot forward/backward according to the input
     * @param distance  Distance in encoder degree , 360 for a full circle.  Positive for forward.
     * @param power Positive value move forward.  Value fom 0 - 1.
     */
    private void moveYAxisDegree(int distance, double power ) {

        wheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Put motor back into run with encoder mode.
        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int currenXPosition = xEncoder.getCurrentPosition();
        Log.d("9010", "current X Position " + currenXPosition);

        //Get current orientation.  Angle is between -180 to 180
        int currentPosition = yEncoder.getCurrentPosition();
        int targetPosition = currentPosition + distance;

        double startHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        Log.d("9010", "Start Heading " + startHeading);

        Log.d("9010", "Start Position: " + currentPosition );
        Log.d("9010", "End Position: " + targetPosition );


        int difference = distance;
        Log.d("9010", "Difference: " + difference );

        PIDFController lnPidfCrtler  = new PIDFController(lnKP, lnKI, lnKD, lnKF);
        Log.d("9010", "lnKp: " + lnKP + "  lnKI: " + lnKI + " lnKD: " + lnKD);
        //Give X compansation more KP
        PIDFController lnXPidfCrtler  = new PIDFController(lnKP, lnKI, lnKD, lnKF);
        Log.d("9010", "lnXKp: " + lnKP + "  lnXKI: " + lnKI + " lnXKD: " + lnKD);
        PIDFController turnPidfCrtler  = new PIDFController(turnKP, turnKI, turnKD, turnKF);
        Log.d("9010", "turnKp: " + turnKP + "  lnKI: " + turnKI + " turnKD: " + turnKD);


        lnPidfCrtler.setSetPoint(0);
        //Set tolerance as 0.5 degrees
        lnPidfCrtler.setTolerance(15);
        //set Integration between -0.5 to 0.5 to avoid saturating PID output.
        lnPidfCrtler.setIntegrationBounds(-1 , 1 );

        lnXPidfCrtler.setSetPoint(0);
        //Set tolerance as 0.5 degrees
        lnXPidfCrtler.setTolerance(20);
        //set Integration between -0.5 to 0.5 to avoid saturating PID output.
        lnXPidfCrtler.setIntegrationBounds(-1 , 1);

        turnPidfCrtler.setSetPoint(0);
        //Set tolerance as 0.5 degrees
        turnPidfCrtler.setTolerance(0.5);
        //set Integration between -0.5 to 0.5 to avoid saturating PID output.
        turnPidfCrtler.setIntegrationBounds(-1 , 1 );

        Log.d("9010", "Before entering Loop ");
        double rx;
        double xVelocity;

        long initMill = System.currentTimeMillis();

        while ( !(lnPidfCrtler.atSetPoint()&&lnXPidfCrtler.atSetPoint() )
                && ( (System.currentTimeMillis() -initMill  )<5000)  ) {
            currentPosition = yEncoder.getCurrentPosition();
            //Calculate new distance
            difference = currentPosition - targetPosition;
            double velocityCaculated = lnPidfCrtler.calculate(difference)*4;
            if (velocityCaculated > ANGULAR_RATE ) {
                velocityCaculated = ANGULAR_RATE;
            }
            if ( velocityCaculated < -ANGULAR_RATE) {
                velocityCaculated = -ANGULAR_RATE;
            }

            Log.d("9010", "=====================");
            Log.d("9010", "Difference: " + difference);
            Log.d("9010", "Current Position: " + currentPosition );
            Log.d("9010", "Calculated Velocity:  " + velocityCaculated );
            double turnError = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)- startHeading;
            rx = turnPidfCrtler.calculate(turnError)*200;
            Log.d("9010", "Turn Error: " + turnError );
            Log.d("9010", "Calculated rx:  " + rx );

            double xError = xEncoder.getCurrentPosition() - currenXPosition;
            Log.d("9010", "X Error " + xError);
            xVelocity = lnXPidfCrtler.calculate(xError)*7;
            Log.d("9010", "X Velocity:  " + xVelocity);

            wheelFrontLeft.setVelocity(velocityCaculated + rx - xVelocity);
            wheelBackLeft.setVelocity(velocityCaculated + rx+ xVelocity);
            wheelFrontRight.setVelocity(velocityCaculated - rx + xVelocity);
            wheelBackRight.setVelocity(velocityCaculated - rx - xVelocity);
        }

        //wheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelFrontRight.setVelocity(0);
        wheelFrontLeft.setVelocity(0);
        wheelBackRight.setVelocity(0);
        wheelBackLeft.setVelocity(0);

    }

    /**
     *
     * @param distance  Distance in inches .  Always positive
     * @param power Positive value move forward
     */
    public void moveYAxis(double distance, double power ) {
        moveYAxisDegree( -(int) Math.round( (float) distance * this.yAxisCoeff ),  -power ) ;
    }

    /**
     * This operation move robot lef/right according to the input
     * @param distance  Distance in encoder degree , 360 for a full circle.  Positive for right.
     * @param power Not used, calculated by PID controller. Kept for backward compatiability
     *
     */
    private void moveXAxisDegree(int distance, double power ) {

        wheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Put motor back into run with encoder mode.
        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int currentYPosition = yEncoder.getCurrentPosition();
        Log.d("9010", "current Y Position " + currentYPosition);

        //Get current orientation.  Angle is between -180 to 180
        int currentPosition = xEncoder.getCurrentPosition();
        int targetPosition = currentPosition + distance;

        double startHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        Log.d("9010", "Start Heading " + startHeading);

        Log.d("9010", "Start Position: " + currentPosition );
        Log.d("9010", "End Position: " + targetPosition );


        int difference = distance;
        Log.d("9010", "Difference: " + difference );



        PIDFController lnPidfCrtler  = new PIDFController(lnKP, lnKI, lnKD, lnKF);
        Log.d("9010", "lnKp: " + lnKP + "  lnKI: " + lnKI + " lnKD: " + lnKD);
        PIDFController lnYPidfCrtler  = new PIDFController(lnKP, lnKI, lnKD, lnKF);
        Log.d("9010", "lnYKp: " + lnKP + "  lnYKI: " + lnKI + " lnYKD: " + lnKD);
        PIDFController turnPidfCrtler  = new PIDFController(turnKP, turnKI, turnKD, turnKF);
        Log.d("9010", "turnKp: " + turnKP + "  lnKI: " + turnKI + " turnKD: " + turnKD);

        //Main PID Controller for X Axis move
        lnPidfCrtler.setSetPoint(0);
        //Set tolerance as 15 clicks in encoder
        lnPidfCrtler.setTolerance(15);
        //set Integration between -0.5 to 0.5 to avoid saturating PID output.
        lnPidfCrtler.setIntegrationBounds(-1 , 1 );

        //AUX PID controller compensating Y Axis move
        lnYPidfCrtler.setSetPoint(0);
        //Set tolerance as clicks in encoder
        lnYPidfCrtler.setTolerance(20);
        //set Integration between -0.5 to 0.5 to avoid saturating PID output.
        lnYPidfCrtler.setIntegrationBounds(-1 , 1 );

        //PID controller compensating for turn .
        turnPidfCrtler.setSetPoint(0);
        //Set tolerance as 0.5 degrees
        turnPidfCrtler.setTolerance(0.5);
        //set Integration between -0.5 to 0.5 to avoid saturating PID output.
        turnPidfCrtler.setIntegrationBounds(-1 , 1);

        Log.d("9010", "Before entering Loop ");
        double rx;
        double yVelocity;

        long initMill = System.currentTimeMillis();

        while ( ! (lnPidfCrtler.atSetPoint() && lnYPidfCrtler.atSetPoint())
                && ( (System.currentTimeMillis() -initMill  )<5000) ) {
            currentPosition = xEncoder.getCurrentPosition();
            //Calculate new distance
            difference = currentPosition - targetPosition;
            double velocityCaculated = lnPidfCrtler.calculate(difference)*4;
            if (velocityCaculated > ANGULAR_RATE ) {
                velocityCaculated = ANGULAR_RATE;
            }
            if ( velocityCaculated < -ANGULAR_RATE) {
                velocityCaculated = -ANGULAR_RATE;
            }

            Log.d("9010", "=====================");
            Log.d("9010", "Difference: " + difference);
            Log.d("9010", "Current Position: " + currentPosition );
            Log.d("9010", "Calculated Velocity:  " + velocityCaculated );
            double turnError = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)- startHeading;
            rx = turnPidfCrtler.calculate(turnError)*200;
            Log.d("9010", "Turn Error: " + turnError );
            Log.d("9010", "Calculated rx:  " + rx );

            double yError = yEncoder.getCurrentPosition() - currentYPosition;
            Log.d("9010", "Y Error " + yError);
            yVelocity = lnYPidfCrtler.calculate(yError)*7;
            Log.d("9010", "Y Vel:  " + yVelocity);

            wheelFrontLeft.setVelocity(-velocityCaculated + rx + yVelocity);
            wheelBackLeft.setVelocity(velocityCaculated + rx+ yVelocity);
            wheelFrontRight.setVelocity(velocityCaculated - rx+ yVelocity);
            wheelBackRight.setVelocity(-velocityCaculated - rx+ yVelocity);
        }


        //wheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelFrontRight.setVelocity(0);
        wheelFrontLeft.setVelocity(0);
        wheelBackRight.setVelocity(0);
        wheelBackLeft.setVelocity(0);
    }


    /**
     * This operation move robot left/right according to the input
     * @param distance  Distance inch , positive to the right.
     * @param power Positive value move right.
     */

    public void moveXAxis(double  distance, double power ) {
        moveXAxisDegree((int) Math.round((float) distance * xAxisCoeff), power);
    }

    private int getXAxisPosition( ) {
        return  wheelFrontLeft.getCurrentPosition() ;
    }

    private int getYAxisPosition( ) {
        return  wheelFrontLeft.getCurrentPosition() ;
    }

    /**
     * Turn robot direction.
     *
     * @param degree  Degrees to turn,  Positive is turn counter clock wise.
     */
    public void turn( double degree) {
        wheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Put motor back into run with encoder mode.
        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Get current orientation.  Angle is between -180 to 180
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double startHeading = orientation.getYaw(AngleUnit.DEGREES);
        double endHeading = regulateDegree( startHeading + degree );

        double currentHeading = startHeading;

        Log.d("9010", "Start Heading: " + startHeading );
        Log.d("9010", "End Heading: " + endHeading );

        double difference = regulateDegree( currentHeading - endHeading   );
        Log.d("9010", "Difference: " + difference );

        PIDFController turnPidfCrtler  = new PIDFController(turnKP, turnKI, turnKD, turnKF);
        Log.d("9010", "Kp: " + turnKP + "  turnKI: " + turnKI + " turnKD: " + turnKD);

        turnPidfCrtler.setSetPoint(0);
        //Set tolerance as 0.5 degrees
        turnPidfCrtler.setTolerance(0.5);
        //set Integration between -0.5 to 0.5 to avoid saturating PID output.
        turnPidfCrtler.setIntegrationBounds(-0.5 , 0.5 );

        Log.d("9010", "Before entering Loop ");

        while ( !turnPidfCrtler.atSetPoint()  ) {
            currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            //Calculate new distance
            difference = regulateDegree(  currentHeading - endHeading );
            double velocityCaculated = turnPidfCrtler.calculate(difference)/10;

            /*
            Log.d("9010", "=====================");
            Log.d("9010", "Difference: " + difference);
            Log.d("9010", "Current Heading: " + currentHeading );
            Log.d("9010", "Calculated Volocity:  " + velocityCaculated );
             */

            wheelFrontLeft.setVelocity(velocityCaculated * Hardware2023.ANGULAR_RATE);
            wheelBackLeft.setVelocity(velocityCaculated * Hardware2023.ANGULAR_RATE);
            wheelFrontRight.setVelocity(-velocityCaculated * Hardware2023.ANGULAR_RATE);
            wheelBackRight.setVelocity(-velocityCaculated * Hardware2023.ANGULAR_RATE);
        }


        //wheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelFrontRight.setVelocity(0);
        wheelFrontLeft.setVelocity(0);
        wheelBackRight.setVelocity(0);
        wheelBackLeft.setVelocity(0);
    }

    /**
     * Regulate degreee between -180 and 180, by adding or subtracting 360.
     *
     * @param degree
     * @return
     */
    private double regulateDegree ( double degree ) {
       if ( degree > 180) {
           degree -= 360;
       } else if ( degree < -180 ) {
           degree += 360;
       }

       return degree;
    }

    /**
     * Move Robot according to the position of the April Tag.  Robot suppose to be square with the
     * april tag
     *
     * @param tagId    Id of the tag to be used for reference.
     * @param targetY   distance of robot to the april tag,  unit in inches.
     * @param targetX   horizontal shift to the center of april tag.  unit in inches.  Positive
     *                  means tag is on the right of robot camera.
     */
    public void moveByAprilTag( int tagId,  double targetY  ,  double targetX  ) throws InterruptedException {
        //visionPortal.setProcessorEnabled(aprilTagProc,true);

        //Set motor to run in encoder mode, use angular velocity to control motor instead of power.
        wheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Start April Tag detection fo find tag, possible multiple tag in camera frame,
        //So result is a list.
        List<AprilTagDetection> currentDetections = aprilTagProc.getDetections();
        if (currentDetections.size()<1 ) {
            //No tag found, do nothing.
            telemetry.addData("No AprilTags Detected ", currentDetections.size());
            telemetry.update();
            Log.d("9010", "No AprilTags Detected "  + tagId );
            //visionPortal.setProcessorEnabled(aprilTagProc,false);
            return;
        } else {
            //Loop though tags found
            for ( AprilTagDetection detection : currentDetections) {
                if ( detection.id == tagId) {
                    //Here we found our target April Tag.
                    //Get Initial Error
                    double initX = detection.ftcPose.x;
                    Log.d("9010", "current X " + initX);
                    double initY = detection.ftcPose.y;
                    Log.d("9010", "current Y  " + initY);
                    double initYaw = detection.ftcPose.yaw;
                    Log.d("9010", "current Yaw " + initYaw);

                    double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                    Log.d("9010", "current heading from IMU: " + currentHeading);
                    double endHeading = regulateDegree( currentHeading + initYaw );
                    Log.d("9010", "End heading by correcting Yaw: " + endHeading);

                    //Initialize PID controller for X, Y and turn
                    PIDFController lnYPidfCrtler  = new PIDFController(lnKP, lnKI, lnKD, lnKF);
                    Log.d("9010", "Y Kp: " + lnKP + "  KI: " + lnKI + " KD: " + lnKD);
                    PIDFController lnXPidfCrtler  = new PIDFController(lnKP, lnKI, lnKD, lnKF);
                    Log.d("9010", "X Kp: " + lnKP + "  KI: " + lnKI + " KD: " + lnKD);
                    PIDFController turnPidfCrtler  = new PIDFController(turnKP, turnKI, turnKD, turnKF);
                    Log.d("9010", "turn Kp: " + turnKP + "  KI: " + turnKI + " KD: " + turnKD);

                    lnYPidfCrtler.setSetPoint(0);
                    //Set Y tolerance as 0.2 inches
                    lnYPidfCrtler.setTolerance(0.2);
                    //set Integration between -0.5 to 0.5 to avoid saturating PID output.
                    lnYPidfCrtler.setIntegrationBounds(-1 , 1 );

                    lnXPidfCrtler.setSetPoint(0);
                    //Set X tolerance as 0.2 inches
                    lnXPidfCrtler.setTolerance(0.2);
                    //set Integration between -0.5 to 0.5 to avoid saturating PID output.
                    lnXPidfCrtler.setIntegrationBounds(-1 , 1 );

                    turnPidfCrtler.setSetPoint(0);
                    //Set tolerance as 0.5 degrees
                    turnPidfCrtler.setTolerance(0.5);
                    //set Integration between -0.5 to 0.5 to avoid saturating PID output.
                    turnPidfCrtler.setIntegrationBounds(-1 , 1 );

                    double xError =0 ;
                    double yError =0;
                    double rx = 0;
                    boolean newDetectionFound = false;

                    Log.d("9010", "Before entering Loop ");
                    long initMill = System.currentTimeMillis();
                    while ( !(lnYPidfCrtler.atSetPoint()&&lnXPidfCrtler.atSetPoint()&& turnPidfCrtler.atSetPoint() )
                            && ( (System.currentTimeMillis() -initMill  )<5000)  ) {

                        //Save CPU
                        Thread.sleep(20);

                        //Get new april tag  detection
                        List<AprilTagDetection> loopDetections = aprilTagProc.getDetections();
                        newDetectionFound = false;
                        for (AprilTagDetection loopDetetion : loopDetections) {
                            if ( loopDetetion.id == tagId) {
                                Log.d("9010", "===================================" );
                                Log.d("9010" , "Find new detection for Tag: " + tagId);
                                //Calculate X, Y and turn by the april tag input
                                xError =   targetX -loopDetetion.ftcPose.x;

                                double xVelocityCaculated = lnXPidfCrtler.calculate(xError) * xAxisCoeff *2;
                                if (xVelocityCaculated > ANGULAR_RATE ) {
                                    xVelocityCaculated = ANGULAR_RATE;
                                }
                                if ( xVelocityCaculated < -ANGULAR_RATE) {
                                    xVelocityCaculated = -ANGULAR_RATE;
                                }

                                Log.d("9010", "x  Error: " + xError );
                                Log.d("9010", "Calculated x Velocity:  " + xVelocityCaculated );

                                yError =  loopDetetion.ftcPose.y - targetY;
                                double yVelocityCaculated = lnYPidfCrtler.calculate(yError)* yAxisCoeff * 1.5;
                                if (yVelocityCaculated > ANGULAR_RATE ) {
                                    yVelocityCaculated = ANGULAR_RATE;
                                }
                                if ( yVelocityCaculated < -ANGULAR_RATE) {
                                    yVelocityCaculated = -ANGULAR_RATE;
                                }

                                Log.d("9010", "Y  Error: " + yError );
                                Log.d("9010", "Calculated Y Velocity:  " + yVelocityCaculated );

                                //Target Yaw is 0
                                //Instead of using april tag reading,  use initial reading by April
                                //Tag and in the loop use IMU.
                                currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                                double turnError = regulateDegree( currentHeading - endHeading );
                                rx = turnPidfCrtler.calculate(turnError)*35;

                                Log.d("9010", "Turn Error: " + turnError );
                                Log.d("9010", "Calculated rx:  " + rx );

                                wheelFrontLeft.setVelocity(yVelocityCaculated + rx - xVelocityCaculated);
                                wheelBackLeft.setVelocity(yVelocityCaculated + rx+ xVelocityCaculated);
                                wheelFrontRight.setVelocity(yVelocityCaculated - rx + xVelocityCaculated);
                                wheelBackRight.setVelocity(yVelocityCaculated - rx - xVelocityCaculated);

                                newDetectionFound = true;
                            }
                        }

                        if(!newDetectionFound) {
                            Log.d("9010" , "Can't find april tag, Abort tag id: " + tagId);
                            wheelFrontRight.setVelocity(0);
                            wheelFrontLeft.setVelocity(0);
                            wheelBackRight.setVelocity(0);
                            wheelBackLeft.setVelocity(0);
                        }

                    } // while loop

                    Log.d("9010", "After PID Loop ");
                    wheelFrontRight.setVelocity(0);
                    wheelFrontLeft.setVelocity(0);
                    wheelBackRight.setVelocity(0);
                    wheelBackLeft.setVelocity(0);
                } // if ( detection.id == tagId) {
            } // for ( AprilTagDetection detection : currentDetections) {
        } //End of  if (currentDetections.size()<1 )   {  } else

     //visionPortal.setProcessorEnabled(aprilTagProc,false);
    } // End of public void moveByAprilTag


    /**
     * Move vertical Slide freely , using game control
     * @param power
     */
    public void freeMoveVerticalSlide(float power ) {

        PIDController slidePID = new PIDController(slideKP, slideKI, slideKD);
        slidePID.setSetPoint(0);
        //Set tolerance as 0.5 degrees
        slidePID.setTolerance(10);
        //set Integration between -0.5 to 0.5 to avoid saturating PID output.
        slidePID.setIntegrationBounds(-0.5 , 0.5 );


        double diff = vSlideM.getCurrentPosition() - vSlideS.getCurrentPosition();

        double calculatedVelocity = slidePID.calculate(diff)   ;
        if (calculatedVelocity > ANGULAR_RATE ) {
            calculatedVelocity = ANGULAR_RATE;
        }
        if ( calculatedVelocity < -ANGULAR_RATE) {
            calculatedVelocity = -ANGULAR_RATE;
        }

        //Control 2 Vslide in Sync
        vSlideM.setVelocity(power * ANGULAR_RATE);
        vSlideS.setVelocity(power *ANGULAR_RATE - calculatedVelocity );
        //vSlideS.setVelocity(power *ANGULAR_RATE );

    }


    /**
     *
     * @param height  This is the encoder reading for the height
     *
     */
    public void moveSlideToHeight ( int height ) {
        int targetPosition = height;
        int currentPosition = vSlideM.getCurrentPosition();

        vSlideM.setTargetPosition(targetPosition);
        vSlideS.setTargetPosition(targetPosition);
        vSlideM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vSlideS.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int sign = 1;

        if ((currentPosition - targetPosition) > 0 ) {
            sign= -1;
        } else {
            //raise slide
            sign = 1;
        }

        while (vSlideM.isBusy()) {
            vSlideM.setVelocity( sign * ANGULAR_RATE* 0.5 );
            vSlideS.setVelocity( sign * ANGULAR_RATE* 0.5 );
            //Log.d("9010", "Inside Moving Loop : " + vSlide.getCurrentPosition() + " Sign: " + sign);
        }
        vSlideM.setVelocity(0);
        vSlideS.setVelocity(0);
        Log.d("9010", "after Moving, slide position : " + vSlideM.getCurrentPosition());

        //Set mode back to Run using encoder.
        vSlideM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vSlideS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void openBox () {
         boxGate.setPosition(.5);

    }


    /**
     * This method detect the posiiton of Team Prop
     *
     * @return
     */
    public TeamPropPosition detectTeamProp (String targetTeamProp ) throws InterruptedException {
        int left = 80;
        int center = 300;
        int right = 500;
        TeamPropPosition foundPosition= null;

        initTfod();

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        if (currentRecognitions.size()<1) {
            //If can't find any object.
            Log.d("9010", "Can't find object " + currentRecognitions.size());
            Thread.sleep(1000);
            currentRecognitions = tfod.getRecognitions();
            if (currentRecognitions.size()<1) {
                Log.d("9010", "Still Can't find object, " + currentRecognitions.size());
                closeVisionPortal();
                return TeamPropPosition.UNKOWN;
            }
        }

        // Find out the correct regconition.
        List<Recognition> filteredRec = new ArrayList<Recognition>();
        for (Recognition recognition : currentRecognitions) {
            if (recognition.getLabel().equals(targetTeamProp)) {
                filteredRec.add(recognition);
            }
        }

        class recComparator implements Comparator<Recognition> {
            @Override
            public int compare(Recognition recognition, Recognition t1) {
                return Math.round( t1.getConfidence() -recognition.getConfidence()  );
            }
        }
        Collections.sort(filteredRec, new recComparator());

        for (Recognition recognition: filteredRec) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;
            Log.d("9010", "Label: " + recognition.getLabel());
            Log.d("9010","Confidence: " + recognition.getConfidence() );
            Log.d("9010","X: " + x );
        }

        Recognition discoveredRec = filteredRec.get(0);
        double x = (discoveredRec.getLeft() + discoveredRec.getRight()) / 2;
        double diff1 = Math.abs(x - left);
        double diff2 = Math.abs(x - center);
        double diff3 = Math.abs(x - right);

        double smallest = Math.min(diff1, Math.min(diff2, diff3));
        if ( smallest == diff1) {
            foundPosition = TeamPropPosition.LEFT;
        } else if (smallest == diff2 ){
            foundPosition = TeamPropPosition.CENTER;
        } else if ( smallest == diff3 ) {
            foundPosition = TeamPropPosition.RIGHT;
        }

        closeVisionPortal();
        return foundPosition;
    }


}
