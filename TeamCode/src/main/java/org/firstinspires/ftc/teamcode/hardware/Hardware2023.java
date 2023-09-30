package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;
import android.util.Size;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * This is the Robot class for 2022-2023 FTC Season
 *
 */
public class Hardware2023 {

    public HardwareMap hwMap;
    private AprilTagProcessor aprilTagProc;
    private VisionPortal visionPortal;

    //motors
    public DcMotorEx wheelFrontRight = null;
    public DcMotorEx wheelFrontLeft = null;
    public DcMotorEx wheelBackRight = null;
    public DcMotorEx wheelBackLeft = null;
    public DcMotorEx yEncoder = null;
    public DcMotorEx xEncoder = null;

    //IMU
    public IMU imu =null ;

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
    private double lnKI = 0.1;
    private double lnKD = 0.005;
    private double lnKF = 0.0;

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

        xEncoder = hwMap.get(DcMotorEx.class, "xEncoder");
        yEncoder = hwMap.get(DcMotorEx.class, "yEncoder");


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
        PIDFController lnXPidfCrtler  = new PIDFController(0.2, lnKI, lnKD, lnKF);
        Log.d("9010", "lnXKp: " + lnKP + "  lnXKI: " + lnKI + " lnXKD: " + lnKD);
        PIDFController turnPidfCrtler  = new PIDFController(turnKP, turnKI, turnKD, turnKF);
        Log.d("9010", "turnKp: " + turnKP + "  lnKI: " + turnKI + " turnKD: " + turnKD);


        lnPidfCrtler.setSetPoint(0);
        //Set tolerance as 0.5 degrees
        lnPidfCrtler.setTolerance(15);
        //set Integration between -0.5 to 0.5 to avoid saturating PID output.
        lnPidfCrtler.setIntegrationBounds(-0.5 , 0.5 );

        lnXPidfCrtler.setSetPoint(0);
        //Set tolerance as 0.5 degrees
        lnXPidfCrtler.setTolerance(20);
        //set Integration between -0.5 to 0.5 to avoid saturating PID output.
        lnXPidfCrtler.setIntegrationBounds(-0.5 , 0.5 );

        turnPidfCrtler.setSetPoint(0);
        //Set tolerance as 0.5 degrees
        turnPidfCrtler.setTolerance(0.5);
        //set Integration between -0.5 to 0.5 to avoid saturating PID output.
        turnPidfCrtler.setIntegrationBounds(-0.5 , 0.5 );

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
        PIDFController lnYPidfCrtler  = new PIDFController(0.3, lnKI, lnKD, lnKF);
        Log.d("9010", "lnYKp: " + lnKP + "  lnYKI: " + lnKI + " lnYKD: " + lnKD);
        PIDFController turnPidfCrtler  = new PIDFController(turnKP, turnKI, turnKD, turnKF);
        Log.d("9010", "turnKp: " + turnKP + "  lnKI: " + turnKI + " turnKD: " + turnKD);

        //Main PID Controller for X Axis move
        lnPidfCrtler.setSetPoint(0);
        //Set tolerance as 15 clicks in encoder
        lnPidfCrtler.setTolerance(15);
        //set Integration between -0.5 to 0.5 to avoid saturating PID output.
        lnPidfCrtler.setIntegrationBounds(-0.5 , 0.5 );

        //AUX PID controller compensating Y Axis move
        lnYPidfCrtler.setSetPoint(0);
        //Set tolerance as clicks in encoder
        lnYPidfCrtler.setTolerance(20);
        //set Integration between -0.5 to 0.5 to avoid saturating PID output.
        lnYPidfCrtler.setIntegrationBounds(-0.5 , 0.5 );

        //PID controller compensating for turn .
        turnPidfCrtler.setSetPoint(0);
        //Set tolerance as 0.5 degrees
        turnPidfCrtler.setTolerance(0.5);
        //set Integration between -0.5 to 0.5 to avoid saturating PID output.
        turnPidfCrtler.setIntegrationBounds(-0.5 , 0.5 );

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
     * @param targetX   horizontal shift to the center of april tag.  unit in inces
     */
    public void moveByAprilTag( int tagId,  double targetY  ,  double targetX  ) {

        wheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Put motor back into run with encoder mode.
        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Start April Tag detection fo find tag
        List<AprilTagDetection> currentDetections = aprilTagProc.getDetections();
        if (currentDetections.size()<1 ) {
            //No tag found, do nothing.
            telemetry.addData("No AprilTags Detected", currentDetections.size());
            Log.d("9010", "No AprilTags Detected"  + tagId );
            return;
        } else {
            //Loop though tags found
            for ( AprilTagDetection detection : currentDetections) {
                if ( detection.id == tagId) {
                    //Here we found our target April Tag.
                    //Get Error
                    double currenX = detection.ftcPose.x;
                    Log.d("9010", "current X " + currenX);
                    double currenY = detection.ftcPose.y;
                    Log.d("9010", "current Y  " + currenY);
                    double currentYaw = detection.ftcPose.yaw;
                    Log.d("9010", "current Yaw " + currentYaw);

                    //Initialize PID controller for X, Y and turn
                    PIDFController lnYPidfCrtler  = new PIDFController(lnKP, lnKI, lnKD, lnKF);
                    Log.d("9010", "Y Kp: " + lnKP + "  KI: " + lnKI + " KD: " + lnKD);
                    PIDFController lnXPidfCrtler  = new PIDFController(lnKP, lnKI, lnKD, lnKF);
                    Log.d("9010", "X Kp: " + lnKP + "  KI: " + lnKI + " KD: " + lnKD);
                    PIDFController turnPidfCrtler  = new PIDFController(turnKP, turnKI, turnKD, turnKF);
                    Log.d("9010", "turn Kp: " + turnKP + "  KI: " + turnKI + " KD: " + turnKD);

                    lnYPidfCrtler.setSetPoint(0);
                    //Set tolerance as 0.5 degrees
                    lnYPidfCrtler.setTolerance(0.2);
                    //set Integration between -0.5 to 0.5 to avoid saturating PID output.
                    lnYPidfCrtler.setIntegrationBounds(-0.5 , 0.5 );

                    lnXPidfCrtler.setSetPoint(0);
                    //Set tolerance as 0.5 degrees
                    lnXPidfCrtler.setTolerance(0.2);
                    //set Integration between -0.5 to 0.5 to avoid saturating PID output.
                    lnXPidfCrtler.setIntegrationBounds(-0.5 , 0.5 );

                    turnPidfCrtler.setSetPoint(0);
                    //Set tolerance as 0.5 degrees
                    turnPidfCrtler.setTolerance(0.5);
                    //set Integration between -0.5 to 0.5 to avoid saturating PID output.
                    turnPidfCrtler.setIntegrationBounds(-0.5 , 0.5 );

                    double xError =0 ;
                    double yError =0;
                    double rx = 0;
                    boolean newDetectionFound = false;

                    Log.d("9010", "Before entering Loop ");
                    long initMill = System.currentTimeMillis();
                    while ( !(lnYPidfCrtler.atSetPoint()&&lnXPidfCrtler.atSetPoint()&& turnPidfCrtler.atSetPoint() )
                            && ( (System.currentTimeMillis() -initMill  )<5000)  ) {

                        //Get new april tag  detection
                        List<AprilTagDetection> loopDetections = aprilTagProc.getDetections();
                        newDetectionFound = false;
                        for (AprilTagDetection loopDetetion : loopDetections) {
                            if ( loopDetetion.id == tagId) {
                                Log.d("9010", "===================================" );
                                Log.d("9010" , "Find new detection for Tag: " + tagId);
                                //Calculate X, Y and turn by the april tag input
                                xError =  loopDetetion.ftcPose.x - targetX ;

                                double xVelocityCaculated = lnXPidfCrtler.calculate(xError) * xAxisCoeff/5;
                                if (xVelocityCaculated > ANGULAR_RATE ) {
                                    xVelocityCaculated = ANGULAR_RATE;
                                }
                                if ( xVelocityCaculated < -ANGULAR_RATE) {
                                    xVelocityCaculated = -ANGULAR_RATE;
                                }

                                Log.d("9010", "x  Error: " + xError );
                                Log.d("9010", "Calculated x Velocity:  " + xVelocityCaculated );

                                yError =  loopDetetion.ftcPose.y - targetY;
                                double yVelocityCaculated = lnYPidfCrtler.calculate(yError)* yAxisCoeff/2;
                                if (yVelocityCaculated > ANGULAR_RATE ) {
                                    yVelocityCaculated = ANGULAR_RATE;
                                }
                                if ( yVelocityCaculated < -ANGULAR_RATE) {
                                    yVelocityCaculated = -ANGULAR_RATE;
                                }

                                Log.d("9010", "Y  Error: " + yError );
                                Log.d("9010", "Calculated Y Velocity:  " + yVelocityCaculated );

                                //Target Yaw is 0
                                double turnError =  loopDetetion.ftcPose.yaw;
                                rx = turnPidfCrtler.calculate(turnError)*20;

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
                            break;
                        }

                    } // while loop

                    Log.d("9010", "After PID Loop ");
                    wheelFrontRight.setVelocity(0);
                    wheelFrontLeft.setVelocity(0);
                    wheelBackRight.setVelocity(0);
                    wheelBackLeft.setVelocity(0);
                } // if ( detection.id == tagId) {
            } // for ( AprilTagDetection detection : currentDetections) {

            telemetry.addData("Targeted AprilTags Not Detected", tagId);
            Log.d("9010", "Targeted AprilTags Not Detected"  + tagId );
        } //End of  if (currentDetections.size()<1 )   {  } else


    } // End of public void moveByAprilTag
}
