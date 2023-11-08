package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.hardware.Hardware2023;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheels;


public class BaseGeneralDriver2023 extends LinearOpMode {

    private boolean debug = true;
    private Hardware2023 hdw;
    private MecanumWheels robotWheel;

    String alliance ;
    String BLUE="Blue";
    String RED="Red";

    @Override
    public void runOpMode() throws InterruptedException {
        hdw = new Hardware2023(hardwareMap, telemetry); //init hardware
        hdw.createHardware();
        robotWheel = new MecanumWheels();

        double powerDrivePercentage = 0.55;

        telemetry.addData("[>]", "All set? For allience: " + alliance);
        telemetry.update();

        waitForStart();
        telemetry.clearAll();

        //This is the main loop of operation.
        while (opModeIsActive()) {

            //Use DPad to move to according April tag
            if (gamepad1.dpad_left) {
                if ( alliance.equals(BLUE)) {
                    hdw.moveByAprilTag(1 , 12, 0);
                } else {
                    hdw.moveByAprilTag(4 , 12, 0);
                }
            }
            if (gamepad1.dpad_up) {
                if ( alliance.equals(BLUE)) {
                    hdw.moveByAprilTag(2 , 12, 0);
                } else {
                    hdw.moveByAprilTag(5 , 12, 0);
                }
            }
            if (gamepad1.dpad_right) {
                if ( alliance.equals(BLUE)) {
                    hdw.moveByAprilTag(3 , 12, 0);
                } else {
                    hdw.moveByAprilTag(6 , 12, 0);
                }
            }

            if (gamepad1.y) {
                if (robotWheel.isHeadingForward()) {
                    robotWheel.setHeadingForward(false);
                } else {
                    robotWheel.setHeadingForward(true);

                }
                telemetry.addData("[>]", "Robot Heading Forword: " + robotWheel.isHeadingForward());
            }


            hdw.freeMoveVerticalSlide(gamepad1.right_trigger - gamepad1.left_trigger);

            //Wheel takes input of gampad 1  ,  turbo is the power factor. Range 0-1 , 1 is 100%
            robotWheel.joystick(gamepad1, 1);

            /* Set the calcuated velocity to wheels according to the gampad input */
            double frontLeftVelocity = robotWheel.wheelFrontLeftPower * powerDrivePercentage * Hardware2023.ANGULAR_RATE;
            double backLeftVelocity = robotWheel.wheelBackLeftPower * powerDrivePercentage * Hardware2023.ANGULAR_RATE;
            double frontRightVelocity = robotWheel.wheelFrontRightPower * powerDrivePercentage * Hardware2023.ANGULAR_RATE;
            double backRightVelocity = robotWheel.wheelBackRightPower * powerDrivePercentage * Hardware2023.ANGULAR_RATE;

            hdw.wheelFrontLeft.setVelocity(frontLeftVelocity);
            hdw.wheelBackLeft.setVelocity(backLeftVelocity);
            hdw.wheelFrontRight.setVelocity(frontRightVelocity);
            hdw.wheelBackRight.setVelocity(backRightVelocity);

        }

    }

}
