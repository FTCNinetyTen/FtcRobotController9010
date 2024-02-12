package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;


import org.firstinspires.ftc.teamcode.hardware.Hardware2023;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheels2023;


public class BaseGeneralDriver2023 extends LinearOpMode {

    private boolean debug = true;
    private Hardware2023 hdw;
    private MecanumWheels2023 robotWheel;

    String alliance ;
    String BLUE="Blue";
    String RED="Red";
    boolean intakePower = false;

    @Override
    public void runOpMode() throws InterruptedException {
        hdw = new Hardware2023(hardwareMap, telemetry); //init hardware
        hdw.createHardware();
        robotWheel = new MecanumWheels2023();

        double powerDrivePercentage = 0.75;

        telemetry.addData("[>]", "All set? For allience: " + alliance);
        telemetry.update();

        waitForStart();
        telemetry.clearAll();

        Gamepad previousGamePad1 = new Gamepad();
        Gamepad currentGamePad1 = new Gamepad();

        //This is the main loop of operation
        currentGamePad1.copy(gamepad1);
        while (opModeIsActive()) {
            //Record previous Gamepad Status
            previousGamePad1.copy(currentGamePad1);
            //Update current gamepad status
            currentGamePad1.copy(gamepad1);

            //Use DPad to move to according April tag
            /* Commend out for 1st meet
            if (gamepad1.dpad_left) {
                Log.d("9010", "alliance " + alliance);
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
            } */


            if (!previousGamePad1.left_bumper & currentGamePad1.left_bumper) {
                if ( !intakePower ) {
                    hdw.intake.setPower(1);
                    intakePower = true;
                } else {
                    hdw.intake.setPower(0);
                    intakePower = false ;
                }
            }

            if (!previousGamePad1.right_bumper & currentGamePad1.right_bumper) {
                hdw.spitOutPixel();

            }

            //This is to toggle the heading, by pushing the button y.
            if ( currentGamePad1.y && ! previousGamePad1.y  ) {
                if (robotWheel.isHeadingForward()) {
                    robotWheel.setHeadingForward(false);
                } else {
                    robotWheel.setHeadingForward(true);
                }
            }

            //Open BOx on pushing down button
            if ( !currentGamePad1.x &&  previousGamePad1.x  ) {
                if (hdw.isBoxOpen()){
                    hdw.closeBox();
                    hdw.setBoxOpen(false);
                }
                else {
                    hdw.openBox();
                    hdw.setBoxOpen(true);
                }
            }


            if ( !currentGamePad1.b && ( previousGamePad1.b && previousGamePad1.back)  ) {
                Log.d("9010", "release and reset drone" + hdw.isDroneReleased());
                if (hdw.isDroneReleased()){
                    hdw.releaseDroneLauncher();
                    hdw.setDroneReleased(false);
                }
                else {
                    hdw.resetDroneLauncher();
                    hdw.setDroneReleased(true);
                }
            }

            if ( !currentGamePad1.a &&  previousGamePad1.a  ) {
               hdw.moveSlideToHeight(1500);
            }


            telemetry.clearAll();
            telemetry.addData("[>]", "Robot Heading Forward: " + robotWheel.isHeadingForward());
            telemetry.update();

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
