package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.hardware.Hardware2023;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheels;

@TeleOp(name="GeneralDriver2023", group="TeleOps")
public class GeneralDriver2023 extends LinearOpMode {

    private boolean debug = true;
    Hardware2023 hdw;

    MecanumWheels robotWheel;

    @Override
    public void runOpMode() throws InterruptedException {
        hdw = new Hardware2023(hardwareMap, telemetry); //init hardware
        hdw.createHardware();
        robotWheel = new MecanumWheels();

        double powerDrivePercentage = 0.55;

        telemetry.addData("[>]", "All set?");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();

        //This is the main loop of operation.
        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                hdw.moveByAprilTag(4 , 12, -1);
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
