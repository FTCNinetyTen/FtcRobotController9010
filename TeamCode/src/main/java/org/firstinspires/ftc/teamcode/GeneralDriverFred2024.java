package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware2024Fred;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheels2023;

@TeleOp(name="GeneralDriverFred2024", group="TeleOps")

public class GeneralDriverFred2024 extends LinearOpMode {

    private boolean debug = true;
    Hardware2024Fred hdw;

    MecanumWheels2023 robotWheel;

    @Override
    public void runOpMode() throws InterruptedException {
        hdw = new Hardware2024Fred(hardwareMap, telemetry); //init hardware
        hdw.createHardware();
        robotWheel = new MecanumWheels2023();

        telemetry.addData("[>]", "All set?");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();

        //This is the main loop of operation.
        while (opModeIsActive()) {
            //hdw.checkAndGrabCone();

            //Wheel takes input of gampad 1  ,  turbo is the power factor. Range 0-1 , 1 is 100%
            robotWheel.joystick(gamepad1, .5);


            hdw.wheelFrontLeft.setPower(robotWheel.wheelFrontLeftPower);
            hdw.wheelBackLeft.setPower(robotWheel.wheelBackLeftPower);
            hdw.wheelFrontRight.setPower(robotWheel.wheelFrontRightPower);
            hdw.wheelBackRight.setPower(robotWheel.wheelBackRightPower);


        }

    }

}

