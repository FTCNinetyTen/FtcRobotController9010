package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.Hardware2023;

@TeleOp(name="HDWTestOp", group="TeleOps")
public class HWTestTele  extends LinearOpMode {
    Hardware2023 hdw;

    double[] pidCoffs = { 0.15,0.1,0.005 };
    int pidCoffIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        hdw = new Hardware2023(hardwareMap, telemetry); //init hardware
        hdw.createHardware();

        telemetry.addData("[>]", "All set?");
        telemetry.update();
        //Initialize April Tag
        hdw.initAprilTag();

        waitForStart();
        telemetry.clearAll();


        //This is the main loop of operation.
        while (opModeIsActive()) {

            if (gamepad1.dpad_left) {
                hdw.setLnKP(pidCoffs[0]);
                hdw.setLnKI(pidCoffs[1]);
                hdw.setLnKD(pidCoffs[2]);
                hdw.moveXAxis(-10.0, -.5);
            }
            if (gamepad1.dpad_right) {
                hdw.setLnKP(pidCoffs[0]);
                hdw.setLnKI(pidCoffs[1]);
                hdw.setLnKD(pidCoffs[2]);
                hdw.moveXAxis(2.0, .5);
            }
            if (gamepad1.dpad_up) {
                telemetry.addLine().addData("[moving y >]  ", " Y ");
                telemetry.update();
                hdw.setLnKP(pidCoffs[0]);
                hdw.setLnKI(pidCoffs[1]);
                hdw.setLnKD(pidCoffs[2]);
                hdw.moveYAxis(10.0, .5);
            }
            if (gamepad1.dpad_down) {
                hdw.setLnKP(pidCoffs[0]);
                hdw.setLnKI(pidCoffs[1]);
                hdw.setLnKD(pidCoffs[2]);
                hdw.moveYAxis (-2.0, -.5);
            }

            //Move to April Tag, using new KID parameters.
            if (gamepad1.a) {
                hdw.setLnKP(pidCoffs[0]);
                hdw.setLnKI(pidCoffs[1]);
                hdw.setLnKD(pidCoffs[2]);
                hdw.moveByAprilTag(4 , 12, -1);
            }

            if ( gamepad1.b) {
                hdw.setLnKP(pidCoffs[0]);
                hdw.setLnKI(pidCoffs[1]);
                hdw.setLnKD(pidCoffs[2]);
                hdw.moveByAprilTag(5 , 12, 1);
            }

            if( gamepad1.x) {
                pidCoffIndex = 0;
                telemetry.addLine().addData("[Kp :]  ", pidCoffs[0]);
                telemetry.addLine().addData("[Ki :]  ", pidCoffs[1]);
                telemetry.addLine().addData("[Kd :]  ", pidCoffs[2]);
                telemetry.update();
            }

            if( gamepad1.y) {
                pidCoffIndex = 1;
                telemetry.addLine().addData("[Kp :]  ", pidCoffs[0]);
                telemetry.addLine().addData("[Ki :]  ", pidCoffs[1]);
                telemetry.addLine().addData("[Kd :]  ", pidCoffs[2]);
                telemetry.update();
            }

            if( gamepad1.back) {
                pidCoffIndex = 2;
                telemetry.addLine().addData("[Kp :]  ", pidCoffs[0]);
                telemetry.addLine().addData("[Ki :]  ", pidCoffs[1]);
                telemetry.addLine().addData("[Kd :]  ", pidCoffs[2]);
                telemetry.update();
            }

            if( gamepad1.left_bumper) {
                if ( pidCoffIndex == 2) {
                    pidCoffs[pidCoffIndex] -= 0.001;
                } else {
                    pidCoffs[pidCoffIndex] -= 0.01;
                }
                telemetry.addLine().addData("[Kp :]  ", pidCoffs[0]);
                telemetry.addLine().addData("[Ki :]  ", pidCoffs[1]);
                telemetry.addLine().addData("[Kd :]  ", pidCoffs[2]);
                telemetry.update();
                sleep(100);
            }

            if( gamepad1.right_bumper) {
                if ( pidCoffIndex == 2) {
                    pidCoffs[pidCoffIndex] += 0.001;
                } else {
                    pidCoffs[pidCoffIndex] += 0.01;
                }
                telemetry.addLine().addData("[Kp :]  ", pidCoffs[0]);
                telemetry.addLine().addData("[Ki :]  ", pidCoffs[1]);
                telemetry.addLine().addData("[Kd :]  ", pidCoffs[2]);
                telemetry.update();
                sleep(100);
            }

            if (gamepad1.left_stick_button) {

            }


        }
    }

}
