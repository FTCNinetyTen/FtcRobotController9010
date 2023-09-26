package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.Hardware2023;

@TeleOp(name="HDWTestOp", group="TeleOps")
public class HWTestTele  extends LinearOpMode {
    Hardware2023 hdw;

    double[] pidCoffs = { 0.2,0.2,0.0 };
    int pidCoffIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        hdw = new Hardware2023(hardwareMap, telemetry); //init hardware
        hdw.createHardware();

        telemetry.addData("[>]", "All set?");
        telemetry.update();
        hdw.initAprilTag();

        waitForStart();
        telemetry.clearAll();


        //This is the main loop of operation.
        while (opModeIsActive()) {

            if (gamepad1.dpad_left) {
                hdw.moveXAxis(-10.0, -.5);
            }
            if (gamepad1.dpad_right) {
                hdw.moveXAxis(2.0, .5);
            }
            if (gamepad1.dpad_up) {
                telemetry.addLine().addData("[moving y >]  ", " Y ");
                telemetry.update();
                hdw.moveYAxis(10.0, .5);
            }
            if (gamepad1.dpad_down) {
                hdw.moveYAxis (-2.0, -.5);
            }

            if (gamepad1.a) {
                hdw.setTurnKP(pidCoffs[0]);
                hdw.setTurnKI(pidCoffs[1]);
                hdw.setTurnKD(pidCoffs[2]);
                hdw.turn(45);
            }

            if ( gamepad1.b) {
                hdw.setTurnKP(pidCoffs[0]);
                hdw.setTurnKI(pidCoffs[1]);
                hdw.setTurnKD(pidCoffs[2]);
                hdw.turn(-45);
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
                pidCoffs[pidCoffIndex] -= 0.01;
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
                hdw.moveByAprilTag(4 , 12, -1);
            }


        }
    }

}
