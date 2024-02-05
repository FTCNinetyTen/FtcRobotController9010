package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.Hardware2023;

@TeleOp(name="HDWTestOp", group="TeleOps")
public class HWTestTele  extends LinearOpMode {
    Hardware2023 hdw;

    double[] pidCoffs = { 1.5,0.15,0.11 };
    int pidCoffIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        hdw = new Hardware2023(hardwareMap, telemetry); //init hardware
        hdw.createHardware();

        telemetry.addData("[>]", "All set?");
        telemetry.update();
        //Initialize April Tag
        //hdw.initVision();
        //hdw.disableTFOD();


        waitForStart();
        telemetry.clearAll();

        Gamepad previousGamePad1 = new Gamepad();
        Gamepad currentGamePad1 = new Gamepad();

        //This is the main loop of operation.
        while (opModeIsActive()) {
            //Record previous Gamepad Status
            previousGamePad1.copy(currentGamePad1);
            //Update current gamepad status
            currentGamePad1.copy(gamepad1);

            if (gamepad1.dpad_left) {
                hdw.setLnKP(pidCoffs[0]);
                hdw.setLnKI(pidCoffs[1]);
                hdw.setLnKD(pidCoffs[2]);
                hdw.moveXAxis(-12.0, -.5);
            }
            if (gamepad1.dpad_right) {
                hdw.setLnKP(pidCoffs[0]);
                hdw.setLnKI(pidCoffs[1]);
                hdw.setLnKD(pidCoffs[2]);
                hdw.moveXAxis(4.0, .5);
            }
            if (gamepad1.dpad_up) {
                telemetry.addLine().addData("[moving y >]  ", " Y ");
                telemetry.update();
                hdw.setLnKP(pidCoffs[0]);
                hdw.setLnKI(pidCoffs[1]);
                hdw.setLnKD(pidCoffs[2]);
                hdw.moveYAxis(2.0, .5);
            }
            if (gamepad1.dpad_down) {
                hdw.setLnKP(pidCoffs[0]);
                hdw.setLnKI(pidCoffs[1]);
                hdw.setLnKD(pidCoffs[2]);
                hdw.moveYAxis (-4.0, -.5);
            }

            //Move to April Tag, using new KID parameters.
            if (gamepad1.a) {
                hdw.setLnKP(pidCoffs[0]);
                hdw.setLnKI(pidCoffs[1]);
                hdw.setLnKD(pidCoffs[2]);
                hdw.moveByAprilTag(4 , 14, -1);
            }

            if ( gamepad1.b) {
                hdw.setLnKP(pidCoffs[0]);
                hdw.setLnKI(pidCoffs[1]);
                hdw.setLnKD(pidCoffs[2]);
                hdw.moveByAprilTag(5 , 18, 1);
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

            /*
            if (!previousGamePad1.left_bumper & currentGamePad1.left_bumper) {
                hdw.moveSlideToHeight(200);
            }
            if (previousGamePad1.left_bumper & !currentGamePad1.left_bumper) {
                hdw.moveSlideToHeight(0);
            }
            if (!previousGamePad1.right_bumper & currentGamePad1.right_bumper) {
                hdw.moveSlideToHeight(500);
            }
            if (previousGamePad1.right_bumper & !currentGamePad1.right_bumper) {
                hdw.moveSlideToHeight(0);
            } */

            if (previousGamePad1.right_stick_button && !currentGamePad1.right_stick_button) {
                hdw.closeBox();
            }
            if (!previousGamePad1.right_stick_button && currentGamePad1.right_stick_button) {
                hdw.openBox();
            }


            //hdw.boxGate.setPosition(gamepad1.left_stick_x);
            //Log.d("9010", "Servo positon: " + gamepad1.left_stick_x);

        }
    }

}
