package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "ProtoTele2023", group = "TeleOp")
public class ProtoTele2023 extends LinearOpMode  {

    //private DcMotorEx wheelIntake = null;
    private DcMotorEx vSlideM = null;
    private DcMotorEx vSlideS = null;

    private double slideKP = 1.27;
    private double slideKI = 0.03;
    private double slideKD = 0.001;

    double[] pidCoffs = { 1.27,0.03,0.001 };
    int pidCoffIndex = 0;

    private double ANGULAR_RATE = 2200;

    @Override
    public void runOpMode() throws InterruptedException {
        //wheelIntake = hardwareMap.get(DcMotorEx.class, "inkWheel");
        vSlideM = hardwareMap.get(DcMotorEx.class, "vSlideM");
        vSlideS = hardwareMap.get(DcMotorEx.class, "vSlideS");
        vSlideM.setDirection(DcMotorSimple.Direction.FORWARD);
        vSlideM.setDirection(DcMotorSimple.Direction.REVERSE);

        vSlideM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlideM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        vSlideS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlideS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        float error = 0 ;
        PIDController slidePID = new PIDController(slideKP, slideKI, slideKD);
        slidePID.setSetPoint(0);
        //Set tolerance as 0.5 degrees
        slidePID.setTolerance(10);
        //set Integration between -0.5 to 0.5 to avoid saturating PID output.
        slidePID.setIntegrationBounds(-0.5 , 0.5 );

        double diff = 0 ;
        double calculatedVelocity = 0 ;

        Log.d("9010", "Slide M init position: " +  vSlideM.getCurrentPosition());
        Log.d("9010", "Slide S init position: " +  vSlideS.getCurrentPosition());

        while (opModeIsActive()) {

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
                    pidCoffs[pidCoffIndex] -= 0.01;
                } else {
                    pidCoffs[pidCoffIndex] -= 0.01;
                }
                telemetry.addLine().addData("[Kp :]  ", pidCoffs[0]);
                telemetry.addLine().addData("[Ki :]  ", pidCoffs[1]);
                telemetry.addLine().addData("[Kd :]  ", pidCoffs[2]);
                telemetry.update();
                sleep(100);
                slideKP = pidCoffs[0];
                slideKI = pidCoffs[1];
                slideKD = pidCoffs[2];

            }

            if( gamepad1.right_bumper) {
                if ( pidCoffIndex == 2) {
                    pidCoffs[pidCoffIndex] += 0.01;
                } else {
                    pidCoffs[pidCoffIndex] += 0.01;
                }
                telemetry.addLine().addData("[Kp :]  ", pidCoffs[0]);
                telemetry.addLine().addData("[Ki :]  ", pidCoffs[1]);
                telemetry.addLine().addData("[Kd :]  ", pidCoffs[2]);
                telemetry.update();
                sleep(100);
                slideKP = pidCoffs[0];
                slideKI = pidCoffs[1];
                slideKD = pidCoffs[2];
            }

            diff = vSlideM.getCurrentPosition() - vSlideS.getCurrentPosition();
            slidePID.setPID(slideKP,slideKI,slideKD);
            calculatedVelocity = slidePID.calculate(diff)   ;
            if (calculatedVelocity > ANGULAR_RATE ) {
                calculatedVelocity = ANGULAR_RATE;
            }
            if ( calculatedVelocity < -ANGULAR_RATE) {
                calculatedVelocity = -ANGULAR_RATE;
            }

            Log.d("9010", "Diff:  " +  diff);
            Log.d("9010", "calculatedVelocity " +  calculatedVelocity);
            Log.d("9010", "======================================\n\n");

            //vSlideM.setPower(gamepad1.left_stick_y);
            //vSlideS.setVelocity(gamepad1.right_stick_y*ANGULAR_RATE);

            //Control 2 Vslide in Sync
            vSlideM.setVelocity(gamepad1.left_stick_y * ANGULAR_RATE);
            //vSlideS.setVelocity(gamepad1.left_stick_y *ANGULAR_RATE - calculatedVelocity );
            vSlideS.setVelocity(gamepad1.left_stick_y *ANGULAR_RATE );


            /*
            Log.d("9010", "Gamepad leftStick: " + gamepad1.left_stick_y  );
            Log.d("9010", "Gamepad Right Stick: " + gamepad1.right_stick_y  );
            Log.d("9010", "Gamepad Left Trigger: " + gamepad1.left_trigger );
            Log.d("9010", "GampPad Right Trigger: " + gamepad1.right_trigger);
            */

        }

    }

}