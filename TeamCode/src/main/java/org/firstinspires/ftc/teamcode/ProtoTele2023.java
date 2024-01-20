package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Hardware2023;

@TeleOp(name = "ProtoTele2023", group = "TeleOp")
public class ProtoTele2023 extends LinearOpMode  {
    private Hardware2023 hdw;

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
        hdw = new Hardware2023(hardwareMap, telemetry); //init hardware
        hdw.createHardware();

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


        Gamepad previousGamePad1 = new Gamepad();
        Gamepad currentGamePad1 = new Gamepad();


        while (opModeIsActive()) {
            //Record previous Gamepad Status
            previousGamePad1.copy(currentGamePad1);
            //Update current gamepad status
            currentGamePad1.copy(gamepad1);


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


            if ( !currentGamePad1.b &&  previousGamePad1.b  ) {
                if (hdw.isPixHookUp()){
                    hdw.releasePixelHook();
                    hdw.setPixHookUp(false);
                }
                else {
                    hdw.resetPixelHook();
                    hdw.setPixHookUp(true);
                }
            }

        }

    }

}