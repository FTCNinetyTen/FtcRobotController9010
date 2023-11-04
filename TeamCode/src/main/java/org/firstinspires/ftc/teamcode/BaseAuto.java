package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Hardware2023;
import org.firstinspires.ftc.teamcode.hardware.TeamPropPosition;

//435 max ticks per second is 383.6

//@Autonomous(name = "BaseAuto")
public abstract class BaseAuto extends LinearOpMode {

    Hardware2023 hdw;
    String targetTeamTP=null;

    abstract void moveBeforeBoard();

    @Override
    public void runOpMode() {
        hdw = new Hardware2023(hardwareMap,telemetry ); //init hardware
        hdw.createHardware();

        waitForStart();

        //1. Detect Team Prop

        //2. Put purple pixel in place

        //3. Move before the board
        moveBeforeBoard();

        //4. Score Yellow Pixel


        while (opModeIsActive())  {
            idle();
        }

    }

}
