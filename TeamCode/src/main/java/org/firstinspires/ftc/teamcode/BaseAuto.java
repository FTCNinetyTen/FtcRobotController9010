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
    String targetTeamTP = null;

    abstract void moveBeforeBoard();

    TeamPropPosition detectedPosition = null;

    @Override
    public void runOpMode() {
        hdw = new Hardware2023(hardwareMap, telemetry); //init hardware
        hdw.createHardware();

        waitForStart();


        //1. detect Team Prop

        try {
            detectedPosition = hdw.detectTeamProp(targetTeamTP);
        } catch (InterruptedException e) {
            detectedPosition = TeamPropPosition.UNKOWN;
        }
        Log.d("9010", "Detected: " + detectedPosition);

        //2. Put purple pixel in place
        putPurplePixel();

        //3. Move before the board
        moveBeforeBoard();

        //4. Score Yellow Pixel

        idle();


    }


    private void putPurplePixel() {
        hdw.moveYAxis(8, 1);

        if (detectedPosition.equals(TeamPropPosition.LEFT) ||
                detectedPosition.equals(TeamPropPosition.UNKOWN)) {
            hdw.turn(-90);
            hdw.moveXAxis(-16, -1);
            hdw.spitOutPixel();
            hdw.turn(-90);
        }

        if (detectedPosition.equals(TeamPropPosition.RIGHT)) {
            hdw.turn(90);
            hdw.moveXAxis(16, 1);
            hdw.spitOutPixel();
            hdw.turn(90);
        }
        if (detectedPosition.equals(TeamPropPosition.CENTER)) {
            hdw.turn(180);
            hdw.moveYAxis(-16, -1);
            hdw.spitOutPixel();
        }

    }

}
