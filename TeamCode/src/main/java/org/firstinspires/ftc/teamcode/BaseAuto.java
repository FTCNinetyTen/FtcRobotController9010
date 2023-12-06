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

    String BLUETP = "BlueTP";
    String REDTP = "RedTP";

    abstract void moveBeforeBoard();

    TeamPropPosition detectedPosition = null;

    @Override
    public void runOpMode() {
        hdw = new Hardware2023(hardwareMap, telemetry); //init hardware
        hdw.createHardware();
        hdw.initTfod();

        waitForStart();


        //1. detect Team Prop

        try {
            detectedPosition = hdw.detectTeamProp(targetTeamTP);
        } catch (InterruptedException e) {
            Log.d("9010", "Interrupt Exception");
            detectedPosition = TeamPropPosition.UNKOWN;

        }
        Log.d("9010", "Detected: " + detectedPosition);
        telemetry.addData("[>]", "Detected:  " + detectedPosition);
        telemetry.update();
        hdw.closeVisionPortal();


        //2. Put purple pixel in place
        putPurplePixel();

        //3. Move before the board
        moveBeforeBoard();


        hdw.initAprilTag();
        //4. Score Yellow Pixel
        //scoreYellow() ;

        idle();
    }

    /**
     * This method put purple pixel on the spark
     * After doing this, robot shall facing backword.
     */
    private void putPurplePixel() {
        hdw.moveYAxis(8, 1);

        if (detectedPosition.equals(TeamPropPosition.LEFT) || detectedPosition.equals(TeamPropPosition.UNKOWN)) {
            hdw.turn(-90);
            hdw.moveXAxis(-10, -1);
            hdw.spitOutPixel();
            hdw.turn(-90);
        }

        if (detectedPosition.equals(TeamPropPosition.RIGHT)) {
            hdw.turn(90);
            hdw.moveXAxis(10, 1);
            hdw.spitOutPixel();
            hdw.turn(90);
        }
        if (detectedPosition.equals(TeamPropPosition.CENTER)) {
            hdw.turn(180);
            hdw.moveYAxis(-10, -1);
            hdw.spitOutPixel();
        }

    }

    private void scoreYellow()  {
        if (detectedPosition.equals(TeamPropPosition.LEFT) || detectedPosition.equals(TeamPropPosition.UNKOWN)) {

            if (targetTeamTP.equals(BLUETP) ) {
                hdw.moveByAprilTag(1, 4, 0 );
            } else {
                hdw.moveByAprilTag(4, 4 , 0);
            }
        } else  if (detectedPosition.equals(TeamPropPosition.RIGHT)) {

            if (targetTeamTP.equals(BLUETP) ) {
                hdw.moveByAprilTag(3, 4, 0 );
            } else {
                hdw.moveByAprilTag(6, 4 , 0);
            }

        } else    if (detectedPosition.equals(TeamPropPosition.CENTER)) {
            if (targetTeamTP.equals(BLUETP) ) {
                hdw.moveByAprilTag(2, 4, 0 );
            } else {
                hdw.moveByAprilTag(5, 4 , 0);
            }
        }

        hdw.moveSlideToHeight(800);
        hdw.openBox();
        sleep(1000);
        hdw.closeBox();
        hdw.moveSlideToHeight(0);
        hdw.moveYAxis(6,1);
    }
}
