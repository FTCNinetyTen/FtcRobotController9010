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
    abstract void park ();

    TeamPropPosition detectedPosition = null;

    @Override
    public void runOpMode() {
        hdw = new Hardware2023(hardwareMap, telemetry); //init hardware
        hdw.createHardware();
        //hdw.initVision();

        waitForStart();


        //1. detect Team Prop
/*
        try {
            detectedPosition = hdw.detectTeamProp(targetTeamTP);
        } catch (InterruptedException e) {
            Log.d("9010", "Interrupt Exception");
            detectedPosition = TeamPropPosition.UNKOWN;

        }
  */
        Log.d("9010", "Detected: " + detectedPosition);
        telemetry.addData("[>]", "Detected:  " + detectedPosition);
        telemetry.update();
        //hdw.disableTFOD();

        //2. Put purple pixel in place
        putPurplePixel();

        //3. Move before the board
        moveBeforeBoard();

        //4. Score Yellow Pixel
        scoreYellow() ;

        hdw.resetPixelHook();
        //Park
        //park();
        idle();
    }

    /**
     * This method put purple pixel on the spark
     *
     */
    private void putPurplePixel() {

        if (detectedPosition.equals(TeamPropPosition.LEFT) || detectedPosition.equals(TeamPropPosition.UNKOWN)) {
            hdw.moveYAxis(27, 1);

            //Turn right, as release is on the back of robot
            hdw.turn(-90);
            hdw.moveYAxis(-1, 1);

            hdw.releasePixelHook();
            sleep(500);

            //move forward , to avoid hiting the pixel
            hdw.moveYAxis(5,1);

            //Turn Left , facing robot forward again.
            hdw.turn(90);
            sleep(500);
        }

        if (detectedPosition.equals(TeamPropPosition.RIGHT)) {
            hdw.moveYAxis(25, 1);

            //Turn Left
            hdw.turn(90);

            hdw.releasePixelHook();
            sleep(500);

            //move forward , to avoid hiting the pixel
            hdw.moveYAxis(3,1);

            //Turn Right, facing Robot forward again.
            hdw.turn(-90);
            sleep(500);
        }
        if (detectedPosition.equals(TeamPropPosition.CENTER)) {
            //Move more , to clear the team Prep
            hdw.moveYAxis(30, 1);
            //Then Move back for the turn
            hdw.moveYAxis(-5,-1);

            //Turn 2 90, to slow down.
            hdw.turn(90);
            sleep(450);
            hdw.turn(90);

            hdw.releasePixelHook();
            sleep(550);

            hdw.moveYAxis(3,1 );

            sleep(450);
            //Turn facing Robot forward again.
            hdw.turn(180);
            sleep(450);
        }

        //hdw.moveSlideToHeight(0);
    }

    private void scoreYellow()  {
        if (detectedPosition.equals(TeamPropPosition.LEFT) || detectedPosition.equals(TeamPropPosition.UNKOWN)) {

            if (targetTeamTP.equals(BLUETP) ) {
                Log.d("9010", "move to april tag 1 ");
                hdw.moveByAprilTag(1, 6, 0 );
            } else {
                Log.d("9010", "move to april tag 4 ");
                hdw.moveByAprilTag(4, 6 , 0);
            }
        } else  if (detectedPosition.equals(TeamPropPosition.RIGHT)) {

            if (targetTeamTP.equals(BLUETP) ) {
                Log.d("9010", "move to april tag 3 ");
                hdw.moveByAprilTag(3, 6, 0 );
            } else {
                Log.d("9010", "move to april tag 6 ");
                hdw.moveByAprilTag(6, 6 , 0);
            }

        } else    if (detectedPosition.equals(TeamPropPosition.CENTER)) {
            if (targetTeamTP.equals(BLUETP) ) {
                Log.d("9010", "move to april tag 2 ");
                hdw.moveByAprilTag(2, 6, 0 );
            } else {
                Log.d("9010", "move to april tag 5 ");
                hdw.moveByAprilTag(5, 6 , 0);
            }
        }

        hdw.moveYAxis(1,1);
        hdw.moveSlideToHeight(1300);
        hdw.openBox();
        sleep(1000);
        hdw.moveSlideToHeight(100);

    }

}
