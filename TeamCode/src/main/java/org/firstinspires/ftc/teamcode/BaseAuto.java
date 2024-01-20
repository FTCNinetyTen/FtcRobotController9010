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
        scoreYellow() ;
        hdw.closeVisionPortal();

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
            hdw.moveYAxis(25, 1);

            //Turn right, as release is on the back of robot
            hdw.turn(-90);

            hdw.releasePixelHook();
            sleep(500);
            //move forward , to avoid hiting the pixel
            hdw.moveYAxis(3,1);

            //Turn Left , facing robot forward again.
            hdw.turn(90);

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

        }
        if (detectedPosition.equals(TeamPropPosition.CENTER)) {
            //Move more , to clear the team Prep
            hdw.moveYAxis(28, 1);
            //Then Move back for the turn
            hdw.moveYAxis(-5,-1);

            //Turn 2 90, to slow down.
            hdw.turn(90);
            sleep(500);
            hdw.turn(90);

            hdw.releasePixelHook();
            hdw.moveYAxis(1,1 );
            sleep(500);

            //Turn facing Robot forward again.
            hdw.turn(180);
        }

        hdw.moveSlideToHeight(0);
    }

    private void scoreYellow()  {
        if (detectedPosition.equals(TeamPropPosition.LEFT) || detectedPosition.equals(TeamPropPosition.UNKOWN)) {

            if (targetTeamTP.equals(BLUETP) ) {
                hdw.moveByAprilTag(1, 18, 0 );
            } else {
                hdw.moveByAprilTag(4, 18 , 0);
            }
        } else  if (detectedPosition.equals(TeamPropPosition.RIGHT)) {

            if (targetTeamTP.equals(BLUETP) ) {
                hdw.moveByAprilTag(3, 18, 0 );
            } else {
                hdw.moveByAprilTag(6, 18 , 0);
            }

        } else    if (detectedPosition.equals(TeamPropPosition.CENTER)) {
            if (targetTeamTP.equals(BLUETP) ) {
                hdw.moveByAprilTag(2, 18, 0 );
            } else {
                hdw.moveByAprilTag(5, 18 , 0);
            }
        }

        hdw.moveYAxis(6,1);
        hdw.moveYAxis(4,1);
        hdw.moveXAxis(2,1);
        hdw.moveSlideToHeight(1000);
        hdw.openBox();
        sleep(1000);
        hdw.closeBox();
        hdw.moveSlideToHeight(100);

    }
}
