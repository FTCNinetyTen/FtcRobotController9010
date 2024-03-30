package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.Hardware2022;

@Autonomous(name = "BlueLeftAuto")
public class BlueLeftAuto extends BaseAuto {

    public BlueLeftAuto () {
        targetTeamTP="BlueTP";
    }
    void moveBeforeBoard() {

        /*
        //MOve to left to avoid hitting trauss
        hdw.moveXAxis(-2,-1);

        //Move Back for the drive before the board
        hdw.moveYAxis(-16,-1) ;
        //Move left
        hdw.moveXAxis(-26,1) ;
        //Move Forward,
        hdw.moveYAxis(20, 1);
        //Turn left
        hdw.turn(90);

         */
        hdw.moveXAxis(-2,-1);
        //Move Back for the drive before the board
        hdw.moveYAxis(-18,-1) ;
        //Move Right
        hdw.moveXAxis(-24,-1) ;
        //Move Forward,
        hdw.moveYAxis(18, 1);
        //Turn right
        hdw.turn(90);

    }

    void park ( ) {
        hdw.moveXAxis(-3,1);
        hdw.moveYAxis(-24,-1);
        hdw.moveXAxis(-34,1);
    }

}

