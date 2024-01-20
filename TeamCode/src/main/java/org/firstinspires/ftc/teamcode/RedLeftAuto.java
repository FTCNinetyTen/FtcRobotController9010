package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.Hardware2022;

@Autonomous(name = "RedLeftAuto")
public class RedLeftAuto extends BaseAuto {

    public RedLeftAuto () {
        targetTeamTP="RedTP";
    }

    void moveBeforeBoard() {
        //Move Back for the drive before the board
        hdw.moveYAxis(-24,-1) ;
        //Move Right
        hdw.moveXAxis(72,1) ;
        //Move Forward,
        hdw.moveYAxis(24, 1);
        //Turn right
        hdw.turn(-90);
    }

    void park ( ) {
        hdw.moveXAxis(3,1);
        hdw.moveYAxis(-24,-1);
        hdw.moveXAxis(80 ,1);
    }

}