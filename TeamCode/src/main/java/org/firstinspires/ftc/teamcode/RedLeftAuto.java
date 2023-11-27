package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.Hardware2022;

@Autonomous(name = "RedLeftAuto")
public class RedLeftAuto extends BaseAuto {

    public RedLeftAuto () {
        targetTeamTP="Pixel";
    }

    void moveBeforeBoard() {
        sleep(10000);
        hdw.moveXAxis(120,1) ;
        //hdw.moveYAxis(24,1) ;
        //hdw.turn(-90);
    }

}