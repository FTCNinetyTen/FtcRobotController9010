package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.Hardware2022;

@Autonomous(name = "RedRightAuto")
public class RedRightAuto extends BaseAuto {


    public RedRightAuto () {
        targetTeamTP="Pixel";
    }
    void moveBeforeBoard() {
        hdw.moveXAxis(48,1) ;
        //hdw.moveYAxis(24,1) ;
        //hdw.turn(-90);

    }


}