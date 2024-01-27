package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "BlueRightAuto")
public class BlueRightAuto extends BaseAuto {

    public BlueRightAuto () {
        targetTeamTP="BlueTP";
    }
    void moveBeforeBoard() {
        //Move Back for the drive before the board
        hdw.moveYAxis(-24,-1) ;
        //Move Right
        hdw.moveXAxis(-40,1) ;
        //Move Forward,
        hdw.moveYAxis(24, 1);
        //Turn left
        hdw.turn(90);
    }

    void park ( ) {
        hdw.moveXAxis(-3,1);
        hdw.moveYAxis(-24,-1);
        hdw.moveXAxis(-80,1);
    }

}
