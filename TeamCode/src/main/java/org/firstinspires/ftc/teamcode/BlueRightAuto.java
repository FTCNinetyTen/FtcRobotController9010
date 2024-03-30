package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "BlueRightAuto")
public class BlueRightAuto extends BaseAuto {

    public BlueRightAuto () {
        targetTeamTP="BlueTP";
    }
    void moveBeforeBoard() {

        hdw.moveXAxis(2,1);
        //Move Back for the drive before the board
        hdw.moveYAxis(-18,-1) ;
        //Move Right
        hdw.moveXAxis(-60,-1) ;
        //Move Forward,
        hdw.moveYAxis(18, 1);
        //Turn right
        hdw.turn(90);

    }

    void park ( ) {
        hdw.moveXAxis(-3,1);
        hdw.moveYAxis(-24,-1);
        hdw.moveXAxis(-80,1);
    }

}
