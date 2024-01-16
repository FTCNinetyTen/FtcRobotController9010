package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "BlueRightAuto")
public class BlueRightAuto extends BaseAuto {

    public BlueRightAuto () {
        targetTeamTP="BlueTP";
    }
    void moveBeforeBoard() {
        sleep(10000);
        hdw.moveXAxis(-120,-1) ;
        //hdw.moveYAxis(24,1) ;
        //hdw.turn(90);
    }

    void park ( ) {
        hdw.moveXAxis(-3,1);
        hdw.moveYAxis(-24,-1);
        hdw.moveXAxis(-80,1);
    }

}
