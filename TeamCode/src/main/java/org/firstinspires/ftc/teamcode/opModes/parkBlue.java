package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * BLUE entry for parkRed. The field plays exactly the same from either side, so this runs
 * parkRed unchanged (same coordinates and Panels settings); only the alliance stored for the
 * TeleOp is blue. Tune in parkRed.
 */
@Autonomous(name = "parkBlue", group = "Main")
public class parkBlue extends parkRed {

    @Override
    protected boolean isBlueAlliance() {
        return true;
    }
}