package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * BLUE entry for fastRed2Tip. The field plays exactly the same from either side, so
 * this runs the red auto unchanged (same coordinates, presets, timings and Panels settings);
 * only the stored alliance and the Limelight tag set are blue. Tune in fastRed2Tip.
 */
@Autonomous(name = "fastBlue2Tip", group = "Main")
public class fastBlue2Tip extends fastRed2Tip {

    @Override
    protected boolean isBlueAlliance() {
        return true;
    }
}