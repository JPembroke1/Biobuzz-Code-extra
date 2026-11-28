package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * BLUE entry for hiveTipRed (HIVE-tip auto). Runs the red auto unchanged
 * (same coordinates, presets, timings and Panels settings); only the stored alliance, the
 * Limelight tag set and the watched tip cell (tipWatchCellBlue) are blue. Tune in
 * hiveTipRed.
 */
@Autonomous(name = "hiveTipBlue", group = "Main")
public class hiveTipBlue extends hiveTipRed {

    @Override
    protected boolean isBlueAlliance() {
        return true;
    }
}