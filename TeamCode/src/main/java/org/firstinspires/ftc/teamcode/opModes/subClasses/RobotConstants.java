package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class RobotConstants {

    // -------------------------------------------------------------------------
    // Intake
    // -------------------------------------------------------------------------

    public static double frontIntakeSpeed = 0.9;
    public static double backIntakeSpeed = 0.5;

    public static double frontFeedSpeed = 1.0;
    public static double backFeedSpeed = 1.0;

    public static double frontFeedSpeedAuto = 1.0;
    public static double backFeedSpeedAuto = 1.0;


    // -------------------------------------------------------------------------
    // Turret
    // -------------------------------------------------------------------------

    public static double turretPosition = 0.5;


    // -------------------------------------------------------------------------
    // Drivetrain
    // -------------------------------------------------------------------------

    public static double driveSpeed = 1.0;


    // -------------------------------------------------------------------------
    // Shooter
    //
    // outtakeTps is the base fixed speed. In TeleOp, G2 bumpers add a +/-20
    // offset on top of it for that match only; the offset resets to 0 each
    // time TeleOp starts, and G2 D-pad left clears it.
    // -------------------------------------------------------------------------

    public static double outtakeTps = 1940.0;
    public static double outtakeTpsStep = 20.0;


    // -------------------------------------------------------------------------
    // Hood
    // -------------------------------------------------------------------------

    public static double hoodPosition = 0.65;


    // -------------------------------------------------------------------------
    // Feeding
    //
    // Feeder strength is Outtake.feederPower (on/off motor, no encoder).
    // Gate delay is CombinedTeleOp.gateOpenDelayS.
    //
    // feedSpeedToleranceTps: shooter must be within this of its target to
    // START a burst. Only checked at burst start now, so it can be tighter
    // than before for a more consistent first shot.
    // -------------------------------------------------------------------------

    public static boolean feedRequiresSpeed = true;

    public static double feedSpeedToleranceTps = 95.0;


    // -------------------------------------------------------------------------
    // Field and HIVE geometry
    //
    // Pedro coordinates are in inches.
    //
    // NOTE: CombinedTeleOp has its own FIELD_SIZE_IN = 141.5 and
    // ROBOT_LENGTH_MM = 380 and does not use these. Measure the robot and
    // keep one set of values.
    // -------------------------------------------------------------------------

    public static double fieldSizeIn = 144.0;
    public static double fieldCenter = 72.0;

    public static double hiveYOffset = 12.75;

    public static double cellXOffset = 24.0;

    // true = blue HIVE is on the lower-Y side.
    public static boolean blueIsLowY = true;


    // -------------------------------------------------------------------------
    // Relocalising against a wall
    // -------------------------------------------------------------------------

    public static double robotLengthMm = 320.0;

    // true = robot back against wall
    // false = robot front against wall
    public static boolean resetBackAgainstWall = false;


    // -------------------------------------------------------------------------
    // Block servo
    // -------------------------------------------------------------------------

    public static double blockClosePos = 0.0;
    public static double blockOpenPos = 0.35;


    // -------------------------------------------------------------------------
    // Flowers
    // -------------------------------------------------------------------------

    public static double flowerLiftPos = 0.0;
    public static double flowerDropPos = 0.39;
}