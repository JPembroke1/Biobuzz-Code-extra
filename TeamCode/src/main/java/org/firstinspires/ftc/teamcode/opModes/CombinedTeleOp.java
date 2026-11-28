package org.firstinspires.ftc.teamcode.opModes;

import com.buildingblockrobotics.expander.BBRDigitalExpander;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opModes.subClasses.Flower;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Intake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.LimelightCam;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Outtake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.subClasses.RobotConstants;
import org.firstinspires.ftc.teamcode.opModes.subClasses.RobotSettings;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

/**
 * Combined TeleOp, Pinpoint-only turret aiming (no camera).
 *
 * Goal naming: the "upper" and "lower" goals here are the two goals at
 * different field Y positions. In Turret they map to the per-goal angle
 * trims:
 *
 *   UPPER goal -> Turret.HIGH_GOAL_ANGLE_TRIM_DEG
 *   LOWER goal -> Turret.LOW_GOAL_ANGLE_TRIM_DEG
 *
 * The goal coordinates themselves are the RED_/BLUE_ UPPER/LOWER fields
 * in this class. The Turret's own RED_LOW_GOAL_X/Y style coordinate
 * fields are not used by this TeleOp.
 *
 * Feeding: the safety checks (shooter on, aim, regression, at speed) are
 * made once when the right trigger starts a burst. While the trigger stays
 * held the burst keeps going, so the flywheel dipping after each ball does
 * not stop and restart the feed. Set checkOnlyAtBurstStart = false to
 * go back to checking every loop.
 *
 * Robustness (disconnect investigation):
 *  1. stop() does not call follower.update(). That reads the Pinpoint over
 *     I2C and can block stop() if the bus has faulted, which makes the SDK
 *     restart the robot. The SDK zeroes all motors on stop anyway.
 *  2. Slow loops are written to the robot log and the worst loop time is
 *     shown in telemetry.
 *  3. ODOMETRY FALLBACK. If follower.update() throws, returns an invalid
 *     pose, or is repeatedly slow (Pinpoint I2C fault after a hit), the
 *     TeleOp stops using Pedro and drives the four wheels directly, so the
 *     robot keeps driving. While in fallback:
 *       - the turret is locked to centre and auto aim is off;
 *       - fixed-turret shooting stays available (fixed TPS, no regression);
 *       - corner resets are blocked (the pose is not being tracked).
 *     It probes the Pinpoint every odometryRetryIntervalMs and returns to
 *     Pedro after odometryRecoverCount good updates in a row. The turret
 *     stays centred after recovery until a corner reset re-establishes the
 *     pose; auto aim and regression then come back by themselves.
 *
 * Shooting mode (see updateShootingMode):
 *   NORMAL  = auto aim + lookup-table distance regression. This is the
 *             default whenever the pose is good.
 *   FIXED   = turret locked centre + fixed TPS (RobotConstants.outtakeTps).
 *             Used only when there is no trustworthy distance: odometry
 *             offline (Pinpoint fault), or no pose yet (do a corner reset).
 *             The manual G2 Square / G1 Circle overrides are off by default
 *             (allowManualFixedShooting, allowManualRegressionToggle).
 *
 * Limelight (on the turret): odometry gives the coarse aim; when the goal
 * cluster is visible its bearing corrects the aim (Turret "Vision
 * correction"). With odometry offline or no pose, the turret aims from the
 * camera alone (VISION ONLY) at fixed TPS, and feeding waits until the
 * camera says it's on target. If the Limelight is missing, everything
 * works as before.
 *
 * Shoot on the move (shootOnMoveEnabled, OFF by default): while driving,
 * the turret and the lookup table aim at a "virtual goal", the real goal
 * shifted against the robot's velocity by the ball's time of flight:
 *   virtualGoal = goal - velocity x timeOfFlight(distance)   (iterated)
 * The robot's pose is also predicted shotLatencyS ahead (gate + feeder +
 * servo delay). The Limelight correction is still measured against the
 * real goal, and only while slower than visionMaxSpeedInPerS. Feeding is
 * blocked above maxShootSpeedInPerS. See "Shoot on the move" below.
 *
 * The four drive motor names below match drivetrainConfig in pedro/Constants.java;
 * test the fallback on blocks (see fallbackStrafeSign / fallbackTurnSign).
 */
@Configurable
@TeleOp(name = "0. Combined TeleOp", group = "Main")
public class CombinedTeleOp extends OpMode {

    public static double FIELD_SIZE_IN = 141.5;
    public static double FIELD_CENTER_IN = 70.75;
    public static double ROBOT_LENGTH_MM = 380.0;
    public static double ROBOT_WIDTH_MM = 308.0;

    public static double RED_LOWER_GOAL_X = 55.00;
    public static double RED_LOWER_GOAL_Y = 70.00;
    public static double RED_UPPER_GOAL_X = 59.06;
    public static double RED_UPPER_GOAL_Y = 94.81;
    public static double BLUE_LOWER_GOAL_X = 80;
    public static double BLUE_LOWER_GOAL_Y = 70;
    public static double BLUE_UPPER_GOAL_X = 80;
    public static double BLUE_UPPER_GOAL_Y = 94.81;

    public static double GOAL_SWITCH_Y_IN = 70.75;
    public static double GOAL_HYSTERESIS_IN = 1.0;

    public static double RED_LOWER_RESET_HEADING_DEG = 270.0;
    public static double RED_UPPER_RESET_HEADING_DEG = 90.0;
    public static double BLUE_LOWER_RESET_HEADING_DEG = 270.0;
    public static double BLUE_UPPER_RESET_HEADING_DEG = 90.0;

    public static double RED_INFRONT_X = 59.06;
    public static double BLUE_INFRONT_X = 80.0;
    public static double RED_INFRONT_HEADING_DEG = 90.0;
    public static double BLUE_INFRONT_HEADING_DEG = 90.0;

    public static double FORWARD_SIGN = -1.0;
    public static double STRAFE_SIGN = -1.0;
    public static double TURN_SIGN = -1.0;
    public static boolean CUBIC_STRAFE_TURN = false;

    public static boolean ALLOW_FIXED_TURRET_FEED = true;

    // ---------------------------------------------------------------------
    // Shoot on the move (26 Nov 2026). OFF (27 Nov 2026, turret erratic): set true in Panels to test.
    //
    // Time of flight (seconds) at four turret-to-goal distances (inches).
    // THESE ARE PLACEHOLDERS: film shots in slow motion at each distance,
    // count frames from the ball leaving to reaching the goal, divide by the
    // frame rate (e.g. 240 fps), and put the real numbers here. Distances
    // must increase from 1 to 4. Outside the table the end value is used.
    //
    // Tuning order: strafe sideways past the goal at one distance and
    // adjust leadScale until moving shots land (1.0 = trust the table);
    // then drive straight toward/away and check range (LUT distance).
    // ---------------------------------------------------------------------
    public static boolean shootOnMoveEnabled = false;  // off 27 Nov 2026: turret too erratic
    public static double tofDist1 = 27.0;
    public static double tofSec1 = 0.30;
    public static double tofDist2 = 39.0;
    public static double tofSec2 = 0.36;
    public static double tofDist3 = 51.0;
    public static double tofSec3 = 0.42;
    public static double tofDist4 = 63.0;
    public static double tofSec4 = 0.48;
    /** Multiplies the lead. Tune this first. */
    public static double leadScale = 1.0;
    /** Predict the robot this far ahead: trigger-to-ball-out delay (gate, feeder, servo). */
    public static double shotLatencyS = 0.0;            // was 0.10 (27 Nov: no prediction while testing)
    /** Velocity smoothing per loop (0-1). Lower = smoother but slower to react. */
    public static double velocityFilterAlpha = 0.15;    // was 0.35 (smoother speed)
    /** Below this speed the robot counts as stopped (no lead, no prediction). */
    public static double velocityDeadbandInPerS = 6.0;  // was 2.0 (ignore small drift)
    /** Below this turn rate (deg/s) heading isn't predicted ahead. */
    public static double turnDeadbandDegPerS = 30.0;    // was 10.0
    /** A new burst won't start above this speed. */
    public static double maxShootSpeedInPerS = 30.0;
    /** Limelight frames only update the correction below this speed. */
    public static double visionMaxSpeedInPerS = 8.0;
    /** Virtual-goal iterations (time of flight depends on distance). */
    public static int leadIterations = 3;

    // ---------------------------------------------------------------------
    // Distance check (28 Nov 2026): LOGGING ONLY, changes nothing.
    // Shows the odometry distance to the goal next to the Limelight's range
    // to the goal cluster, so we can see how they compare (and how far the
    // Pinpoint drifts after a hit) before letting the camera set the shooter
    // speed. Also written to the robot log every distanceLogIntervalMs while
    // the robot is nearly still and the cluster is in view, and at the start
    // of every burst. Search the log for "DistCheck".
    // ---------------------------------------------------------------------
    public static boolean distanceCheckLogging = true;
    public static double distanceLogIntervalMs = 500.0;
    /** Only the periodic log lines: robot must be slower than this. */
    public static double distanceLogMaxSpeedInPerS = 3.0;
    /** Smoothing for the Limelight range (0-1). */
    public static double limelightRangeAlpha = 0.3;

    /** Use the Limelight for aiming. Off = odometry-only aim, as before. */
    public static boolean visionEnabled = true;
    /** With odometry offline or no pose, aim from the camera instead of locking centre. */
    public static boolean visionOnlyWhenNoPose = true;
    public static boolean FIXED_TURRET_BYPASSES_REGRESSION = true;

    /*
     * Distance regression (lookup table) on when TeleOp starts.
     */
    public static boolean REGRESSION_ON_AT_START = true;

    /*
     * Manual overrides. Both false = the robot always uses auto aim +
     * regression, and only drops to fixed turret + fixed TPS by itself
     * when odometry is offline or the pose is not set.
     *   allowManualFixedShooting:    G2 Square forces fixed shooting,
     *                                G1 Square returns to auto.
     *   allowManualRegressionToggle: G1 Circle turns regression off/on.
     */
    public static boolean allowManualFixedShooting = false;
    public static boolean allowManualRegressionToggle = false;

    /*
     * Feeding.
     * gateOpenDelayS: time for the gate servo to open before the feeder
     * and intake push balls through. Real seconds, so it no longer depends
     * on loop speed. Tune this down until balls start catching the gate.
     * checkOnlyAtBurstStart: see class comment.
     */
    // 0.15 -> 0.10 (24 Nov 2026): logs showed every normal shot waiting ~185 ms
    // on the gate. Lower to 0.08 if balls don't catch; raise if they do.
    public static double gateOpenDelayS = 0.10;
    public static boolean checkOnlyAtBurstStart = true;

    /*
     * false = fire even if the turret thinks the goal is outside its range
     * or the aim isn't settled. true = old behaviour (block the shot).
     */
    public static boolean requireAimReady = false;

    /*
     * Auto spin-up: the intake sensor watches the dedicated fourth-ball
     * spot. When a ball is confirmed there (held for
     * Intake.ballConfirmDelaySeconds) the robot is full, so the shooter
     * turns on by itself once RB (collect) is released, provided the spot
     * is still full. Fires once each time that spot fills.
     */
    public static boolean autoSpinUpEnabled = true;

    /*
     * false (default): auto spin-up only arms once the fourth-ball sensor has
     * been seen EMPTY at least once after Start, so a sensor that reads
     * "full" at start-up (or a ball already sitting there) cannot turn the
     * shooter on by itself. The drivers use G1 Cross if they start loaded.
     * true: old behaviour, a robot that starts full spins up straight away.
     */
    public static boolean spinUpIfFullAtStart = false;

    /*
     * Flywheel idle (24 Nov 2026). Logs showed shots waiting 0.5-2.5 s
     * because collecting (RB) turned the shooter off and it had to spin up
     * from rest. Now collecting drops the flywheel to an IDLE speed instead,
     * and it keeps idling until G1 Triangle turns it fully off. Pulling RT
     * while idle (or off) brings it straight up to full speed.
     *   shooterOffWhileCollecting = true  -> old behaviour (off while RB held)
     *   collectIdleFraction: idle speed as a fraction of the shot speed
     *                        (0.7 = 70%; 1.0 = stay at full speed)
     *   triggerStartsShooter: RT spins the shooter up if it isn't at full
     */
    public static boolean shooterOffWhileCollecting = false;
    public static double collectIdleFraction = 0.7;
    public static boolean triggerStartsShooter = true;

    // Telemetry is built and sent at most this often (ms). Saves loop time.
    public static double telemetryIntervalMs = 100.0;

    /*
     * Diagnostics. Any loop slower than slowLoopWarnMs is written to the
     * robot log as a warning.
     */
    public static double slowLoopWarnMs = 100.0;

    /*
     * Odometry fallback.
     *
     * Motor names MUST match drivetrainConfig in pedro/Constants.java
     * (frontLeftName, frontRightName, backLeftName, backRightName). Pedro's
     * Mecanum has already set each motor's direction from that config, and
     * hardwareMap hands back the same motor objects, so no direction
     * settings are needed here.
     *
     * If the robot strafes or turns the wrong way in fallback (test with the
     * wheels off the ground by setting forceFallbackDrive = true), flip
     * fallbackStrafeSign or fallbackTurnSign to -1.
     */
    public static boolean fallbackDriveEnabled = true;
    public static boolean forceFallbackDrive = false;   // testing only
    public static String frontLeftName = "frontLeft";
    public static String frontRightName = "frontRight";
    public static String backLeftName = "backLeft";
    public static String backRightName = "backRight";
    public static double fallbackStrafeSign = 1.0;
    public static double fallbackTurnSign = 1.0;

    // follower.update() slower than this counts as a bad update
    public static double odometrySlowUpdateMs = 50.0;
    // this many slow updates in a row switches to fallback
    // (an exception or an invalid pose switches immediately)
    public static int odometrySlowLimit = 2;
    /** Heading snap (G2 D-pad right) refuses if the heading is further than this from a wall angle. */
    public static double headingSnapMaxErrorDeg = 20.0;
    // while in fallback, how often to test whether the Pinpoint is back
    public static double odometryRetryIntervalMs = 1000.0;
    // good updates in a row needed to switch back to Pedro
    public static int odometryRecoverCount = 5;

    /*
     * Controller feedback for the drive team (both gamepads):
     *   light bar RED    = odometry offline, turret locked centre
     *   light bar YELLOW = odometry OK but pose not set: do a corner reset
     *                      (fixed turret + fixed TPS until then)
     *   light bar GREEN  = pose good: auto aim + regression active
     *   long rumble      = odometry just dropped out
     *   two short blips  = odometry back
     * Light bar works on PS4/PS5 controllers; rumble on most controllers.
     */
    public static boolean controllerFeedback = true;

    public enum AllianceMode { STORED, RED, BLUE }
    public static AllianceMode ALLIANCE_MODE = AllianceMode.STORED;

    private static final String logTag = "CombinedTeleOp";

    private PoseStorage.Alliance activeAlliance = PoseStorage.Alliance.UNKNOWN;
    private Follower follower;
    private Intake intake;
    private Outtake outtake;
    private Turret turret;
    private LimelightCam limelightCam;
    private boolean visionAvailable;
    private double visionMs;
    private Flower flower;
    private RobotSettings robotSettings;
    private List<LynxModule> hubs;

    // Direct drive motors for the odometry fallback
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private boolean fallbackAvailable;

    private boolean shooterActive;   // full speed
    private boolean shooterIdle;     // spinning at idle speed
    private boolean autoAimMode = false;
    private boolean manualFixedRequested;
    private String fixedReason = "";
    private boolean flowerActive;
    private boolean poseEstablished;
    private boolean poseResetThisLoop;
    private boolean upperGoalSelected;
    private boolean goalSelectionInitialized;
    private boolean manualGoalMode;
    private boolean targetValid;
    private double goalX = Double.NaN;
    private double goalY = Double.NaN;
    private double distanceToGoalIn = Double.NaN;

    // Shoot on the move state
    private double velX;              // field frame, in/s (filtered)
    private double velY;
    private double omega;             // rad/s (filtered)
    private double robotSpeed;        // in/s
    private Pose lastVelocityPose;
    private long lastVelocityNs;
    private double leadX;             // virtual goal - real goal, inches
    private double leadY;
    private double tofS = Double.NaN;
    private double realDistanceIn = Double.NaN;

    // Distance check state
    private double llRangeIn = Double.NaN;          // latest raw Limelight range, inches
    private double llRangeSmoothedIn = Double.NaN;  // smoothed
    private int llTagCount;
    private long llLastSeenNs;
    private double odoDistanceIn = Double.NaN;      // odometry turret-to-goal distance
    private final ElapsedTime distanceLogTimer = new ElapsedTime();
    private boolean previousBurstActive;

    private boolean feederRequested;
    private boolean feederPermitted;
    private boolean fixedTurretFeedOverride;
    private boolean burstActive;

    // Fire-delay diagnostics: time from pulling RT to the feeder starting
    private final ElapsedTime triggerTimer = new ElapsedTime();
    private boolean previousShootTrigger;
    private boolean waitingToFire;
    private String currentFireWaitReason = "none";
    private double lastFireDelayMs;
    private String lastFireWaitReason = "none";
    private boolean gateOpen;
    private boolean previousFullSlot;
    private boolean autoSpinUpPending;
    private boolean autoSpinUpArmed;
    private double shooterTpsOffset;
    private String feederBlockReason = "NOT REQUESTED";

    private final ElapsedTime gateTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private final ElapsedTime followerUpdateTimer = new ElapsedTime();
    private final ElapsedTime odometryRetryTimer = new ElapsedTime();
    private double loopMs;
    private double avgLoopMs;

    // Diagnostics
    private double maxLoopMs;
    private int slowLoopCount;

    /*
     * Loop profiler: how long each part of loop() took last time (ms).
     * "sdk" is everything outside loop(): the SDK's own work between loops,
     * including sending telemetry and gamepad data.
     */
    public static boolean loopProfilerEnabled = false;   // true to diagnose loop time
    private final ElapsedTime sectionTimer = new ElapsedTime();
    private double controlsMs;
    private double driveMs;
    private double aimMs;
    private double outtakeMs;
    private double feedMs;
    private double displayMs;
    private double loopBodyMs;
    private String bulkTimingText = "";
    private int odometryErrorCount;
    private String lastOdometryError = "none";
    private double lastFollowerUpdateMs;

    // Odometry fallback state
    private boolean odometryFaulted;
    private boolean lastFailureWasSlowOnly;
    private int consecutiveSlowUpdates;
    private int consecutiveGoodProbes;
    private int odometryDropoutCount;

    // Controller light bar state last sent (-1 = none yet)
    private static final int ledOffline = 0;
    private static final int ledNeedsReset = 1;
    private static final int ledReady = 2;
    private int controllerLedState = -1;

    // Heading health (BBR expander). A hard hit can clip the expander's gyro;
    // the expander latches a flag, and the heading may be off from then on.
    // headingSuspect turns the light bar yellow until a corner reset,
    // goal reset or heading snap.
    private boolean headingSuspect;
    private boolean gyroSaturationSeen;
    private boolean previousHeadingSnapButton;

    private boolean previousFlowerButton;
    private boolean previousRegressionButton;
    private boolean previousAimButton;
    private boolean previousCentreButton;
    private boolean previousLowerResetButton;
    private boolean previousUpperResetButton;
    private boolean previousInfrontResetButton;
    private boolean previousLowerGoalButton;
    private boolean previousUpperGoalButton;
    private boolean previousAutomaticGoalButton;
    private String status = "Initialising";

    @Override
    public void init() {
        ALLIANCE_MODE = AllianceMode.STORED;
        follower = Constants.create(hardwareMap);
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // Same motor objects Pedro uses, so their directions are already set
        frontLeft = hardwareMap.tryGet(DcMotorEx.class, frontLeftName);
        frontRight = hardwareMap.tryGet(DcMotorEx.class, frontRightName);
        backLeft = hardwareMap.tryGet(DcMotorEx.class, backLeftName);
        backRight = hardwareMap.tryGet(DcMotorEx.class, backRightName);
        fallbackAvailable = frontLeft != null && frontRight != null
                && backLeft != null && backRight != null;
        if (!fallbackAvailable) {
            RobotLog.ww(logTag, "Fallback drive unavailable: check drive motor names");
        }

        intake = new Intake();
        outtake = new Outtake();
        turret = new Turret();
        flower = new Flower();
        robotSettings = new RobotSettings();
        intake.init(hardwareMap);
        outtake.init(hardwareMap);
        turret.init(hardwareMap);
        initLimelight();
        flower.init(hardwareMap);
        robotSettings.init(hardwareMap);

        Outtake.DISTANCE_REGRESSION_ENABLED = REGRESSION_ON_AT_START;
        Outtake.HOOD_COMP_ENABLED = false;
        Turret.velocityCompensationActive = false;   // old fixed lead; shoot on the move replaces it
        turret.setCentre();
        flower.flowerActivate(RobotConstants.flowerLiftPos);
        robotSettings.block();
        gateOpen = false;

        activeAlliance = resolveAlliance();
        Pose savedPose = PoseStorage.getPose();
        if (validPose(savedPose)) {
            follower.setPose(savedPose);
            poseEstablished = true;
            status = "Stored pose restored; verify robot has not moved";
        } else {
            poseEstablished = false;
            status = "No stored pose; perform a corner reset";
        }
        if (!fallbackAvailable && fallbackDriveEnabled) {
            status += " | WARNING: fallback drive motor names not found";
        }
        displayTelemetry();
    }

    @Override
    public void init_loop() {
        clearBulkCache();
        if (gamepad1.dpad_left) ALLIANCE_MODE = AllianceMode.RED;
        else if (gamepad1.dpad_right) ALLIANCE_MODE = AllianceMode.BLUE;
        else if (gamepad1.dpad_up) ALLIANCE_MODE = AllianceMode.STORED;
        activeAlliance = resolveAlliance();
        updateGoalFromPose(poseEstablished ? follower.pose() : null);
        turret.setCentre();
        flowerActive = false;
        maybeDisplayTelemetry();
    }

    @Override
    public void start() {
        activeAlliance = resolveAlliance();
        if (activeAlliance == PoseStorage.Alliance.RED) PoseStorage.setRed();
        else if (activeAlliance == PoseStorage.Alliance.BLUE) PoseStorage.setBlue();

        shooterActive = false;
        shooterIdle = false;
        // updateShootingMode() decides auto vs fixed on the first loop
        autoAimMode = false;
        manualFixedRequested = false;
        fixedReason = "";
        flowerActive = false;
        manualGoalMode = false;
        upperGoalSelected = false;
        goalSelectionInitialized = false;
        targetValid = false;
        goalX = goalY = distanceToGoalIn = Double.NaN;
        fixedTurretFeedOverride = false;
        burstActive = false;
        previousFullSlot = false;
        autoSpinUpPending = false;
        // Armed now only if starting full should spin up; otherwise the slot
        // must read empty first (see spinUpIfFullAtStart)
        autoSpinUpArmed = spinUpIfFullAtStart;
        shooterTpsOffset = 0.0;
        feederBlockReason = "NOT REQUESTED";
        flower.flowerActivate(RobotConstants.flowerLiftPos);
        Outtake.DISTANCE_REGRESSION_ENABLED = REGRESSION_ON_AT_START;
        Outtake.HOOD_COMP_ENABLED = false;

        previousFlowerButton = gamepad1.dpad_right;
        previousRegressionButton = gamepad1.circle;
        previousAimButton = gamepad1.square;
        previousCentreButton = gamepad2.square;
        previousLowerResetButton = gamepad2.dpad_down;
        previousUpperResetButton = gamepad2.dpad_up;
        previousLowerGoalButton = gamepad2.cross;
        previousUpperGoalButton = gamepad2.circle;
        previousAutomaticGoalButton = gamepad2.triangle;
        previousInfrontResetButton = gamepad1.dpad_down;
        previousHeadingSnapButton = gamepad2.dpad_right;

        String regressionText = Outtake.DISTANCE_REGRESSION_ENABLED ? " | REGRESSION ON" : " | FIXED TPS";
        if (activeAlliance == PoseStorage.Alliance.RED) status = "LOCKED ALLIANCE: RED" + regressionText;
        else if (activeAlliance == PoseStorage.Alliance.BLUE) status = "LOCKED ALLIANCE: BLUE" + regressionText;
        else status = "UNKNOWN ALLIANCE - fixed shooting only";

        maxLoopMs = 0.0;
        slowLoopCount = 0;
        odometryErrorCount = 0;
        lastOdometryError = "none";
        lastFollowerUpdateMs = 0.0;
        odometryFaulted = false;
        consecutiveSlowUpdates = 0;
        consecutiveGoodProbes = 0;
        odometryDropoutCount = 0;
        controllerLedState = -1;
        RobotLog.ii(logTag, "TeleOp started, alliance %s", allianceLabel());

        if (forceFallbackDrive) enterOdometryFallback("FORCED FOR TEST");

        loopTimer.reset();
        avgLoopMs = 0.0;
    }

    @Override
    public void loop() {
        loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        avgLoopMs = avgLoopMs == 0.0 ? loopMs : avgLoopMs + 0.1 * (loopMs - avgLoopMs);
        recordLoopTime(loopMs);

        double bodyStart = loopTimer.milliseconds();
        clearBulkCacheTimed();
        poseResetThisLoop = false;

        sectionTimer.reset();
        handleCornerResets();
        handleHiveResets();
        handleHeadingSnap();
        handleShooterControls();
        handleFlowerControls();
        handleRegressionToggle();
        controlsMs = lap();

        if (!poseResetThisLoop) driveRobot();
        else if (odometryFaulted) setDirectDrivePowers(0.0, 0.0, 0.0);
        else follower.manual(new DrivePowers(0.0, 0.0, 0.0));
        driveMs = lap();
        checkHeadingHealth();

        Pose currentPose = follower.pose();
        if (poseEstablished && validPose(currentPose)) PoseStorage.setPose(currentPose);
        updateVelocity(currentPose);

        handleManualGoalSelection();
        if (manualGoalMode) applySelectedGoal();
        else updateGoalFromPose(currentPose);

        handleTurretModeControls();
        updateShootingMode();
        updateVision();
        updateDistanceCheck(currentPose);
        if (autoAimMode) updateTurretAim(currentPose);
        else if (canAimVisionOnly()) aimVisionOnly();
        else lockTurretCentre();

        boolean fixedShootingMode = ALLOW_FIXED_TURRET_FEED
                && !autoAimMode
                && turret.isFixedPositionMode();

        // Distance only at full speed, so the lookup table doesn't override idle
        outtake.setGoalDistance(shooterActive
                && !fixedShootingMode
                && targetValid
                && poseEstablished
                && Double.isFinite(distanceToGoalIn)
                ? distanceToGoalIn : Double.NaN);
        aimMs = lap();

        // One update only. Calling Outtake.update() twice may distort its timing.
        outtake.update();
        outtakeMs = lap();

        handleIntakeAndFeeder();
        intake.update();
        feedMs = lap();

        updateControllerLed();
        maybeDisplayTelemetry();
        displayMs = lap();

        loopBodyMs = loopTimer.milliseconds() - bodyStart;
    }

    /** Milliseconds since the last lap() (or sectionTimer reset). */
    private double lap() {
        double ms = sectionTimer.milliseconds();
        sectionTimer.reset();
        return ms;
    }

    /**
     * Clears the bulk cache and, when the profiler is on, reads each hub
     * straight away so the time for every hub's bulk read is measured on
     * its own. A hub that is slow or dropping out shows up here.
     */
    private void clearBulkCacheTimed() {
        if (hubs == null) return;
        if (!loopProfilerEnabled) {
            clearBulkCache();
            return;
        }
        StringBuilder text = new StringBuilder();
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
            sectionTimer.reset();
            hub.getBulkData();
            double ms = sectionTimer.milliseconds();
            if (text.length() > 0) text.append(" | ");
            text.append(hub.isParent() ? "CH" : "EH" + hub.getModuleAddress())
                    .append(String.format(" %.0f", ms));
        }
        bulkTimingText = text.toString();
    }

    /**
     * Shows the odometry / pose state on both controllers' light bars.
     * Only sends a command when the state changes.
     */
    private void updateControllerLed() {
        if (!controllerFeedback) return;
        int state = odometryFaulted ? ledOffline
                : (!poseEstablished || headingSuspect) ? ledNeedsReset
                  : ledReady;
        if (state == controllerLedState) return;
        controllerLedState = state;

        double r = state == ledReady ? 0.0 : 1.0;
        double g = state == ledOffline ? 0.0 : 1.0;
        gamepad1.setLedColor(r, g, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
        gamepad2.setLedColor(r, g, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
    }

    /** Tracks the worst loop and logs any slow loop. */
    private void recordLoopTime(double ms) {
        if (ms > maxLoopMs) maxLoopMs = ms;
        if (ms > slowLoopWarnMs) {
            slowLoopCount++;
            RobotLog.ww(logTag, "Slow loop: %.0f ms (count %d)", ms, slowLoopCount);
        }
    }

    private void handleShooterControls() {
        boolean collect = gamepad1.right_bumper;
        boolean fullSlot = intake.isBallConfirmed();

        // The slot has been seen empty, so the next fill is a real fourth ball
        if (!Intake.sensorTriggered) autoSpinUpArmed = true;

        // Fourth-ball spot just filled: arm a spin-up
        if (autoSpinUpEnabled && autoSpinUpArmed && fullSlot && !previousFullSlot) {
            autoSpinUpPending = true;
        }
        // Spot emptied again (false detection or ball moved): cancel it
        if (!fullSlot) autoSpinUpPending = false;
        previousFullSlot = fullSlot;

        // Collecting: drop to idle (or off, old behaviour). The auto spin-up
        // waits until RB is released, and only happens if the spot is still full.
        if (collect) {
            shooterActive = false;
            shooterIdle = !shooterOffWhileCollecting;
        } else if (autoSpinUpPending) {
            autoSpinUpPending = false;
            shooterActive = true;
            status = "Loaded: shooter auto spin-up";
        }

        // RT while not at full speed: spin straight up (from idle is quick)
        if (triggerStartsShooter && !collect && !shooterActive
                && gamepad1.right_trigger > 0.3) {
            shooterActive = true;
        }

        if (gamepad1.cross) shooterActive = true;
        if (gamepad1.triangle) {
            shooterActive = false;
            shooterIdle = false;
            autoSpinUpPending = false;
        }
        // G2 bumpers nudge speed for this match only; D-pad left clears it
        if (gamepad2.leftBumperWasPressed()) shooterTpsOffset -= RobotConstants.outtakeTpsStep;
        if (gamepad2.rightBumperWasPressed()) shooterTpsOffset += RobotConstants.outtakeTpsStep;
        if (gamepad2.dpadLeftWasPressed()) shooterTpsOffset = 0.0;

        double shooterTps = Math.max(0.0, RobotConstants.outtakeTps + shooterTpsOffset);
        double idleTps = shooterTps * Math.max(0.0, Math.min(1.0, collectIdleFraction));
        outtake.setTargetTPS(shooterActive ? shooterTps : shooterIdle ? idleTps : 0.0);
        outtake.setHoodPosition(RobotConstants.hoodPosition);
    }

    private void handleRegressionToggle() {
        boolean pressed = gamepad1.circle;
        if (!allowManualRegressionToggle) {
            // Regression stays on; fixed TPS is only used in FIXED mode
            Outtake.DISTANCE_REGRESSION_ENABLED = true;
            previousRegressionButton = pressed;
            return;
        }
        if (pressed && !previousRegressionButton) {
            Outtake.DISTANCE_REGRESSION_ENABLED = !Outtake.DISTANCE_REGRESSION_ENABLED;
            stopFeederOnly();
            intake.stopIntake();
            status = Outtake.DISTANCE_REGRESSION_ENABLED ? "REGRESSION ENABLED" : "FIXED TPS MODE";
        }
        previousRegressionButton = pressed;
    }

    // ---------------------------------------------------------------------
    // Intake and feeder
    // ---------------------------------------------------------------------

    /** Runs the intake/feeder logic, then records the fire delay. */
    private void handleIntakeAndFeeder() {
        runIntakeAndFeeder();
        measureFireDelay();

    }

    /**
     * Measures how long it takes from pulling RT to the feeder actually
     * starting, and remembers what it was waiting for (other than the gate).
     * Shown in telemetry as "Fire Delay" and written to the robot log.
     */
    private void measureFireDelay() {
        boolean trigger = gamepad1.right_trigger > 0.3 && !gamepad1.right_bumper;
        if (trigger && !previousShootTrigger) {
            triggerTimer.reset();
            waitingToFire = true;
            currentFireWaitReason = "none";
        }
        if (!trigger) waitingToFire = false;

        if (waitingToFire) {
            if (feederPermitted) {
                lastFireDelayMs = triggerTimer.milliseconds();
                lastFireWaitReason = currentFireWaitReason.equals("none")
                        ? "gate only" : currentFireWaitReason;
                waitingToFire = false;
                RobotLog.ii(logTag, "Fire delay %.0f ms (waited on: %s)",
                        lastFireDelayMs, lastFireWaitReason);
            } else if (!feederBlockReason.equals("GATE OPENING")
                    && !feederBlockReason.startsWith("FEEDING")) {
                // Something other than the gate is holding the shot
                currentFireWaitReason = feederBlockReason;
            }
        }
        previousShootTrigger = trigger;
    }

    private void runIntakeAndFeeder() {
        feederRequested = false;
        feederPermitted = false;

        // Collect
        if (gamepad1.right_bumper) {
            burstActive = false;
            setGate(false);
            intake.intake(RobotConstants.frontIntakeSpeed, RobotConstants.backIntakeSpeed);
            outtake.stopFeeder();
            feederBlockReason = "COLLECTING";
            return;
        }

        // Shoot
        if (gamepad1.right_trigger > 0.3) {
            feederRequested = true;

            // Never feed into a stopped flywheel, even mid-burst
            if (!shooterActive) {
                burstActive = false;
            }

            boolean needCheck = !burstActive || !checkOnlyAtBurstStart;
            if (needCheck && !feederAllowed()) {
                burstActive = false;
                setGate(false);
                intake.stopIntake();
                outtake.stopFeeder();
                return;
            }

            if (!burstActive) {
                burstActive = true;
                feederBlockReason = fixedTurretFeedOverride
                        ? "FEEDING: FIXED TURRET" : "FEEDING: AUTO AIM";
            }
            setGate(true);

            if (gateTimer.seconds() >= gateOpenDelayS) {
                intake.intake(RobotConstants.frontFeedSpeed, RobotConstants.backFeedSpeed);
                outtake.setFeederOn(true);
                feederPermitted = true;
            } else {
                intake.stopIntake();
                outtake.stopFeeder();
                feederBlockReason = "GATE OPENING";
            }
            return;
        }

        // Trigger released
        burstActive = false;
        setGate(false);

        // Reverse intake
        if (gamepad1.left_bumper) {
            intake.intake(-RobotConstants.frontIntakeSpeed, -RobotConstants.backIntakeSpeed);
            outtake.stopFeeder();
            feederBlockReason = "REVERSING INTAKE";
            return;
        }

        intake.stopIntake();
        outtake.stopFeeder();
        feederBlockReason = "NOT REQUESTED";
    }

    /** Opens or closes the gate, only talking to the servo when it changes. */
    private void setGate(boolean open) {
        if (open == gateOpen) return;
        gateOpen = open;
        if (open) {
            robotSettings.release();
            gateTimer.reset();
        } else {
            robotSettings.block();
        }
    }

    private boolean feederAllowed() {
        fixedTurretFeedOverride = ALLOW_FIXED_TURRET_FEED
                && !autoAimMode
                && (turret.isFixedPositionMode() || turret.isVisionOnlyAim());

        if (!shooterActive) return blockFeed("SHOOTER OFF");

        if (shootOnMoveEnabled && robotSpeed > maxShootSpeedInPerS)
            return blockFeed(String.format("MOVING TOO FAST (%.0f in/s)", robotSpeed));

        if (turret.isVisionOnlyAim() && !turret.isVisionOnTarget())
            return blockFeed("VISION NOT ON TARGET");

        if (!fixedTurretFeedOverride) {
            if (!autoAimMode) return blockFeed("AUTO AIM DISABLED");
            if (!poseEstablished) return blockFeed("POSE NOT ESTABLISHED");
            if (!targetValid) return blockFeed("TARGET INVALID");
            if (requireAimReady) {
                if (!turret.isTargetReachable()) return blockFeed("TURRET UNREACHABLE");
                if (!turret.isAimReady()) return blockFeed("AIM NOT READY");
            }
        }

        boolean regressionRequired = Outtake.DISTANCE_REGRESSION_ENABLED
                && (!fixedTurretFeedOverride || !FIXED_TURRET_BYPASSES_REGRESSION);
        if (regressionRequired && !outtake.isUsingDistanceRegression())
            return blockFeed("REGRESSION INVALID");

        if (RobotConstants.feedRequiresSpeed
                && !outtake.isAtSpeed(RobotConstants.feedSpeedToleranceTps))
            return blockFeed("SHOOTER NOT AT SPEED");

        return true;
    }

    private boolean blockFeed(String reason) {
        feederBlockReason = reason;
        return false;
    }

    /**
     * Stops the feeder and ends the current burst, so the next shot re-runs
     * the safety checks. Used when the goal, aim mode or pose changes.
     */
    private void stopFeederOnly() {
        feederRequested = false;
        feederPermitted = false;
        burstActive = false;
        outtake.stopFeeder();
    }

    // ---------------------------------------------------------------------
    // Goal selection and aiming
    // ---------------------------------------------------------------------

    private void handleManualGoalSelection() {
        boolean lower = gamepad2.cross;
        boolean upper = gamepad2.circle;
        boolean automatic = gamepad2.triangle;
        if (automatic && !previousAutomaticGoalButton) {
            manualGoalMode = false;
            goalSelectionInitialized = false;
            stopFeederOnly();
            status = "Automatic Y target selection enabled";
        }
        if (lower && !previousLowerGoalButton) {
            manualGoalMode = true;
            upperGoalSelected = false;
            goalSelectionInitialized = true;
            applySelectedGoal();
            status = allianceLabel() + " LOWER target manually selected";
        }
        if (upper && !previousUpperGoalButton) {
            manualGoalMode = true;
            upperGoalSelected = true;
            goalSelectionInitialized = true;
            applySelectedGoal();
            status = allianceLabel() + " UPPER target manually selected";
        }
        previousLowerGoalButton = lower;
        previousUpperGoalButton = upper;
        previousAutomaticGoalButton = automatic;
    }

    private void applySelectedGoal() {
        /*
         * Tell the Turret which goal is selected so it applies that goal's
         * angle trim (UPPER = HIGH trim, LOWER = LOW trim). Done every time,
         * before anything can return early.
         */
        turret.setGoalHeight(upperGoalSelected
                ? Turret.GoalHeight.HIGH
                : Turret.GoalHeight.LOW);

        targetValid = false;
        goalX = goalY = Double.NaN;
        if (activeAlliance == PoseStorage.Alliance.RED) {
            goalX = upperGoalSelected ? RED_UPPER_GOAL_X : RED_LOWER_GOAL_X;
            goalY = upperGoalSelected ? RED_UPPER_GOAL_Y : RED_LOWER_GOAL_Y;
        } else if (activeAlliance == PoseStorage.Alliance.BLUE) {
            goalX = upperGoalSelected ? BLUE_UPPER_GOAL_X : BLUE_LOWER_GOAL_X;
            goalY = upperGoalSelected ? BLUE_UPPER_GOAL_Y : BLUE_LOWER_GOAL_Y;
        } else return;
        targetValid = validFieldPoint(goalX, goalY);
        if (!targetValid) return;
        Pose existing = turret.getTargetPose();
        boolean changed = turret.getSelectedGoal() != Turret.Goal.CUSTOM
                || existing == null
                || Double.compare(existing.x(), goalX) != 0
                || Double.compare(existing.y(), goalY) != 0;
        if (changed) {
            turret.setTargetPose(new Pose(goalX, goalY, 0.0));
            turret.realignToSelectedGoal();
            stopFeederOnly();
        }
    }

    private void updateGoalFromPose(Pose pose) {
        targetValid = false;
        goalX = goalY = Double.NaN;
        if (!allianceKnown() || !poseEstablished || !validPose(pose)) return;
        double hysteresis = Math.max(0.0, GOAL_HYSTERESIS_IN);
        double y = pose.y();
        boolean next = upperGoalSelected;
        if (!goalSelectionInitialized || hysteresis == 0.0) next = y >= GOAL_SWITCH_Y_IN;
        else if (upperGoalSelected && y < GOAL_SWITCH_Y_IN - hysteresis) next = false;
        else if (!upperGoalSelected && y >= GOAL_SWITCH_Y_IN + hysteresis) next = true;
        upperGoalSelected = next;
        goalSelectionInitialized = true;
        applySelectedGoal();
    }

    private void handleFlowerControls() {
        boolean pressed = gamepad1.dpad_right;
        if (pressed && !previousFlowerButton) {
            flowerActive = !flowerActive;
            // Only command the servo when it actually changes
            flower.flowerActivate(flowerActive ? RobotConstants.flowerDropPos : RobotConstants.flowerLiftPos);
        }
        previousFlowerButton = pressed;
    }

    /**
     * Manual overrides only. Whether the robot is in auto aim or fixed
     * shooting is decided in updateShootingMode().
     */
    private void handleTurretModeControls() {
        boolean aim = gamepad1.square;
        if (aim && !previousAimButton && manualFixedRequested) {
            manualFixedRequested = false;
            status = "Manual fixed shooting cleared";
        }
        previousAimButton = aim;

        boolean centre = gamepad2.square;
        if (centre && !previousCentreButton) {
            if (allowManualFixedShooting) {
                manualFixedRequested = true;
                status = "Manual fixed shooting (G1 Square to return to auto)";
            } else {
                status = "Manual fixed shooting disabled (allowManualFixedShooting)";
            }
        }
        previousCentreButton = centre;
    }

    /**
     * NORMAL: auto aim + lookup-table regression (the default).
     * FIXED:  turret centre + fixed TPS, only when there is no trustworthy
     *         distance to the goal.
     * Switches automatically in both directions, so after a Pinpoint
     * dropout and a corner reset the robot goes straight back to NORMAL.
     */
    private void updateShootingMode() {
        String reason = odometryFaulted ? "ODOMETRY OFFLINE"
                : !allianceKnown() ? "ALLIANCE UNKNOWN"
                  : !poseEstablished ? "NO POSE - CORNER RESET"
                    : !targetValid ? "NO TARGET"
                      : (allowManualFixedShooting && manualFixedRequested) ? "MANUAL"
                        : "";
        boolean needFixed = !reason.isEmpty();

        if (needFixed) {
            if (autoAimMode || !reason.equals(fixedReason)) {
                // Mode changed mid-burst: stop feeding so checks re-run
                stopFeederOnly();
                status = "FIXED SHOOTING: " + reason;
            }
            fixedReason = reason;
            autoAimMode = false;
            return;
        }

        if (!autoAimMode) {
            autoAimMode = true;
            fixedTurretFeedOverride = false;
            fixedReason = "";
            turret.realignToSelectedGoal();
            stopFeederOnly();
            status = "AUTO AIM + REGRESSION";
        }
    }

    private void updateTurretAim(Pose pose) {
        distanceToGoalIn = Double.NaN;
        realDistanceIn = Double.NaN;
        leadX = leadY = 0.0;
        tofS = Double.NaN;
        if (!targetValid || !poseEstablished || !validPose(pose)) {
            if (autoAimMode) turret.setCentre();
            return;
        }

        if (shootOnMoveEnabled) {
            // Aim + lookup table use the virtual goal from the predicted pose
            Pose predicted = computeVirtualGoal(pose);
            if (!autoAimMode) return;
            turret.aimTurretWithLead(predicted, leadX, leadY);
            return;
        }

        distanceToGoalIn = calculateDistanceToGoal(pose);
        realDistanceIn = distanceToGoalIn;
        if (!autoAimMode) return;
        turret.aimTurret(pose, 0.0, 0.0);
    }

    // ---------------------------------------------------------------------
    // Shoot on the move
    // ---------------------------------------------------------------------

    /**
     * Field-frame velocity from the change in pose between loops, smoothed.
     * Works the same with the Pinpoint or the expander. Reset whenever the
     * pose jumps (reset, heading snap) or isn't being tracked.
     */
    private void updateVelocity(Pose pose) {
        long now = System.nanoTime();
        if (!validPose(pose) || odometryFaulted || !poseEstablished || poseResetThisLoop) {
            velX = velY = omega = robotSpeed = 0.0;
            lastVelocityPose = null;
            turret.setVisionUpdatesAllowed(true);
            return;
        }
        if (lastVelocityPose != null) {
            double dt = (now - lastVelocityNs) / 1e9;
            if (dt > 0.002 && dt < 0.25) {
                double rawX = (pose.x() - lastVelocityPose.x()) / dt;
                double rawY = (pose.y() - lastVelocityPose.y()) / dt;
                double dh = Math.atan2(Math.sin(pose.heading() - lastVelocityPose.heading()),
                        Math.cos(pose.heading() - lastVelocityPose.heading()));
                double rawW = dh / dt;
                double a = Math.max(0.0, Math.min(1.0, velocityFilterAlpha));
                velX += a * (rawX - velX);
                velY += a * (rawY - velY);
                omega += a * (rawW - omega);
            } else if (dt >= 0.25) {
                velX = velY = omega = 0.0;   // long gap: don't trust old speed
            }
        }
        lastVelocityPose = pose;
        lastVelocityNs = now;
        robotSpeed = Math.hypot(velX, velY);

        // Blurred, late frames while moving fast would corrupt the correction
        turret.setVisionUpdatesAllowed(!shootOnMoveEnabled || robotSpeed <= visionMaxSpeedInPerS);
    }

    /**
     * Predicts the pose shotLatencyS ahead, then finds the virtual goal:
     * the goal shifted against the velocity by the time of flight, iterated
     * because time of flight depends on distance. Sets leadX/leadY,
     * distanceToGoalIn (virtual, for the lookup table), realDistanceIn and
     * tofS. Returns the predicted pose to aim from.
     */
    private Pose computeVirtualGoal(Pose pose) {
        boolean moving = robotSpeed > velocityDeadbandInPerS;
        boolean turning = Math.abs(Math.toDegrees(omega)) > turnDeadbandDegPerS;
        double vx = moving ? velX : 0.0;
        double vy = moving ? velY : 0.0;
        double w = turning ? omega : 0.0;
        double latency = Math.max(0.0, shotLatencyS);

        Pose predicted = new Pose(
                pose.x() + vx * latency,
                pose.y() + vy * latency,
                pose.heading() + w * latency);

        double h = predicted.heading();
        double pivotX = predicted.x() + Turret.TURRET_OFFSET_X_IN * Math.cos(h)
                - Turret.TURRET_OFFSET_Y_IN * Math.sin(h);
        double pivotY = predicted.y() + Turret.TURRET_OFFSET_X_IN * Math.sin(h)
                + Turret.TURRET_OFFSET_Y_IN * Math.cos(h);

        double scale = Double.isFinite(leadScale) ? leadScale : 1.0;
        double virtualX = goalX;
        double virtualY = goalY;
        double t = 0.0;
        for (int i = 0; i < Math.max(1, leadIterations); i++) {
            t = timeOfFlight(Math.hypot(virtualX - pivotX, virtualY - pivotY));
            virtualX = goalX - vx * t * scale;
            virtualY = goalY - vy * t * scale;
        }

        leadX = virtualX - goalX;
        leadY = virtualY - goalY;
        tofS = t;
        realDistanceIn = Math.hypot(goalX - pivotX, goalY - pivotY);
        distanceToGoalIn = Math.hypot(virtualX - pivotX, virtualY - pivotY);
        return predicted;
    }

    /** Time of flight (s) from the 4-point table, linear between points, held at the ends. */
    private double timeOfFlight(double distance) {
        double[] d = {tofDist1, tofDist2, tofDist3, tofDist4};
        double[] t = {tofSec1, tofSec2, tofSec3, tofSec4};
        if (!Double.isFinite(distance)) return t[0];
        if (distance <= d[0]) return t[0];
        for (int i = 1; i < d.length; i++) {
            if (distance <= d[i]) {
                double span = d[i] - d[i - 1];
                if (span <= 0.0) return t[i];
                return t[i - 1] + (t[i] - t[i - 1]) * (distance - d[i - 1]) / span;
            }
        }
        return t[d.length - 1];
    }

    /**
     * FIXED shooting: with no trustworthy pose, aiming from it would point
     * the turret at the wrong place, so hold the turret at centre and shoot
     * at fixed TPS. setCentre() only writes to the servo when the position
     * changes, so calling it every loop is cheap.
     */
    private void lockTurretCentre() {
        autoAimMode = false;
        distanceToGoalIn = Double.NaN;
        turret.setCentre();
        fixedTurretFeedOverride = ALLOW_FIXED_TURRET_FEED;
    }

    // ---------------------------------------------------------------------
    // Limelight
    // ---------------------------------------------------------------------

    /** Starts the Limelight. If it is missing, vision is just switched off. */
    private void initLimelight() {
        visionAvailable = false;
        if (!visionEnabled) return;
        try {
            limelightCam = new LimelightCam(hardwareMap);
            limelightCam.init();
            visionAvailable = true;
        } catch (RuntimeException e) {
            limelightCam = null;
            RobotLog.ee(logTag, e, "Limelight not available: odometry-only aiming");
        }
    }

    /**
     * Reads the newest Limelight result and hands the goal cluster's bearing
     * to the turret. Cheap: the SDK polls the Limelight in the background.
     */
    private void updateVision() {
        long startNs = System.nanoTime();
        if (!visionAvailable || !visionEnabled || !allianceKnown()) {
            turret.clearVisionMeasurement();
            visionMs = 0.0;
            return;
        }
        try {
            limelightCam.update();
            LimelightCam.HiveDetection d = limelightCam.getAimDetection(
                    activeAlliance == PoseStorage.Alliance.RED
                            ? LimelightCam.Alliance.RED : LimelightCam.Alliance.BLUE);
            if (d != null) {
                turret.setVisionMeasurement(true, d.bearing, limelightCam.getFrameId());
                recordLimelightRange(d.range, d.tagCount);
            } else {
                turret.clearVisionMeasurement();
            }
        } catch (RuntimeException e) {
            turret.clearVisionMeasurement();
            RobotLog.ee(logTag, e, "Limelight read failed");
        }
        visionMs = (System.nanoTime() - startNs) / 1_000_000.0;
    }

    /** No pose to aim from (odometry offline / not reset), but the camera can aim. */
    private boolean canAimVisionOnly() {
        return visionAvailable && visionEnabled && visionOnlyWhenNoPose
                && allianceKnown()
                && (odometryFaulted || !poseEstablished)
                && !(allowManualFixedShooting && manualFixedRequested);
    }

    /**
     * Turret aims from the camera, flywheel at fixed TPS (no trustworthy
     * distance). Feeding waits for isVisionOnTarget() (feederAllowed()).
     */
    private void aimVisionOnly() {
        autoAimMode = false;
        distanceToGoalIn = Double.NaN;
        turret.aimVisionOnly();
        fixedTurretFeedOverride = ALLOW_FIXED_TURRET_FEED;
    }

    // ---------------------------------------------------------------------
    // Distance check (logging only)
    // ---------------------------------------------------------------------

    private void recordLimelightRange(double rangeIn, int tags) {
        if (!Double.isFinite(rangeIn) || rangeIn <= 0.0) return;
        long now = System.nanoTime();
        // Restart the smoothing after a gap, so an old value doesn't drag
        boolean gap = (now - llLastSeenNs) / 1e6 > 300.0;
        double a = Math.max(0.0, Math.min(1.0, limelightRangeAlpha));
        llRangeSmoothedIn = (gap || !Double.isFinite(llRangeSmoothedIn))
                ? rangeIn : llRangeSmoothedIn + a * (rangeIn - llRangeSmoothedIn);
        llRangeIn = rangeIn;
        llTagCount = tags;
        llLastSeenNs = now;
    }

    /** True if the Limelight saw the goal cluster in the last 300 ms. */
    private boolean limelightRangeFresh() {
        return Double.isFinite(llRangeSmoothedIn)
                && (System.nanoTime() - llLastSeenNs) / 1e6 <= 300.0;
    }

    /**
     * Works out the odometry distance (independent of aim mode) and writes
     * both distances to the robot log: periodically while nearly still, and
     * at the start of each burst.
     */
    private void updateDistanceCheck(Pose pose) {
        odoDistanceIn = targetValid && poseEstablished && validPose(pose)
                ? calculateDistanceToGoal(pose) : Double.NaN;

        boolean burstStarted = burstActive && !previousBurstActive;
        previousBurstActive = burstActive;
        if (!distanceCheckLogging || !limelightRangeFresh()) return;

        boolean periodic = robotSpeed <= distanceLogMaxSpeedInPerS
                && distanceLogTimer.milliseconds() >= distanceLogIntervalMs;
        if (!periodic && !burstStarted) return;
        distanceLogTimer.reset();

        Pose p = validPose(pose) ? pose : null;
        RobotLog.ii(logTag, "DistCheck %s odo %.1f | LL %.1f (raw %.1f, %d tags) | diff %+.1f"
                        + " | pose (%.1f, %.1f, %.0f) | speed %.0f | TPS %.0f/%.0f",
                burstStarted ? "SHOT" : "still",
                odoDistanceIn, llRangeSmoothedIn, llRangeIn, llTagCount,
                llRangeSmoothedIn - odoDistanceIn,
                p == null ? Double.NaN : p.x(), p == null ? Double.NaN : p.y(),
                p == null ? Double.NaN : Math.toDegrees(p.heading()),
                robotSpeed, outtake.getCurrentTPS(), outtake.getTargetTPS());
    }

    private double calculateDistanceToGoal(Pose pose) {
        double h = pose.heading();
        double cos = Math.cos(h);
        double sin = Math.sin(h);
        double pivotX = pose.x() + Turret.TURRET_OFFSET_X_IN * cos
                - Turret.TURRET_OFFSET_Y_IN * sin;
        double pivotY = pose.y() + Turret.TURRET_OFFSET_X_IN * sin
                + Turret.TURRET_OFFSET_Y_IN * cos;
        return Math.hypot(goalX - pivotX, goalY - pivotY);
    }

    // ---------------------------------------------------------------------
    // Pose resets
    // ---------------------------------------------------------------------

    private void handleCornerResets() {
        boolean lower = gamepad2.dpad_down;
        boolean upper = gamepad2.dpad_up;
        if (lower && !previousLowerResetButton) resetAtAllianceCorner(false);
        else if (upper && !previousUpperResetButton) resetAtAllianceCorner(true);
        previousLowerResetButton = lower;
        previousUpperResetButton = upper;
    }

    private void handleHiveResets() {
        boolean infront = gamepad1.dpad_down;

        if (infront && !previousInfrontResetButton) resetInfrontOfGoal();
        previousInfrontResetButton = infront;
    }

    /**
     * G2 D-pad right: square the robot flat against ANY wall, then press.
     * Snaps the heading to the nearest 90 degrees and keeps X and Y. Heading
     * is the biggest source of aim error, so this fixes most of it from far
     * more places than the corners. Refused if the heading is more than
     * headingSnapMaxErrorDeg from a wall angle (robot not square, or heading
     * too far gone to guess which wall).
     */
    private void handleHeadingSnap() {
        boolean pressed = gamepad2.dpad_right;
        if (pressed && !previousHeadingSnapButton) snapHeadingToWall();
        previousHeadingSnapButton = pressed;
    }

    private void snapHeadingToWall() {
        if (odometryFaulted) { status = "Heading snap blocked: odometry offline"; return; }
        if (!poseEstablished) { status = "Heading snap blocked: do a corner reset first"; return; }

        Pose pose = follower.pose();
        if (!validPose(pose)) { status = "Heading snap blocked: no valid pose"; return; }

        double headingDeg = Math.toDegrees(pose.heading());
        double snappedDeg = Math.round(headingDeg / 90.0) * 90.0;
        double errorDeg = headingDeg - snappedDeg;
        if (Math.abs(errorDeg) > headingSnapMaxErrorDeg) {
            status = String.format("Heading snap blocked: %.0f deg off a wall angle", errorDeg);
            return;
        }

        Pose snapped = new Pose(pose.x(), pose.y(), Math.toRadians(snappedDeg));
        try {
            follower.setPose(snapped);
        } catch (RuntimeException e) {
            odometryErrorCount++;
            lastOdometryError = e.getClass().getSimpleName();
            RobotLog.ee(logTag, e, "follower.setPose() failed during heading snap");
            status = "Heading snap failed: odometry not responding";
            return;
        }
        poseResetThisLoop = true;
        follower.manual(new DrivePowers(0.0, 0.0, 0.0));
        PoseStorage.setPose(snapped);
        headingSuspect = false;
        turret.realignToSelectedGoal();
        status = String.format("Heading snapped to %.0f deg (corrected %.1f deg)", snappedDeg, -errorDeg);
        RobotLog.ii(logTag, "Heading snap: %.1f -> %.0f deg", headingDeg, snappedDeg);
    }

    /**
     * Watches the expander's latched gyro-saturation flag. The first time it
     * trips, rumbles both controllers and turns the light bar yellow until
     * the drivers re-establish heading (corner reset, goal reset or heading
     * snap). Auto aim keeps running; the drivers decide when to reset.
     *
     * The flag latches until the next full recalibration (init), so this
     * only warns about the FIRST hard hit each match.
     */
    private void checkHeadingHealth() {
        if (Constants.expanderLocalizer == null || odometryFaulted) return;
        if (gyroSaturationSeen) return;
        if (!Constants.expanderLocalizer.hasGyroEverSaturated()) return;

        gyroSaturationSeen = true;
        headingSuspect = true;
        if (controllerFeedback) {
            gamepad1.rumbleBlips(3);
            gamepad2.rumbleBlips(3);
        }
        status = "HEADING SUSPECT after hit: corner reset or heading snap (G2 D-pad right)";
        RobotLog.ww(logTag, "Expander gyro saturated: heading may have drifted");
    }

    private void resetInfrontOfGoal() {
        if (!allianceKnown()) { status = "Goal reset blocked: alliance unknown"; return; }
        if (odometryFaulted) { status = "Reset blocked: odometry offline"; return; }

        boolean red = activeAlliance == PoseStorage.Alliance.RED;
        double halfLength = ROBOT_LENGTH_MM / 25.4 / 2.0;

        double x = red ? RED_INFRONT_X : BLUE_INFRONT_X;
        double y = FIELD_SIZE_IN - halfLength;   // against the back wall
        double heading = red ? RED_INFRONT_HEADING_DEG : BLUE_INFRONT_HEADING_DEG;

        if (!validFieldPoint(x, y)) { status = "Goal reset blocked: invalid geometry"; return; }

        if (applyPoseReset(new Pose(x, y, Math.toRadians(heading)))) {
            status = (red ? "RED" : "BLUE") + " in-front-of-goal reset";
        }
    }

    private void resetAtAllianceCorner(boolean upper) {
        if (!allianceKnown()) { status = "Corner reset blocked: alliance unknown"; return; }
        if (odometryFaulted) { status = "Corner reset blocked: odometry offline"; return; }
        double halfLength = ROBOT_LENGTH_MM / 25.4 / 2.0;
        double halfWidth = ROBOT_WIDTH_MM / 25.4 / 2.0;
        boolean red = activeAlliance == PoseStorage.Alliance.RED;
        double heading = red
                ? (upper ? RED_UPPER_RESET_HEADING_DEG : RED_LOWER_RESET_HEADING_DEG)
                : (upper ? BLUE_UPPER_RESET_HEADING_DEG : BLUE_LOWER_RESET_HEADING_DEG);
        double x = red ? halfWidth : FIELD_SIZE_IN - halfWidth;
        double y = upper ? FIELD_SIZE_IN - halfLength : halfLength;
        if (!validFieldPoint(x, y)) { status = "Corner reset blocked: invalid geometry"; return; }

        if (applyPoseReset(new Pose(x, y, Math.toRadians(heading)))) {
            status = (red ? "RED " : "BLUE ") + (upper ? "UPPER" : "LOWER") + " corner reset";
        }
    }

    /**
     * Shared by both resets: stop everything, set the pose, re-pick the goal.
     * Returns false if the Pinpoint rejected the new pose (I2C fault).
     */
    private boolean applyPoseReset(Pose reset) {
        shooterActive = false;
        shooterIdle = false;
        outtake.setTargetTPS(0.0);
        stopFeederOnly();
        intake.stopIntake();
        follower.manual(new DrivePowers(0.0, 0.0, 0.0));
        poseResetThisLoop = true;
        try {
            follower.setPose(reset);
        } catch (RuntimeException e) {
            odometryErrorCount++;
            lastOdometryError = e.getClass().getSimpleName();
            RobotLog.ee(logTag, e, "follower.setPose() failed during reset");
            status = "Reset failed: odometry not responding";
            return false;
        }
        PoseStorage.setPose(reset);
        poseEstablished = true;
        headingSuspect = false;
        manualGoalMode = false;
        goalSelectionInitialized = false;
        updateGoalFromPose(reset);
        turret.realignToSelectedGoal();
        return true;
    }

    // ---------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------

    private PoseStorage.Alliance resolveAlliance() {
        if (ALLIANCE_MODE == AllianceMode.RED) return PoseStorage.Alliance.RED;
        if (ALLIANCE_MODE == AllianceMode.BLUE) return PoseStorage.Alliance.BLUE;
        return PoseStorage.lastAlliance == null ? PoseStorage.Alliance.UNKNOWN : PoseStorage.lastAlliance;
    }

    private boolean allianceKnown() {
        return activeAlliance == PoseStorage.Alliance.RED || activeAlliance == PoseStorage.Alliance.BLUE;
    }

    private String allianceLabel() {
        return activeAlliance == PoseStorage.Alliance.RED ? "RED"
                : activeAlliance == PoseStorage.Alliance.BLUE ? "BLUE" : "UNKNOWN";
    }

    // ---------------------------------------------------------------------
    // Driving, with odometry fallback
    // ---------------------------------------------------------------------

    private void driveRobot() {
        double strafeInput = gamepad1.left_stick_x;
        double turnInput = gamepad1.right_stick_x;
        if (CUBIC_STRAFE_TURN) {
            strafeInput = strafeInput * strafeInput * strafeInput;
            turnInput = turnInput * turnInput * turnInput;
        }
        double forward = FORWARD_SIGN * gamepad1.left_stick_y * RobotConstants.driveSpeed;
        double strafe = STRAFE_SIGN * strafeInput * RobotConstants.driveSpeed;
        double turn = TURN_SIGN * turnInput * RobotConstants.driveSpeed;
        if (gamepad2.dpad_down || gamepad2.dpad_up) forward = strafe = turn = 0.0;

        if (odometryFaulted) {
            runFallbackDrive(forward, strafe, turn);
            return;
        }

        follower.manual(new DrivePowers(forward, strafe, turn));
        if (timedFollowerUpdate()) {
            consecutiveSlowUpdates = 0;
            return;
        }

        consecutiveSlowUpdates++;
        boolean hardFault = !lastFailureWasSlowOnly;
        if (fallbackDriveEnabled && fallbackAvailable
                && (hardFault || consecutiveSlowUpdates >= Math.max(1, odometrySlowLimit))) {
            enterOdometryFallback(lastOdometryError);
            // Pedro may not have written the drive powers; do it now
            setDirectDrivePowers(forward, strafe, turn);
        }
    }

    /**
     * Runs follower.update() and reports whether it was healthy: no
     * exception, a valid pose, and not slower than odometrySlowUpdateMs.
     */
    private boolean timedFollowerUpdate() {
        followerUpdateTimer.reset();
        try {
            follower.update();
        } catch (RuntimeException e) {
            lastFollowerUpdateMs = followerUpdateTimer.milliseconds();
            odometryErrorCount++;
            lastOdometryError = e.getClass().getSimpleName();
            lastFailureWasSlowOnly = false;
            RobotLog.ee(logTag, e, "follower.update() failed (count %d)", odometryErrorCount);
            return false;
        }
        lastFollowerUpdateMs = followerUpdateTimer.milliseconds();

        // BBR expander: a failed read serves the last good pose instead of
        // throwing, so check freshness. Counted like a slow update (two in a
        // row switch to fallback) so a single missed read doesn't.
        if (Constants.expanderLocalizer != null
                && !Constants.expanderLocalizer.getExpander().isDataFresh()) {
            lastOdometryError = "EXPANDER DATA STALE";
            lastFailureWasSlowOnly = true;
            RobotLog.ww(logTag, "Expander localizer data not fresh");
            return false;
        }

        // BBR expander: if the localizer has left RUNNING the pose has
        // stopped updating, so don't aim or drive paths with it.
        if (Constants.expanderLocalizer != null
                && !Constants.expanderLocalizer.isRunning()) {
            odometryErrorCount++;
            BBRDigitalExpander.LocalizerState st =
                    Constants.expanderLocalizer.getLastState();
            lastOdometryError = "EXPANDER " + (st == null ? "NO DATA" : String.valueOf(st.status));
            lastFailureWasSlowOnly = false;
            RobotLog.ww(logTag, "Expander localizer not RUNNING: %s", lastOdometryError);
            return false;
        }

        if (!validPose(follower.pose())) {
            odometryErrorCount++;
            lastOdometryError = "INVALID POSE";
            lastFailureWasSlowOnly = false;
            RobotLog.ww(logTag, "follower.update() gave an invalid pose");
            return false;
        }

        if (lastFollowerUpdateMs > odometrySlowUpdateMs) {
            lastOdometryError = String.format("SLOW %.0f ms", lastFollowerUpdateMs);
            lastFailureWasSlowOnly = true;
            RobotLog.ww(logTag, "follower.update() slow: %.0f ms", lastFollowerUpdateMs);
            return false;
        }
        return true;
    }

    private void enterOdometryFallback(String reason) {
        if (!fallbackAvailable) return;
        odometryFaulted = true;
        odometryDropoutCount++;
        consecutiveGoodProbes = 0;
        odometryRetryTimer.reset();

        // Pose is no longer being tracked: stop trusting it
        poseEstablished = false;
        targetValid = false;
        stopFeederOnly();
        lockTurretCentre();

        if (controllerFeedback) {
            gamepad1.rumble(1000);
            gamepad2.rumble(1000);
        }

        status = "ODOMETRY OFFLINE (" + reason + "): driving direct, turret centred";
        RobotLog.ww(logTag, "Odometry fallback ON (%s), dropout %d", reason, odometryDropoutCount);
    }

    /**
     * Drives the wheels directly, and every odometryRetryIntervalMs tests
     * whether the Pinpoint is answering again. Once it gives
     * odometryRecoverCount good updates in a row, Pedro takes over again.
     */
    private void runFallbackDrive(double forward, double strafe, double turn) {
        if (!forceFallbackDrive
                && odometryRetryTimer.milliseconds() >= odometryRetryIntervalMs) {
            // Give Pedro the same powers so a probe update can't jerk the robot
            follower.manual(new DrivePowers(forward, strafe, turn));
            if (timedFollowerUpdate()) {
                consecutiveGoodProbes++;
                // keep probing every loop while it looks healthy
                if (consecutiveGoodProbes >= Math.max(1, odometryRecoverCount)) {
                    exitOdometryFallback();
                    return;   // Pedro just wrote this loop's powers
                }
            } else {
                consecutiveGoodProbes = 0;
                odometryRetryTimer.reset();
            }
        }
        setDirectDrivePowers(forward, strafe, turn);
    }

    private void exitOdometryFallback() {
        odometryFaulted = false;
        consecutiveSlowUpdates = 0;
        consecutiveGoodProbes = 0;
        // Turret stays centred (poseEstablished is false) until the drivers
        // do a corner reset; updateShootingMode() then restores auto aim.
        if (controllerFeedback) {
            gamepad1.rumbleBlips(2);
            gamepad2.rumbleBlips(2);
        }
        status = "Odometry back: corner reset to restore auto aim + regression";
        RobotLog.ii(logTag, "Odometry fallback OFF, Pedro driving again");
    }

    /**
     * Robot-centric mecanum mix in Pedro's convention:
     * forward +, strafe + = left, turn + = counter-clockwise.
     */
    private void setDirectDrivePowers(double forward, double strafe, double turn) {
        if (!fallbackAvailable) return;
        double s = strafe * (Double.isFinite(fallbackStrafeSign) ? fallbackStrafeSign : 1.0);
        double t = turn * (Double.isFinite(fallbackTurnSign) ? fallbackTurnSign : 1.0);

        double frontLeftPower = forward - s - t;
        double backLeftPower = forward + s - t;
        double frontRightPower = forward + s + t;
        double backRightPower = forward - s + t;

        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));

        frontLeft.setPower(frontLeftPower / max);
        backLeft.setPower(backLeftPower / max);
        frontRight.setPower(frontRightPower / max);
        backRight.setPower(backRightPower / max);
    }

    private void clearBulkCache() {
        if (hubs != null) for (LynxModule hub : hubs) hub.clearBulkCache();
    }

    private boolean validPose(Pose pose) {
        return pose != null && Double.isFinite(pose.x())
                && Double.isFinite(pose.y()) && Double.isFinite(pose.heading());
    }

    private boolean validFieldPoint(double x, double y) {
        return Double.isFinite(x) && Double.isFinite(y)
                && x >= 0.0 && y >= 0.0 && x <= FIELD_SIZE_IN && y <= FIELD_SIZE_IN;
    }

    // ---------------------------------------------------------------------
    // Telemetry
    // ---------------------------------------------------------------------

    /** Builds telemetry only every telemetryIntervalMs; formatting is slow. */
    private void maybeDisplayTelemetry() {
        if (telemetryTimer.milliseconds() < telemetryIntervalMs) return;
        telemetryTimer.reset();
        displayTelemetry();
    }

    private void displayTelemetry() {
        Pose pose = follower == null ? null : follower.pose();
        telemetry.addLine("=== STATUS ===");
        telemetry.addData("Status", status);
        telemetry.addData("Drive Mode", odometryFaulted
                ? "FALLBACK - odometry offline, turret locked centre"
                : "PEDRO");
        telemetry.addData("Shooting Mode", autoAimMode
                ? "AUTO AIM + LUT REGRESSION"
                : "FIXED TURRET + FIXED TPS (" + fixedReason + ")");
        telemetry.addData("Loop", "%.1f ms (avg %.1f, %.0f Hz)",
                loopMs, avgLoopMs, avgLoopMs > 0.0 ? 1000.0 / avgLoopMs : 0.0);
        telemetry.addData("Max Loop / Slow", "%.0f ms / %d", maxLoopMs, slowLoopCount);
        if (loopProfilerEnabled) {
            telemetry.addData("Bulk read ms", bulkTimingText);
            telemetry.addData("Loop ms", "ctrl %.0f | drive %.0f | aim %.0f | outtake %.0f"
                            + " | feed %.0f | display %.0f | sdk %.0f",
                    controlsMs, driveMs, aimMs, outtakeMs, feedMs, displayMs,
                    Math.max(0.0, loopMs - loopBodyMs));
        }
        telemetry.addData("Odometry", "%s | update %.0f ms | errors %d | dropouts %d | last: %s",
                Constants.useExpanderLocalizer ? "EXPANDER" : "PINPOINT",
                lastFollowerUpdateMs, odometryErrorCount, odometryDropoutCount, lastOdometryError);
        if (!fallbackAvailable) telemetry.addLine("WARNING: fallback drive motors not found");
        if (headingSuspect) telemetry.addLine("HEADING SUSPECT (hit clipped gyro): corner reset or G2 D-pad right");
        else if (gyroSaturationSeen) telemetry.addLine("Gyro clipped earlier this match (heading re-set since)");
        telemetry.addData("Alliance", activeAlliance);
        telemetry.addData("Pose Established", poseEstablished);
        if (validPose(pose)) telemetry.addData("Pose", "(%.2f, %.2f, %.1f deg)",
                pose.x(), pose.y(), normalizeDegrees0To360(Math.toDegrees(pose.heading())));

        telemetry.addLine("=== SHOOTER / FEEDER ===");
        telemetry.addData("Shooter", shooterActive ? "FULL"
                : shooterIdle ? String.format("IDLE (%.0f%%)", collectIdleFraction * 100.0)
                  : "OFF");
        telemetry.addData("4th Ball Slot", "%s%s%s",
                intake.isBallConfirmed() ? "FULL" : "empty",
                !autoSpinUpEnabled ? " (auto spin-up off)"
                        : autoSpinUpArmed ? "" : " (spin-up not armed: slot not seen empty yet)",
                "");
        telemetry.addData("Ball Sensor", "raw %s -> %s (triggeredWhenHigh %s)",
                Intake.rawSensorState ? "HIGH" : "LOW",
                Intake.sensorTriggered ? "BALL" : "none",
                Intake.sensorTriggeredWhenHigh);
        telemetry.addData("Regression", "%s (%s)",
                Outtake.DISTANCE_REGRESSION_ENABLED ? "ON" : "OFF",
                outtake.getRegressionStatus());
        telemetry.addData("TPS", "%.0f / %.0f", outtake.getCurrentTPS(), outtake.getTargetTPS());
        telemetry.addData("Hood", "%.3f (%s)", outtake.getHoodPosition(),
                outtake.isUsingDistanceRegression() ? "from lookup table" : "fixed");
        telemetry.addData("Fixed TPS", "%.0f (base %.0f, offset %+.0f)",
                RobotConstants.outtakeTps + shooterTpsOffset,
                RobotConstants.outtakeTps, shooterTpsOffset);
        telemetry.addData("At Speed", outtake.isAtSpeed(RobotConstants.feedSpeedToleranceTps));
        telemetry.addData("Raw / Filtered TPS", "%.0f / %.0f", outtake.getRawTPS(), outtake.getCurrentTPS());
        telemetry.addData("Feed Link", "scale %.2f | %.0f TPS",
                outtake.getFeedLinkScale(), outtake.getFeedLinkTPS());
        telemetry.addData("Feeder", "%s (burst %s, gate %s)",
                feederPermitted ? "ON" : "OFF",
                burstActive ? "active" : "idle",
                gateOpen ? "open" : "closed");
        telemetry.addData("Feeder Status", feederBlockReason);
        telemetry.addData("Fire Delay", "%.0f ms (waited on: %s)",
                lastFireDelayMs, lastFireWaitReason);
        telemetry.addData("Fixed Override", fixedTurretFeedOverride);

        telemetry.addLine("=== TURRET ===");
        String turretState;
        if (turret.isVisionOnlyAim()) {
            turretState = "VISION ONLY (" + fixedReason + ")"
                    + (turret.isVisionOnTarget() ? " - ON TARGET" : "");
        } else if (odometryFaulted) {
            turretState = "LOCKED CENTRE (odometry offline)";
        } else if (!autoAimMode) {
            turretState = "LOCKED CENTRE (" + fixedReason + ")";
        } else if (!targetValid || !poseEstablished) {
            turretState = "NO TARGET/POSE - turret centred";
        } else if (turret.isTargetReachable()) {
            turretState = "TRACKING";
        } else {
            turretState = "AT LIMIT " + (Turret.lastHoldSide > 0 ? "(+)" : "(-)")
                    + " - goal outside arc, check heading or turn robot";
        }
        telemetry.addData("Turret State", turretState);
        telemetry.addData("Auto Aim", autoAimMode);
        telemetry.addData("Aim Source", turret.getAimSource());
        if (visionAvailable) {
            telemetry.addData("Vision", "%s | bearing %.1f | correction %.1f deg | %s | %.0f ms",
                    Turret.lastVisionStatus, Turret.lastVisionBearingDeg,
                    Turret.lastVisionCorrectionDeg, LimelightCam.cameraStatus, visionMs);
        } else {
            telemetry.addData("Vision", visionEnabled ? "LIMELIGHT NOT FOUND" : "OFF");
        }
        telemetry.addData("Aim Ready", turret.isAimReady());
        telemetry.addData("Goal", "%s %s (%s)",
                allianceLabel(),
                upperGoalSelected ? "UPPER" : "LOWER",
                manualGoalMode ? "manual" : "auto");
        telemetry.addData("Target", "(%.2f, %.2f)", goalX, goalY);
        telemetry.addData("Distance", "%.2f in", distanceToGoalIn);
        telemetry.addData("Distance Check", limelightRangeFresh()
                ? String.format("odo %.1f | LL %.1f (%d tags) | diff %+.1f in",
                odoDistanceIn, llRangeSmoothedIn, llTagCount, llRangeSmoothedIn - odoDistanceIn)
                : String.format("odo %.1f | LL not in view", odoDistanceIn));
        telemetry.addData("Shoot on Move", shootOnMoveEnabled
                ? String.format("ON | %.0f in/s, %.0f deg/s | lead (%.1f, %.1f) in | tof %.2f s | dist %.0f -> %.0f in",
                robotSpeed, Math.toDegrees(omega), leadX, leadY, tofS, realDistanceIn, distanceToGoalIn)
                : String.format("OFF | %.0f in/s", robotSpeed));
        telemetry.addData("Angle Trim", "%.2f deg", Turret.lastAngleTrimDeg);
        telemetry.addData("Requested / Commanded", "%.1f / %.1f deg",
                Turret.lastRelativeAngleDeg, Turret.lastFinalAngleDeg);
        telemetry.addData("Servo", "%.3f", turret.getServoPosition());
        telemetry.addData("Reachable", turret.isTargetReachable());
        telemetry.addData("Turret used pose", "(%.2f, %.2f, %.0f deg)",
                Turret.lastRobotX, Turret.lastRobotY, Turret.lastRobotHeadingDeg);

        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("G1 Cross: shooter on | Triangle: off");
        telemetry.addLine("G1 RT: hold to fire | LB: reverse intake");
        telemetry.addLine(shooterOffWhileCollecting
                ? "G1 RB: collect / shooter off (auto-starts on release if full)"
                : "G1 RB: collect / shooter idles (full on release if 4 balls)");
        telemetry.addLine("G1 RT also spins the shooter up | Triangle: shooter fully off");
        if (allowManualRegressionToggle) telemetry.addLine("G1 Circle: regression on/off");
        if (allowManualFixedShooting) {
            telemetry.addLine("G2 Square: force fixed shooting | G1 Square: back to auto");
        }
        telemetry.addLine("G2 Cross/Circle/Triangle: lower/upper/auto goal");
        telemetry.addLine("G2 D-pad up/down: corner reset");
        telemetry.addLine("G2 D-pad right: heading snap (square against any wall)");
        telemetry.addLine("G2 LB/RB: fixed TPS -/+ | D-pad left: clear offset");
        telemetry.update();
    }

    private double normalizeDegrees0To360(double degrees) {
        double result = degrees % 360.0;
        return result < 0.0 ? result + 360.0 : result;
    }

    /**
     * Keep stop() short and free of I2C reads. The SDK zeroes every motor
     * when the OpMode stops, so there's no need to call follower.update().
     */
    @Override
    public void stop() {
        RobotLog.ii(logTag, "TeleOp stopping. Max loop %.0f ms, slow loops %d, "
                        + "odometry errors %d, dropouts %d",
                maxLoopMs, slowLoopCount, odometryErrorCount, odometryDropoutCount);
        if (outtake != null) outtake.stopOuttake();
        if (intake != null) intake.stopIntake();
        if (turret != null) turret.setCentre();
        if (limelightCam != null) limelightCam.close();
        if (hubs != null) for (LynxModule hub : hubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }
}