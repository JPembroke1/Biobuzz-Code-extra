package org.firstinspires.ftc.teamcode.opModes;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

@Configurable
@Autonomous(
        name = "01 - Hive Tip Flower Red Path 5 Auto",
        group = "Main"
)
public class DelayedFlowerRedPath5Auto extends OpMode {

    // ---------------------------------------------------------------------
    // Hardware and subsystems
    // ---------------------------------------------------------------------

    private List<LynxModule> allHubs;

    private Follower follower;

    private Outtake outtake;
    private Intake intake;
    private Turret turret;
    private Flower flower;
    private RobotSettings robotSettings;

    private final PoseFactory poseFactory =
            PoseFactory.degrees();

    // ---------------------------------------------------------------------
    // HIVE-tip detection (Limelight 3A on the turret)
    //
    // The Limelight has no cluster support, so LimelightCam groups the HIVE
    // member tags (IDs 30-45) into cells itself. tipWatchCell picks the cell
    // to watch, e.g. "BLUE SCORING" (same as the old SDK cluster name).
    // SCORING and FAR mean the same side (away from the audience).
    //
    // The camera turns with the turret, so during the wait the turret sits
    // at tipWatchTurretPosition, which must point the camera at that cell.
    // NaN = use FAR_WALL_TURRET_POSITION (no extra turret move before the
    // first shot). Check "Watched Cell" in telemetry with the turret there.
    //
    // Pipeline setup (exposure, decimation, resolution) is done in the
    // Limelight web UI now, not here.
    // ---------------------------------------------------------------------

    public static boolean VISION_ENABLED = true;

    /* The cell the camera watches for the tip. */
    public static String tipWatchCell = "BLUE SCORING";

    /* Turret position during the tip wait. NaN = FAR_WALL_TURRET_POSITION. */
    public static double tipWatchTurretPosition = Double.NaN;

    /*
     * Limelight fine-aim for each shot: once the turret has settled at its
     * preset, if the camera sees the RED goal cluster (LimelightCam
     * aimCellSide) it steers onto it, then holds that position while
     * feeding. No tags = preset exactly as before.
     */
    public static boolean useVisionAim = true;
    public static double visionAimTimeoutSeconds = 0.6;

    /*
     * true  = the tip is any FLIP of the watched cell: the first state seen
     *         after Start (up or down) is the baseline, and the tip is the
     *         cell holding the opposite state for HIVE_TIP_CONFIRM_FRAMES.
     *         Works whichever way the hive starts, and never fires just
     *         because the cell is already up.
     * false = old behaviour: fire as soon as the cell reads upright.
     */
    public static boolean TIP_ON_FLIP = true;

    public static double UPRIGHT_ROLL_LIMIT_DEG = 90.0;

    /*
     * When true, the blocker opens during the tip wait (feeder and intake
     * stopped), and the first shot goes straight to FEED after the tip,
     * skipping BLOCK_RELEASE_SECONDS. Bench test first: only safe if
     * nothing reaches the flywheel with the feeder stopped.
     */
    public static boolean PRE_RELEASE_DURING_WAIT = false;

    /*
     * Backup: if no tip is detected within HIVE_TIP_WAIT_SECONDS, then
     * SHOOT_ON_TIP_TIMEOUT = true  -> shoot anyway and run the full auto
     * SHOOT_ON_TIP_TIMEOUT = false -> skip shooting and park (old behaviour)
     */
    public static double HIVE_TIP_WAIT_SECONDS = 10.0;
    public static boolean SHOOT_ON_TIP_TIMEOUT = true;

    /*
     * Number of consecutive NEW camera frames required before the tip is
     * accepted.
     */
    public static int HIVE_TIP_CONFIRM_FRAMES = 2;

    /*
     * true  = the tip is simply RED SCORING being SEEN for
     *         HIVE_TIP_CONFIRM_FRAMES frames (roll is ignored). Overrides
     *         TIP_ON_FLIP. If the cell is already visible at Start, the
     *         auto fires TIP_FIRE_DELAY_SECONDS after Start.
     * false = use TIP_ON_FLIP / upright logic.
     */
    public static boolean TIP_ON_VISIBLE = true;

    /* Wait this long after the tip is confirmed before shooting. */
    public static double TIP_FIRE_DELAY_SECONDS = 1.0;

    private double hiveTipConfirmedTime = 0.0;

    private LimelightCam limelightCam;
    private boolean visionAvailable = false;

    // "redFar" = the watched cell (tipWatchCell); names kept from the webcam version.
    private boolean redFarVisible = false;
    private double redFarRoll = Double.NaN;
    private double redFarPercent = 0.0;
    private long lastTipFrameId = -1L;
    private String visibleClusterText = "none";

    // Limelight shot fine-aim state
    private boolean visionLocked = false;
    private String lastVisionShot = "NONE";

    /* First state seen after Start: null = not seen yet. */
    private Boolean tipBaselineUpright = null;

    // ---------------------------------------------------------------------
    // Turret positions
    // ---------------------------------------------------------------------

    public static double FAR_WALL_TURRET_POSITION = 0.4;
    public static double SECOND_FLOWER_TURRET_POSITION = 0.45;
    public static double TRAVEL_TURRET_POSITION = 0.50;

    public static double AUTO_TURRET_TRIM = 0.0;

    // ---------------------------------------------------------------------
    // Drive speeds
    // ---------------------------------------------------------------------

    public static double DRIVE_SPEED = 1.00;
    public static double FLOWER_COLLECT_SPEED = 0.35;
    public static double PARK_SPEED = 1.00;

    // ---------------------------------------------------------------------
    // Shooting timing
    // ---------------------------------------------------------------------

    public static double TURRET_SETTLE_SECONDS = 0.20;

    public static double SHOOT_READY_TIMEOUT_SECONDS = 1.0;

    public static double BLOCK_RELEASE_SECONDS = 0.40;

    public static double PRELOAD_FEED_SECONDS = 1.00;
    public static double FLOWER_FEED_SECONDS = 1.60;

    public static double SECOND_FLOWER_INTAKE_HOLD_SECONDS = 0.45;

    public static double FLOWER_LIFT_DELAY_SECONDS = 0.20;

    public static double TPS_READY_FRACTION = 0.95;
    public static double PRE_RELEASE_TPS_FRACTION = 0.80;

    /*
     * Shooter speed for each shot. The first is spun up from the start and
     * held through the HIVE wait. The second is set as the robot leaves the
     * first shot, so the flywheel has slowed to it before the flower shot.
     */
    public static double FIRST_SHOT_TPS = 2050.0;
    public static double SECOND_SHOT_TPS = 1970.0;

    public static boolean PRE_AIM_TURRET = true;

    // ---------------------------------------------------------------------
    // Path timing and completion
    // ---------------------------------------------------------------------

    public static double PATH6_TIMEOUT_SECONDS = 4.0;
    public static double PATH7_TIMEOUT_SECONDS = 2.5;
    public static double PATH8_TIMEOUT_SECONDS = 5.0;

    public static double MIN_PATH_RUNTIME_SECONDS = 0.05;

    public static double END_POSITION_TOLERANCE_IN = 1.5;
    public static double END_HEADING_TOLERANCE_DEG = 10.0;

    // ---------------------------------------------------------------------
    // Telemetry
    // ---------------------------------------------------------------------

    public static double TELEMETRY_INTERVAL_SECONDS = 0.25;

    // ---------------------------------------------------------------------
    // Mechanism and path state
    // ---------------------------------------------------------------------

    private double activeTurretPosition = 0.50;

    private int shotCount = 0;

    private double shooterReadyStartTime = 0.0;

    private boolean shooterReadyTimeoutFired = false;

    private boolean anyPathTimeoutFired = false;
    private boolean currentPathTimedOut = false;

    private String lastTimedOutPath = "NONE";

    private boolean autoFinished = false;

    private double pathStartTime = 0.0;
    private double turretMoveTime = 0.0;

    // ---------------------------------------------------------------------
    // HIVE wait state
    // ---------------------------------------------------------------------

    private double hiveTipWaitStartTime = 0.0;

    private boolean hiveTipWaitActive = false;

    private int hiveTipConfirmCount = 0;

    private boolean hiveTipConfirmed = false;
    private boolean hiveTipTimedOut = false;

    /*
     * After a HIVE timeout:
     *
     * - shooting is skipped;
     * - Paths 6 and 7 are skipped;
     * - flower collection is skipped;
     * - Path 8 still runs from the robot's actual current pose.
     */
    private boolean skipToPark = false;

    // ---------------------------------------------------------------------
    // Shooting state machine
    // ---------------------------------------------------------------------

    private enum ShootPhase {
        WAIT,
        RELEASE,
        FEED,
        DONE
    }

    private ShootPhase shootPhase =
            ShootPhase.DONE;

    private double releaseStartTime = 0.0;
    private double feedStartTime = 0.0;
    private double shootFeedSeconds = 0.0;

    /*
     * True when the current shot started with the blocker already open
     * (first shot with PRE_RELEASE_DURING_WAIT enabled).
     */
    private boolean preReleasedShot = false;

    // ---------------------------------------------------------------------
    // Loop timing
    // ---------------------------------------------------------------------

    private double lastLoopTime = 0.0;
    private double loopMs = 0.0;
    private double maxLoopMs = 0.0;

    private double lastTelemetryTime = -1.0;

    // ---------------------------------------------------------------------
    // Poses
    //
    // Start is the original Path 5 endpoint.
    // ---------------------------------------------------------------------

    private final Pose start =
            poseFactory.of(
                    23.7064,
                    130.1432,
                    90.0
            );

    private final Pose point6 =
            poseFactory.of(
                    42.0,
                    104.1657,
                    90.0
            );

    private final Pose point7Start =
            poseFactory.of(
                    42.0,
                    104.1657,
                    90.0
            );

    private final Pose point7 =
            poseFactory.of(
                    42.0,
                    127.0,
                    90.0
            );

    private final Pose point8Start =
            poseFactory.of(
                    42.0,
                    127.0,
                    90.0
            );

    private final Pose point8 =
            poseFactory.of(
                    0.0,
                    116.1719,
                    0.0
            );

    // ---------------------------------------------------------------------
    // Path leg
    // ---------------------------------------------------------------------

    private static class Leg {

        final Pose plannedStart;
        final Pose end;

        final double speed;
        final double timeoutSeconds;

        final String name;

        Leg(
                Pose plannedStart,
                Pose end,
                double speed,
                double timeoutSeconds,
                String name
        ) {

            this.plannedStart = plannedStart;
            this.end = end;

            this.speed = speed;
            this.timeoutSeconds = timeoutSeconds;

            this.name = name;
        }
    }

    private Leg leg6() {

        return new Leg(
                start,
                point6,
                DRIVE_SPEED,
                PATH6_TIMEOUT_SECONDS,
                "PATH 6"
        );
    }

    private Leg leg7() {

        return new Leg(
                point7Start,
                point7,
                FLOWER_COLLECT_SPEED,
                PATH7_TIMEOUT_SECONDS,
                "PATH 7"
        );
    }

    private Leg leg8() {

        return new Leg(
                point8Start,
                point8,
                PARK_SPEED,
                PATH8_TIMEOUT_SECONDS,
                "PATH 8"
        );
    }

    // ---------------------------------------------------------------------
    // Path creation
    // ---------------------------------------------------------------------

    private Path buildPath(
            Leg leg
    ) {

        Pose from =
                follower.pose();

        if (from == null) {
            from = leg.plannedStart;
        }

        return line(
                from,
                leg.end
        )
                .linear(
                        from,
                        leg.end
                )
                .with(
                        Constants.foresightConfig
                                .maxPathSpeed
                                .at(
                                        leg.speed
                                )
                );
    }

    private Command followLeg(
            Leg leg
    ) {

        return followLeg(
                leg,
                null
        );
    }

    /**
     * Path 8 always executes.
     *
     * Paths 6 and 7 complete immediately when skipToPark is true.
     */
    private Command followLeg(
            Leg leg,
            Runnable onStart
    ) {

        return new CommandBuilder()
                .setStart(() -> {

                    currentPathTimedOut = false;

                    if (skipToPark
                            && !"PATH 8".equals(
                            leg.name
                    )) {

                        releaseFollower();

                        return;
                    }

                    if (onStart != null) {
                        onStart.run();
                    }

                    pathStartTime =
                            getRuntime();

                    follower.follow(
                            buildPath(leg)
                    );
                })
                .setDone(() -> {

                    if (skipToPark
                            && !"PATH 8".equals(
                            leg.name
                    )) {

                        return true;
                    }

                    boolean finished =
                            isLegFinished(leg);

                    if (finished) {
                        releaseFollower();
                    }

                    return finished;
                });
    }

    private boolean isLegFinished(
            Leg leg
    ) {

        double now =
                getRuntime();

        double timeout =
                Math.max(
                        0.0,
                        leg.timeoutSeconds
                );

        if (now - pathStartTime >= timeout) {

            currentPathTimedOut = true;
            anyPathTimeoutFired = true;
            lastTimedOutPath = leg.name;

            return true;
        }

        if (now - pathStartTime
                < Math.max(
                0.0,
                MIN_PATH_RUNTIME_SECONDS
        )) {

            return false;
        }

        if (follower.atParametricEnd()) {
            return true;
        }

        Pose currentPose =
                follower.pose();

        if (currentPose == null) {
            return false;
        }

        double distance =
                Math.hypot(
                        leg.end.x()
                                - currentPose.x(),
                        leg.end.y()
                                - currentPose.y()
                );

        double headingErrorDegrees =
                Math.abs(
                        angleDiffDeg(
                                Math.toDegrees(
                                        leg.end.heading()
                                ),
                                Math.toDegrees(
                                        currentPose.heading()
                                )
                        )
                );

        return distance
                <= Math.max(
                0.0,
                END_POSITION_TOLERANCE_IN
        )
                && headingErrorDegrees
                <= Math.max(
                0.0,
                END_HEADING_TOLERANCE_DEG
        );
    }

    private double angleDiffDeg(
            double targetDegrees,
            double currentDegrees
    ) {

        double difference =
                (targetDegrees - currentDegrees)
                        % 360.0;

        if (difference > 180.0) {
            difference -= 360.0;
        }

        if (difference < -180.0) {
            difference += 360.0;
        }

        return difference;
    }

    private void releaseFollower() {

        if (follower != null) {
            follower.stop();
        }
    }

    // ---------------------------------------------------------------------
    // Initialization
    // ---------------------------------------------------------------------

    @Override
    public void init() {

        Scheduler.reset();

        configureBulkCaching();
        clearBulkCache();

        PoseStorage.setRed();
        PoseStorage.setPose(start);

        follower =
                Constants.create(
                        hardwareMap
                );

        follower.setPose(start);
        follower.update();

        outtake = new Outtake();
        intake = new Intake();
        turret = new Turret();
        flower = new Flower();
        robotSettings = new RobotSettings();

        outtake.init(hardwareMap);
        intake.init(hardwareMap);
        turret.init(hardwareMap);
        flower.init(hardwareMap);
        robotSettings.init(hardwareMap);

        initLimelight();

        Turret.velocityCompensationActive = false;

        intake.stopIntake();
        outtake.stopOuttake();

        robotSettings.block();

        flower.flowerActivate(
                RobotConstants.flowerLiftPos
        );

        resetRuntimeState();

        setTravelTurretPosition();
    }

    // ---------------------------------------------------------------------
    // Limelight
    // ---------------------------------------------------------------------

    private void initLimelight() {

        limelightCam = null;
        visionAvailable = false;

        if (!VISION_ENABLED) {
            return;
        }

        try {
            limelightCam = new LimelightCam(hardwareMap);
            limelightCam.init();
            visionAvailable = true;
        } catch (RuntimeException e) {
            /*
             * Limelight missing or misconfigured: the HIVE wait simply
             * times out (see SHOOT_ON_TIP_TIMEOUT) and shots use presets.
             */
            limelightCam = null;
        }
    }

    private boolean isCameraStreaming() {

        return limelightCam != null && limelightCam.isConnected();
    }

    /**
     * Reads the newest Limelight frame once per loop and hands the red goal
     * cluster's bearing to the turret for shot fine-aim.
     */
    private void updateVision() {

        if (!visionAvailable) {
            turret.clearVisionMeasurement();
            return;
        }

        try {
            limelightCam.update();
            LimelightCam.HiveDetection goal =
                    limelightCam.getAimDetection(LimelightCam.Alliance.RED);
            if (goal != null) {
                turret.setVisionMeasurement(true, goal.bearing, limelightCam.getFrameId());
            } else {
                turret.clearVisionMeasurement();
            }
        } catch (RuntimeException e) {
            turret.clearVisionMeasurement();
        }
    }

    /**
     * Parses tipWatchCell ("BLUE SCORING", "RED AUDIENCE", ...) into a HIVE
     * cell. SCORING and FAR are the same side (away from the audience).
     */
    private static LimelightCam.HiveCell parseWatchCell(String text) {

        String t = text == null ? "" : text.trim().toUpperCase();

        LimelightCam.Alliance alliance =
                t.contains("BLUE") ? LimelightCam.Alliance.BLUE : LimelightCam.Alliance.RED;

        LimelightCam.HiveSide side =
                t.contains("AUDIENCE") ? LimelightCam.HiveSide.AUDIENCE : LimelightCam.HiveSide.FAR;

        return new LimelightCam.HiveCell(alliance, side);
    }

    /**
     * Returns the watched HIVE cell from the current Limelight frame, or
     * null if it is not visible. Also refreshes the telemetry summary.
     * Call after limelightCam.update() (updateVision()).
     */
    private LimelightCam.HiveDetection readWatchedCell() {

        redFarVisible = false;
        redFarPercent = 0.0;

        if (!visionAvailable) {
            visibleClusterText = "camera off";
            return null;
        }

        StringBuilder summary = new StringBuilder();
        for (LimelightCam.HiveDetection d : limelightCam.getClusterDetections()) {
            if (summary.length() > 0) summary.append(" | ");
            summary.append(d.name).append(String.format(" r%.0f (%d tags)", d.roll, d.tagCount));
        }
        visibleClusterText = summary.length() > 0 ? summary.toString() : "none";

        LimelightCam.HiveDetection watched =
                limelightCam.getHiveDetection(parseWatchCell(tipWatchCell));

        if (watched != null) {
            redFarVisible = true;
            redFarRoll = watched.roll;
            redFarPercent = watched.percentClusterFound;
        }

        return watched;
    }

    /*
     * Only used when TIP_ON_VISIBLE is false. RE-CHECK UPRIGHT_ROLL_LIMIT_DEG
     * for the Limelight: its roll comes from a different pose solver than the
     * SDK clusters. Watch "Watched Cell" roll in telemetry with the cell up
     * and down.
     */
    private boolean isUpright(
            double roll
    ) {

        return Double.isFinite(roll)
                && Math.abs(roll)
                < UPRIGHT_ROLL_LIMIT_DEG;
    }

    private void closeCamera() {

        if (limelightCam != null) {
            try {
                limelightCam.close();
            } catch (RuntimeException ignored) {
                // Already closed.
            }
            limelightCam = null;
            visionAvailable = false;
        }
    }

    /** Turret position while waiting for the tip (camera faces the watched cell). */
    private double tipWatchPosition() {

        return Double.isFinite(tipWatchTurretPosition)
                ? tipWatchTurretPosition
                : FAR_WALL_TURRET_POSITION;
    }

    // ---------------------------------------------------------------------
    // Limelight shot fine-aim
    // ---------------------------------------------------------------------

    /**
     * Vision may steer the turret during a shot's WAIT and RELEASE phases,
     * once the turret has physically reached its preset.
     */
    private boolean visionWindowOpen() {

        return useVisionAim && visionAvailable
                && (shootPhase == ShootPhase.WAIT || shootPhase == ShootPhase.RELEASE)
                && getRuntime() - turretMoveTime >= TURRET_SETTLE_SECONDS;
    }

    private void updateTurret() {

        if (visionWindowOpen() && (visionLocked || turret.isVisionTargetVisible())) {
            // Starts from the preset angle and steps toward the cluster each new frame.
            visionLocked = true;
            turret.aimVisionOnly();
        } else {
            turret.setFixedServoPosition(activeTurretPosition);
        }
    }

    /**
     * A shot may move on once: vision isn't in use, the camera never saw the
     * goal cluster, the camera says it's on target, or
     * visionAimTimeoutSeconds has passed since the turret settled.
     */
    private boolean visionAimDone(double now) {

        if (!useVisionAim || !visionAvailable || !visionLocked) return true;
        if (turret.isVisionOnTarget()) return true;
        return now - turretMoveTime - TURRET_SETTLE_SECONDS >= visionAimTimeoutSeconds;
    }

    /** At feed start, hold the turret exactly where vision put it. */
    private void freezeVisionAim() {

        if (visionLocked) {
            activeTurretPosition = Turret.lastServoPosition;   // no turretMoveTime change
            lastVisionShot = String.format("shot %d: %.3f (%s)", shotCount,
                    activeTurretPosition, turret.isVisionOnTarget() ? "on target" : "timeout");
        } else if (useVisionAim && visionAvailable) {
            lastVisionShot = String.format("shot %d: preset, no tags", shotCount);
        }
        visionLocked = false;
    }

    // ---------------------------------------------------------------------
    // INIT loop
    // ---------------------------------------------------------------------

    @Override
    public void init_loop() {

        clearBulkCache();

        PoseStorage.setRed();

        intake.update();

        if (visionAvailable) {
            try {
                limelightCam.update();
            } catch (RuntimeException ignored) {
                // shown as not seen
            }
        }

        readWatchedCell();

        Pose currentPose =
                follower.pose();

        if (currentPose != null) {

            PoseStorage.setPose(
                    currentPose
            );
        }

        telemetry.addLine(
                "===== RED PATH 5 HIVE-TIP AUTO ====="
        );

        telemetry.addData(
                "Start Pose",
                start
        );

        telemetry.addData(
                "HIVE Tip Timeout",
                "%.2f sec",
                HIVE_TIP_WAIT_SECONDS
        );

        telemetry.addData(
                "Required Tip Frames",
                HIVE_TIP_CONFIRM_FRAMES
        );

        telemetry.addData(
                "Tip Target",
                "%s (turret %.3f)",
                tipWatchCell,
                tipWatchPosition()
        );

        telemetry.addData(
                "Limelight",
                visionAvailable
                        ? LimelightCam.cameraStatus
                        : "NOT FOUND (tip wait will time out)"
        );

        telemetry.addData(
                "Camera Streaming",
                isCameraStreaming()
        );


        telemetry.addData(
                "Visible Clusters",
                visibleClusterText
        );

        telemetry.addData(
                "Watched Cell",
                redFarVisible
                        ? String.format(
                        "roll %.1f (%s), %.0f%% found",
                        redFarRoll,
                        isUpright(redFarRoll)
                        ? "UP"
                        : "down",
                        redFarPercent
                )
                        : "not seen"
        );

        telemetry.addData(
                "Far Wall Turret",
                "%.3f",
                FAR_WALL_TURRET_POSITION
        );

        telemetry.addData(
                "Second Flower Turret",
                "%.3f",
                SECOND_FLOWER_TURRET_POSITION
        );

        telemetry.update();
    }

    // ---------------------------------------------------------------------
    // Start
    // ---------------------------------------------------------------------

    @Override
    public void start() {

        clearBulkCache();

        PoseStorage.setRed();

        follower.setPose(start);
        PoseStorage.setPose(start);

        resetRuntimeState();

        stopIntakeAndBlock();

        flower.flowerActivate(
                RobotConstants.flowerLiftPos
        );

        setTravelTurretPosition();

        lastLoopTime =
                getRuntime();

        schedule(
                autoRoutine()
        );
    }

    // ---------------------------------------------------------------------
    // Main loop
    // ---------------------------------------------------------------------

    @Override
    public void loop() {

        double loopNow =
                getRuntime();

        loopMs =
                (loopNow - lastLoopTime)
                        * 1000.0;

        lastLoopTime =
                loopNow;

        if (loopMs > maxLoopMs) {
            maxLoopMs = loopMs;
        }

        clearBulkCache();

        /*
         * VisionPortal processing is asynchronous. This reads the most
         * recent completed frame before the scheduler checks whether the
         * HIVE wait command is complete.
         */
        updateVision();

        updateHiveTipObservation();

        if (!autoFinished) {

            Scheduler.execute();
            follower.update();

        } else {

            maintainFinishedState();
        }

        Pose currentPose =
                follower.pose();

        if (currentPose != null) {

            PoseStorage.setPose(
                    currentPose
            );
        }

        if (!autoFinished) {

            updateTurret();

            intake.update();

            outtake.setHoodPosition(
                    RobotConstants.hoodPosition
            );

            outtake.update();
        }

        if (lastTelemetryTime < 0.0
                || loopNow - lastTelemetryTime
                >= Math.max(
                0.05,
                TELEMETRY_INTERVAL_SECONDS
        )) {

            lastTelemetryTime =
                    loopNow;

            displayTelemetry(
                    currentPose
            );
        }
    }

    // ---------------------------------------------------------------------
    // HIVE tip observation
    // ---------------------------------------------------------------------

    /**
     * Confirms the Red HIVE tip when the RED FAR cluster is seen
     * right-side up for HIVE_TIP_CONFIRM_FRAMES consecutive new frames.
     *
     * The loop runs faster than the camera, so the same frame is only
     * counted once (using the detection's frame timestamp).
     */
    private void updateHiveTipObservation() {

        if (!hiveTipWaitActive
                || !visionAvailable
                || hiveTipConfirmed
                || hiveTipTimedOut) {

            return;
        }

        LimelightCam.HiveDetection redFar =
                readWatchedCell();

        if (redFar == null) {

            hiveTipConfirmCount = 0;

            return;
        }

        long frameId =
                limelightCam.getFrameId();

        if (frameId == lastTipFrameId) {
            return;
        }

        lastTipFrameId =
                frameId;

        boolean upright =
                isUpright(redFar.roll);

        boolean tipState;

        if (TIP_ON_VISIBLE) {

            // Seen at all = tipped
            tipState = true;

        } else if (TIP_ON_FLIP) {

            if (tipBaselineUpright == null) {

                // First sighting after Start sets the starting state
                tipBaselineUpright = upright;
                hiveTipConfirmCount = 0;

                return;
            }

            tipState = upright != tipBaselineUpright;

        } else {

            tipState = upright;
        }

        if (tipState) {

            hiveTipConfirmCount++;

        } else {

            hiveTipConfirmCount = 0;
        }

        if (hiveTipConfirmCount
                >= Math.max(
                1,
                HIVE_TIP_CONFIRM_FRAMES
        )) {

            hiveTipConfirmed = true;
            hiveTipConfirmedTime = getRuntime();
            hiveTipWaitActive = false;

        }
    }

    /**
     * Completes when either:
     *
     * - RED FAR is confirmed upright; or
     * - HIVE_TIP_WAIT_SECONDS expires.
     */
    private Command waitForHiveTipOrTimeout() {

        return new CommandBuilder()
                .setStart(() -> {

                    hiveTipWaitStartTime =
                            getRuntime();

                    hiveTipWaitActive = true;

                    hiveTipConfirmCount = 0;
                    lastTipFrameId = -1L;
                    tipBaselineUpright = null;

                    hiveTipConfirmed = false;
                    hiveTipTimedOut = false;

                    skipToPark = false;
                })
                .setDone(() -> {

                    if (hiveTipConfirmed) {

                        hiveTipWaitActive = false;

                        // Hold for the fire delay; the timeout no longer applies
                        return getRuntime() - hiveTipConfirmedTime
                                >= Math.max(0.0, TIP_FIRE_DELAY_SECONDS);
                    }

                    double timeout =
                            Math.max(
                                    0.0,
                                    HIVE_TIP_WAIT_SECONDS
                            );

                    if (getRuntime()
                            - hiveTipWaitStartTime
                            >= timeout) {

                        hiveTipWaitActive = false;
                        hiveTipTimedOut = true;


                        if (SHOOT_ON_TIP_TIMEOUT) {

                            // Backup: carry on and shoot as if tipped
                            skipToPark = false;

                        } else {

                            skipToPark = true;
                            prepareDirectPark();
                        }

                        return true;
                    }

                    return false;
                });
    }

    /**
     * Safely abandons all shot and collection actions before parking.
     */
    private void prepareDirectPark() {

        shootPhase =
                ShootPhase.DONE;

        stopIntakeAndBlock();

        outtake.setTargetTPS(0.0);
        outtake.stopFeeder();

        flower.flowerActivate(
                RobotConstants.flowerLiftPos
        );

        setTravelTurretPosition();

        releaseFollower();
    }

    // ---------------------------------------------------------------------
    // Stop
    // ---------------------------------------------------------------------

    @Override
    public void stop() {

        Scheduler.reset();

        if (follower != null) {

            Pose finalPose =
                    follower.pose();

            if (finalPose != null) {

                PoseStorage.setPose(
                        finalPose
                );
            }

            follower.stop();
        }

        PoseStorage.setRed();

        closeCamera();

        shutdownAllSystems();

        restoreBulkCaching();
    }

    // ---------------------------------------------------------------------
    // Autonomous sequence
    // ---------------------------------------------------------------------

    private Command autoRoutine() {

        return sequential(

                instant(
                        this::preparePreload
                ),

                race(
                        waitForHiveTipOrTimeout(),
                        infinite(
                                this::holdForTip
                        )
                ),

                shootCycle(
                        FAR_WALL_TURRET_POSITION,
                        PRELOAD_FEED_SECONDS,
                        FIRST_SHOT_TPS
                ),

                followTravel(
                        leg6()
                ),

                collectFlower(
                        leg7(),
                        SECOND_FLOWER_INTAKE_HOLD_SECONDS,
                        SECOND_FLOWER_TURRET_POSITION
                ),

                shootCycle(
                        SECOND_FLOWER_TURRET_POSITION,
                        FLOWER_FEED_SECONDS,
                        SECOND_SHOT_TPS
                ),

                parkAndLiftFlower(),

                instant(
                        this::shutdownAllSystems
                )
        );
    }

    private void preparePreload() {

        stopIntakeAndBlock();

        flower.flowerActivate(
                RobotConstants.flowerLiftPos
        );

        // Camera (on the turret) faces the watched cell during the tip wait.
        setFixedTurretPosition(
                tipWatchPosition()
        );

        outtake.setTargetTPS(
                FIRST_SHOT_TPS
        );
    }

    // ---------------------------------------------------------------------
    // Travel and flower collection
    // ---------------------------------------------------------------------

    private Command followTravel(
            Leg leg
    ) {

        return followLeg(
                leg,
                () -> {

                    stopIntakeAndBlock();

                    setTravelTurretPosition();

                    /*
                     * Drop to the second-shot speed now, so the flywheel
                     * has settled there before the flower shot.
                     */
                    outtake.setTargetTPS(
                            SECOND_SHOT_TPS
                    );
                }
        );
    }

    private Command collectFlower(
            Leg leg,
            double intakeHoldSeconds,
            double aimAfterPosition
    ) {

        return sequential(

                followLeg(
                        leg,
                        () -> {

                            if (!skipToPark) {
                                holdFlowerCollection();
                            }
                        }
                ),

                race(
                        race(
                                waitSeconds(
                                        intakeHoldSeconds
                                ),
                                waitUntil(
                                        () -> skipToPark
                                )
                        ),
                        infinite(
                                () -> {

                                    if (!skipToPark) {

                                        holdFlowerCollection();

                                        preAimTurret(
                                                aimAfterPosition
                                        );
                                    }
                                }
                        )
                )
        );
    }

    private void holdFlowerCollection() {

        if (skipToPark) {
            return;
        }

        robotSettings.block();

        outtake.stopFeeder();

        flower.flowerActivate(
                RobotConstants.flowerDropPos
        );

        intake.intake(
                RobotConstants.frontIntakeSpeed,
                RobotConstants.backIntakeSpeed
        );
    }

    private void preAimTurret(
            double position
    ) {

        if (skipToPark) {
            return;
        }

        if (PRE_AIM_TURRET
                && Double.isFinite(position)) {

            setFixedTurretPosition(
                    position
            );
        }
    }

    // ---------------------------------------------------------------------
    // Shooting
    // ---------------------------------------------------------------------

    private Command shootCycle(
            double turretPosition,
            double feedSeconds,
            double shotTPS
    ) {

        return sequential(

                instant(
                        () -> beginShot(
                                turretPosition,
                                feedSeconds,
                                shotTPS
                        )
                ),

                race(
                        infinite(
                                this::runShotStateMachine
                        ),
                        waitUntil(
                                () -> shootPhase
                                        == ShootPhase.DONE
                        )
                )
        );
    }

    private void beginShot(
            double turretPosition,
            double feedSeconds,
            double shotTPS
    ) {

        if (skipToPark) {

            shootPhase =
                    ShootPhase.DONE;

            stopIntakeAndBlock();

            outtake.setTargetTPS(0.0);

            return;
        }

        shotCount++;

        setFixedTurretPosition(
                turretPosition
        );

        outtake.setTargetTPS(
                shotTPS
        );

        startShooterReadyWait();

        /*
         * First shot after a pre-released tip wait keeps the blocker open.
         */
        preReleasedShot =
                PRE_RELEASE_DURING_WAIT
                        && shotCount == 1;

        if (preReleasedShot) {
            holdFeederStoppedReleased();
        } else {
            stopIntakeAndBlock();
        }

        shootFeedSeconds =
                Math.max(
                        0.0,
                        feedSeconds
                );

        shootPhase =
                ShootPhase.WAIT;

        visionLocked = false;
    }

    /**
     * Held during the tip wait. Shooter keeps spinning (set in
     * preparePreload); feeder and intake stay stopped. The blocker is
     * opened early only when PRE_RELEASE_DURING_WAIT is enabled.
     */
    private void holdForTip() {

        intake.stopIntake();

        outtake.stopFeeder();

        if (PRE_RELEASE_DURING_WAIT) {
            robotSettings.release();
        } else {
            robotSettings.block();
        }
    }

    private void holdFeederStoppedReleased() {

        intake.stopIntake();

        outtake.stopFeeder();

        robotSettings.release();
    }

    private void runShotStateMachine() {

        if (skipToPark) {

            shootPhase =
                    ShootPhase.DONE;

            stopIntakeAndBlock();

            outtake.setTargetTPS(0.0);

            return;
        }

        double now =
                getRuntime();

        if (shootPhase == ShootPhase.WAIT
                && preReleasedShot) {

            boolean turretSettled =
                    now - turretMoveTime
                            >= Math.max(
                            0.0,
                            TURRET_SETTLE_SECONDS
                    );

            /*
             * Blocker is already open: skip RELEASE and feed as soon as
             * the turret is settled and the shooter is ready.
             */
            if (turretSettled
                    && visionAimDone(now)
                    && shooterReadyOrTimedOut()) {

                shootPhase =
                        ShootPhase.FEED;

                feedStartTime =
                        now;

                freezeVisionAim();
            }

        } else if (shootPhase == ShootPhase.WAIT) {

            boolean turretSettled =
                    now - turretMoveTime
                            >= Math.max(
                            0.0,
                            TURRET_SETTLE_SECONDS
                    );

            boolean releaseAllowed =
                    isShooterAtFraction(
                            PRE_RELEASE_TPS_FRACTION
                    )
                            || shooterReadyOrTimedOut();

            if (turretSettled
                    && visionAimDone(now)
                    && releaseAllowed) {

                shootPhase =
                        ShootPhase.RELEASE;

                releaseStartTime =
                        now;
            }
        }

        if (shootPhase == ShootPhase.RELEASE
                && now - releaseStartTime
                >= Math.max(
                0.0,
                BLOCK_RELEASE_SECONDS
        )
                && shooterReadyOrTimedOut()) {

            shootPhase =
                    ShootPhase.FEED;

            feedStartTime =
                    now;

            freezeVisionAim();
        }

        if (shootPhase == ShootPhase.FEED
                && now - feedStartTime
                >= shootFeedSeconds) {

            shootPhase =
                    ShootPhase.DONE;

            preReleasedShot = false;

            stopIntakeAndBlock();

            return;
        }

        switch (shootPhase) {

            case WAIT:

                if (preReleasedShot) {
                    holdFeederStoppedReleased();
                } else {
                    holdShooterAndBlock();
                }

                break;

            case RELEASE:

                intake.stopIntake();

                outtake.stopFeeder();

                robotSettings.release();

                break;

            case FEED:

                runShootFeed();

                break;

            case DONE:
            default:

                break;
        }
    }

    private void holdShooterAndBlock() {

        intake.stopIntake();

        outtake.stopFeeder();

        robotSettings.block();
    }

    private void runShootFeed() {

        robotSettings.release();

        intake.intake(
                RobotConstants.frontFeedSpeed,
                RobotConstants.backFeedSpeed
        );

        outtake.setFeederOn(true);
    }

    private void stopIntakeAndBlock() {

        intake.stopIntake();

        outtake.stopFeeder();

        robotSettings.block();
    }

    private void startShooterReadyWait() {

        shooterReadyStartTime =
                getRuntime();

        shooterReadyTimeoutFired =
                false;
    }

    private boolean shooterReadyOrTimedOut() {

        if (isShooterAtSpeed()) {
            return true;
        }

        if (getRuntime()
                - shooterReadyStartTime
                >= Math.max(
                0.0,
                SHOOT_READY_TIMEOUT_SECONDS
        )) {

            shooterReadyTimeoutFired = true;

            return true;
        }

        return false;
    }

    private boolean isShooterAtFraction(
            double fraction
    ) {

        double targetTPS =
                outtake.getTargetTPS();

        double currentTPS =
                outtake.getCurrentTPS();

        return targetTPS > 0.0
                && Double.isFinite(currentTPS)
                && currentTPS
                >= targetTPS
                * clamp(
                fraction,
                0.0,
                1.0
        );
    }

    private boolean isShooterAtSpeed() {

        return isShooterAtFraction(
                TPS_READY_FRACTION
        );
    }

    // ---------------------------------------------------------------------
    // Final parking
    // ---------------------------------------------------------------------

    private Command parkAndLiftFlower() {

        return parallel(

                followLeg(
                        leg8(),
                        () -> {

                            stopIntakeAndBlock();

                            outtake.setTargetTPS(0.0);

                            setTravelTurretPosition();
                        }
                ),

                sequential(

                        waitSeconds(
                                FLOWER_LIFT_DELAY_SECONDS
                        ),

                        instant(
                                () -> flower.flowerActivate(
                                        RobotConstants.flowerLiftPos
                                )
                        )
                )
        );
    }

    // ---------------------------------------------------------------------
    // End-of-auto safety
    // ---------------------------------------------------------------------

    private void shutdownAllSystems() {

        autoFinished = true;
        hiveTipWaitActive = false;

        if (follower != null) {
            follower.stop();
        }

        if (intake != null) {
            intake.stopIntake();
        }

        if (outtake != null) {

            outtake.setTargetTPS(0.0);
            outtake.stopFeeder();

            outtake.stopOuttake();
        }

        if (robotSettings != null) {
            robotSettings.block();
        }

        if (flower != null) {

            flower.flowerActivate(
                    RobotConstants.flowerLiftPos
            );
        }

        activeTurretPosition =
                TRAVEL_TURRET_POSITION;

        if (turret != null) {
            turret.setCentre();
        }
    }

    private void maintainFinishedState() {

        if (follower != null) {
            follower.stop();
        }

        if (intake != null) {
            intake.stopIntake();
        }

        if (outtake != null) {

            outtake.setTargetTPS(0.0);
            outtake.stopFeeder();
        }

        if (robotSettings != null) {
            robotSettings.block();
        }
    }

    // ---------------------------------------------------------------------
    // Fixed autonomous turret commands
    // ---------------------------------------------------------------------

    private void moveTurretTo(
            double position
    ) {

        double target =
                clamp(
                        position,
                        0.0,
                        1.0
                );

        if (Math.abs(
                target - activeTurretPosition
        ) > 1e-6) {

            turretMoveTime =
                    getRuntime();
        }

        activeTurretPosition =
                target;

        turret.setFixedServoPosition(
                activeTurretPosition
        );
    }

    private void setFixedTurretPosition(
            double position
    ) {

        moveTurretTo(
                position
                        + AUTO_TURRET_TRIM
        );
    }

    private void setTravelTurretPosition() {

        moveTurretTo(
                TRAVEL_TURRET_POSITION
        );
    }

    // ---------------------------------------------------------------------
    // Command helpers and runtime reset
    // ---------------------------------------------------------------------

    private Command waitSeconds(
            double seconds
    ) {

        return waitMs(
                Math.round(
                        Math.max(
                                0.0,
                                seconds
                        )
                                * 1000.0
                )
        );
    }

    private void resetRuntimeState() {

        activeTurretPosition =
                TRAVEL_TURRET_POSITION;

        shotCount = 0;

        shooterReadyStartTime = 0.0;
        shooterReadyTimeoutFired = false;

        anyPathTimeoutFired = false;
        currentPathTimedOut = false;
        lastTimedOutPath = "NONE";

        autoFinished = false;

        pathStartTime = 0.0;
        turretMoveTime = 0.0;

        shootPhase =
                ShootPhase.DONE;

        releaseStartTime = 0.0;
        feedStartTime = 0.0;
        shootFeedSeconds = 0.0;

        preReleasedShot = false;

        hiveTipWaitStartTime = 0.0;
        hiveTipWaitActive = false;

        hiveTipConfirmCount = 0;
        lastTipFrameId = -1L;
        tipBaselineUpright = null;

        hiveTipConfirmed = false;
        hiveTipTimedOut = false;

        skipToPark = false;

        visionLocked = false;
        lastVisionShot = "NONE";

        lastLoopTime = 0.0;
        loopMs = 0.0;
        maxLoopMs = 0.0;

        lastTelemetryTime = -1.0;
    }

    // ---------------------------------------------------------------------
    // Telemetry
    // ---------------------------------------------------------------------

    private void displayTelemetry(
            Pose currentPose
    ) {

        telemetry.addLine(
                "===== RED PATH 5 HIVE-TIP AUTO ====="
        );

        if (currentPose != null) {

            telemetry.addData(
                    "Pose",
                    "(%.1f, %.1f, %.1f deg)",
                    currentPose.x(),
                    currentPose.y(),
                    Math.toDegrees(
                            currentPose.heading()
                    )
            );
        }

        telemetry.addLine(
                "===== AUTO TIME ====="
        );

        telemetry.addData(
                "Runtime",
                "%.2f sec",
                getRuntime()
        );

        telemetry.addData(
                "Time Remaining",
                "%.2f sec",
                Math.max(
                        0.0,
                        30.0 - getRuntime()
                )
        );

        telemetry.addData(
                "Loop",
                "%.1f ms (max %.1f)",
                loopMs,
                maxLoopMs
        );

        telemetry.addData(
                "Auto Finished",
                autoFinished
        );

        telemetry.addData(
                "Shot Count",
                shotCount
        );

        telemetry.addData(
                "Shot Phase",
                shootPhase
        );

        telemetry.addData(
                "Pre-Released Shot",
                preReleasedShot
        );

        telemetry.addLine(
                "===== HIVE TIP WAIT ====="
        );

        double tipWaitElapsed =
                hiveTipWaitActive
                        || hiveTipConfirmed
                        || hiveTipTimedOut
                        ? Math.max(
                        0.0,
                        getRuntime()
                        - hiveTipWaitStartTime
                )
                        : 0.0;

        telemetry.addData(
                "Tip Target",
                tipWatchCell
        );

        telemetry.addData(
                "Wait Active",
                hiveTipWaitActive
        );

        telemetry.addData(
                "Tip Wait",
                "%.2f / %.2f sec",
                tipWaitElapsed,
                HIVE_TIP_WAIT_SECONDS
        );

        telemetry.addData(
                "Tip Start State",
                TIP_ON_VISIBLE
                        ? String.format("n/a (fires %.1fs after seen)", TIP_FIRE_DELAY_SECONDS)
                        : !TIP_ON_FLIP
                          ? "n/a (fires on UP)"
                          : tipBaselineUpright == null
                            ? "not seen yet"
                            : tipBaselineUpright ? "UP (waiting for down)" : "down (waiting for UP)"
        );

        telemetry.addData(
                "Tip Frames",
                "%d / %d",
                hiveTipConfirmCount,
                Math.max(
                        1,
                        HIVE_TIP_CONFIRM_FRAMES
                )
        );

        telemetry.addData(
                "Tip Confirmed",
                hiveTipConfirmed
        );

        telemetry.addData(
                "Tip Timed Out",
                hiveTipTimedOut
        );

        telemetry.addData(
                "Skip To Park",
                skipToPark
        );

        telemetry.addData(
                "Camera Streaming",
                isCameraStreaming()
        );

        telemetry.addData(
                "Watched Cell",
                redFarVisible
                        ? String.format(
                        "roll %.1f (%s), %.0f%% found",
                        redFarRoll,
                        isUpright(redFarRoll)
                        ? "UP"
                        : "down",
                        redFarPercent
                )
                        : "not seen"
        );

        telemetry.addData(
                "Visible Clusters",
                visibleClusterText
        );

        telemetry.addData(
                "Vision Aim",
                visionAvailable
                        ? String.format("%s | bearing %.1f | %s",
                        Turret.lastVisionStatus, Turret.lastVisionBearingDeg, lastVisionShot)
                        : "presets only"
        );

        telemetry.addLine(
                "===== PATH STATUS ====="
        );

        telemetry.addData(
                "Any Timeout",
                anyPathTimeoutFired
        );

        telemetry.addData(
                "Last Timeout",
                lastTimedOutPath
        );

        telemetry.addData(
                "Current Path Timed Out",
                currentPathTimedOut
        );

        telemetry.addData(
                "Follower Mode",
                follower.mode()
        );

        telemetry.addLine(
                "===== TURRET ====="
        );

        telemetry.addData(
                "Active Position",
                "%.3f",
                activeTurretPosition
        );

        telemetry.addData(
                "Far Wall Position",
                "%.3f",
                FAR_WALL_TURRET_POSITION
        );

        telemetry.addData(
                "Second Flower Position",
                "%.3f",
                SECOND_FLOWER_TURRET_POSITION
        );

        telemetry.addData(
                "Servo Position",
                "%.3f",
                Turret.lastServoPosition
        );

        telemetry.addData(
                "Fixed Mode",
                Turret.lastFixedPositionMode
        );

        telemetry.addLine(
                "===== SHOOTER ====="
        );

        telemetry.addData(
                "Shooter TPS",
                "%.0f / %.0f",
                outtake.getCurrentTPS(),
                outtake.getTargetTPS()
        );

        telemetry.addData(
                "Shooter Ready",
                isShooterAtSpeed()
        );

        telemetry.addData(
                "Ready Timeout",
                shooterReadyTimeoutFired
        );

        // Feeder is open loop now (no encoder), so show its power, not TPS
        telemetry.addData(
                "Feeder Power",
                "%.2f",
                outtake.getFeederCommandedPower()
        );

        telemetry.addData(
                "Hood Base",
                "%.3f",
                outtake.getHoodBase()
        );

        telemetry.addData(
                "Hood Compensation",
                "%.4f",
                outtake.getHoodCompensation()
        );

        telemetry.addData(
                "Hood Command",
                "%.3f",
                outtake.getHoodPosition()
        );

        telemetry.addLine(
                "===== FLOWER ====="
        );

        telemetry.addData(
                "Second Hold",
                "%.2f sec",
                SECOND_FLOWER_INTAKE_HOLD_SECONDS
        );

        telemetry.addData(
                "Second Pickup",
                "(%.1f, %.1f, %.1f deg)",
                point7.x(),
                point7.y(),
                Math.toDegrees(
                        point7.heading()
                )
        );

        telemetry.addLine(
                "===== INTAKE ====="
        );

        telemetry.addData(
                "Front Power",
                "%.2f",
                intake.getFrontMotorPower()
        );

        telemetry.addData(
                "Back Power",
                "%.2f",
                intake.getBackMotorPower()
        );

        telemetry.update();
    }

    // ---------------------------------------------------------------------
    // Bulk caching
    // ---------------------------------------------------------------------

    private void configureBulkCaching() {

        allHubs =
                hardwareMap.getAll(
                        LynxModule.class
                );

        for (LynxModule hub : allHubs) {

            hub.setBulkCachingMode(
                    LynxModule.BulkCachingMode.MANUAL
            );
        }
    }

    private void clearBulkCache() {

        if (allHubs == null) {
            return;
        }

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    private void restoreBulkCaching() {

        if (allHubs == null) {
            return;
        }

        for (LynxModule hub : allHubs) {

            hub.setBulkCachingMode(
                    LynxModule.BulkCachingMode.AUTO
            );
        }
    }

    // ---------------------------------------------------------------------
    // Utility
    // ---------------------------------------------------------------------

    private double clamp(
            double value,
            double minimum,
            double maximum
    ) {

        return Math.max(
                minimum,
                Math.min(
                        maximum,
                        value
                )
        );
    }
}