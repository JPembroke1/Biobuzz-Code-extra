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

/**
 * HIVE-tip partner auto, rebuilt on the BACK END of fastRed2Tip (28 Nov 2026).
 *
 * The robot starts where the fast auto's path 5 ends (against the wall, facing +Y), with the
 * turret already at the wall-shot position so the Limelight (on the turret) is looking at the
 * HIVE from the first frame. Then:
 *
 *   1. Wait for the HIVE tip (Limelight, tipWatchCell), flywheel already spinning.
 *   2. Wall shot (fast auto shot 2: turret FAR_WALL_TURRET_POSITION, shot1Tps here).
 *   3. Path 6 (pass-through) -> path 7 second flower -> bump -> second-flower shot.
 *   4. Park (path 8) and lift the flower.
 *
 * Everything after the tip is the fast auto's code and settings: stall detection on every
 * path, pass-through, stable-band spool, fixed per-shot TPS, the flower bump, auto feed speeds
 * from RobotConstants. If you retune the fast auto's back end, copy the same values here.
 *
 * Tip backup: if no tip within HIVE_TIP_WAIT_SECONDS, SHOOT_ON_TIP_TIMEOUT = true shoots and
 * runs the rest anyway; false skips everything and only parks.
 */
@Configurable
@Autonomous(name = "hiveTipRed", group = "Main")
public class hiveTipRed extends OpMode {

    private List<LynxModule> allHubs;
    private Follower follower;
    private Outtake outtake;
    private Intake intake;
    private Turret turret;
    private Flower flower;
    private RobotSettings robotSettings;
    private LimelightCam limelightCam;
    private boolean visionAvailable = false;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // ---------------------------------------------------------------------------------------
    // HIVE-tip detection (Limelight 3A on the turret), unchanged from the previous version
    // ---------------------------------------------------------------------------------------

    public static boolean VISION_ENABLED = true;

    /* The cell the camera watches for the tip ("BLUE SCORING", "RED AUDIENCE", ...). */
    public static String tipWatchCell = "BLUE SCORING";
    /*
     * The same physical cell as seen when playing BLUE (colours swap). VERIFY on the blue side
     * in INIT: "Visible Clusters" shows the names of the cells the camera can see.
     */
    public static String tipWatchCellBlue = "RED SCORING";

    /*
     * Turret position during the tip wait (the camera turns with the turret).
     * NaN = FAR_WALL_TURRET_POSITION, i.e. facing the goal for the first shot, so there is no
     * turret move (or settle wait) between the tip and the shot. The turret is set here in
     * INIT, so check "Watched Cell" on the Driver Station before pressing start.
     */
    public static double tipWatchTurretPosition = Double.NaN;

    /*
     * true  = the tip is the watched cell being SEEN for HIVE_TIP_CONFIRM_FRAMES frames.
     * false = TIP_ON_FLIP / upright logic below.
     */
    public static boolean TIP_ON_VISIBLE = true;
    /* Tip = the watched cell flipping from its first-seen state (only if TIP_ON_VISIBLE false). */
    public static boolean TIP_ON_FLIP = true;
    public static double UPRIGHT_ROLL_LIMIT_DEG = 90.0;
    public static int HIVE_TIP_CONFIRM_FRAMES = 2;
    /* Wait this long after the tip is confirmed before shooting. */
    public static double TIP_FIRE_DELAY_SECONDS = 1.0;
    public static double HIVE_TIP_WAIT_SECONDS = 10.0;
    public static boolean SHOOT_ON_TIP_TIMEOUT = true;

    // ---------------------------------------------------------------------------------------
    // From here down: fastRed2Tip's back end (paths 6-8, shots 2 and 3)
    // ---------------------------------------------------------------------------------------

    public static double FAR_WALL_TURRET_POSITION = 0.35;       // fast auto wall shot
    public static double SECOND_FLOWER_TURRET_POSITION = 0.45;
    public static double TRAVEL_TURRET_POSITION = 0.50;
    public static double AUTO_TURRET_TRIM = 0.0;

    public static double DRIVE_SPEED = 0.95;
    public static double FLOWER_COLLECT_SPEED = 0.45;
    public static double PARK_SPEED = 1.00;

    public static double TURRET_SETTLE_SECONDS = 0.15;
    public static double SHOOT_READY_TIMEOUT_SECONDS = 1.5;
    // Shooter must stay within +/- shooterStableBandFraction of target for this long to feed.
    public static double spoolUpExtraSeconds = 0.75;
    public static double shooterStableBandFraction = 0.03;
    // The first shot may come only TIP_FIRE_DELAY_SECONDS after start, so it gets longer.
    public static double firstShotReadyTimeoutSeconds = 2.5;
    public static double BLOCK_RELEASE_SECONDS = 0.35;
    public static double FIRST_SHOT_FEED_SECONDS = 0.80;   // the preload balls
    public static double FLOWER_FEED_SECONDS = 1.30;

    public static double SECOND_FLOWER_INTAKE_HOLD_SECONDS = 0.35;
    public static double FLOWER_LIFT_DELAY_SECONDS = 0.15;

    // Bump at the second flower before the last shot (same as the fast auto). 0 = off.
    public static int secondBumpRepeats = 1;
    public static double bumpBackIn = 8.0;
    public static double bumpSpeed = 0.45;
    public static double bumpHoldSeconds = 0.25;
    public static double bumpTimeoutSeconds = 1.5;

    public static boolean passThroughEnabled = true;
    public static double passThroughToleranceIn = 4.0;
    public static double passThroughHeadingDeg = 25.0;

    // Limelight fine aim on the shots (off in the fast auto: fixed presets). The Limelight is
    // still used for the tip either way.
    public static boolean useVisionAim = false;
    public static double visionAimTimeoutSeconds = 0.40;

    // Fixed flywheel speed per shot. After a shot the flywheel goes straight to the next one.
    // Hood: NaN = RobotConstants.hoodPosition.
    public static double shot1Tps = 2000.0;   // wall shot (fast auto shot 2)
    public static double shot2Tps = 1970.0;   // second flower (fast auto shot 3)
    public static double shot1Hood = Double.NaN;
    public static double shot2Hood = Double.NaN;

    public static double TPS_READY_FRACTION = 0.95;
    public static double PRE_RELEASE_TPS_FRACTION = 0.75;

    public static double PATH6_TIMEOUT_SECONDS = 4.0;
    public static double PATH7_TIMEOUT_SECONDS = 2.5;
    public static double PATH8_TIMEOUT_SECONDS = 5.0;

    public static double MIN_PATH_RUNTIME_SECONDS = 0.05;
    public static double END_POSITION_TOLERANCE_IN = 1.5;
    public static double END_HEADING_TOLERANCE_DEG = 10.0;

    // Stall detection on every path (fast auto, 27 Nov 2026).
    public static boolean stallDetectAllPaths = true;
    public static double stallGraceSeconds = 0.40;
    public static double stallWindowSeconds = 0.35;
    public static double stallMinMoveIn = 0.5;
    public static double stallMinTurnDeg = 3.0;

    public static boolean PRE_AIM_TURRET = true;
    public static double TELEMETRY_INTERVAL_SECONDS = 0.50;

    // ---------------------------------------------------------------------------------------
    // State
    // ---------------------------------------------------------------------------------------

    private double activeTurretPosition = 0.50;
    private int shotCount = 0;
    private double shooterReadyStartTime = 0.0;
    private boolean shooterReadyTimeoutFired = false;
    private double activeReadyTimeoutSeconds = 1.5;
    private double shooterFirstReadyTime = -1.0;
    private boolean anyPathTimeoutFired = false;
    private boolean currentPathTimedOut = false;
    private String lastTimedOutPath = "NONE";
    private String lastBlockedPath = "NONE";
    private String lastPassThroughPath = "NONE";
    private int blockedCount = 0;
    private Pose blockedRefPose = null;
    private double blockedRefTime = 0.0;
    private double pathStartTime = 0.0;
    private double turretMoveTime = 0.0;
    private double lastLoopTime = 0.0;
    private double loopMs = 0.0;
    private double maxLoopMs = 0.0;
    private double lastTelemetryTime = -1.0;

    private enum ShootPhase { WAIT, RELEASE, FEED, DONE }
    private ShootPhase shootPhase = ShootPhase.DONE;
    private double releaseStartTime = 0.0;
    private double feedStartTime = 0.0;
    private double shootFeedSeconds = 0.0;

    private boolean visionLocked = false;
    private String lastVisionShot = "NONE";

    // HIVE wait
    private boolean hiveTipWaitActive = false;
    private double hiveTipWaitStartTime = 0.0;
    private int hiveTipConfirmCount = 0;
    private boolean hiveTipConfirmed = false;
    private double hiveTipConfirmedTime = 0.0;
    private boolean hiveTipTimedOut = false;
    private Boolean tipBaselineUpright = null;
    private long lastTipFrameId = -1L;
    /* Tip timed out with SHOOT_ON_TIP_TIMEOUT false: skip shots and the flower, only park. */
    private boolean skipToPark = false;

    private boolean watchedVisible = false;
    private double watchedRoll = Double.NaN;
    private double watchedPercent = 0.0;
    private String visibleClusterText = "none";

    // ---------------------------------------------------------------------------------------
    // Poses (fast auto points 5-8)
    // ---------------------------------------------------------------------------------------

    private final Pose start = poseFactory.of(23.7064, 130.1432, 90.0);   // fast auto point5
    private final Pose point6 = poseFactory.of(42.0, 104.1657, 90.0);
    private final Pose point7Start = poseFactory.of(42.0, 104.1657, 90.0);
    private final Pose point7 = poseFactory.of(43.0, 120.0, 90.0);
    private final Pose point8Start = poseFactory.of(43.0, 120.0, 90.0);
    private final Pose point8 = poseFactory.of(0.0, 116.1719, 0.0);

    private static class Leg {
        final Pose plannedStart;
        final Pose end;
        final double speed;
        final double timeoutSeconds;
        final String name;
        final boolean passThrough;
        final boolean runsWhenSkipping;   // the park leg always runs

        Leg(Pose plannedStart, Pose end, double speed, double timeoutSeconds, String name,
            boolean passThrough, boolean runsWhenSkipping) {
            this.plannedStart = plannedStart;
            this.end = end;
            this.speed = speed;
            this.timeoutSeconds = timeoutSeconds;
            this.name = name;
            this.passThrough = passThrough;
            this.runsWhenSkipping = runsWhenSkipping;
        }
    }

    private Leg leg6() {
        return new Leg(start, point6, DRIVE_SPEED, PATH6_TIMEOUT_SECONDS, "PATH 6", true, false);
    }

    private Leg leg7() {
        return new Leg(point7Start, point7, FLOWER_COLLECT_SPEED, PATH7_TIMEOUT_SECONDS,
                "PATH 7", false, false);
    }

    private Leg leg8() {
        return new Leg(point8Start, point8, PARK_SPEED, PATH8_TIMEOUT_SECONDS, "PATH 8",
                false, true);
    }

    // ---------------------------------------------------------------------------------------
    // Path following (fast auto)
    // ---------------------------------------------------------------------------------------

    private Path buildPath(Leg leg) {
        Pose from = follower.pose();
        if (from == null) from = leg.plannedStart;
        return line(from, leg.end).linear(from, leg.end)
                .with(Constants.foresightConfig.maxPathSpeed.at(leg.speed));
    }

    private Command followLeg(Leg leg) {
        return followLeg(leg, null);
    }

    private Command followLeg(Leg leg, Runnable onStart) {
        return new CommandBuilder()
                .setStart(() -> {
                    currentPathTimedOut = false;
                    blockedRefPose = null;
                    if (skipToPark && !leg.runsWhenSkipping) return;
                    if (onStart != null) onStart.run();
                    pathStartTime = getRuntime();
                    follower.follow(buildPath(leg));
                })
                .setDone(() -> {
                    if (skipToPark && !leg.runsWhenSkipping) return true;
                    if (isPassThroughDone(leg)) {
                        lastPassThroughPath = leg.name;
                        return true;   // keep moving: the next leg's follow() takes over
                    }
                    boolean finished = isLegFinished(leg);
                    if (finished) releaseFollower();
                    return finished;
                });
    }

    private boolean isPassThroughDone(Leg leg) {
        if (!passThroughEnabled || !leg.passThrough) return false;
        if (getRuntime() - pathStartTime < MIN_PATH_RUNTIME_SECONDS) return false;
        Pose pose = follower.pose();
        if (pose == null) return false;
        double headingErrDeg = Math.abs(angleDiffDeg(
                Math.toDegrees(leg.end.heading()), Math.toDegrees(pose.heading())));
        double dist = Math.hypot(leg.end.x() - pose.x(), leg.end.y() - pose.y());
        return dist <= passThroughToleranceIn && headingErrDeg <= passThroughHeadingDeg;
    }

    private double distanceToEnd(Leg leg) {
        Pose pose = follower.pose();
        if (pose == null) return Double.MAX_VALUE;
        return Math.hypot(leg.end.x() - pose.x(), leg.end.y() - pose.y());
    }

    private boolean isLegFinished(Leg leg) {
        double now = getRuntime();
        if (now - pathStartTime >= leg.timeoutSeconds) {
            currentPathTimedOut = true;
            anyPathTimeoutFired = true;
            lastTimedOutPath = leg.name;
            return true;
        }
        if (now - pathStartTime < MIN_PATH_RUNTIME_SECONDS) return false;
        if (follower.atParametricEnd()) return true;

        Pose pose = follower.pose();
        if (pose == null) return false;

        double headingErrDeg = Math.abs(angleDiffDeg(
                Math.toDegrees(leg.end.heading()), Math.toDegrees(pose.heading())));
        if (distanceToEnd(leg) <= END_POSITION_TOLERANCE_IN
                && headingErrDeg <= END_HEADING_TOLERANCE_DEG) {
            return true;
        }
        return isBlocked(leg, pose, now);
    }

    /** No movement or turning for stallWindowSeconds (after the grace period) ends the leg. */
    private boolean isBlocked(Leg leg, Pose pose, double now) {
        if (!stallDetectAllPaths) return false;
        if (now - pathStartTime < stallGraceSeconds) {
            blockedRefPose = null;
            return false;
        }
        if (blockedRefPose == null) {
            blockedRefPose = pose;
            blockedRefTime = now;
            return false;
        }
        double moved = Math.hypot(pose.x() - blockedRefPose.x(), pose.y() - blockedRefPose.y());
        double turned = Math.abs(angleDiffDeg(Math.toDegrees(pose.heading()),
                Math.toDegrees(blockedRefPose.heading())));
        if (moved > stallMinMoveIn || turned > stallMinTurnDeg) {
            blockedRefPose = pose;
            blockedRefTime = now;
            return false;
        }
        if (now - blockedRefTime >= stallWindowSeconds) {
            blockedCount++;
            lastBlockedPath = String.format("%s at %.1fs, %.1f in from end",
                    leg.name, now - pathStartTime, distanceToEnd(leg));
            return true;
        }
        return false;
    }

    private double angleDiffDeg(double a, double b) {
        double d = (a - b) % 360.0;
        if (d > 180.0) d -= 360.0;
        if (d < -180.0) d += 360.0;
        return d;
    }

    private void releaseFollower() {
        if (follower != null) follower.stop();
    }


    // ---------------------------------------------------------------------------------------
    // Alliance. The field plays exactly the same from either side, so the blue auto is this
    // class run in the same (red) coordinates with the same presets and timings. Only the
    // stored alliance, the Limelight tag set and the watched tip cell change. The blue Driver Station
    // entry is a tiny subclass that overrides isBlueAlliance().
    // ---------------------------------------------------------------------------------------

    /** Overridden to true by the blue subclass. */
    protected boolean isBlueAlliance() {
        return false;
    }

    private void storeAlliance() {
        if (isBlueAlliance()) PoseStorage.setBlue();
        else PoseStorage.setRed();
    }

    private LimelightCam.Alliance aimAlliance() {
        return isBlueAlliance() ? LimelightCam.Alliance.BLUE : LimelightCam.Alliance.RED;
    }

    private String allianceName() {
        return isBlueAlliance() ? "BLUE" : "RED";
    }

    // ---------------------------------------------------------------------------------------
    // OpMode
    // ---------------------------------------------------------------------------------------

    @Override
    public void init() {
        Scheduler.reset();
        configureBulkCaching();
        clearBulkCache();

        storeAlliance();
        PoseStorage.setPose(start);

        follower = Constants.create(hardwareMap);
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
        flower.flowerActivate(RobotConstants.flowerLiftPos);

        resetRuntimeState();
        // Camera (on the turret) faces the HIVE / goal from INIT
        setTipWatchTurretPosition();
    }

    @Override
    public void init_loop() {
        clearBulkCache();
        storeAlliance();
        intake.update();

        if (visionAvailable) {
            try {
                limelightCam.update();
            } catch (RuntimeException ignored) {
                // shown as not seen
            }
        }
        readWatchedCell();
        turret.setFixedServoPosition(activeTurretPosition);

        Pose currentPose = follower.pose();
        if (currentPose != null) PoseStorage.setPose(currentPose);

        telemetry.addLine("===== " + allianceName() + " HIVE-TIP AUTO (FAST BACK END) =====");
        telemetry.addData("Start Pose", start);
        telemetry.addData("Tip Target", "%s (turret %.3f)", watchCell(), tipWatchPosition());
        telemetry.addData("Tip Rule", TIP_ON_VISIBLE ? "cell SEEN" : TIP_ON_FLIP ? "cell FLIPS" : "cell UP");
        telemetry.addData("Tip Timeout", "%.1f s -> %s", HIVE_TIP_WAIT_SECONDS,
                SHOOT_ON_TIP_TIMEOUT ? "shoot anyway" : "park only");
        telemetry.addData("Limelight", visionAvailable
                ? LimelightCam.cameraStatus : "NOT FOUND (tip wait will time out)");
        telemetry.addData("Visible Clusters", visibleClusterText);
        telemetry.addData("Watched Cell", watchedText());
        if (TIP_ON_VISIBLE && watchedVisible) {
            telemetry.addLine("NOTE: watched cell already visible - will fire "
                    + TIP_FIRE_DELAY_SECONDS + " s after start");
        }
        telemetry.addData("Shots", "wall %.0f TPS @ %.3f | flower %.0f TPS @ %.3f",
                shot1Tps, FAR_WALL_TURRET_POSITION, shot2Tps, SECOND_FLOWER_TURRET_POSITION);
        telemetry.update();
    }

    @Override
    public void start() {
        clearBulkCache();
        storeAlliance();
        follower.setPose(start);
        PoseStorage.setPose(start);
        resetRuntimeState();
        setTipWatchTurretPosition();
        stopIntakeAndBlock();
        flower.flowerActivate(RobotConstants.flowerLiftPos);
        // Spin up for the wall shot now; it holds through the tip wait
        outtake.setTargetTPS(shotTps(1));
        lastLoopTime = getRuntime();
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        double loopNow = getRuntime();
        loopMs = (loopNow - lastLoopTime) * 1000.0;
        lastLoopTime = loopNow;
        if (loopMs > maxLoopMs) maxLoopMs = loopMs;

        clearBulkCache();

        // Newest camera frame first, so the tip wait sees it this loop
        updateVision();
        updateHiveTipObservation();

        Scheduler.execute();
        follower.update();

        Pose currentPose = follower.pose();
        if (currentPose != null) PoseStorage.setPose(currentPose);

        updateTurret();
        intake.update();
        outtake.setHoodPosition(shotHood(upcomingShot()));
        outtake.setGoalDistance(Double.NaN);   // fixed per-shot TPS
        outtake.update();

        if (lastTelemetryTime < 0.0 || loopNow - lastTelemetryTime >= TELEMETRY_INTERVAL_SECONDS) {
            lastTelemetryTime = loopNow;
            displayTelemetry(currentPose);
        }
    }

    @Override
    public void stop() {
        Scheduler.reset();
        if (follower != null) {
            Pose finalPose = follower.pose();
            if (finalPose != null) PoseStorage.setPose(finalPose);
            follower.stop();
        }
        storeAlliance();
        if (outtake != null) outtake.stopOuttake();
        if (intake != null) intake.stopIntake();
        if (robotSettings != null) robotSettings.block();
        if (flower != null) flower.flowerActivate(RobotConstants.flowerLiftPos);
        if (turret != null) turret.setCentre();
        closeCamera();
        restoreBulkCaching();
    }

    // ---------------------------------------------------------------------------------------
    // Routine
    // ---------------------------------------------------------------------------------------

    private Command autoRoutine() {
        return sequential(
                race(
                        waitForHiveTipOrTimeout(),
                        infinite(this::holdForTip)
                ),
                shootCycle(FAR_WALL_TURRET_POSITION, FIRST_SHOT_FEED_SECONDS),
                followTravel(leg6()),
                collectFlower(leg7(), SECOND_FLOWER_INTAKE_HOLD_SECONDS,
                        SECOND_FLOWER_TURRET_POSITION),
                bumpFlower(point7, point7Start, secondBumpRepeats, "BUMP 2"),
                shootCycle(SECOND_FLOWER_TURRET_POSITION, FLOWER_FEED_SECONDS),
                parkAndLiftFlower(),
                instant(this::finishAutoMechanisms)
        );
    }

    /** Wait that ends early if the auto has switched to park-only. */
    private Command skippableWait(double seconds) {
        return race(waitSeconds(seconds), waitUntil(() -> skipToPark));
    }

    private Command collectFlower(Leg leg, double intakeHoldSeconds, double aimAfterPosition) {
        return sequential(
                followLeg(leg, this::holdFlowerCollection),
                race(
                        skippableWait(intakeHoldSeconds),
                        infinite(() -> {
                            holdFlowerCollection();
                            preAimTurret(aimAfterPosition);
                        })
                )
        );
    }

    /** Back off the pickup toward approachFrom by bumpBackIn, drive back in, hold the intake. */
    private Command bumpFlower(Pose pickup, Pose approachFrom, int repeatCount, String name) {
        int repeats = Math.max(0, repeatCount);
        if (repeats == 0) return instant(() -> { });

        Pose back = bumpBackPose(pickup, approachFrom);
        Command[] steps = new Command[repeats * 3];
        for (int i = 0; i < repeats; i++) {
            steps[i * 3] = race(
                    followLeg(new Leg(pickup, back, bumpSpeed, bumpTimeoutSeconds,
                            name + " BACK", false, false), this::holdFlowerCollection),
                    infinite(this::holdFlowerCollection)
            );
            steps[i * 3 + 1] = race(
                    followLeg(new Leg(back, pickup, bumpSpeed, bumpTimeoutSeconds,
                            name + " IN", false, false), this::holdFlowerCollection),
                    infinite(this::holdFlowerCollection)
            );
            steps[i * 3 + 2] = race(
                    skippableWait(bumpHoldSeconds),
                    infinite(this::holdFlowerCollection)
            );
        }
        return sequential(steps);
    }

    private Pose bumpBackPose(Pose pickup, Pose approachFrom) {
        double dx = approachFrom.x() - pickup.x();
        double dy = approachFrom.y() - pickup.y();
        double len = Math.hypot(dx, dy);
        if (len < 1e-6) return pickup;
        double back = Math.max(0.0, bumpBackIn);
        return new Pose(pickup.x() + dx / len * back, pickup.y() + dy / len * back,
                pickup.heading());
    }

    private void holdFlowerCollection() {
        if (skipToPark) return;
        robotSettings.block();
        outtake.stopFeeder();
        flower.flowerActivate(RobotConstants.flowerDropPos);
        intake.intake(RobotConstants.frontIntakeSpeed, RobotConstants.backIntakeSpeed);
    }

    private void preAimTurret(double position) {
        if (skipToPark) return;
        if (PRE_AIM_TURRET && !Double.isNaN(position)) setFixedTurretPosition(position);
    }

    private Command followTravel(Leg leg) {
        return followLeg(leg, () -> {
            stopIntakeAndBlock();
            setTravelTurretPosition();
        });
    }

    private Command parkAndLiftFlower() {
        return parallel(
                followLeg(leg8(), () -> {
                    stopIntakeAndBlock();
                    outtake.setTargetTPS(0.0);
                    setTravelTurretPosition();
                }),
                sequential(
                        waitSeconds(FLOWER_LIFT_DELAY_SECONDS),
                        instant(() -> flower.flowerActivate(RobotConstants.flowerLiftPos))
                )
        );
    }

    private void finishAutoMechanisms() {
        stopIntakeAndBlock();
        outtake.setTargetTPS(0.0);
        outtake.stopFeeder();
        flower.flowerActivate(RobotConstants.flowerLiftPos);
        setTravelTurretPosition();
    }

    // ---------------------------------------------------------------------------------------
    // HIVE tip
    // ---------------------------------------------------------------------------------------

    /** Held during the tip wait: flywheel spinning, feeder/intake stopped, blocker closed. */
    private void holdForTip() {
        intake.stopIntake();
        outtake.stopFeeder();
        robotSettings.block();
    }

    private Command waitForHiveTipOrTimeout() {
        return new CommandBuilder()
                .setStart(() -> {
                    hiveTipWaitStartTime = getRuntime();
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
                        return getRuntime() - hiveTipConfirmedTime
                                >= Math.max(0.0, TIP_FIRE_DELAY_SECONDS);
                    }
                    if (getRuntime() - hiveTipWaitStartTime >= Math.max(0.0, HIVE_TIP_WAIT_SECONDS)) {
                        hiveTipWaitActive = false;
                        hiveTipTimedOut = true;
                        if (!SHOOT_ON_TIP_TIMEOUT) {
                            skipToPark = true;
                            shootPhase = ShootPhase.DONE;
                            stopIntakeAndBlock();
                            outtake.setTargetTPS(0.0);
                            setTravelTurretPosition();
                        }
                        return true;
                    }
                    return false;
                });
    }

    /** Counts NEW Limelight frames where the watched cell is in the tipped state. */
    private void updateHiveTipObservation() {
        if (!hiveTipWaitActive || !visionAvailable || hiveTipConfirmed || hiveTipTimedOut) return;

        LimelightCam.HiveDetection watched = readWatchedCell();
        if (watched == null) {
            hiveTipConfirmCount = 0;
            return;
        }

        long frameId = limelightCam.getFrameId();
        if (frameId == lastTipFrameId) return;
        lastTipFrameId = frameId;

        boolean upright = isUpright(watched.roll);
        boolean tipState;
        if (TIP_ON_VISIBLE) {
            tipState = true;
        } else if (TIP_ON_FLIP) {
            if (tipBaselineUpright == null) {
                tipBaselineUpright = upright;
                hiveTipConfirmCount = 0;
                return;
            }
            tipState = upright != tipBaselineUpright;
        } else {
            tipState = upright;
        }

        hiveTipConfirmCount = tipState ? hiveTipConfirmCount + 1 : 0;
        if (hiveTipConfirmCount >= Math.max(1, HIVE_TIP_CONFIRM_FRAMES)) {
            hiveTipConfirmed = true;
            hiveTipConfirmedTime = getRuntime();
            hiveTipWaitActive = false;
        }
    }

    private static LimelightCam.HiveCell parseWatchCell(String text) {
        String t = text == null ? "" : text.trim().toUpperCase();
        LimelightCam.Alliance alliance =
                t.contains("BLUE") ? LimelightCam.Alliance.BLUE : LimelightCam.Alliance.RED;
        LimelightCam.HiveSide side =
                t.contains("AUDIENCE") ? LimelightCam.HiveSide.AUDIENCE : LimelightCam.HiveSide.FAR;
        return new LimelightCam.HiveCell(alliance, side);
    }

    /** The watched cell from the current frame (null if not seen); refreshes telemetry text. */
    private LimelightCam.HiveDetection readWatchedCell() {
        watchedVisible = false;
        watchedPercent = 0.0;
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

        LimelightCam.HiveDetection watched = limelightCam.getHiveDetection(parseWatchCell(watchCell()));
        if (watched != null) {
            watchedVisible = true;
            watchedRoll = watched.roll;
            watchedPercent = watched.percentClusterFound;
        }
        return watched;
    }

    private boolean isUpright(double roll) {
        return Double.isFinite(roll) && Math.abs(roll) < UPRIGHT_ROLL_LIMIT_DEG;
    }

    private String watchedText() {
        return watchedVisible
                ? String.format("roll %.1f (%s), %.0f%% found", watchedRoll,
                isUpright(watchedRoll) ? "UP" : "down", watchedPercent)
                : "not seen";
    }

    /** The cell watched for the tip on this alliance. */
    private String watchCell() {
        return isBlueAlliance() ? tipWatchCellBlue : tipWatchCell;
    }

    private double tipWatchPosition() {
        return Double.isFinite(tipWatchTurretPosition)
                ? tipWatchTurretPosition : FAR_WALL_TURRET_POSITION;
    }

    /** Turret straight to the tip-watch position, counted as already settled. */
    private void setTipWatchTurretPosition() {
        activeTurretPosition = clamp(tipWatchPosition() + AUTO_TURRET_TRIM, 0.0, 1.0);
        turret.setFixedServoPosition(activeTurretPosition);
        turretMoveTime = -1000.0;   // long ago: no settle wait if the first shot uses it
    }

    // ---------------------------------------------------------------------------------------
    // Shooting (fast auto state machine)
    // ---------------------------------------------------------------------------------------

    private Command shootCycle(double turretPosition, double feedSeconds) {
        return sequential(
                instant(() -> beginShot(turretPosition, feedSeconds)),
                race(
                        infinite(this::runShotStateMachine),
                        waitUntil(() -> shootPhase == ShootPhase.DONE)
                )
        );
    }

    private void beginShot(double turretPosition, double feedSeconds) {
        if (skipToPark) {
            shootPhase = ShootPhase.DONE;
            return;
        }
        shotCount++;
        turret.clearVisionCorrection();
        // Only records a move (and a settle wait) if the turret isn't already there
        setFixedTurretPosition(turretPosition);
        outtake.setTargetTPS(shotTps(shotCount));
        activeReadyTimeoutSeconds = shotCount == 1
                ? firstShotReadyTimeoutSeconds : SHOOT_READY_TIMEOUT_SECONDS;
        startShooterReadyWait();
        stopIntakeAndBlock();
        shooterFirstReadyTime = -1.0;
        shootFeedSeconds = feedSeconds;
        shootPhase = ShootPhase.WAIT;
        visionLocked = false;
    }

    private void runShotStateMachine() {
        if (skipToPark) {
            shootPhase = ShootPhase.DONE;
            return;
        }
        double now = getRuntime();
        if (shootPhase != ShootPhase.DONE) shooterSpooled(now);

        if (shootPhase == ShootPhase.WAIT) {
            boolean turretSettled = now - turretMoveTime >= TURRET_SETTLE_SECONDS
                    && visionAimDone(now);
            if (turretSettled
                    && (isShooterAtFraction(PRE_RELEASE_TPS_FRACTION) || shooterReadyOrTimedOut())) {
                shootPhase = ShootPhase.RELEASE;
                releaseStartTime = now;
            }
        }

        if (shootPhase == ShootPhase.RELEASE
                && now - releaseStartTime >= BLOCK_RELEASE_SECONDS
                && shooterSpooled(now)) {
            shootPhase = ShootPhase.FEED;
            feedStartTime = now;
            freezeVisionAim();
        }

        if (shootPhase == ShootPhase.FEED && now - feedStartTime >= shootFeedSeconds) {
            shootPhase = ShootPhase.DONE;
            stopIntakeAndBlock();
            // Straight to the next shot's speed while driving there
            outtake.setTargetTPS(shotTps(shotCount + 1));
            return;
        }

        switch (shootPhase) {
            case WAIT:
                holdShooterAndBlock();
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
        intake.intake(RobotConstants.frontFeedSpeedAuto, RobotConstants.backFeedSpeedAuto);
        outtake.setFeederOn(true);
    }

    private void stopIntakeAndBlock() {
        intake.stopIntake();
        outtake.stopFeeder();
        robotSettings.block();
    }

    private int upcomingShot() {
        return shootPhase == ShootPhase.DONE ? shotCount + 1 : shotCount;
    }

    /** 1 = wall shot, 2 = second flower; anything after keeps the second flower's speed. */
    private double shotTps(int n) {
        return n <= 1 ? shot1Tps : shot2Tps;
    }

    private double shotHood(int n) {
        double hood = n <= 1 ? shot1Hood : shot2Hood;
        return Double.isFinite(hood) ? hood : RobotConstants.hoodPosition;
    }

    private void startShooterReadyWait() {
        shooterReadyStartTime = getRuntime();
        shooterReadyTimeoutFired = false;
    }

    /** Stable band held for spoolUpExtraSeconds (overspeed is not stable), or the timeout. */
    private boolean shooterSpooled(double now) {
        if (shooterInStableBand()) {
            if (shooterFirstReadyTime < 0.0) shooterFirstReadyTime = now;
            if (now - shooterFirstReadyTime >= Math.max(0.0, spoolUpExtraSeconds)) return true;
        } else {
            shooterFirstReadyTime = -1.0;
        }
        if (now - shooterReadyStartTime >= activeReadyTimeoutSeconds) {
            shooterReadyTimeoutFired = true;
            return true;
        }
        return false;
    }

    private boolean shooterInStableBand() {
        double targetTPS = outtake.getTargetTPS();
        if (targetTPS <= 0.0) return false;
        double band = targetTPS * Math.max(0.0, shooterStableBandFraction);
        return Math.abs(outtake.getCurrentTPS() - targetTPS) <= band;
    }

    private boolean shooterReadyOrTimedOut() {
        if (isShooterAtFraction(TPS_READY_FRACTION)) return true;
        if (getRuntime() - shooterReadyStartTime >= activeReadyTimeoutSeconds) {
            shooterReadyTimeoutFired = true;
            return true;
        }
        return false;
    }

    private boolean isShooterAtFraction(double fraction) {
        double targetTPS = outtake.getTargetTPS();
        double currentTPS = outtake.getCurrentTPS();
        return targetTPS > 0.0 && currentTPS >= targetTPS * clamp(fraction, 0.0, 1.0);
    }

    // ---------------------------------------------------------------------------------------
    // Turret
    // ---------------------------------------------------------------------------------------

    private void moveTurretTo(double position) {
        double target = clamp(position, 0.0, 1.0);
        if (Math.abs(target - activeTurretPosition) > 1e-6) turretMoveTime = getRuntime();
        activeTurretPosition = target;
        turret.setFixedServoPosition(activeTurretPosition);
    }

    private void setFixedTurretPosition(double position) {
        moveTurretTo(position + AUTO_TURRET_TRIM);
    }

    private void setTravelTurretPosition() {
        moveTurretTo(TRAVEL_TURRET_POSITION);
    }

    private void updateTurret() {
        if (visionWindowOpen() && (visionLocked || turret.isVisionTargetVisible())) {
            visionLocked = true;
            turret.aimVisionOnly();
        } else {
            turret.setFixedServoPosition(activeTurretPosition);
        }
    }

    // ---------------------------------------------------------------------------------------
    // Limelight
    // ---------------------------------------------------------------------------------------

    private void initLimelight() {
        limelightCam = null;
        visionAvailable = false;
        if (!VISION_ENABLED) return;
        try {
            limelightCam = new LimelightCam(hardwareMap);
            limelightCam.init();
            visionAvailable = true;
        } catch (RuntimeException e) {
            limelightCam = null;   // tip wait times out (SHOOT_ON_TIP_TIMEOUT), presets only
        }
    }

    /** Reads the newest frame every loop (for the tip); feeds the turret only if useVisionAim. */
    private void updateVision() {
        if (!visionAvailable) {
            turret.clearVisionMeasurement();
            return;
        }
        try {
            limelightCam.update();
            LimelightCam.HiveDetection d = useVisionAim
                    ? limelightCam.getAimDetection(aimAlliance()) : null;
            if (d != null) turret.setVisionMeasurement(true, d.bearing, limelightCam.getFrameId());
            else turret.clearVisionMeasurement();
        } catch (RuntimeException e) {
            turret.clearVisionMeasurement();
        }
    }

    private boolean visionWindowOpen() {
        return useVisionAim && visionAvailable
                && (shootPhase == ShootPhase.WAIT || shootPhase == ShootPhase.RELEASE)
                && getRuntime() - turretMoveTime >= TURRET_SETTLE_SECONDS;
    }

    private boolean visionAimDone(double now) {
        if (!useVisionAim || !visionAvailable || !visionLocked) return true;
        if (turret.isVisionOnTarget()) return true;
        return now - turretMoveTime - TURRET_SETTLE_SECONDS >= visionAimTimeoutSeconds;
    }

    private void freezeVisionAim() {
        if (visionLocked) {
            activeTurretPosition = Turret.lastServoPosition;
            lastVisionShot = String.format("shot %d: %.3f (%s)", shotCount,
                    activeTurretPosition, turret.isVisionOnTarget() ? "on target" : "timeout");
        } else if (useVisionAim && visionAvailable) {
            lastVisionShot = String.format("shot %d: preset, no tags", shotCount);
        }
        visionLocked = false;
    }

    private void closeCamera() {
        if (limelightCam != null) {
            try {
                limelightCam.close();
            } catch (RuntimeException ignored) {
                // already closed
            }
            limelightCam = null;
            visionAvailable = false;
        }
    }

    // ---------------------------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------------------------

    private Command waitSeconds(double seconds) {
        return waitMs(Math.round(Math.max(0.0, seconds) * 1000.0));
    }

    private void resetRuntimeState() {
        activeTurretPosition = TRAVEL_TURRET_POSITION;
        shotCount = 0;
        shooterReadyStartTime = 0.0;
        shooterReadyTimeoutFired = false;
        activeReadyTimeoutSeconds = SHOOT_READY_TIMEOUT_SECONDS;
        shooterFirstReadyTime = -1.0;
        anyPathTimeoutFired = false;
        currentPathTimedOut = false;
        lastTimedOutPath = "NONE";
        lastBlockedPath = "NONE";
        lastPassThroughPath = "NONE";
        blockedCount = 0;
        blockedRefPose = null;
        blockedRefTime = 0.0;
        pathStartTime = 0.0;
        turretMoveTime = 0.0;
        lastLoopTime = 0.0;
        loopMs = 0.0;
        maxLoopMs = 0.0;
        lastTelemetryTime = -1.0;
        shootPhase = ShootPhase.DONE;
        releaseStartTime = 0.0;
        feedStartTime = 0.0;
        shootFeedSeconds = 0.0;
        visionLocked = false;
        lastVisionShot = "NONE";
        hiveTipWaitActive = false;
        hiveTipWaitStartTime = 0.0;
        hiveTipConfirmCount = 0;
        hiveTipConfirmed = false;
        hiveTipConfirmedTime = 0.0;
        hiveTipTimedOut = false;
        tipBaselineUpright = null;
        lastTipFrameId = -1L;
        skipToPark = false;
    }

    private void displayTelemetry(Pose currentPose) {
        telemetry.addLine("===== " + allianceName() + " HIVE-TIP AUTO (FAST BACK END) =====");
        if (currentPose != null) {
            telemetry.addData("Pose", "(%.1f, %.1f, %.1f deg)", currentPose.x(), currentPose.y(),
                    Math.toDegrees(currentPose.heading()));
        }
        telemetry.addData("Runtime", "%.2f s (%.2f left)", getRuntime(),
                Math.max(0.0, 30.0 - getRuntime()));
        telemetry.addData("Loop", "%.1f ms (max %.1f)", loopMs, maxLoopMs);

        telemetry.addLine("===== HIVE TIP =====");
        double tipElapsed = hiveTipWaitActive || hiveTipConfirmed || hiveTipTimedOut
                ? Math.max(0.0, getRuntime() - hiveTipWaitStartTime) : 0.0;
        telemetry.addData("Tip", "%s | %.1f / %.1f s | frames %d/%d",
                hiveTipConfirmed ? "CONFIRMED" : hiveTipTimedOut ? "TIMED OUT"
                                                 : hiveTipWaitActive ? "WAITING" : "-",
                tipElapsed, HIVE_TIP_WAIT_SECONDS,
                hiveTipConfirmCount, Math.max(1, HIVE_TIP_CONFIRM_FRAMES));
        telemetry.addData("Skip To Park", skipToPark);
        telemetry.addData("Watched Cell", "%s: %s", watchCell(), watchedText());
        telemetry.addData("Visible Clusters", visibleClusterText);

        telemetry.addLine("===== PATHS =====");
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Last Timeout", lastTimedOutPath);
        telemetry.addData("Blocked", "%d | last: %s", blockedCount, lastBlockedPath);
        telemetry.addData("Last Pass-Through", lastPassThroughPath);

        telemetry.addLine("===== SHOOTER =====");
        telemetry.addData("Shot", "%d | %s", shotCount, shootPhase);
        telemetry.addData("Shooter TPS", "%.0f / %.0f (next shot %d -> %.0f)",
                outtake.getCurrentTPS(), outtake.getTargetTPS(),
                upcomingShot(), shotTps(upcomingShot()));
        telemetry.addData("Spool", shooterFirstReadyTime < 0.0
                ? String.format("not stable (band +/-%.0f%%)", shooterStableBandFraction * 100.0)
                : String.format("stable %.2f s (need %.2f)",
                getRuntime() - shooterFirstReadyTime, spoolUpExtraSeconds));
        telemetry.addData("Ready Timeout", shooterReadyTimeoutFired);
        telemetry.addData("Feeder Power", "%.2f", outtake.getFeederCommandedPower());
        telemetry.addData("Feed Link", "scale %.2f | %.0f TPS",
                outtake.getFeedLinkScale(), outtake.getFeedLinkTPS());
        telemetry.addData("Hood", "%.3f", outtake.getHoodPosition());

        telemetry.addLine("===== TURRET =====");
        telemetry.addData("Active / Servo", "%.3f / %.3f", activeTurretPosition,
                Turret.lastServoPosition);
        if (useVisionAim) {
            telemetry.addData("Vision Aim", visionAvailable
                    ? String.format("%s | bearing %.1f | %s", Turret.lastVisionStatus,
                    Turret.lastVisionBearingDeg, lastVisionShot)
                    : "LIMELIGHT NOT FOUND - presets only");
        }
        telemetry.update();
    }

    private void configureBulkCaching() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    private void clearBulkCache() {
        if (allHubs == null) return;
        for (LynxModule hub : allHubs) hub.clearBulkCache();
    }

    private void restoreBulkCaching() {
        if (allHubs == null) return;
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }

    private double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }
}