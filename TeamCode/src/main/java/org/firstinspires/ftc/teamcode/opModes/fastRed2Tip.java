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
 * FAST version of FlowerRedAuto. Same route and the same shots; the original
 * FlowerRedAuto is untouched, so you can switch back from the Driver Station.
 *
 * Where the time comes from (every change is a Panels tunable; the old value
 * is in the comment next to it):
 *
 *  1. PASS-THROUGH legs. Paths 1, 3, 4 and 6 no longer stop at their end
 *     point: the next path starts as soon as the robot is within
 *     passThroughToleranceIn, without braking. Only legs that end at a pickup,
 *     the wall or a shot still stop properly.
 *  2. Faster path speeds (approach, collect, wall approach, drive).
 *  3. Shooter spins up in start(), so the preload shot doesn't wait for it.
 *  4. Shorter settle, blocker release, feed, intake hold and flower timings.
 *  5. Limelight fine aim on each shot (same as the other autos), with a short
 *     timeout so it never costs more than visionAimTimeoutSeconds.
 *
 * Shooting (26 Nov 2026): the same aiming and regression as the TeleOp.
 *  - POSE AIM: for each shot the turret aims from odometry at the red goal
 *    (goal coordinates and upper/lower switch taken from CombinedTeleOp, so
 *    there is one place to tune them), with the Limelight correcting the aim
 *    when the goal cluster is in view (Turret "vision correction").
 *    If the pose is bad or the goal is outside the turret's arc, the shot
 *    falls back to its preset position + Limelight fine aim, as before.
 *  - LUT REGRESSION: the flywheel speed and hood come from the Outtake
 *    lookup table, using the distance from the turret to the goal. The
 *    distance is sent every loop, so the flywheel is already near the right
 *    speed when the robot arrives. Outside the table it clamps (Outtake
 *    CLAMP_DISTANCE_TO_RANGE); with no valid distance it uses outtakeTps.
 *
 * Tune it back toward the old values if anything gets unreliable. The things
 * most likely to need it: FLOWER_FEED_SECONDS (all balls out?), the hold
 * times (all pollen in?) and DRIVE_SPEED on paths 4/6 (heading wobble).
 */
@Configurable
@Autonomous(name = "fastRed2Tip", group = "Main")
public class fastRed2Tip extends OpMode {

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

    // Route / mechanism values unchanged from the supplied auto unless marked CHANGED.
    public static double PRELOAD_TURRET_POSITION = 0.0;
    public static double FAR_WALL_TURRET_POSITION = 0.35;
    public static double SECOND_FLOWER_TURRET_POSITION = 0.45;
    public static double TRAVEL_TURRET_POSITION = 0.50;
    public static double AUTO_TURRET_TRIM = 0.0;

    // FAST: paths 4 and 6 translate and rotate together; if the heading wobbles on them,
    // drop DRIVE_SPEED back toward 0.85.
    public static double DRIVE_SPEED = 0.95;            // was 0.85
    public static double FLOWER_APPROACH_SPEED = 0.85;  // was 0.70
    public static double FLOWER_COLLECT_SPEED = 0.45;   // was 0.35
    public static double WALL_APPROACH_SPEED = 0.50;    // was 0.35 (stall exit still ends it at the wall)
    public static double PARK_SPEED = 1.00;

    public static double TURRET_SETTLE_SECONDS = 0.15;        // was 0.20
    public static double SHOOT_READY_TIMEOUT_SECONDS = 1.5;  // was 1.0 (27 Nov: +0.5 s spool)
    // Extra spool time (27 Nov 2026): once the shooter first reads at speed (or times out),
    // keep spinning this much longer before feeding, so it's settled, not just touching speed.
    public static double spoolUpExtraSeconds = 0.75;   // was 0.5 (27 Nov: shots overshooting)
    // The shooter only counts as settled while it is WITHIN this fraction of its target, above
    // OR below. Before, anything over 95% counted, so a flywheel still overshooting after
    // spin-up (running fast) fed straight away and the balls went long. Leaving the band
    // restarts the spoolUpExtraSeconds clock.
    public static double shooterStableBandFraction = 0.03;
    // The preload spins up from rest, so it gets longer before the "shoot anyway" timeout.
    public static double preloadReadyTimeoutSeconds = 2.5;
    // The turret is set to the preload position in init, so the first shot doesn't wait for it.
    public static boolean firstShotSkipSettle = true;
    public static double BLOCK_RELEASE_SECONDS = 0.35;        // was 0.40
    public static double PRELOAD_FEED_SECONDS = 0.80;         // was 1.00
    public static double FLOWER_FEED_SECONDS = 1.30;          // was 1.60

    public static double FIRST_FLOWER_INTAKE_HOLD_SECONDS = 0.50;   // was 0.75
    public static double SECOND_FLOWER_INTAKE_HOLD_SECONDS = 0.35;  // was 0.45
    public static double FLOWER_REVERSE_CLEAR_SECONDS = 0.25;       // was 0.35
    public static double FLOWER_LIFT_DELAY_SECONDS = 0.15;          // was 0.20

    // First flower "bump" (27 Nov 2026): after the first intake hold, back off a little and
    // drive back into the flower, to knock down any pollen that didn't drop. The intake runs
    // (re-commanded every loop) and the flower stays down the whole time. bumpRepeats = 0 turns it off.
    public static int bumpRepeats = 1;
    // Same bump at the second (last) flower before the final shot. 0 turns it off.
    // Uses the same bumpBackIn / bumpSpeed / bumpHoldSeconds / bumpTimeoutSeconds.
    public static int secondBumpRepeats = 1;
    public static double bumpBackIn = 8.0;          // how far to back off, inches (was 4.0)
    public static double bumpSpeed = 0.45;          // path speed for both moves
    public static double bumpHoldSeconds = 0.25;    // intake hold after driving back in
    public static double bumpTimeoutSeconds = 1.5;  // per move (stall detection usually ends it first)

    // Long drive toward the wall (path 4) moved toward the HIVE (27 Nov 2026), to keep clear of
    // the partner parking on our side. Shifts the end of path 3 and both ends of path 4 in +X
    // (toward the field centre / HIVE); path 5 then angles from there to the same wall point.
    // Was 0 (path 4 at x = 32.4-32.7). CHECK the robot still clears the HIVE base.
    public static double longDriveShiftIn = 4.0;   // was 6.0 (27 Nov: pushed over too far)

    // FAST: pass-through legs finish when this close to their end point (and heading within
    // passThroughHeadingDeg), without stopping, so the next leg carries on at speed.
    public static boolean passThroughEnabled = true;
    public static double passThroughToleranceIn = 4.0;
    public static double passThroughHeadingDeg = 25.0;

    // Path 4 -> 5 hand-off (28 Nov 2026). Pedro slows every path down to stop at its end point,
    // so path 4 was braking hard just before the hand-off, then path 5 had to accelerate again:
    // the stop-and-jitter. Now path 4 aims path4OvershootIn PAST its real end, so the robot is
    // still at speed when it reaches the hand-off point, and path 5 takes over there. If the
    // robot passes the hand-off point off to one side, crossing it still counts. 0 = old way.
    public static double path4OvershootIn = 12.0;

    // FAST: spin the shooter up in start() so the preload shot doesn't wait for it.
    public static boolean spinUpAtStart = true;

    // Limelight fine aim for each shot (see the other autos). Short timeout keeps it fast.
    public static boolean useVisionAim = false;   // 27 Nov 2026: fixed turret presets only
    public static double visionAimTimeoutSeconds = 0.40;

    // Shot aiming from odometry (+ Limelight correction) instead of fixed presets.
    public static boolean usePoseAim = false;     // 27 Nov 2026: fixed turret presets only
    // Flywheel speed + hood from the Outtake lookup table (distance to goal).
    // Ignored while useShotTps is on.
    public static boolean useLutRegression = true;

    // Fixed flywheel speed per shot (27 Nov 2026). When on, these replace the lookup table for
    // this auto. After each shot the flywheel goes straight to the NEXT shot's speed, so it is
    // already there when the robot arrives. Hood: NaN = RobotConstants.hoodPosition.
    public static boolean useShotTps = true;
    public static double shot1Tps = 1970.0;   // preload
    public static double shot2Tps = 2000.0;   // far wall
    public static double shot3Tps = 1970.0;   // second flower
    public static double shot1Hood = Double.NaN;
    public static double shot2Hood = Double.NaN;
    public static double shot3Hood = Double.NaN;
    // Longest a shot waits for the pose aim (turret settled, Limelight locked) before firing.
    public static double poseAimMaxWaitSeconds = 0.60;
    // Sanity check on the pose aim. Your presets are tuned and known good, so if the odometry
    // aim disagrees with a shot's preset by more than this AND the Limelight hasn't confirmed
    // the goal, the odometry pose is probably off (drift after pickups / pass-throughs): the
    // shot uses its preset + Limelight fine aim instead. A Limelight lock always wins.
    public static double poseAimMaxDeviationDeg = 10.0;

    public static double TPS_READY_FRACTION = 0.95;

    public static double PATH1_TIMEOUT_SECONDS = 4.0;
    public static double PATH2_TIMEOUT_SECONDS = 3.0;
    public static double PATH3_TIMEOUT_SECONDS = 3.0;
    public static double PATH4_TIMEOUT_SECONDS = 5.0;
    public static double PATH5_TIMEOUT_SECONDS = 1.5;
    public static double PATH6_TIMEOUT_SECONDS = 4.0;
    public static double PATH7_TIMEOUT_SECONDS = 2.5;
    public static double PATH8_TIMEOUT_SECONDS = 5.0;

    // NEW: path completion tuning.
    // Ignore "done" for this long after a path starts, so a stale atParametricEnd from the
    // previous path can't finish the new one instantly.
    public static double MIN_PATH_RUNTIME_SECONDS = 0.05;
    // Finish when this close to the end point AND heading is close (don't wait for t = 1).
    public static double END_POSITION_TOLERANCE_IN = 1.5;
    public static double END_HEADING_TOLERANCE_DEG = 10.0;
    // Wall stall exit (only legs flagged stallExit, i.e. path 5): if within
    // STALL_EXIT_MAX_DISTANCE_IN of the target and moving less than STALL_MOVE_TOLERANCE_IN
    // for STALL_TIME_SECONDS, treat the leg as finished instead of pushing until timeout.
    public static double STALL_EXIT_MAX_DISTANCE_IN = 8.0;
    public static double STALL_MOVE_TOLERANCE_IN = 0.3;
    public static double STALL_TIME_SECONDS = 0.30;

    // Stall detection on EVERY path (27 Nov 2026). If the odometry shows the robot hasn't
    // moved more than stallMinMoveIn AND hasn't turned more than stallMinTurnDeg for
    // stallWindowSeconds, it's blocked (another robot, a wall, a ball under a wheel) and the
    // leg ends straight away instead of pushing until its PATHn_TIMEOUT_SECONDS. The fixed
    // timeouts stay as a backstop. stallGraceSeconds skips the start of each leg, while the
    // robot is still accelerating. Also catches a frozen odometry reading (I2C fault).
    public static boolean stallDetectAllPaths = true;
    public static double stallGraceSeconds = 0.40;
    public static double stallWindowSeconds = 0.35;
    public static double stallMinMoveIn = 0.5;
    public static double stallMinTurnDeg = 3.0;

    // NEW: shot sequencing.
    // The blocker starts opening once the turret has settled and the shooter is above this
    // fraction of target, instead of waiting for TPS_READY_FRACTION first. Feeding still waits
    // for TPS_READY_FRACTION (or its timeout). Set this >= TPS_READY_FRACTION to get the old
    // "ready, then release" order back.
    public static double PRE_RELEASE_TPS_FRACTION = 0.75;   // was 0.80

    // NEW: start moving the turret to its shot position before the robot arrives (far wall shot:
    // within PRE_AIM_DISTANCE_IN of the end of path 5; second flower shot: during the intake
    // hold) so TURRET_SETTLE_SECONDS has already elapsed by the time the shot starts.
    // Set false if the turret can't safely move while the robot is driving/intaking.
    public static boolean PRE_AIM_TURRET = true;
    public static double PRE_AIM_DISTANCE_IN = 20.0;

    // NEW: telemetry is throttled so the loop stays short and consistent.
    public static double TELEMETRY_INTERVAL_SECONDS = 0.50;   // was 0.25

    private double activeTurretPosition = 0.50;
    private int shotCount = 0;
    private double shooterReadyStartTime = 0.0;
    private boolean shooterReadyTimeoutFired = false;
    private boolean anyPathTimeoutFired = false;
    private boolean currentPathTimedOut = false;
    private String lastTimedOutPath = "NONE";
    private String lastStallExitPath = "NONE";
    private String lastBlockedPath = "NONE";
    private int blockedCount = 0;
    private Pose blockedRefPose = null;
    private double blockedRefTime = 0.0;

    // NEW: per-path tracking state and loop timing.
    private double pathStartTime = 0.0;
    private Pose stallRefPose = null;
    private double stallRefTime = 0.0;
    private double lastLoopTime = 0.0;
    private double loopMs = 0.0;
    private double maxLoopMs = 0.0;
    private double lastTelemetryTime = -1.0;

    // NEW: shot state machine and turret/leg tracking.
    private enum ShootPhase { WAIT, RELEASE, FEED, DONE }
    private ShootPhase shootPhase = ShootPhase.DONE;
    private double shootStartTime = 0.0;
    private double releaseStartTime = 0.0;
    private double feedStartTime = 0.0;
    private double shootFeedSeconds = 0.0;
    private double turretMoveTime = 0.0;
    private boolean legDone = false;
    private double shooterFirstReadyTime = -1.0;   // when the shooter entered the stable band
    private double activeReadyTimeoutSeconds = 1.5;

    // Limelight shot state
    private boolean visionLocked = false;
    private String lastVisionShot = "NONE";
    private String lastPassThroughPath = "NONE";

    // Pose aim + regression state
    private boolean poseAimThisShot = false;   // this shot is aiming from odometry
    private double shotPresetPosition = 0.5;   // fallback if pose aim isn't possible
    private boolean upperGoalSelected = false;
    private boolean goalInitialised = false;
    private double goalX = Double.NaN;
    private double goalY = Double.NaN;
    private double distanceToGoalIn = Double.NaN;
    private String lastAimShot = "NONE";

    // Route coordinates intentionally unchanged.
    private final Pose start = poseFactory.of(52.3532, 8.0, 180.0);
    private final Pose path1End = poseFactory.of(22.4811, 50.4390, 180.0);
    private final Pose point2Start = poseFactory.of(22.4811, 47.4390, 180.0);
    private final Pose point2 = poseFactory.of(11.0, 47.9688, 170.0);
    private final Pose point3 = poseFactory.of(32.3555, 50.1482, -178.9251);
    private final Pose point4Start = poseFactory.of(32.3555, 50.1482, 180.0);
    private final Pose point4 = poseFactory.of(32.6871, 94.7054, 90.0);
    private final Pose point5Start = poseFactory.of(32.6871, 94.7054, 90.0);
    private final Pose point5 = poseFactory.of(23.7064, 130.1432, 90.0);
    private final Pose point6Start = poseFactory.of(23.7064, 130.1432, 90.0);
    private final Pose point6 = poseFactory.of(42.0, 104.1657, 90.0);
    private final Pose point7Start = poseFactory.of(42.0, 104.1657, 90.0);
    private final Pose point7 = poseFactory.of(43.0, 120.0, 90.0);
    private final Pose point8Start = poseFactory.of(43.0, 120.0, 90.0);
    private final Pose point8 = poseFactory.of(0.0, 116.1719, 0.0);

    /**
     * Description of one drive leg. The Path itself is built when the leg starts, from the
     * robot's real pose, rather than from the planned start point. That stops the follower
     * lurching to correct accumulated error at the start of every leg.
     */
    private static class Leg {
        final Pose plannedStart;   // fallback only, used if the follower has no pose
        final Pose end;
        final double speed;
        final boolean reverseTangent;
        final boolean stallExit;
        final double timeoutSeconds;
        final String name;
        final boolean passThrough;   // FAST: don't stop at the end, hand straight to the next leg
        final Pose passPoint;        // where a pass-through hands over (default: end)

        Leg(Pose plannedStart, Pose end, double speed, boolean reverseTangent,
            boolean stallExit, double timeoutSeconds, String name) {
            this(plannedStart, end, speed, reverseTangent, stallExit, timeoutSeconds, name, false);
        }

        Leg(Pose plannedStart, Pose end, double speed, boolean reverseTangent,
            boolean stallExit, double timeoutSeconds, String name, boolean passThrough) {
            this.plannedStart = plannedStart;
            this.end = end;
            this.speed = speed;
            this.reverseTangent = reverseTangent;
            this.stallExit = stallExit;
            this.timeoutSeconds = timeoutSeconds;
            this.name = name;
            this.passThrough = passThrough;
            this.passPoint = end;
        }

        /** Pass-through leg that drives toward `end` but hands over at `passPoint`. */
        Leg(Pose plannedStart, Pose end, Pose passPoint, double speed, double timeoutSeconds,
            String name) {
            this.plannedStart = plannedStart;
            this.end = end;
            this.speed = speed;
            this.reverseTangent = false;
            this.stallExit = false;
            this.timeoutSeconds = timeoutSeconds;
            this.name = name;
            this.passThrough = true;
            this.passPoint = passPoint;
        }
    }

    private Leg leg1() {
        return new Leg(start, path1End, FLOWER_APPROACH_SPEED, false, false,
                PATH1_TIMEOUT_SECONDS, "PATH 1", true);
    }

    private Leg leg2() {
        return new Leg(point2Start, point2, FLOWER_COLLECT_SPEED, false, false,
                PATH2_TIMEOUT_SECONDS, "PATH 2");
    }

    /** Moves a pose toward the HIVE (+X) by longDriveShiftIn. */
    private Pose shiftTowardHive(Pose p) {
        double shift = Double.isFinite(longDriveShiftIn) ? longDriveShiftIn : 0.0;
        return new Pose(p.x() + shift, p.y(), p.heading());
    }

    private Leg leg3() {
        return new Leg(point2, shiftTowardHive(point3), FLOWER_APPROACH_SPEED, true, false,
                PATH3_TIMEOUT_SECONDS, "PATH 3", true);
    }

    private Leg leg4() {
        Pose from = shiftTowardHive(point4Start);
        Pose handoff = shiftTowardHive(point4);
        double over = passThroughEnabled ? Math.max(0.0, path4OvershootIn) : 0.0;
        if (over <= 0.0) {
            return new Leg(from, handoff, DRIVE_SPEED, false, false,
                    PATH4_TIMEOUT_SECONDS, "PATH 4", true);
        }
        // Aim past the hand-off point along the direction of travel, same end heading
        double dx = handoff.x() - from.x();
        double dy = handoff.y() - from.y();
        double len = Math.hypot(dx, dy);
        Pose beyond = len < 1e-6 ? handoff
                : new Pose(handoff.x() + dx / len * over, handoff.y() + dy / len * over,
                handoff.heading());
        return new Leg(from, beyond, handoff, DRIVE_SPEED, PATH4_TIMEOUT_SECONDS, "PATH 4");
    }

    private Leg leg5() {
        return new Leg(shiftTowardHive(point5Start), point5, WALL_APPROACH_SPEED, false, true,
                PATH5_TIMEOUT_SECONDS, "PATH 5");
    }

    private Leg leg6() {
        return new Leg(point6Start, point6, DRIVE_SPEED, false, false,
                PATH6_TIMEOUT_SECONDS, "PATH 6", true);
    }

    private Leg leg7() {
        return new Leg(point7Start, point7, FLOWER_COLLECT_SPEED, false, false,
                PATH7_TIMEOUT_SECONDS, "PATH 7");
    }

    private Leg leg8() {
        return new Leg(point8Start, point8, PARK_SPEED, false, false,
                PATH8_TIMEOUT_SECONDS, "PATH 8");
    }

    /** Builds the path from where the robot actually is right now to the leg's end pose. */
    private Path buildPath(Leg leg) {
        Pose from = follower.pose();
        if (from == null) from = leg.plannedStart;

        if (leg.reverseTangent) {
            return line(from, leg.end).reverseTangent()
                    .with(Constants.foresightConfig.maxPathSpeed.at(leg.speed));
        }
        return line(from, leg.end).linear(from, leg.end)
                .with(Constants.foresightConfig.maxPathSpeed.at(leg.speed));
    }

    private Command followLeg(Leg leg) {
        return followLeg(leg, null);
    }

    /**
     * Follows a leg. onStart (optional) runs in the same instant the path starts, so mechanism
     * changes like deploying the flower or stopping the intake need no separate command.
     * Timeout, normal finish and stall exit are all handled here: no race/wait commands, and
     * the follower is released the moment the leg ends.
     */
    // Local adapter avoids the incompatible PedroCommands.follow(PathChain) overload.
    private Command followLeg(Leg leg, Runnable onStart) {
        return new CommandBuilder()
                .setStart(() -> {
                    if (onStart != null) onStart.run();
                    pathStartTime = getRuntime();
                    stallRefPose = null;
                    blockedRefPose = null;
                    currentPathTimedOut = false;
                    legDone = false;
                    follower.follow(buildPath(leg));
                })
                .setDone(() -> {
                    if (isPassThroughDone(leg)) {
                        // Keep moving: the next leg's follow() replaces this path.
                        legDone = true;
                        lastPassThroughPath = leg.name;
                        return true;
                    }
                    boolean finished = isLegFinished(leg);
                    if (finished) {
                        legDone = true;
                        releaseFollower();
                    }
                    return finished;
                });
    }

    /** FAST: a pass-through leg is done once it's close, without stopping. */
    private boolean isPassThroughDone(Leg leg) {
        if (!passThroughEnabled || !leg.passThrough) return false;
        if (getRuntime() - pathStartTime < MIN_PATH_RUNTIME_SECONDS) return false;
        Pose pose = follower.pose();
        if (pose == null) return false;
        Pose pass = leg.passPoint;
        double headingErrDeg = Math.abs(angleDiffDeg(
                Math.toDegrees(pass.heading()), Math.toDegrees(pose.heading())));
        double dist = Math.hypot(pass.x() - pose.x(), pass.y() - pose.y());
        if (dist <= passThroughToleranceIn && headingErrDeg <= passThroughHeadingDeg) return true;

        // Overshoot legs: crossing the hand-off point counts, even if off to one side
        if (pass != leg.end) {
            double dirX = leg.end.x() - pass.x();
            double dirY = leg.end.y() - pass.y();
            double along = (pose.x() - pass.x()) * dirX + (pose.y() - pass.y()) * dirY;
            if (along >= 0.0) return true;
        }
        return false;
    }

    private double distanceToEnd(Leg leg) {
        Pose pose = follower.pose();
        if (pose == null) return Double.MAX_VALUE;
        return Math.hypot(leg.end.x() - pose.x(), leg.end.y() - pose.y());
    }

    private boolean isLegFinished(Leg leg) {
        double now = getRuntime();

        // Timeout, measured from when this leg actually started.
        if (now - pathStartTime >= leg.timeoutSeconds) {
            currentPathTimedOut = true;
            anyPathTimeoutFired = true;
            lastTimedOutPath = leg.name;
            return true;
        }

        // Guard against a stale atParametricEnd from the previous path.
        if (now - pathStartTime < MIN_PATH_RUNTIME_SECONDS) return false;

        if (follower.atParametricEnd()) return true;

        Pose pose = follower.pose();
        if (pose == null) return false;

        double dist = distanceToEnd(leg);

        // Close enough in position and heading: don't wait for t to reach 1.
        double headingErrDeg = Math.abs(angleDiffDeg(
                Math.toDegrees(leg.end.heading()), Math.toDegrees(pose.heading())));
        if (dist <= END_POSITION_TOLERANCE_IN && headingErrDeg <= END_HEADING_TOLERANCE_DEG) {
            return true;
        }

        // Blocked anywhere on the path: no movement or turning for stallWindowSeconds.
        if (isBlocked(leg, pose, now)) return true;

        // Wall stall exit: near the target but not moving means we're against the wall.
        if (leg.stallExit && dist <= STALL_EXIT_MAX_DISTANCE_IN) {
            if (stallRefPose == null) {
                stallRefPose = pose;
                stallRefTime = now;
            } else {
                double moved = Math.hypot(pose.x() - stallRefPose.x(),
                        pose.y() - stallRefPose.y());
                if (moved > STALL_MOVE_TOLERANCE_IN) {
                    stallRefPose = pose;
                    stallRefTime = now;
                } else if (now - stallRefTime >= STALL_TIME_SECONDS) {
                    lastStallExitPath = leg.name;
                    return true;
                }
            }
        } else {
            stallRefPose = null;
        }

        return false;
    }

    /**
     * General stall check for every leg. Measures movement and turning from a reference pose;
     * whenever the robot moves or turns enough the reference restarts. If neither happens for
     * stallWindowSeconds (after the grace period), the leg is over.
     */
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

    /**
     * Stops the follower from continuing to drive toward the old end pose once a leg is over.
     * Without this the robot keeps pushing at the end point during shooting and intake.
     * If follower.stop() doesn't zero the drive in your Pedro version, swap this one line
     * for follower.breakFollowing().
     */
    private void releaseFollower() {
        follower.stop();
    }


    // ---------------------------------------------------------------------------------------
    // Alliance. The field plays exactly the same from either side, so the blue auto is this
    // class run in the same (red) coordinates with the same presets and timings. Only the
    // stored alliance, the Limelight tag set change. The blue Driver Station
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
        Outtake.DISTANCE_REGRESSION_ENABLED = useLutRegression;
        intake.stopIntake();
        outtake.stopOuttake();
        robotSettings.block();
        flower.flowerActivate(RobotConstants.flowerLiftPos);

        resetRuntimeState();
        // Turret starts at the first shot's position (27 Nov 2026), not the travel position
        setPreloadTurretPosition();
    }

    /** Turret straight to the preload position, counted as already settled. */
    private void setPreloadTurretPosition() {
        activeTurretPosition = clamp(PRELOAD_TURRET_POSITION + AUTO_TURRET_TRIM, 0.0, 1.0);
        turret.setFixedServoPosition(activeTurretPosition);
        turretMoveTime = -1000.0;   // long ago: no settle wait
    }

    @Override
    public void init_loop() {
        clearBulkCache();
        storeAlliance();
        intake.update();

        Pose currentPose = follower.pose();
        if (currentPose != null) PoseStorage.setPose(currentPose);

        telemetry.addLine("===== FLOWER " + allianceName() + " AUTO (FAST VERSION) =====");
        telemetry.addData("Start Pose", start);
        telemetry.addData("Flower Approach Speed", "%.2f", FLOWER_APPROACH_SPEED);
        telemetry.addData("Flower Collect Speed", "%.2f", FLOWER_COLLECT_SPEED);
        telemetry.addData("Wall Approach Speed", "%.2f", WALL_APPROACH_SPEED);
        telemetry.addData("First Hold", "%.2f sec", FIRST_FLOWER_INTAKE_HOLD_SECONDS);
        telemetry.addData("Second Hold", "%.2f sec", SECOND_FLOWER_INTAKE_HOLD_SECONDS);
        telemetry.addData("Far Wall Turret", "%.3f", FAR_WALL_TURRET_POSITION);
        telemetry.update();
    }

    @Override
    public void start() {
        clearBulkCache();
        storeAlliance();
        follower.setPose(start);
        PoseStorage.setPose(start);
        resetRuntimeState();
        setPreloadTurretPosition();
        stopIntakeAndBlock();
        if (spinUpAtStart) outtake.setTargetTPS(useShotTps ? shotTps(1) : RobotConstants.outtakeTps);
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
        Scheduler.execute();
        follower.update();

        Pose currentPose = follower.pose();
        if (currentPose != null) PoseStorage.setPose(currentPose);

        updateGoal(currentPose);
        updateVision();
        updateTurret(currentPose);
        intake.update();
        outtake.setHoodPosition(useShotTps ? shotHood(upcomingShot()) : RobotConstants.hoodPosition);
        // Lookup table: needs a fresh distance every loop (Outtake DISTANCE_TIMEOUT_SECONDS).
        // Off while the per-shot speeds are in use.
        outtake.setGoalDistance(useLutRegression && !useShotTps ? distanceToGoalIn : Double.NaN);
        outtake.update();

        // Throttled: building and sending ~60 telemetry lines every loop skews loop timing.
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
        if (limelightCam != null) limelightCam.close();
    }

    private Command autoRoutine() {
        // followTravel / parkAndLift set the travel turret position themselves, so no separate
        // "set travel turret" commands are needed between steps.
        return sequential(
                shootCycle(PRELOAD_TURRET_POSITION, PRELOAD_FEED_SECONDS),
                followTravel(leg1()),
                collectFlower(leg2(), FIRST_FLOWER_INTAKE_HOLD_SECONDS, Double.NaN),
                bumpFirstFlower(),
                reverseAndLiftFlower(),
                followTravel(leg4()),
                followTravelPreAim(leg5(), FAR_WALL_TURRET_POSITION),
                shootCycle(FAR_WALL_TURRET_POSITION, FLOWER_FEED_SECONDS),
                followTravel(leg6()),
                collectFlower(leg7(), SECOND_FLOWER_INTAKE_HOLD_SECONDS,
                        SECOND_FLOWER_TURRET_POSITION),
                bumpSecondFlower(),
                shootCycle(SECOND_FLOWER_TURRET_POSITION, FLOWER_FEED_SECONDS),
                parkAndLiftFlower(),
                instant(this::finishAutoMechanisms)
        );
    }

    /**
     * Flower and intake deploy at the moment the path starts (no separate deploy wait), then
     * the intake holds at the pickup. aimAfterPosition (NaN for none) pre-aims the turret during
     * the hold. The intake is left running; whatever runs next stops it.
     */
    private Command collectFlower(Leg leg, double intakeHoldSeconds, double aimAfterPosition) {
        return sequential(
                followLeg(leg, this::holdFlowerCollection),
                race(
                        waitSeconds(intakeHoldSeconds),
                        infinite(() -> {
                            holdFlowerCollection();
                            preAimTurret(aimAfterPosition);
                        })
                )
        );
    }

    /**
     * Backs straight out along the approach line by bumpBackIn, drives back in to the pickup
     * point, then holds the intake for bumpHoldSeconds. Repeated bumpRepeats times.
     */
    private Command bumpFirstFlower() {
        return bumpFlower(point2, point2Start, bumpRepeats, "BUMP 1");
    }

    /**
     * Same bump at the second flower. The turret has already been pre-aimed for the last shot
     * during the intake hold, and the bump doesn't move it, so the shot starts settled.
     */
    private Command bumpSecondFlower() {
        return bumpFlower(point7, point7Start, secondBumpRepeats, "BUMP 2");
    }

    /**
     * Backs straight out from the pickup toward approachFrom by bumpBackIn, drives back in,
     * then holds the intake. Repeated `repeats` times; 0 does nothing.
     */
    private Command bumpFlower(Pose pickup, Pose approachFrom, int repeatCount, String name) {
        int repeats = Math.max(0, repeatCount);
        if (repeats == 0) return instant(() -> { });

        Pose back = bumpBackPose(pickup, approachFrom);
        Command[] steps = new Command[repeats * 3];
        for (int i = 0; i < repeats; i++) {
            // Intake re-commanded every loop during both moves, not just at the start
            steps[i * 3] = race(
                    followLeg(new Leg(pickup, back, bumpSpeed, false, false,
                            bumpTimeoutSeconds, name + " BACK"), this::holdFlowerCollection),
                    infinite(this::holdFlowerCollection)
            );
            steps[i * 3 + 1] = race(
                    followLeg(new Leg(back, pickup, bumpSpeed, false, false,
                            bumpTimeoutSeconds, name + " IN"), this::holdFlowerCollection),
                    infinite(this::holdFlowerCollection)
            );
            steps[i * 3 + 2] = race(
                    waitSeconds(bumpHoldSeconds),
                    infinite(this::holdFlowerCollection)
            );
        }
        return sequential(steps);
    }

    /** Pickup point moved back toward where the robot came from (approachFrom). */
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
        robotSettings.block();
        outtake.stopFeeder();
        flower.flowerActivate(RobotConstants.flowerDropPos);
        intake.intake(
                RobotConstants.frontIntakeSpeed,
                RobotConstants.backIntakeSpeed
        );
    }

    private void preAimTurret(double position) {
        if (PRE_AIM_TURRET && !Double.isNaN(position)) setFixedTurretPosition(position);
    }

    // Backs out of the first flower with the intake still running (the next travel leg stops it),
    // so any remaining pollen is pulled out. The turret is already at travel position.
    private Command reverseAndLiftFlower() {
        return parallel(
                followLeg(leg3(), this::holdFlowerCollection),
                sequential(
                        waitSeconds(FLOWER_REVERSE_CLEAR_SECONDS),
                        instant(() -> flower.flowerActivate(
                                RobotConstants.flowerLiftPos
                        ))
                )
        );
    }

    private Command followTravel(Leg leg) {
        return followLeg(leg, () -> {
            stopIntakeAndBlock();
            setTravelTurretPosition();
        });
    }

    /** Travel leg that moves the turret to aimPosition when close to the end of the leg. */
    private Command followTravelPreAim(Leg leg, double aimPosition) {
        return sequential(
                instant(() -> {
                    stopIntakeAndBlock();
                    setTravelTurretPosition();
                    legDone = false;
                }),
                parallel(
                        followLeg(leg),
                        sequential(
                                waitUntil(() -> legDone
                                        || (PRE_AIM_TURRET
                                        && distanceToEnd(leg) <= PRE_AIM_DISTANCE_IN)),
                                instant(() -> preAimTurret(aimPosition))
                        )
                )
        );
    }

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
        shotCount++;
        shotPresetPosition = turretPosition;
        // Each shot measures its own Limelight correction: one carried over from a shot at a
        // different spot (held up to visionHoldMs + visionFadeMs) would point this one wrong.
        turret.clearVisionCorrection();
        poseAimThisShot = usePoseAim && canPoseAim(follower.pose());
        if (poseAimThisShot) {
            // The turret is about to move to the aimed angle: time the settle from now.
            turretMoveTime = getRuntime();
        } else {
            setFixedTurretPosition(turretPosition);
        }
        outtake.setTargetTPS(useShotTps ? shotTps(shotCount) : RobotConstants.outtakeTps);
        activeReadyTimeoutSeconds = shotCount == 1
                ? preloadReadyTimeoutSeconds : SHOOT_READY_TIMEOUT_SECONDS;
        startShooterReadyWait();
        stopIntakeAndBlock();
        shootStartTime = getRuntime();
        shooterFirstReadyTime = -1.0;
        shootFeedSeconds = feedSeconds;
        shootPhase = ShootPhase.WAIT;
        visionLocked = false;
    }

    /**
     * One state machine instead of several chained wait commands:
     *   WAIT    turret settling and shooter spinning up run together; blocker closed.
     *   RELEASE blocker opens (starting before the shooter is fully at speed); no feeding yet.
     *   FEED    feeds once the blocker has had time to open AND the shooter is ready/timed out.
     *   DONE    everything stopped.
     */
    private void runShotStateMachine() {
        double now = getRuntime();

        // Keep the stable-band clock running from the start of the shot, so the settle time
        // overlaps with the turret and gate where it can
        if (shootPhase != ShootPhase.DONE) shooterSpooled(now);

        if (shootPhase == ShootPhase.WAIT) {
            boolean settleDone = (firstShotSkipSettle && shotCount == 1)
                    || now - turretMoveTime >= TURRET_SETTLE_SECONDS;
            boolean turretSettled = settleDone
                    && (poseAimThisShot ? poseAimDone(now) : visionAimDone(now));
            if (turretSettled
                    && (isShooterAtFraction(PRE_RELEASE_TPS_FRACTION)
                    || shooterReadyOrTimedOut())) {
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
            // Head straight for the next shot's speed while driving there
            if (useShotTps) outtake.setTargetTPS(shotTps(shotCount + 1));
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
        intake.intake(
                RobotConstants.frontFeedSpeedAuto,
                RobotConstants.backFeedSpeedAuto
        );
        outtake.setFeederOn(true);
    }

    private void stopIntakeAndBlock() {
        intake.stopIntake();
        outtake.stopFeeder();
        robotSettings.block();
    }

    /** The shot being taken now, or the next one if between shots (1-based). */
    private int upcomingShot() {
        return shootPhase == ShootPhase.DONE ? shotCount + 1 : shotCount;
    }

    /** Flywheel speed for shot n (1 = preload, 2 = far wall, 3 = second flower). */
    private double shotTps(int n) {
        switch (n) {
            case 1: return shot1Tps;
            case 2: return shot2Tps;
            case 3: return shot3Tps;
            default: return RobotConstants.outtakeTps;
        }
    }

    /** Hood for shot n; NaN in the setting means RobotConstants.hoodPosition. */
    private double shotHood(int n) {
        double hood = n == 1 ? shot1Hood : n == 2 ? shot2Hood : n == 3 ? shot3Hood : Double.NaN;
        return Double.isFinite(hood) ? hood : RobotConstants.hoodPosition;
    }

    private void startShooterReadyWait() {
        shooterReadyStartTime = getRuntime();
        shooterReadyTimeoutFired = false;
    }

    /**
     * True once the shooter has stayed inside the stable band (target +/- shooterStableBand-
     * Fraction) for spoolUpExtraSeconds without leaving it. Overspeed counts as NOT stable,
     * so a flywheel still overshooting after spin-up waits instead of firing long. If it never
     * settles, the ready timeout fires the shot anyway so the auto can't stall.
     */
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
        if (isShooterAtSpeed()) return true;

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

    private boolean isShooterAtSpeed() {
        return isShooterAtFraction(TPS_READY_FRACTION);
    }

    private Command parkAndLiftFlower() {
        return parallel(
                followLeg(leg8(), () -> {
                    stopIntakeAndBlock();
                    setTravelTurretPosition();
                }),
                sequential(
                        waitSeconds(FLOWER_LIFT_DELAY_SECONDS),
                        instant(() -> flower.flowerActivate(
                                RobotConstants.flowerLiftPos
                        ))
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

    // Records when the turret was last told to move, so settle time is counted from the real
    // move (including pre-aims) rather than from the start of each shot.
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

    // ---------------------------------------------------------------------------------------
    // Limelight
    // ---------------------------------------------------------------------------------------

    private void initLimelight() {
        visionAvailable = false;
        if (!useVisionAim) return;
        try {
            limelightCam = new LimelightCam(hardwareMap);
            limelightCam.init();
            visionAvailable = true;
        } catch (RuntimeException e) {
            limelightCam = null;   // presets only
        }
    }

    /** Hands the red goal cluster's bearing (if visible) to the turret. */
    private void updateVision() {
        if (!visionAvailable) {
            turret.clearVisionMeasurement();
            return;
        }
        try {
            limelightCam.update();
            LimelightCam.HiveDetection d = limelightCam.getAimDetection(aimAlliance());
            if (d != null) turret.setVisionMeasurement(true, d.bearing, limelightCam.getFrameId());
            else turret.clearVisionMeasurement();
        } catch (RuntimeException e) {
            turret.clearVisionMeasurement();
        }
    }

    /** Vision may steer during WAIT/RELEASE once the turret has reached its preset. */
    private boolean visionWindowOpen() {
        return useVisionAim && visionAvailable
                && (shootPhase == ShootPhase.WAIT || shootPhase == ShootPhase.RELEASE)
                && getRuntime() - turretMoveTime >= TURRET_SETTLE_SECONDS;
    }

    private void updateTurret(Pose pose) {
        boolean aiming = shootPhase == ShootPhase.WAIT || shootPhase == ShootPhase.RELEASE;

        // Pose aim (odometry + Limelight correction) during the shot's WAIT/RELEASE
        if (aiming && poseAimThisShot) {
            String why = "pose lost";
            if (canPoseAim(pose)) {
                turret.aimTurret(pose);
                boolean visionConfirmed = turret.isVisionTargetVisible()
                        && "LOCKED".equals(Turret.lastVisionStatus);
                double deviation = Math.abs(Turret.lastFinalAngleDeg
                        - presetAngleDeg(shotPresetPosition));
                if (!turret.isTargetReachable()) {
                    why = "goal outside turret arc";
                } else if (!visionConfirmed && deviation > poseAimMaxDeviationDeg) {
                    why = String.format("pose aim %.0f deg off preset, no camera lock", deviation);
                } else {
                    return;
                }
            }
            // Fall back to the preset (+ Limelight fine aim below)
            poseAimThisShot = false;
            setFixedTurretPosition(shotPresetPosition);
            lastAimShot = String.format("shot %d: PRESET (%s) at (%.1f, %.1f, %.0f deg)",
                    shotCount, why,
                    pose == null ? Double.NaN : pose.x(),
                    pose == null ? Double.NaN : pose.y(),
                    pose == null ? Double.NaN : Math.toDegrees(pose.heading()));
        }

        if (visionWindowOpen() && (visionLocked || turret.isVisionTargetVisible())) {
            visionLocked = true;
            turret.aimVisionOnly();
        } else {
            turret.setFixedServoPosition(activeTurretPosition);
        }
    }

    // ---------------------------------------------------------------------------------------
    // Goal, distance and pose aim (same goal coordinates as CombinedTeleOp)
    // ---------------------------------------------------------------------------------------

    private static boolean validPose(Pose pose) {
        return pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading());
    }

    /**
     * Picks the red upper/lower goal from the robot's Y (with the TeleOp's hysteresis), hands
     * it to the turret, and works out the turret-to-goal distance for the lookup table.
     */
    private void updateGoal(Pose pose) {
        distanceToGoalIn = Double.NaN;
        if (!validPose(pose)) return;

        double y = pose.y();
        double hyst = Math.max(0.0, CombinedTeleOp.GOAL_HYSTERESIS_IN);
        if (!goalInitialised) upperGoalSelected = y >= CombinedTeleOp.GOAL_SWITCH_Y_IN;
        else if (upperGoalSelected && y < CombinedTeleOp.GOAL_SWITCH_Y_IN - hyst) upperGoalSelected = false;
        else if (!upperGoalSelected && y >= CombinedTeleOp.GOAL_SWITCH_Y_IN + hyst) upperGoalSelected = true;
        goalInitialised = true;

        double gx = upperGoalSelected ? CombinedTeleOp.RED_UPPER_GOAL_X : CombinedTeleOp.RED_LOWER_GOAL_X;
        double gy = upperGoalSelected ? CombinedTeleOp.RED_UPPER_GOAL_Y : CombinedTeleOp.RED_LOWER_GOAL_Y;
        if (!Double.isFinite(gx) || !Double.isFinite(gy)) return;

        turret.setGoalHeight(upperGoalSelected ? Turret.GoalHeight.HIGH : Turret.GoalHeight.LOW);
        if (Double.compare(gx, goalX) != 0 || Double.compare(gy, goalY) != 0) {
            goalX = gx;
            goalY = gy;
            turret.setTargetPose(new Pose(goalX, goalY, 0.0));
            turret.realignToSelectedGoal();
        }

        // Distance from the turret pivot, as in the TeleOp
        double h = pose.heading();
        double pivotX = pose.x() + Turret.TURRET_OFFSET_X_IN * Math.cos(h)
                - Turret.TURRET_OFFSET_Y_IN * Math.sin(h);
        double pivotY = pose.y() + Turret.TURRET_OFFSET_X_IN * Math.sin(h)
                + Turret.TURRET_OFFSET_Y_IN * Math.cos(h);
        distanceToGoalIn = Math.hypot(goalX - pivotX, goalY - pivotY);
    }

    private boolean canPoseAim(Pose pose) {
        return validPose(pose) && Double.isFinite(goalX) && Double.isFinite(goalY);
    }

    /** Turret angle (deg, Turret's convention) that a preset servo position points at. */
    private static double presetAngleDeg(double presetPosition) {
        double pos = presetPosition + AUTO_TURRET_TRIM;
        double sign = Turret.REVERSE_AIM_MAPPING ? -1.0 : 1.0;
        return sign * (pos - Turret.getCalibratedCentrePosition()) * Turret.SERVO_RANGE_DEG;
    }

    /**
     * Pose-aimed shot may fire once the turret has settled on its aimed angle and, if the
     * Limelight can see the goal, has applied at least one correction. Capped at
     * poseAimMaxWaitSeconds so a bad view can't stall the auto.
     */
    private boolean poseAimDone(double now) {
        if (now - shootStartTime >= poseAimMaxWaitSeconds) return true;
        if (!turret.isTurretSettled()) return false;
        if (useVisionAim && visionAvailable && turret.isVisionTargetVisible()) {
            return "LOCKED".equals(Turret.lastVisionStatus);
        }
        return true;
    }

    private boolean visionAimDone(double now) {
        if (!useVisionAim || !visionAvailable || !visionLocked) return true;
        if (turret.isVisionOnTarget()) return true;
        return now - turretMoveTime - TURRET_SETTLE_SECONDS >= visionAimTimeoutSeconds;
    }

    /** At feed start, hold the turret exactly where vision put it. */
    private void freezeVisionAim() {
        if (poseAimThisShot) {
            activeTurretPosition = Turret.lastServoPosition;   // no turretMoveTime change
            Pose p = follower.pose();
            lastAimShot = String.format(
                    "shot %d: POSE %.3f (preset %.3f), vision %+.1f deg, %.0f in, at (%.1f, %.1f, %.0f deg)",
                    shotCount, activeTurretPosition, shotPresetPosition,
                    Turret.lastVisionCorrectionDeg, distanceToGoalIn,
                    p == null ? Double.NaN : p.x(), p == null ? Double.NaN : p.y(),
                    p == null ? Double.NaN : Math.toDegrees(p.heading()));
            poseAimThisShot = false;
            visionLocked = false;
            return;
        }
        if (visionLocked) {
            activeTurretPosition = Turret.lastServoPosition;   // no turretMoveTime change
            lastVisionShot = String.format("shot %d: %.3f (%s)", shotCount,
                    activeTurretPosition, turret.isVisionOnTarget() ? "on target" : "timeout");
        } else if (useVisionAim && visionAvailable) {
            lastVisionShot = String.format("shot %d: preset, no tags", shotCount);
        }
        visionLocked = false;
    }

    private Command waitSeconds(double seconds) {
        return waitMs(Math.round(Math.max(0.0, seconds) * 1000.0));
    }

    private void resetRuntimeState() {
        activeTurretPosition = TRAVEL_TURRET_POSITION;
        shotCount = 0;
        shooterReadyStartTime = 0.0;
        shooterReadyTimeoutFired = false;
        anyPathTimeoutFired = false;
        currentPathTimedOut = false;
        lastTimedOutPath = "NONE";
        lastStallExitPath = "NONE";
        lastBlockedPath = "NONE";
        blockedCount = 0;
        blockedRefPose = null;
        blockedRefTime = 0.0;
        pathStartTime = 0.0;
        stallRefPose = null;
        stallRefTime = 0.0;
        lastLoopTime = 0.0;
        loopMs = 0.0;
        maxLoopMs = 0.0;
        lastTelemetryTime = -1.0;
        shootPhase = ShootPhase.DONE;
        shootStartTime = 0.0;
        releaseStartTime = 0.0;
        feedStartTime = 0.0;
        shootFeedSeconds = 0.0;
        turretMoveTime = 0.0;
        legDone = false;
        visionLocked = false;
        lastVisionShot = "NONE";
        lastPassThroughPath = "NONE";
        poseAimThisShot = false;
        goalInitialised = false;
        goalX = goalY = distanceToGoalIn = Double.NaN;
        lastAimShot = "NONE";
    }

    private void displayTelemetry(Pose currentPose) {
        telemetry.addLine("===== FLOWER " + allianceName() + " AUTO (FAST VERSION) =====");

        if (currentPose != null) {
            telemetry.addData(
                    "Pose",
                    "(%.1f, %.1f, %.1f deg)",
                    currentPose.x(),
                    currentPose.y(),
                    Math.toDegrees(currentPose.heading())
            );
        }

        telemetry.addLine("===== AUTO TIME =====");
        telemetry.addData("Runtime", "%.2f sec", getRuntime());
        telemetry.addData("Time Remaining", "%.2f sec", Math.max(0.0, 30.0 - getRuntime()));
        telemetry.addData("Loop", "%.1f ms (max %.1f)", loopMs, maxLoopMs);
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Shot Count", shotCount);
        telemetry.addData("Shot Phase", shootPhase);

        telemetry.addLine("===== PATH STATUS =====");
        telemetry.addData("Any Timeout", anyPathTimeoutFired);
        telemetry.addData("Last Timeout", lastTimedOutPath);
        telemetry.addData("Last Stall Exit", lastStallExitPath);
        telemetry.addData("Blocked", "%d | last: %s", blockedCount, lastBlockedPath);
        telemetry.addData("Last Pass-Through", lastPassThroughPath);
        telemetry.addData("Current Path Timed Out", currentPathTimedOut);

        telemetry.addLine("===== SPEED SETTINGS =====");
        telemetry.addData("Drive", "%.2f", DRIVE_SPEED);
        telemetry.addData("Flower Approach", "%.2f", FLOWER_APPROACH_SPEED);
        telemetry.addData("Flower Collect", "%.2f", FLOWER_COLLECT_SPEED);
        telemetry.addData("Wall Approach", "%.2f", WALL_APPROACH_SPEED);

        telemetry.addLine("===== FLOWER =====");
        telemetry.addData("First Hold", "%.2f sec", FIRST_FLOWER_INTAKE_HOLD_SECONDS);
        telemetry.addData("Second Hold", "%.2f sec", SECOND_FLOWER_INTAKE_HOLD_SECONDS);
        telemetry.addData("Reverse Clear", "%.2f sec", FLOWER_REVERSE_CLEAR_SECONDS);
        telemetry.addData(
                "First Pickup",
                "(%.1f, %.1f, %.1f deg)",
                point2.x(), point2.y(), Math.toDegrees(point2.heading())
        );
        telemetry.addData(
                "Second Pickup",
                "(%.1f, %.1f, %.1f deg)",
                point7.x(), point7.y(), Math.toDegrees(point7.heading())
        );

        telemetry.addLine("===== TURRET =====");
        telemetry.addData("Active Position", "%.3f", activeTurretPosition);
        telemetry.addData("Far Wall Position", "%.3f", FAR_WALL_TURRET_POSITION);
        telemetry.addData("Servo Position", "%.3f", Turret.lastServoPosition);
        telemetry.addData("Fixed Mode", Turret.lastFixedPositionMode);
        telemetry.addData("Goal", "RED %s (%.1f, %.1f) | %.1f in",
                upperGoalSelected ? "UPPER" : "LOWER", goalX, goalY, distanceToGoalIn);
        telemetry.addData("Aim", "%s | last: %s",
                poseAimThisShot ? "POSE AIM" : "PRESET", lastAimShot);
        if (visionAvailable) {
            telemetry.addData("Vision", "%s | bearing %.1f | correction %.1f | %s",
                    Turret.lastVisionStatus, Turret.lastVisionBearingDeg,
                    Turret.lastVisionCorrectionDeg, lastVisionShot);
        } else {
            telemetry.addData("Vision", useVisionAim ? "LIMELIGHT NOT FOUND - presets only" : "OFF");
        }

        telemetry.addLine("===== SHOOTER =====");
        telemetry.addData(
                "Shooter TPS",
                "%.0f / %.0f",
                outtake.getCurrentTPS(),
                outtake.getTargetTPS()
        );
        telemetry.addData("Regression", useShotTps
                ? String.format("PER-SHOT TPS: shot %d -> %.0f (hood %.3f)",
                upcomingShot(), shotTps(upcomingShot()), shotHood(upcomingShot()))
                : String.format("%s (%s)", outtake.isUsingDistanceRegression() ? "LUT" : "FIXED TPS",
                outtake.getRegressionStatus()));
        telemetry.addData("Shooter Ready", isShooterAtSpeed());
        telemetry.addData("Ready Timeout", shooterReadyTimeoutFired);
        telemetry.addData("Spool", shooterFirstReadyTime < 0.0
                ? String.format("not stable (%.0f / %.0f, band +/-%.0f%%)",
                outtake.getCurrentTPS(), outtake.getTargetTPS(), shooterStableBandFraction * 100.0)
                : String.format("stable %.2f s (need %.2f)",
                getRuntime() - shooterFirstReadyTime, spoolUpExtraSeconds));
        // Feeder is open loop now (no encoder), so show its power, not TPS
        telemetry.addData("Feeder Power", "%.2f", outtake.getFeederCommandedPower());
        telemetry.addData("Feed Link", "scale %.2f | %.0f TPS",
                outtake.getFeedLinkScale(), outtake.getFeedLinkTPS());
        telemetry.addData("Hood Base", "%.3f", outtake.getHoodBase());
        telemetry.addData("Hood Compensation", "%.4f", outtake.getHoodCompensation());
        telemetry.addData("Hood Command", "%.3f", outtake.getHoodPosition());

        telemetry.addLine("===== INTAKE =====");
        telemetry.addData("Front Power", "%.2f", intake.getFrontMotorPower());
        telemetry.addData("Back Power", "%.2f", intake.getBackMotorPower());
        telemetry.update();
    }

    private void configureBulkCaching() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    private void clearBulkCache() {
        if (allHubs == null) return;
        for (LynxModule hub : allHubs) hub.clearBulkCache();
    }

    private double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }
}