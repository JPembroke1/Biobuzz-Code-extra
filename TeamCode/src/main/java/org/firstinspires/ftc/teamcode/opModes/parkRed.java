package org.firstinspires.ftc.teamcode.opModes;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
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
import org.firstinspires.ftc.teamcode.opModes.subClasses.Outtake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.subClasses.RobotConstants;
import org.firstinspires.ftc.teamcode.opModes.subClasses.RobotSettings;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

/**
 * Park-only auto (final, 28 Nov 2026). Drives one short leg from the start pose to the park
 * pose, with the mechanisms held safe, then stops.
 *
 * The field plays exactly the same from either side, so parkBlue runs this class unchanged
 * (same coordinates and settings); only the alliance stored for the TeleOp is blue.
 *
 * The path ends on: reaching the end pose, the robot being blocked (stall detection, e.g.
 * a partner robot in the way), or parkTimeoutSeconds.
 */
@Configurable
@Autonomous(name = "parkRed", group = "Main")
public class parkRed extends OpMode {

    // Optional wait before driving, e.g. to let a partner clear the way
    public static double startDelaySeconds = 0.0;

    public static double parkSpeed = 0.20;
    public static double parkTimeoutSeconds = 4.0;

    public static double travelTurretPosition = 0.50;

    public static double minPathRuntimeSeconds = 0.05;
    public static double endPositionToleranceIn = 1.5;
    public static double endHeadingToleranceDeg = 10.0;

    // Stall detection (same as the other autos): blocked for stallWindowSeconds ends the path
    public static boolean stallDetectEnabled = true;
    public static double stallGraceSeconds = 0.40;
    public static double stallWindowSeconds = 0.35;
    public static double stallMinMoveIn = 0.5;
    public static double stallMinTurnDeg = 3.0;

    public static double telemetryIntervalSeconds = 0.25;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(8.0, 76.5, 90.0);
    private final Pose parkPose = poseFactory.of(11.0, 89.0, 90.0);

    private List<LynxModule> allHubs;
    private Follower follower;
    private Outtake outtake;
    private Intake intake;
    private Turret turret;
    private Flower flower;
    private RobotSettings robotSettings;

    private double pathStartTime = 0.0;
    private Pose blockedRefPose = null;
    private double blockedRefTime = 0.0;
    private String parkResult = "NOT STARTED";
    private boolean autoFinished = false;
    private double lastTelemetryTime = -1.0;

    // ---------------------------------------------------------------------------------------
    // Alliance: the blue entry (parkBlue) overrides isBlueAlliance()
    // ---------------------------------------------------------------------------------------

    protected boolean isBlueAlliance() {
        return false;
    }

    private void storeAlliance() {
        if (isBlueAlliance()) PoseStorage.setBlue();
        else PoseStorage.setRed();
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
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
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

        Turret.velocityCompensationActive = false;
        holdMechanismsSafe();
    }

    @Override
    public void init_loop() {
        clearBulkCache();
        storeAlliance();
        intake.update();

        Pose pose = follower.pose();
        if (pose != null) PoseStorage.setPose(pose);

        telemetry.addLine("===== PARK " + allianceName() + " =====");
        telemetry.addData("Start", "(%.1f, %.1f, %.0f deg)", start.x(), start.y(),
                Math.toDegrees(start.heading()));
        telemetry.addData("Park", "(%.1f, %.1f, %.0f deg)", parkPose.x(), parkPose.y(),
                Math.toDegrees(parkPose.heading()));
        telemetry.addData("Speed / Delay", "%.2f / %.1f s", parkSpeed, startDelaySeconds);
        telemetry.update();
    }

    @Override
    public void start() {
        clearBulkCache();
        storeAlliance();
        follower.setPose(start);
        PoseStorage.setPose(start);
        autoFinished = false;
        parkResult = "WAITING";
        lastTelemetryTime = -1.0;
        holdMechanismsSafe();
        schedule(sequential(
                waitSecondsCommand(startDelaySeconds),
                parkCommand(),
                instant(this::finishAuto)
        ));
    }

    @Override
    public void loop() {
        double now = getRuntime();
        clearBulkCache();
        Scheduler.execute();
        follower.update();

        Pose pose = follower.pose();
        if (pose != null) PoseStorage.setPose(pose);

        turret.setFixedServoPosition(travelTurretPosition);
        intake.update();
        outtake.setHoodPosition(RobotConstants.hoodPosition);
        outtake.update();

        if (lastTelemetryTime < 0.0 || now - lastTelemetryTime >= telemetryIntervalSeconds) {
            lastTelemetryTime = now;
            telemetry.addLine("===== PARK " + allianceName() + " =====");
            if (pose != null) {
                telemetry.addData("Pose", "(%.1f, %.1f, %.1f deg)", pose.x(), pose.y(),
                        Math.toDegrees(pose.heading()));
            }
            telemetry.addData("Runtime", "%.2f s", now);
            telemetry.addData("Park", parkResult);
            telemetry.addData("Finished", autoFinished);
            telemetry.update();
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
        if (allHubs != null) {
            for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    // ---------------------------------------------------------------------------------------
    // Park
    // ---------------------------------------------------------------------------------------

    private Command parkCommand() {
        return new CommandBuilder()
                .setStart(() -> {
                    holdMechanismsSafe();
                    pathStartTime = getRuntime();
                    blockedRefPose = null;
                    parkResult = "DRIVING";
                    follower.follow(buildPath());
                })
                .setDone(() -> {
                    boolean done = isParkFinished();
                    if (done) follower.stop();
                    return done;
                });
    }

    private Path buildPath() {
        Pose from = follower.pose();
        if (from == null) from = start;
        return line(from, parkPose).linear(from, parkPose)
                .with(Constants.foresightConfig.maxPathSpeed.at(parkSpeed));
    }

    private boolean isParkFinished() {
        double now = getRuntime();
        double elapsed = now - pathStartTime;

        if (elapsed >= parkTimeoutSeconds) {
            parkResult = String.format("TIMED OUT at %.1f s", elapsed);
            return true;
        }
        if (elapsed < minPathRuntimeSeconds) return false;

        Pose pose = follower.pose();
        if (follower.atParametricEnd()) {
            parkResult = String.format("ARRIVED at %.1f s", elapsed);
            return true;
        }
        if (pose == null) return false;

        double dist = Math.hypot(parkPose.x() - pose.x(), parkPose.y() - pose.y());
        double headingErr = Math.abs(angleDiffDeg(Math.toDegrees(parkPose.heading()),
                Math.toDegrees(pose.heading())));
        if (dist <= endPositionToleranceIn && headingErr <= endHeadingToleranceDeg) {
            parkResult = String.format("ARRIVED at %.1f s", elapsed);
            return true;
        }

        if (isBlocked(pose, now)) {
            parkResult = String.format("BLOCKED at %.1f s, %.1f in short", elapsed, dist);
            return true;
        }
        return false;
    }

    private boolean isBlocked(Pose pose, double now) {
        if (!stallDetectEnabled) return false;
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
        return now - blockedRefTime >= stallWindowSeconds;
    }

    // ---------------------------------------------------------------------------------------
    // Mechanisms
    // ---------------------------------------------------------------------------------------

    /** Shooter off, intake and feeder stopped, blocker closed, flower up, turret at travel. */
    private void holdMechanismsSafe() {
        intake.stopIntake();
        outtake.stopOuttake();
        robotSettings.block();
        flower.flowerActivate(RobotConstants.flowerLiftPos);
        turret.setFixedServoPosition(travelTurretPosition);
    }

    private void finishAuto() {
        autoFinished = true;
        follower.stop();
        holdMechanismsSafe();
    }

    // ---------------------------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------------------------

    private Command waitSecondsCommand(double seconds) {
        final double wait = Math.max(0.0, seconds);
        final double[] startTime = new double[1];
        return new CommandBuilder()
                .setStart(() -> startTime[0] = getRuntime())
                .setDone(() -> getRuntime() - startTime[0] >= wait);
    }

    private void clearBulkCache() {
        if (allHubs == null) return;
        for (LynxModule hub : allHubs) hub.clearBulkCache();
    }

    private static double angleDiffDeg(double a, double b) {
        double d = (a - b) % 360.0;
        if (d > 180.0) d -= 360.0;
        if (d < -180.0) d += 360.0;
        return d;
    }
}