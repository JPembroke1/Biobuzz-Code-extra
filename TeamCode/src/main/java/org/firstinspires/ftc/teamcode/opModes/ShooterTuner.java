package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Flower;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Intake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Outtake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.RobotConstants;
import org.firstinspires.ftc.teamcode.opModes.subClasses.RobotSettings;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Shooter distance tuner.
 *
 * Place the robot in the RED UPPER corner exactly as for the CombinedTeleOp
 * corner reset (G2 D-pad up), then push it by hand
 * to each tuning spot (Pinpoint tracks it). Distance to the goal is shown
 * live and logged with every shot. The flower arm is held up the whole time.
 *
 * gamepad1:
 *   Y              shooter on / off
 *   RB / LB        target TPS + / -
 *   D-pad up/down  hood + / -
 *   D-pad L/R      turret - / +
 *   A              fire one shot (waits for READY_TOLERANCE_TPS)
 *   X / B          mark last shot HIT / MISS (writes it to the log)
 *   Back           toggle MANUAL / TABLE mode
 *   L stick btn    copy the table prediction into manual TPS/hood
 *   R stick btn    reset pose to the start pose
 *
 * Log file: /sdcard/FIRST/shot_log.csv (pull with Android Studio Device
 * Explorer or adb pull).
 */
@Configurable
@TeleOp(name = "Shooter Tuner", group = "Test")
public class ShooterTuner extends OpMode {

    /*
     * Goal point used for distance. Defaults to the UPPER goals from
     * CombinedTeleOp, so the table matches what TeleOp feeds Outtake.
     */
    public static boolean RED_ALLIANCE = true;
    public static double RED_GOAL_X = CombinedTeleOp.RED_UPPER_GOAL_X;
    public static double RED_GOAL_Y = CombinedTeleOp.RED_UPPER_GOAL_Y;
    public static double BLUE_GOAL_X = CombinedTeleOp.BLUE_UPPER_GOAL_X;
    public static double BLUE_GOAL_Y = CombinedTeleOp.BLUE_UPPER_GOAL_Y;

    /*
     * Measure distance from the turret pivot (same as CombinedTeleOp).
     * Turn off only if you want robot-centre distance.
     */
    public static boolean DISTANCE_FROM_TURRET_PIVOT = true;

    /*
     * Start pose. When true, uses the CombinedTeleOp RED UPPER corner reset
     * (robot in the (0, 144) corner, same size and heading values), so both
     * OpModes agree. Set false to use START_X/Y/HEADING_DEG instead.
     */
    public static boolean USE_TELEOP_RED_UPPER_RESET = true;
    public static double START_X = 72.0;
    public static double START_Y = 72.0;
    public static double START_HEADING_DEG = 90.0;

    // Adjustment steps
    public static double TPS_STEP = 25.0;
    public static double HOOD_STEP = 0.01;
    public static double TURRET_STEP = 0.005;

    // Starting values
    public static double START_TPS = 2000.0;
    public static double START_HOOD = 0.60;
    public static double START_TURRET = 0.30;

    // Shot timing
    public static double READY_TIMEOUT_SECONDS = 1.5;
    public static double RELEASE_SECONDS = 0.30;
    public static double FEED_SECONDS = 0.50;

    public static String LOG_FILE_NAME = "shot_log.csv";

    private enum ShotState { IDLE, WAIT_READY, RELEASE, FEED }

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private Follower follower;
    private Outtake outtake;
    private Intake intake;
    private Turret turret;
    private Flower flower;
    private RobotSettings robotSettings;

    private boolean shooterOn = false;
    private boolean tableMode = false;
    private boolean savedRegressionFlag;

    private double manualTPS;
    private double manualHood;
    private double turretPos;

    private ShotState shotState = ShotState.IDLE;
    private double stateStartTime = 0.0;
    private boolean readyTimedOut = false;

    // Data captured for the most recent shot, waiting for HIT/MISS
    private boolean pendingShot = false;
    private int shotNumber = 0;
    private String pendingLine = "";
    private double feedTPS = 0.0;
    private double feedMinTPS = 0.0;

    private final List<String> recentLines = new ArrayList<>();
    private String logStatus = "no shots yet";

    private final boolean[] prev = new boolean[12];

    @Override
    public void init() {
        savedRegressionFlag = Outtake.DISTANCE_REGRESSION_ENABLED;
        Outtake.DISTANCE_REGRESSION_ENABLED = false;

        follower = Constants.create(hardwareMap);
        resetPose();

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

        // Flower arm up and out of the way
        flower.flowerActivate(RobotConstants.flowerLiftPos);

        manualTPS = START_TPS;
        manualHood = START_HOOD;
        turretPos = START_TURRET;

        holdIdle();
    }

    @Override
    public void init_loop() {
        follower.update();
        flower.flowerActivate(RobotConstants.flowerLiftPos);
        telemetry.addLine("Shooter Tuner ready. Robot at the start pose?");
        telemetry.addData("Start pose", "(%.2f, %.2f, %.0f) %s",
                startX(), startY(), startHeadingDeg(),
                USE_TELEOP_RED_UPPER_RESET ? "RED UPPER corner" : "custom");
        if (!goalSet()) {
            telemetry.addLine("!! Set RED/BLUE_GOAL_X/Y in Panels !!");
        }
        telemetry.update();
    }

    /*
     * Pinpoint can zero its own position while it calibrates during init,
     * wiping the pose set in init(). Set it again at Start.
     */
    @Override
    public void start() {
        resetPose();
        follower.update();
    }

    @Override
    public void loop() {
        double now = getRuntime();

        follower.update();
        Pose pose = follower.pose();
        double distance = distanceToGoal(pose);

        handleButtons(distance);

        // Shooter targets
        Outtake.DISTANCE_REGRESSION_ENABLED = tableMode;
        if (shooterOn) {
            outtake.setTargetTPS(manualTPS); // fallback / manual value
            if (Double.isFinite(distance)) outtake.setGoalDistance(distance);
        } else {
            outtake.setTargetTPS(0.0);
        }
        outtake.setHoodPosition(manualHood);

        runShot(now);

        turret.setFixedServoPosition(turretPos);
        flower.flowerActivate(RobotConstants.flowerLiftPos);
        intake.update();
        outtake.update();

        showTelemetry(pose, distance);
    }

    @Override
    public void stop() {
        if (pendingShot) writePending("?");
        if (outtake != null) outtake.stopOuttake();
        if (intake != null) intake.stopIntake();
        if (robotSettings != null) robotSettings.block();
        Outtake.DISTANCE_REGRESSION_ENABLED = savedRegressionFlag;
    }

    // ---------------------------------------------------------------------
    // Controls
    // ---------------------------------------------------------------------

    private void handleButtons(double distance) {
        if (pressed(0, gamepad1.y)) shooterOn = !shooterOn;

        if (pressed(1, gamepad1.right_bumper)) manualTPS += TPS_STEP;
        if (pressed(2, gamepad1.left_bumper)) manualTPS = Math.max(0.0, manualTPS - TPS_STEP);

        if (pressed(3, gamepad1.dpad_up)) manualHood = clamp(manualHood + HOOD_STEP, 0.0, 1.0);
        if (pressed(4, gamepad1.dpad_down)) manualHood = clamp(manualHood - HOOD_STEP, 0.0, 1.0);

        if (pressed(5, gamepad1.dpad_left)) turretPos = clamp(turretPos - TURRET_STEP, 0.0, 1.0);
        if (pressed(6, gamepad1.dpad_right)) turretPos = clamp(turretPos + TURRET_STEP, 0.0, 1.0);

        if (pressed(7, gamepad1.a) && shooterOn && shotState == ShotState.IDLE) {
            if (pendingShot) writePending("?");
            startShot();
        }

        if (pressed(8, gamepad1.x) && pendingShot) writePending("HIT");
        if (pressed(9, gamepad1.b) && pendingShot) writePending("MISS");

        if (pressed(10, gamepad1.back)) tableMode = !tableMode;

        if (pressed(11, gamepad1.left_stick_button) && Double.isFinite(distance)) {
            Outtake.Prediction p = outtake.predict(distance);
            if (p.valid) {
                manualTPS = p.tps;
                manualHood = p.hood;
            }
        }

        if (gamepad1.right_stick_button && shotState == ShotState.IDLE) resetPose();
    }

    private boolean pressed(int index, boolean now) {
        boolean edge = now && !prev[index];
        prev[index] = now;
        return edge;
    }

    // ---------------------------------------------------------------------
    // Shot sequence
    // ---------------------------------------------------------------------

    private void startShot() {
        shotState = ShotState.WAIT_READY;
        stateStartTime = getRuntime();
        readyTimedOut = false;
    }

    private void runShot(double now) {
        switch (shotState) {
            case WAIT_READY:
                holdIdle();
                boolean timedOut = now - stateStartTime >= READY_TIMEOUT_SECONDS;
                if (outtake.isReady() || timedOut) {
                    readyTimedOut = !outtake.isReady();
                    shotState = ShotState.RELEASE;
                    stateStartTime = now;
                }
                break;

            case RELEASE:
                intake.stopIntake();
                outtake.stopFeeder();
                robotSettings.release();
                if (now - stateStartTime >= RELEASE_SECONDS) {
                    shotState = ShotState.FEED;
                    stateStartTime = now;
                    feedTPS = outtake.getCurrentTPS();
                    feedMinTPS = feedTPS;
                    captureShot();
                }
                break;

            case FEED:
                robotSettings.release();
                intake.intake(RobotConstants.frontFeedSpeed, RobotConstants.backFeedSpeed);
                outtake.setFeederOn(true);
                // Raw (unfiltered) velocity so the real dip isn't smoothed away
                feedMinTPS = Math.min(feedMinTPS, outtake.getRawTPS());
                if (now - stateStartTime >= FEED_SECONDS) {
                    shotState = ShotState.IDLE;
                    holdIdle();
                    updatePendingDip();
                }
                break;

            case IDLE:
            default:
                holdIdle();
                break;
        }
    }

    private void holdIdle() {
        intake.stopIntake();
        outtake.stopFeeder();
        robotSettings.block();
    }

    // ---------------------------------------------------------------------
    // Logging
    // ---------------------------------------------------------------------

    private double pendingX, pendingY, pendingDist, pendingHood, pendingTarget, pendingVolt;
    private String pendingMode = "";

    private void captureShot() {
        Pose pose = follower.pose();
        shotNumber++;
        pendingShot = true;
        pendingX = pose != null ? pose.x() : Double.NaN;
        pendingY = pose != null ? pose.y() : Double.NaN;
        pendingDist = distanceToGoal(pose);
        pendingHood = outtake.getHoodBase();
        pendingTarget = outtake.getTargetTPS();
        pendingVolt = outtake.getVoltage();
        pendingMode = tableMode ? "TABLE" : "MANUAL";
        logStatus = "shot " + shotNumber + ": press X (hit) or B (miss)";
    }

    private void updatePendingDip() {
        if (pendingShot) {
            logStatus = String.format("shot %d: dip to %.0f TPS. X = hit, B = miss",
                    shotNumber, feedMinTPS);
        }
    }

    private void writePending(String result) {
        pendingLine = String.format(
                "%d,%s,%.1f,%.1f,%.2f,%.3f,%.0f,%.0f,%.0f,%.2f,%s,%s",
                shotNumber, pendingMode, pendingX, pendingY, pendingDist,
                pendingHood, pendingTarget, feedTPS, feedMinTPS, pendingVolt,
                readyTimedOut ? "Y" : "N", result);

        pendingShot = false;
        recentLines.add(pendingLine);
        while (recentLines.size() > 6) recentLines.remove(0);

        File file = new File(AppUtil.ROOT_FOLDER, LOG_FILE_NAME);
        boolean isNew = !file.exists();
        try (FileWriter writer = new FileWriter(file, true)) {
            if (isNew) {
                writer.write("shot,mode,x,y,distance,hood,targetTPS,feedTPS,minTPS,"
                        + "voltage,readyTimeout,result\n");
            }
            writer.write(pendingLine + "\n");
            logStatus = "saved shot " + shotNumber + " (" + result + ")";
        } catch (IOException e) {
            logStatus = "LOG WRITE FAILED: " + e.getMessage();
        }
    }

    // ---------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------

    private boolean goalSet() {
        return RED_ALLIANCE
                ? Double.isFinite(RED_GOAL_X) && Double.isFinite(RED_GOAL_Y)
                : Double.isFinite(BLUE_GOAL_X) && Double.isFinite(BLUE_GOAL_Y);
    }

    private double distanceToGoal(Pose pose) {
        if (pose == null || !goalSet()) return Double.NaN;
        double gx = RED_ALLIANCE ? RED_GOAL_X : BLUE_GOAL_X;
        double gy = RED_ALLIANCE ? RED_GOAL_Y : BLUE_GOAL_Y;

        double px = pose.x();
        double py = pose.y();
        if (DISTANCE_FROM_TURRET_PIVOT) {
            double h = pose.heading();
            px += Turret.TURRET_OFFSET_X_IN * Math.cos(h) - Turret.TURRET_OFFSET_Y_IN * Math.sin(h);
            py += Turret.TURRET_OFFSET_X_IN * Math.sin(h) + Turret.TURRET_OFFSET_Y_IN * Math.cos(h);
        }
        return Math.hypot(gx - px, gy - py);
    }

    private double startX() {
        return USE_TELEOP_RED_UPPER_RESET
                ? CombinedTeleOp.ROBOT_WIDTH_MM / 25.4 / 2.0
                : START_X;
    }

    private double startY() {
        return USE_TELEOP_RED_UPPER_RESET
                ? CombinedTeleOp.FIELD_SIZE_IN - CombinedTeleOp.ROBOT_LENGTH_MM / 25.4 / 2.0
                : START_Y;
    }

    private double startHeadingDeg() {
        return USE_TELEOP_RED_UPPER_RESET
                ? CombinedTeleOp.RED_UPPER_RESET_HEADING_DEG
                : START_HEADING_DEG;
    }

    private void resetPose() {
        follower.setPose(poseFactory.of(startX(), startY(), startHeadingDeg()));
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void showTelemetry(Pose pose, double distance) {
        telemetry.addLine("===== SHOOTER TUNER =====");
        telemetry.addData("Mode", tableMode ? "TABLE (using lookup table)" : "MANUAL");
        telemetry.addData("Shooter", shooterOn ? "ON" : "off");
        if (pose != null) {
            telemetry.addData("Pose", "(%.1f, %.1f, %.0f)",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        }
        telemetry.addData("Distance", goalSet()
                ? String.format("%.1f in", distance) : "SET GOAL COORDS");

        telemetry.addLine("--- Shot ---");
        telemetry.addData("Manual TPS / Hood", "%.0f / %.3f", manualTPS, manualHood);
        telemetry.addData("Target TPS", "%.0f", outtake.getTargetTPS());
        telemetry.addData("Current TPS", "%.0f (err %.0f)",
                outtake.getCurrentTPS(), outtake.getErrorTPS());
        telemetry.addData("Ready", outtake.isReady());
        telemetry.addData("Hood", "%.3f", outtake.getHoodPosition());
        telemetry.addData("Turret", "%.3f", turretPos);
        telemetry.addData("Voltage", "%.2f V", outtake.getVoltage());
        telemetry.addData("Shot State", shotState);

        telemetry.addLine("--- Table ---");
        if (Double.isFinite(distance)) {
            Outtake.Prediction p = outtake.predict(distance);
            telemetry.addData("Prediction", p.valid
                    ? String.format("%.0f TPS, hood %.3f (%s)", p.tps, p.hood, p.status)
                    : p.status.toString());
        }
        telemetry.addData("Regression", outtake.getRegressionStatus());

        telemetry.addLine("--- Log ---");
        telemetry.addData("Status", logStatus);
        telemetry.addLine("shot,mode,x,y,dist,hood,tgt,feed,min,V,timeout,result");
        for (String line : recentLines) telemetry.addLine(line);

        telemetry.update();
    }
}