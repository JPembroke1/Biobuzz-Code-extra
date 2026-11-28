package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Turret aiming: odometry for the coarse aim, Limelight for the final lock.
 *
 * The turret angle is calculated from the robot pose and the goal
 * coordinates. When the Limelight (mounted on the turret, on the pivot)
 * can see the goal cluster, its bearing corrects that aim, so odometry
 * drift after a hit doesn't move the shots. See "Vision correction" below.
 *
 * Aim chain:
 *
 *   robot pose -> turret pivot position -> field angle to goal
 *   -> angle relative to robot -> turret angle -> servo position
 *
 * An error anywhere in that chain shows up as a missed shot, so each link
 * has its own calibration value below.
 */
@Configurable
public class Turret {

    // ---------------------------------------------------------------------
    // Hardware
    // ---------------------------------------------------------------------

    public static String TURRET_SERVO_NAME = "turretServo";

    private Servo turretServo;

    // ---------------------------------------------------------------------
    // Servo calibration
    //
    // With robot heading = 180 degrees:
    //
    // Servo 0.0 -> shooter field heading  90 degrees
    // Servo 0.5 -> shooter field heading   0 degrees
    // Servo 1.0 -> shooter field heading 270 degrees
    //
    // Therefore:
    //
    // - servo centre is 0.5;
    // - centre points 180 degrees behind the robot;
    // - increasing turret angle DECREASES servo position.
    // ---------------------------------------------------------------------

    public static double SERVO_CENTER = 0.5;

    public static double RIGHT_SERVO_LIMIT = 0.0;
    public static double LEFT_SERVO_LIMIT = 1.0;

    /*
     * Confirmed true from physical endpoint testing.
     */
    public static boolean REVERSE_AIM_MAPPING = true;

    /*
     * Turret rotation produced by moving the servo across its FULL range
     * (0.0 to 1.0).
     *
     * 180 matches your endpoint measurements (servo 0.0 = +90 degrees,
     * 1.0 = -90 degrees from centre). If shots are accurate near the
     * middle but increasingly wrong at larger turret angles, this value is
     * the first thing to check. See the mid-range check in the notes.
     */
    public static double SERVO_RANGE_DEG = 180.0;

    // ---------------------------------------------------------------------
    // Automatic aiming
    //
    // 90 = the full servo range (+/-90 degrees, servo 0.0 to 1.0).
    // This is the maximum: it is capped at SERVO_RANGE_DEG / 2.
    // Fixed servo commands retain the full 0.0 to 1.0 range.
    // ---------------------------------------------------------------------

    public static double AUTO_AIM_MAX_ANGLE_DEG = 90.0;

    /*
     * AIM_OFFSET is a servo-position offset, not degrees.
     * AIM_GAIN scales the requested angle before it is limited and
     * converted. Leave at 1.0 and use SERVO_RANGE_DEG for scale errors.
     */
    public static double AIM_OFFSET = 0.0;
    public static double AIM_GAIN = 1.0;

    /*
     * Auto-aim only sends a new servo position when it differs from the
     * last one sent by at least this much. 0.002 is about 0.36 degrees.
     * Saves loop time and stops jitter from pose noise. Keep 0.001-0.003;
     * above 0.005 starts to cost accuracy. 0 = send every loop.
     */
    public static double servoDeadband = 0.002;

    /*
     * At servo centre, the turret points behind the robot.
     */
    public static double TURRET_ZERO_OFFSET_DEG = 180.0;

    // ---------------------------------------------------------------------
    // Turret pivot offset from the Pedro/Pinpoint reference point
    //
    // Robot frame:
    //
    // +X = robot forward
    // +Y = robot left
    //
    // Units: inches.
    //
    // MEASURE THESE. They are the distance from the Pinpoint reference
    // point to the turret's axis of rotation. Left at 0 the error is small
    // from far away but large up close (a 3 inch offset at 12 inches is
    // about 14 degrees).
    // ---------------------------------------------------------------------

    public static double TURRET_OFFSET_X_IN = 0.0;
    public static double TURRET_OFFSET_Y_IN = 0.0;

    public static double MIN_AIM_DISTANCE_IN = 1.0;

    // ---------------------------------------------------------------------
    // Optional movement compensation
    //
    // velocityX and velocityY must be field-frame velocities in inches/s.
    // ---------------------------------------------------------------------

    public static boolean velocityCompensationActive = false;
    public static double VELOCITY_LEAD_GAIN = 0.3;

    // ---------------------------------------------------------------------
    // Vision correction (Limelight on the turret)
    //
    // The camera turns with the turret, so the cluster's bearing is the aim
    // error itself. Each new frame, while the turret is settled:
    //
    //   angle vision wants = commanded angle + bearing + cameraYawOffsetDeg
    //   correction         = angle vision wants - angle odometry wants
    //
    // The correction is smoothed and added on top of the odometry aim, so:
    //   - the turret still swings onto the goal from odometry when no tags
    //     are in view;
    //   - heading drift after a hit is cancelled while tags are visible;
    //   - when tags are lost the correction is held for visionHoldMs, then
    //     fades out over visionFadeMs;
    //   - a pose reset clears it (realignToSelectedGoal()).
    //
    // ASSUMES the cluster the camera tracks sits at the aim point. If the
    // camera locks on but shots go consistently left/right, use
    // cameraYawOffsetDeg (+ = aim further left).
    // ---------------------------------------------------------------------

    public static boolean visionAimEnabled = true;

    /** Camera mounting / cluster trim, degrees. + = aim further left. */
    public static double cameraYawOffsetDeg = 0.0;

    /** Fraction of each new frame's error applied (0-1). Lower = smoother. */
    public static double visionCorrectionGain = 0.5;

    /** Ignore frames that disagree with odometry by more than this (wrong tag, bad frame). */
    public static double visionMaxCorrectionDeg = 15.0;

    /** Turret must have been still (within visionSettleDeg) this long before a frame counts. */
    public static double visionSettleMs = 120.0;
    public static double visionSettleDeg = 1.0;

    /** Keep the correction this long after tags are lost, then fade it out. */
    public static double visionHoldMs = 3000.0;
    public static double visionFadeMs = 2000.0;

    /** Vision-only mode (odometry offline): fraction of the bearing applied per frame. */
    public static double visionOnlyGain = 0.7;

    /** Vision-only mode: on target when the last bearing was within this. */
    public static double visionOnTargetDeg = 2.0;

    // ---------------------------------------------------------------------
    // Goal coordinates (field frame, inches)
    // ---------------------------------------------------------------------

    public enum Goal {
        RED,
        BLUE,
        CUSTOM
    }

    public enum GoalHeight {
        HIGH,
        LOW
    }

    /*
     * Which goal height is being aimed at. A plain boolean so it can be
     * switched live from Panels as well as from code
     * (setHighGoal / setLowGoal / toggleGoalHeight).
     */
    public static boolean LOW_GOAL_SELECTED = false;

    /*
     * HIGH goal coordinates.
     *
     * These keep their original names so existing code and Panels values
     * still work.
     */
    public static double RED_GOAL_X = Double.NaN;
    public static double RED_GOAL_Y = Double.NaN;

    public static double BLUE_GOAL_X = Double.NaN;
    public static double BLUE_GOAL_Y = Double.NaN;

    /*
     * LOW goal coordinates.
     *
     * Edit these live in Panels while the turret is aiming. Changes are
     * picked up on the next loop, so you can nudge the point until the
     * shots land.
     */
    public static double RED_LOW_GOAL_X = Double.NaN;
    public static double RED_LOW_GOAL_Y = Double.NaN;

    public static double BLUE_LOW_GOAL_X = Double.NaN;
    public static double BLUE_LOW_GOAL_Y = Double.NaN;

    /*
     * Per-height angle trim, in degrees, added to the requested turret
     * angle for that goal height.
     *
     * Positive turns the aim counter-clockwise (left, seen from above).
     * Use the trim for a consistent angular error at every distance, and
     * the goal coordinates for an error that changes with distance.
     */
    public static double HIGH_GOAL_ANGLE_TRIM_DEG = 0.0;
    public static double LOW_GOAL_ANGLE_TRIM_DEG = 0.0;

    private Goal selectedGoal = Goal.CUSTOM;
    private Pose targetPose = null;

    // ---------------------------------------------------------------------
    // Aim source
    // ---------------------------------------------------------------------

    public enum AimSource {
        NONE,
        PINPOINT,
        PINPOINT_VISION,
        VISION_ONLY,
        FIXED
    }

    private AimSource currentAimSource = AimSource.NONE;

    // Last position actually sent to the servo. NaN forces the first write.
    private double lastCommandedServo = Double.NaN;

    // Vision state
    private boolean visionSeen = false;
    private double visionBearingDeg = Double.NaN;
    private long visionFrameId = -1L;
    private long lastUsedVisionFrameId = -1L;
    private long lastVisionSeenNs = 0L;

    private double visionCorrectionDeg = 0.0;
    private boolean visionCorrectionValid = false;
    private long lastVisionCorrectionNs = 0L;

    private boolean visionOnlyMode = false;

    // Shoot on the move: frames are skipped (correction held) while moving fast
    private boolean visionUpdatesAllowed = true;

    // Turret settle tracking (commanded angle)
    private double settleRefAngleDeg = Double.NaN;
    private long settleSinceNs = 0L;

    // ---------------------------------------------------------------------
    // Diagnostics
    // ---------------------------------------------------------------------

    public static double lastRobotX = 0.0;
    public static double lastRobotY = 0.0;
    public static double lastRobotHeadingDeg = 0.0;

    public static double lastPivotX = 0.0;
    public static double lastPivotY = 0.0;

    public static double lastTargetX = Double.NaN;
    public static double lastTargetY = Double.NaN;

    public static double lastDistanceToTargetIn = Double.NaN;

    public static double lastFieldTargetAngleDeg = Double.NaN;

    /*
     * Requested turret angle before the +/- limit is applied.
     */
    public static double lastRelativeAngleDeg = Double.NaN;

    public static double lastVelocityLeadX = 0.0;
    public static double lastVelocityLeadY = 0.0;

    /*
     * Turret angle actually commanded (after the limit), and the servo
     * position sent to the hardware.
     */
    public static double lastFinalAngleDeg = 0.0;
    public static double lastServoPosition = 0.5;

    public static boolean lastTargetReachable = false;
    public static boolean lastHoldingBehind = false;

    public static int lastHoldSide = 1;

    public static String lastSelectedGoal = "CUSTOM";
    public static boolean lastFixedPositionMode = true;

    public static String lastAimSource = "NONE";

    public static String lastGoalHeight = "HIGH";
    public static double lastAngleTrimDeg = 0.0;

    /*
     * True when an alliance goal is selected but the coordinates for the
     * current goal height are not set (NaN). The turret centres instead of
     * aiming at the wrong goal.
     */
    public static boolean lastTargetMissing = false;

    /** Vision diagnostics. */
    public static double lastVisionBearingDeg = Double.NaN;
    public static double lastVisionCorrectionDeg = 0.0;
    public static double lastVisionMeasuredDeg = Double.NaN;
    public static String lastVisionStatus = "NO DATA";

    // ---------------------------------------------------------------------
    // Initialization
    // ---------------------------------------------------------------------

    public void init(HardwareMap hardwareMap) {

        turretServo =
                hardwareMap.get(
                        Servo.class,
                        TURRET_SERVO_NAME
                );

        turretServo.setDirection(
                Servo.Direction.FORWARD
        );

        lastCommandedServo = Double.NaN;

        selectedGoal = Goal.CUSTOM;
        targetPose = null;

        clearAimDiagnostics();

        selectAllianceGoal();

        setCentre();
    }

    // ---------------------------------------------------------------------
    // Target selection
    // ---------------------------------------------------------------------

    public void selectAllianceGoal() {

        if (PoseStorage.isRed()) {

            selectRedGoal();

        } else if (PoseStorage.isBlue()) {

            selectBlueGoal();
        }
    }

    public void selectGoal(Goal goal) {

        if (goal == null
                || goal == Goal.CUSTOM) {

            return;
        }

        double targetX;
        double targetY;

        boolean low = LOW_GOAL_SELECTED;

        if (goal == Goal.RED) {

            targetX = low ? RED_LOW_GOAL_X : RED_GOAL_X;
            targetY = low ? RED_LOW_GOAL_Y : RED_GOAL_Y;

        } else {

            targetX = low ? BLUE_LOW_GOAL_X : BLUE_GOAL_X;
            targetY = low ? BLUE_LOW_GOAL_Y : BLUE_GOAL_Y;
        }

        if (!Double.isFinite(targetX)
                || !Double.isFinite(targetY)) {

            /*
             * Coordinates for this goal height are not set.
             *
             * Do not keep aiming at the previous goal. Drop the target so
             * the turret centres, and keep the goal selected so the
             * coordinates are picked up as soon as they are entered.
             *
             * An explicit custom target is left alone.
             */
            if (targetPose == null
                    || selectedGoal != Goal.CUSTOM) {

                selectedGoal = goal;

                lastSelectedGoal =
                        goal.name();

                lastTargetMissing = true;

                if (targetPose != null) {

                    targetPose = null;

                    clearAimDiagnostics();
                }
            }

            return;
        }

        applyTarget(
                goal,
                targetX,
                targetY
        );
    }

    public void selectRedGoal() {

        selectGoal(
                Goal.RED
        );
    }

    public void selectBlueGoal() {

        selectGoal(
                Goal.BLUE
        );
    }

    // ---------------------------------------------------------------------
    // Goal height (high / low)
    // ---------------------------------------------------------------------

    public void setGoalHeight(GoalHeight height) {

        if (height == null) {
            return;
        }

        LOW_GOAL_SELECTED =
                height == GoalHeight.LOW;

        refreshGoalTarget();
    }

    public void setHighGoal() {

        setGoalHeight(
                GoalHeight.HIGH
        );
    }

    public void setLowGoal() {

        setGoalHeight(
                GoalHeight.LOW
        );
    }

    public void toggleGoalHeight() {

        LOW_GOAL_SELECTED =
                !LOW_GOAL_SELECTED;

        refreshGoalTarget();
    }

    public GoalHeight getGoalHeight() {

        return LOW_GOAL_SELECTED
                ? GoalHeight.LOW
                : GoalHeight.HIGH;
    }

    private double activeAngleTrimDeg() {

        double trim =
                LOW_GOAL_SELECTED
                        ? LOW_GOAL_ANGLE_TRIM_DEG
                        : HIGH_GOAL_ANGLE_TRIM_DEG;

        return Double.isFinite(trim)
                ? trim
                : 0.0;
    }

    /**
     * Re-resolves the alliance goal for the current goal height.
     *
     * Called every aim loop, so coordinates and the goal height edited
     * live in Panels take effect immediately.
     */
    private void refreshGoalTarget() {

        lastGoalHeight =
                getGoalHeight().name();

        lastAngleTrimDeg =
                activeAngleTrimDeg();

        if (selectedGoal == Goal.RED
                || selectedGoal == Goal.BLUE) {

            selectGoal(
                    selectedGoal
            );
        }
    }

    /**
     * Supplies an explicit field-coordinate target.
     *
     * Pose heading is ignored.
     */
    public void setTargetPose(Pose pose) {

        if (pose == null
                || !Double.isFinite(pose.x())
                || !Double.isFinite(pose.y())) {

            return;
        }

        applyTarget(
                Goal.CUSTOM,
                pose.x(),
                pose.y()
        );
    }

    private void applyTarget(
            Goal goal,
            double targetX,
            double targetY
    ) {

        boolean targetChanged =
                selectedGoal != goal
                        || targetPose == null
                        || Double.compare(
                        targetPose.x(),
                        targetX
                ) != 0
                        || Double.compare(
                        targetPose.y(),
                        targetY
                ) != 0;

        if (targetChanged) {

            selectedGoal = goal;

            targetPose =
                    new Pose(
                            targetX,
                            targetY,
                            0.0
                    );

            clearAimDiagnostics();
        }

        lastSelectedGoal =
                selectedGoal.name();

        lastTargetX = targetX;
        lastTargetY = targetY;

        lastTargetMissing = false;
    }

    public Goal getSelectedGoal() {

        return selectedGoal;
    }

    public Pose getTargetPose() {

        if (targetPose == null) {
            return null;
        }

        return new Pose(
                targetPose.x(),
                targetPose.y(),
                targetPose.heading()
        );
    }

    public void realignToAllianceGoal() {

        selectAllianceGoal();
        clearAimDiagnostics();
        clearVisionCorrection();
    }

    public void realignToSelectedGoal() {

        clearAimDiagnostics();
        clearVisionCorrection();
    }

    private void clearAimDiagnostics() {

        lastDistanceToTargetIn = Double.NaN;

        lastFieldTargetAngleDeg = Double.NaN;
        lastRelativeAngleDeg = Double.NaN;

        lastVelocityLeadX = 0.0;
        lastVelocityLeadY = 0.0;

        lastTargetReachable = false;
        lastHoldingBehind = false;
        lastHoldSide = 1;

        currentAimSource = AimSource.NONE;
        lastAimSource = currentAimSource.name();
    }

    // ---------------------------------------------------------------------
    // Pinpoint aiming
    // ---------------------------------------------------------------------

    public void aimTurret(Pose robotPose) {

        aimTurret(
                robotPose,
                0.0,
                0.0
        );
    }

    public void aimTurret(
            Pose robotPose,
            double velocityX,
            double velocityY
    ) {

        // Old simple lead: a fixed VELOCITY_LEAD_GAIN seconds of travel.
        double leadX = 0.0;
        double leadY = 0.0;

        if (velocityCompensationActive
                && Double.isFinite(velocityX)
                && Double.isFinite(velocityY)
                && Double.isFinite(VELOCITY_LEAD_GAIN)) {

            leadX = -velocityX * VELOCITY_LEAD_GAIN;
            leadY = -velocityY * VELOCITY_LEAD_GAIN;
        }

        aimTurretWithLead(robotPose, leadX, leadY);
    }

    /**
     * Shoot on the move: aims at the goal shifted by (leadX, leadY) inches
     * (the "virtual goal"), worked out by the caller from the robot's
     * velocity and the ball's time of flight. The Limelight correction is
     * still measured against the REAL goal, so the camera never pulls the
     * aim back off the lead.
     */
    public void aimTurretWithLead(
            Pose robotPose,
            double leadX,
            double leadY
    ) {

        refreshGoalTarget();

        AimCalculation calculation =
                calculateAim(
                        robotPose,
                        Double.isFinite(leadX) ? leadX : 0.0,
                        Double.isFinite(leadY) ? leadY : 0.0
                );

        if (calculation == null) {

            setCentre();

            return;
        }

        visionOnlyMode = false;

        double correctionRad =
                Math.toRadians(
                        updateVisionCorrection(
                                calculation.realRequestedAngleRad
                        )
                );

        applyAutomaticAngle(
                calculation.requestedAngleRad,
                correctionRad
        );

        updateDiagnostics(
                robotPose,
                calculation
        );
    }

    // ---------------------------------------------------------------------
    // Field-coordinate aim calculation
    // ---------------------------------------------------------------------

    private AimCalculation calculateAim(
            Pose robotPose,
            double leadX,
            double leadY
    ) {

        if (!validPose(robotPose)
                || targetPose == null
                || !Double.isFinite(TURRET_OFFSET_X_IN)
                || !Double.isFinite(TURRET_OFFSET_Y_IN)
                || !Double.isFinite(TURRET_ZERO_OFFSET_DEG)
                || !Double.isFinite(MIN_AIM_DISTANCE_IN)) {

            return null;
        }

        double heading =
                robotPose.heading();

        double headingCos =
                Math.cos(heading);

        double headingSin =
                Math.sin(heading);

        /*
         * Turret pivot position in field coordinates.
         */
        double pivotX =
                robotPose.x()
                        + TURRET_OFFSET_X_IN * headingCos
                        - TURRET_OFFSET_Y_IN * headingSin;

        double pivotY =
                robotPose.y()
                        + TURRET_OFFSET_X_IN * headingSin
                        + TURRET_OFFSET_Y_IN * headingCos;

        /*
         * Vector from the turret pivot to the goal.
         */
        double targetDx =
                targetPose.x()
                        - pivotX;

        double targetDy =
                targetPose.y()
                        - pivotY;

        double physicalDistance =
                Math.sqrt(
                        targetDx * targetDx
                                + targetDy * targetDy
                );

        double minimumDistance =
                Math.max(
                        0.0,
                        MIN_AIM_DISTANCE_IN
                );

        if (!Double.isFinite(physicalDistance)
                || physicalDistance <= minimumDistance) {

            return null;
        }

        /*
         * Movement compensation: aim at the goal shifted by the lead.
         */
        double aimDx =
                targetDx
                        + leadX;

        double aimDy =
                targetDy
                        + leadY;

        double compensatedDistance =
                Math.sqrt(
                        aimDx * aimDx
                                + aimDy * aimDy
                );

        if (!Double.isFinite(compensatedDistance)
                || compensatedDistance <= minimumDistance) {

            return null;
        }

        /*
         * Pedro heading:
         * 0 degrees = +X
         * 90 degrees = +Y
         *
         * Turret servo centre points 180 degrees behind the robot.
         */
        double fieldTargetAngle =
                Math.atan2(
                        aimDy,
                        aimDx
                );

        double requestedAngle =
                normalizeRadians(
                        fieldTargetAngle
                                - heading
                                - Math.toRadians(
                                TURRET_ZERO_OFFSET_DEG
                        )
                                + Math.toRadians(
                                activeAngleTrimDeg()
                        )
                );

        if (!Double.isFinite(requestedAngle)) {
            return null;
        }

        // Same angle to the REAL goal (no lead), for the Limelight correction
        double realRequestedAngle =
                normalizeRadians(
                        Math.atan2(targetDy, targetDx)
                                - heading
                                - Math.toRadians(TURRET_ZERO_OFFSET_DEG)
                                + Math.toRadians(activeAngleTrimDeg())
                );

        return new AimCalculation(
                requestedAngle,
                realRequestedAngle,
                fieldTargetAngle,
                pivotX,
                pivotY,
                physicalDistance,
                leadX,
                leadY
        );
    }

    // ---------------------------------------------------------------------
    // Automatic command application
    // ---------------------------------------------------------------------

    private void applyAutomaticAngle(
            double requestedAngleRad,
            double visionCorrectionRad
    ) {

        if (turretServo == null
                || !Double.isFinite(requestedAngleRad)
                || !Double.isFinite(AIM_GAIN)
                || !Double.isFinite(AIM_OFFSET)) {

            setCentre();

            return;
        }

        double gainedAngle =
                requestedAngleRad
                        * AIM_GAIN
                        + (Double.isFinite(visionCorrectionRad)
                        ? visionCorrectionRad
                        : 0.0);

        if (!Double.isFinite(gainedAngle)) {

            setCentre();

            return;
        }

        double maximumAngle =
                getAutoAimMaxAngleRad();

        if (!Double.isFinite(maximumAngle)
                || maximumAngle <= 0.0) {

            setCentre();

            return;
        }

        lastFixedPositionMode = false;

        lastRelativeAngleDeg =
                Math.toDegrees(
                        requestedAngleRad
                );

        lastTargetReachable =
                Math.abs(gainedAngle)
                        <= maximumAngle;

        lastHoldingBehind =
                !lastTargetReachable;

        if (gainedAngle > 0.0) {

            lastHoldSide = 1;

        } else if (gainedAngle < 0.0) {

            lastHoldSide = -1;
        }

        double commandedAngle =
                clamp(
                        gainedAngle,
                        -maximumAngle,
                        maximumAngle
                );

        double servoPosition =
                angleToServoPosition(
                        commandedAngle
                )
                        + AIM_OFFSET;

        servoPosition =
                clampToServoLimits(
                        servoPosition
                );

        double deadband =
                Double.isFinite(servoDeadband)
                        ? Math.abs(servoDeadband)
                        : 0.0;

        if (Double.isNaN(lastCommandedServo)
                || Math.abs(servoPosition - lastCommandedServo) >= deadband) {

            turretServo.setPosition(
                    servoPosition
            );

            lastCommandedServo = servoPosition;
        }

        lastFinalAngleDeg =
                Math.toDegrees(
                        commandedAngle
                );

        trackSettle(lastFinalAngleDeg);

        lastServoPosition =
                lastCommandedServo;

        currentAimSource =
                visionCorrectionRad != 0.0
                        ? AimSource.PINPOINT_VISION
                        : AimSource.PINPOINT;

        lastAimSource =
                currentAimSource.name();
    }

    // ---------------------------------------------------------------------
    // Diagnostics
    // ---------------------------------------------------------------------

    private void updateDiagnostics(
            Pose robotPose,
            AimCalculation calculation
    ) {

        lastRobotX =
                robotPose.x();

        lastRobotY =
                robotPose.y();

        lastRobotHeadingDeg =
                normalizeDegrees0To360(
                        Math.toDegrees(
                                robotPose.heading()
                        )
                );

        lastPivotX =
                calculation.pivotX;

        lastPivotY =
                calculation.pivotY;

        lastTargetX =
                targetPose.x();

        lastTargetY =
                targetPose.y();

        lastDistanceToTargetIn =
                calculation.physicalDistance;

        lastFieldTargetAngleDeg =
                normalizeDegrees0To360(
                        Math.toDegrees(
                                calculation.fieldTargetAngleRad
                        )
                );

        lastVelocityLeadX =
                calculation.leadX;

        lastVelocityLeadY =
                calculation.leadY;

        lastSelectedGoal =
                selectedGoal.name();
    }

    // ---------------------------------------------------------------------
    // Reachability and shooting readiness
    // ---------------------------------------------------------------------

    public boolean isTargetReachable() {

        return lastTargetReachable
                && !lastFixedPositionMode
                && Double.isFinite(
                lastRelativeAngleDeg
        )
                && Double.isFinite(
                lastFinalAngleDeg
        );
    }

    /**
     * Ready when the Pinpoint aim is active and the goal is inside the
     * automatic turret limit.
     */
    public boolean isAimReady() {

        return (currentAimSource == AimSource.PINPOINT
                || currentAimSource == AimSource.PINPOINT_VISION)
                && isTargetReachable();
    }

    public boolean isUsingPinpointAim() {

        return currentAimSource == AimSource.PINPOINT
                || currentAimSource == AimSource.PINPOINT_VISION;
    }

    public AimSource getAimSource() {

        return currentAimSource;
    }

    // ---------------------------------------------------------------------
    // Automatic limit
    //
    // Limited to the turret's physical half-range.
    // ---------------------------------------------------------------------

    private double getAutoAimMaxAngleDeg() {

        if (!Double.isFinite(
                AUTO_AIM_MAX_ANGLE_DEG
        )) {

            return 90.0;
        }

        return clamp(
                Math.abs(
                        AUTO_AIM_MAX_ANGLE_DEG
                ),
                0.0,
                servoRangeDeg() / 2.0
        );
    }

    private double getAutoAimMaxAngleRad() {

        return Math.toRadians(
                getAutoAimMaxAngleDeg()
        );
    }

    // ---------------------------------------------------------------------
    // Fixed servo commands
    //
    // These bypass the automatic limit.
    // ---------------------------------------------------------------------

    public void setFixedServoPosition(
            double position
    ) {

        if (turretServo == null
                || !Double.isFinite(position)) {

            return;
        }

        double clippedPosition =
                clampToServoLimits(
                        position
                );

        // Fixed positions skip the deadband but don't resend an identical value
        if (clippedPosition != lastCommandedServo) {

            turretServo.setPosition(
                    clippedPosition
            );

            lastCommandedServo = clippedPosition;
        }

        lastFixedPositionMode = true;
        lastServoPosition = clippedPosition;
        visionOnlyMode = false;

        lastFinalAngleDeg =
                servoPositionToAngleDeg(
                        clippedPosition
                );

        trackSettle(lastFinalAngleDeg);

        lastTargetReachable = false;
        lastHoldingBehind = false;

        if (lastFinalAngleDeg > 0.0) {

            lastHoldSide = 1;

        } else if (lastFinalAngleDeg < 0.0) {

            lastHoldSide = -1;
        }

        lastRelativeAngleDeg = Double.NaN;
        lastFieldTargetAngleDeg = Double.NaN;

        currentAimSource = AimSource.FIXED;
        lastAimSource = currentAimSource.name();
    }

    public void setCentre() {

        setFixedServoPosition(
                centrePosition()
        );

        lastFinalAngleDeg = 0.0;
        lastServoPosition = centrePosition();
        lastFixedPositionMode = true;
    }

    /**
     * Compatibility method.
     *
     * On this robot, servo centre points behind the robot.
     */
    public void setForward() {

        setCentre();
    }

    public void setFullRight() {

        setFixedServoPosition(
                RIGHT_SERVO_LIMIT
        );
    }

    public void setFullLeft() {

        setFixedServoPosition(
                LEFT_SERVO_LIMIT
        );
    }

    // ---------------------------------------------------------------------
    // Servo mapping
    // ---------------------------------------------------------------------

    private double angleToServoPosition(
            double angleRadians
    ) {

        double mappingSign =
                REVERSE_AIM_MAPPING
                        ? -1.0
                        : 1.0;

        return centrePosition()
                + mappingSign
                * Math.toDegrees(
                angleRadians
        )
                / servoRangeDeg();
    }

    private double servoPositionToAngleDeg(
            double servoPosition
    ) {

        double mappingSign =
                REVERSE_AIM_MAPPING
                        ? -1.0
                        : 1.0;

        return mappingSign
                * (servoPosition
                - centrePosition())
                * servoRangeDeg();
    }

    private static double servoRangeDeg() {

        return Double.isFinite(SERVO_RANGE_DEG)
                && SERVO_RANGE_DEG > 0.0
                ? SERVO_RANGE_DEG
                : 180.0;
    }

    private double clampToServoLimits(
            double position
    ) {

        double rightLimit =
                Double.isFinite(
                        RIGHT_SERVO_LIMIT
                )
                        ? clamp(
                        RIGHT_SERVO_LIMIT,
                        0.0,
                        1.0
                )
                        : 0.0;

        double leftLimit =
                Double.isFinite(
                        LEFT_SERVO_LIMIT
                )
                        ? clamp(
                        LEFT_SERVO_LIMIT,
                        0.0,
                        1.0
                )
                        : 1.0;

        return clamp(
                position,
                Math.min(
                        rightLimit,
                        leftLimit
                ),
                Math.max(
                        rightLimit,
                        leftLimit
                )
        );
    }

    private static double centrePosition() {

        return Double.isFinite(
                SERVO_CENTER
        )
                ? clamp(
                SERVO_CENTER,
                0.0,
                1.0
        )
                : 0.5;
    }

    // ---------------------------------------------------------------------
    // Status
    // ---------------------------------------------------------------------

    public double getDistanceToTarget() {

        return lastDistanceToTargetIn;
    }

    public double getServoPosition() {

        return turretServo == null
                ? lastServoPosition
                : turretServo.getPosition();
    }

    public boolean isHoldingBehind() {

        return lastHoldingBehind;
    }

    public boolean isFixedPositionMode() {

        return lastFixedPositionMode;
    }

    public static double getCalibratedCentrePosition() {

        return centrePosition();
    }

    // ---------------------------------------------------------------------
    // Vision input
    // ---------------------------------------------------------------------

    /**
     * Call once per loop BEFORE aimTurret()/aimVisionOnly().
     *
     * @param seen       true if the goal cluster is visible this loop
     * @param bearingDeg its bearing from the camera, degrees, + = LEFT
     * @param frameId    the camera frame it came from (only new frames count)
     */
    public void setVisionMeasurement(boolean seen, double bearingDeg, long frameId) {

        visionSeen = seen && Double.isFinite(bearingDeg);
        visionBearingDeg = visionSeen ? bearingDeg : Double.NaN;
        visionFrameId = frameId;

        if (visionSeen) {
            lastVisionSeenNs = System.nanoTime();
        }

        lastVisionBearingDeg = visionBearingDeg;
    }

    public void clearVisionMeasurement() {

        setVisionMeasurement(false, Double.NaN, visionFrameId);
    }

    /**
     * Shoot on the move: false stops new Limelight frames from updating the
     * correction (frames taken while moving fast are blurred and late). The
     * current correction is kept.
     */
    public void setVisionUpdatesAllowed(boolean allowed) {

        visionUpdatesAllowed = allowed;
    }

    /** Drops the vision correction (pose reset, goal change). */
    public void clearVisionCorrection() {

        visionCorrectionDeg = 0.0;
        visionCorrectionValid = false;
        lastVisionCorrectionNs = 0L;
        lastUsedVisionFrameId = -1L;

        lastVisionCorrectionDeg = 0.0;
        lastVisionMeasuredDeg = Double.NaN;
    }

    /**
     * Where the turret should point relative to the camera's view of the goal:
     * the camera mounting offset PLUS the goal's angle trim. Before 27 Nov
     * 2026 the trim was left out here, so as soon as the Limelight locked on
     * it cancelled HIGH/LOW_GOAL_ANGLE_TRIM_DEG and the aim moved back by
     * the trim. The trim now applies the same with or without the camera.
     */
    private double visionAimOffsetDeg() {

        return finiteOr(cameraYawOffsetDeg, 0.0) + activeAngleTrimDeg();
    }

    /** True if the camera can currently see the goal cluster. */
    public boolean isVisionTargetVisible() {

        return visionSeen;
    }

    /** True if a vision correction is currently being applied. */
    public boolean isVisionCorrecting() {

        return visionCorrectionValid
                && Math.abs(lastVisionCorrectionDeg) > 1e-6;
    }

    /**
     * Updates the smoothed correction from a new, settled frame and returns
     * the correction to apply now (degrees), including hold and fade.
     */
    private double updateVisionCorrection(double requestedAngleRad) {

        if (!visionAimEnabled) {
            lastVisionStatus = "OFF";
            lastVisionCorrectionDeg = 0.0;
            return 0.0;
        }

        long now = System.nanoTime();

        if (visionSeen
                && visionFrameId != lastUsedVisionFrameId
                && !visionUpdatesAllowed) {

            lastVisionStatus = visionCorrectionValid
                    ? "MOVING - HOLDING CORRECTION" : "MOVING - NOT MEASURING";

        } else if (visionSeen
                && visionFrameId != lastUsedVisionFrameId) {

            if (!isTurretSettled()) {

                lastVisionStatus = "WAITING FOR TURRET TO SETTLE";

            } else {

                lastUsedVisionFrameId = visionFrameId;

                double odometryDeg =
                        Math.toDegrees(requestedAngleRad * AIM_GAIN);

                double visionWantsDeg =
                        lastFinalAngleDeg
                                + visionBearingDeg
                                + visionAimOffsetDeg();

                double measured =
                        normalizeDegrees180(visionWantsDeg - odometryDeg);

                lastVisionMeasuredDeg = measured;

                if (Math.abs(measured)
                        > Math.abs(finiteOr(visionMaxCorrectionDeg, 15.0))) {

                    lastVisionStatus = String.format(
                            "REJECTED %.1f deg (> visionMaxCorrectionDeg)", measured);

                } else {

                    double gain = clamp(finiteOr(visionCorrectionGain, 0.5), 0.0, 1.0);

                    visionCorrectionDeg = visionCorrectionValid
                            ? visionCorrectionDeg + gain * (measured - visionCorrectionDeg)
                            : measured;

                    visionCorrectionValid = true;
                    lastVisionCorrectionNs = now;
                    lastVisionStatus = "LOCKED";
                }
            }

        } else if (!visionSeen) {

            lastVisionStatus = visionCorrectionValid ? "TAGS LOST - HOLDING" : "NO TAGS";
        }

        if (!visionCorrectionValid) {
            lastVisionCorrectionDeg = 0.0;
            return 0.0;
        }

        // Hold, then fade out, once frames stop coming in.
        double sinceMs = (now - lastVisionCorrectionNs) / 1_000_000.0;
        double holdMs = Math.max(0.0, finiteOr(visionHoldMs, 3000.0));
        double fadeMs = Math.max(1.0, finiteOr(visionFadeMs, 2000.0));

        double scale = 1.0;
        if (sinceMs > holdMs) {
            scale = 1.0 - (sinceMs - holdMs) / fadeMs;
        }

        if (scale <= 0.0) {
            clearVisionCorrection();
            lastVisionStatus = "NO TAGS";
            return 0.0;
        }

        if (scale < 1.0) {
            lastVisionStatus = "TAGS LOST - FADING";
        }

        lastVisionCorrectionDeg = visionCorrectionDeg * scale;
        return lastVisionCorrectionDeg;
    }

    // ---------------------------------------------------------------------
    // Vision-only aiming (odometry offline)
    // ---------------------------------------------------------------------

    /**
     * Aims from the camera alone, for when odometry is offline. Each new,
     * settled frame steps the turret by visionOnlyGain x bearing. With no
     * tags it holds its angle for visionHoldMs, then centres.
     *
     * Call setVisionMeasurement() first.
     */
    public void aimVisionOnly() {

        if (turretServo == null) {
            return;
        }

        long now = System.nanoTime();

        boolean recentlySeen = lastVisionSeenNs != 0L
                && (now - lastVisionSeenNs) / 1_000_000.0
                <= Math.max(0.0, finiteOr(visionHoldMs, 3000.0));

        if (!recentlySeen) {
            setCentre();
            lastVisionStatus = "VISION ONLY: NO TAGS - CENTRED";
            return;
        }

        // Starting from a fixed position: begin from where the servo is.
        if (!visionOnlyMode) {
            visionOnlyMode = true;
            lastUsedVisionFrameId = -1L;
        }

        double angleDeg = lastFinalAngleDeg;

        if (visionSeen
                && visionFrameId != lastUsedVisionFrameId
                && isTurretSettled()) {

            lastUsedVisionFrameId = visionFrameId;

            double gain = clamp(finiteOr(visionOnlyGain, 0.7), 0.0, 1.0);
            angleDeg = lastFinalAngleDeg
                    + gain * (visionBearingDeg + visionAimOffsetDeg());

            lastVisionStatus = "VISION ONLY: TRACKING";

        } else if (!visionSeen) {

            lastVisionStatus = "VISION ONLY: TAGS LOST - HOLDING";
        }

        double maximumDeg = getAutoAimMaxAngleDeg();
        double commandedDeg = clamp(angleDeg, -maximumDeg, maximumDeg);

        double servoPosition = clampToServoLimits(
                angleToServoPosition(Math.toRadians(commandedDeg))
                        + finiteOr(AIM_OFFSET, 0.0));

        double deadband = Double.isFinite(servoDeadband) ? Math.abs(servoDeadband) : 0.0;

        if (Double.isNaN(lastCommandedServo)
                || Math.abs(servoPosition - lastCommandedServo) >= deadband) {
            turretServo.setPosition(servoPosition);
            lastCommandedServo = servoPosition;
        }

        lastFinalAngleDeg = commandedDeg;
        trackSettle(lastFinalAngleDeg);

        lastServoPosition = lastCommandedServo;
        lastFixedPositionMode = false;
        lastTargetReachable = Math.abs(angleDeg) <= maximumDeg;
        lastHoldingBehind = !lastTargetReachable;
        lastRelativeAngleDeg = Double.NaN;
        lastFieldTargetAngleDeg = Double.NaN;

        currentAimSource = AimSource.VISION_ONLY;
        lastAimSource = currentAimSource.name();
    }

    /** True while vision-only aiming is in control of the turret. */
    public boolean isVisionOnlyAim() {

        return currentAimSource == AimSource.VISION_ONLY;
    }

    /**
     * Vision-only: the cluster is visible, the turret is settled and the
     * last bearing was within visionOnTargetDeg.
     */
    public boolean isVisionOnTarget() {

        return visionSeen
                && isTurretSettled()
                && Math.abs(visionBearingDeg + visionAimOffsetDeg())
                <= Math.abs(finiteOr(visionOnTargetDeg, 2.0));
    }

    // ---------------------------------------------------------------------
    // Settle tracking
    // ---------------------------------------------------------------------

    private void trackSettle(double commandedDeg) {

        if (Double.isNaN(settleRefAngleDeg)
                || Math.abs(commandedDeg - settleRefAngleDeg)
                > Math.abs(finiteOr(visionSettleDeg, 1.0))) {

            settleRefAngleDeg = commandedDeg;
            settleSinceNs = System.nanoTime();
        }
    }

    /** True once the commanded angle has been steady for visionSettleMs. */
    public boolean isTurretSettled() {

        return settleSinceNs != 0L
                && (System.nanoTime() - settleSinceNs) / 1_000_000.0
                >= Math.max(0.0, finiteOr(visionSettleMs, 120.0));
    }

    // ---------------------------------------------------------------------
    // Internal calculations
    // ---------------------------------------------------------------------

    private static class AimCalculation {

        final double requestedAngleRad;
        final double realRequestedAngleRad;
        final double fieldTargetAngleRad;

        final double pivotX;
        final double pivotY;

        final double physicalDistance;

        final double leadX;
        final double leadY;

        AimCalculation(
                double requestedAngleRad,
                double realRequestedAngleRad,
                double fieldTargetAngleRad,
                double pivotX,
                double pivotY,
                double physicalDistance,
                double leadX,
                double leadY
        ) {

            this.requestedAngleRad =
                    requestedAngleRad;

            this.realRequestedAngleRad =
                    realRequestedAngleRad;

            this.fieldTargetAngleRad =
                    fieldTargetAngleRad;

            this.pivotX = pivotX;
            this.pivotY = pivotY;

            this.physicalDistance =
                    physicalDistance;

            this.leadX = leadX;
            this.leadY = leadY;
        }
    }

    // ---------------------------------------------------------------------
    // Utilities
    // ---------------------------------------------------------------------

    private boolean validPose(Pose pose) {

        return pose != null
                && Double.isFinite(pose.x())
                && Double.isFinite(pose.y())
                && Double.isFinite(
                pose.heading()
        );
    }

    private static double normalizeRadians(
            double angle
    ) {

        return Math.atan2(
                Math.sin(angle),
                Math.cos(angle)
        );
    }

    private static double normalizeDegrees0To360(
            double angleDegrees
    ) {

        double normalized =
                angleDegrees % 360.0;

        if (normalized < 0.0) {
            normalized += 360.0;
        }

        return normalized;
    }

    private static double normalizeDegrees180(double angleDegrees) {

        double a = angleDegrees % 360.0;
        if (a >= 180.0) a -= 360.0;
        if (a < -180.0) a += 360.0;
        return a;
    }

    private static double finiteOr(double value, double fallback) {

        return Double.isFinite(value) ? value : fallback;
    }

    private static double clamp(
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