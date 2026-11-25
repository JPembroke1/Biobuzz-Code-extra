package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.Map;

@Configurable
public class Outtake {

    // Hardware
    public static String SHOOTER_MOTOR_NAME = "shooterMotor1";
    public static String FEEDER_MOTOR_NAME = "shooterMotor";
    public static String HOOD_SERVO_NAME = "hoodServo";
    public static boolean SHOOTER_REVERSED = true;
    public static boolean FEEDER_REVERSED = true;

    // Shooter controller
    public static double kF = 0.00045; //0.00045
    public static double kP = 0.015; //0.015

    // Feeder: no encoder, open loop. Power used when the feeder is on.
    public static double feederPower = 0.9;

    // Limits and filtering
    public static double MAX_POWER = 1.0;
    public static double VELOCITY_FILTER_ALPHA = 0.15;

    // Ready gate: shooter must be within this many TPS of target to fire
    public static double READY_TOLERANCE_TPS = 30.0;

    // Voltage compensation (shooter only)
    public static boolean VOLTAGE_COMP_ENABLED = true;
    public static double NOMINAL_VOLTAGE = 13.2;
    public static double VOLTAGE_FILTER_ALPHA = 0.10;
    /*
     * Battery voltage is not part of the hub's bulk read, so every
     * getVoltage() is its own hub command. Read it at most this often (ms)
     * instead of every loop. Battery voltage changes slowly, so this does
     * not hurt the compensation.
     */
    public static double voltageSampleIntervalMs = 100.0;
    public static double MIN_VOLTAGE = 9.0;
    public static double MAX_VOLTAGE = 14.5;
    public static double MIN_VOLTAGE_SCALE = 0.80;
    public static double MAX_VOLTAGE_SCALE = 1.20;

    // Hood calibration
    public static double HOOD_HOME = 0.50;
    public static double HOOD_MIN = 0.40;
    public static double HOOD_MAX = 0.80;
    public static double HOOD_MIN_STEP = 0.002;

    // Hood velocity compensation (keep OFF while collecting tuning data)
    public static boolean HOOD_COMP_ENABLED = false;
    public static double HOOD_COMP_GAIN = 0.05;
    public static double HOOD_COMP_DEADBAND_TPS = 50.0;
    public static double HOOD_COMP_MAX = 0.08;

    // ---------------------------------------------------------------------
    // Distance model
    // ---------------------------------------------------------------------

    /*
     * On by default. Regression only takes over when an OpMode supplies a
     * valid goal distance (setGoalDistance); with no distance it falls back
     * to the fixed TPS from setTargetTPS, so autos that never send a
     * distance keep shooting at their fixed speeds.
     */
    public static boolean DISTANCE_REGRESSION_ENABLED = true;

    /*
     * true  = interpolated lookup table (LUT_* below)
     * false = straight-line model (TPS_/HOOD_ SLOPE and INTERCEPT)
     */
    public static boolean USE_LOOKUP_TABLE = true;

    /*
     * true  = distances outside the tuned range use the nearest end of
     *         the model (no snapping back to the fixed values)
     * false = outside the range falls back to fixed TPS/hood (old behaviour)
     */
    public static boolean CLAMP_DISTANCE_TO_RANGE = true;

    // Straight-line model
    public static double TPS_SLOPE = 5.779158656;
    public static double TPS_INTERCEPT = 1724.035454492;
    public static double HOOD_SLOPE = 0.0;
    public static double HOOD_INTERCEPT = 0.60;
    public static double SHOT_DISTANCE_MIN = 39.09;
    public static double SHOT_DISTANCE_MAX = 65.06;

    /*
     * Lookup table. Distances (inches) must be strictly increasing.
     * Only the first LUT_POINTS rows are used (2 to 7).
     * Tuned 15 Nov 2026 (hood 0.60, red upper goal, smoothed fit of
     * 39.1 in = 1950, 45.0 in = 2050, 57.1 in = 2100). 51 in is
     * interpolated, not yet shot: confirm 45 and 51 on the field.
     *
     * 27 in and 33 in added 23 Nov 2026 by extending the existing trend
     * (about 7.5-7.8 TPS per inch, i.e. ~45 TPS per 6 in step). They are
     * ESTIMATES, not shot yet: test both on the field and adjust. Close
     * shots often need a different hood angle rather than just less speed.
     */
    public static int LUT_POINTS = 6;

    public static double LUT_DIST_1 = 27.0;   // NEW, estimated
    public static double LUT_DIST_2 = 33.0;   // NEW, estimated
    public static double LUT_DIST_3 = 39.0;
    public static double LUT_DIST_4 = 45.0;
    public static double LUT_DIST_5 = 51.0;
    public static double LUT_DIST_6 = 57.0;
    public static double LUT_DIST_7 = 0.0;

    public static double LUT_TPS_1 = 1880.0;  // NEW, estimated
    public static double LUT_TPS_2 = 1925.0;  // NEW, estimated
    public static double LUT_TPS_3 = 1970.0;
    public static double LUT_TPS_4 = 2015.0;
    public static double LUT_TPS_5 = 2065.0;
    public static double LUT_TPS_6 = 2110.0;
    public static double LUT_TPS_7 = 0.0;

    public static double LUT_HOOD_1 = 0.70;   // NEW, close-shot hood angle (test)
    public static double LUT_HOOD_2 = 0.70;   // NEW, close-shot hood angle (test)
    public static double LUT_HOOD_3 = 0.60;
    public static double LUT_HOOD_4 = 0.60;
    public static double LUT_HOOD_5 = 0.60;
    public static double LUT_HOOD_6 = 0.60;
    public static double LUT_HOOD_7 = 0.60;

    /*
     * Safety limits on any predicted TPS. With CLAMP_DISTANCE_TO_RANGE the
     * prediction is clamped into this window; without it, out-of-window
     * predictions are rejected.
     */
    public static double REGRESSION_MIN_TPS = 1200.0;
    public static double REGRESSION_MAX_TPS = 2600.0;

    public static double DISTANCE_TIMEOUT_SECONDS = 0.50;

    public enum RegressionStatus {
        DISABLED,
        SHOOTER_STOPPED,
        NO_DISTANCE,
        INVALID_DISTANCE,
        STALE_DISTANCE,
        INVALID_RANGE,
        BELOW_RANGE,
        ABOVE_RANGE,
        INVALID_MODEL,
        TPS_OUT_OF_RANGE,
        ACTIVE,
        ACTIVE_CLAMPED_LOW,
        ACTIVE_CLAMPED_HIGH
    }

    private RegressionStatus regressionStatus = RegressionStatus.DISABLED;

    /** Result of evaluating the distance model. */
    public static class Prediction {
        public final boolean valid;
        public final double tps;
        public final double hood;
        public final RegressionStatus status;

        Prediction(boolean valid, double tps, double hood, RegressionStatus status) {
            this.valid = valid;
            this.tps = tps;
            this.hood = hood;
            this.status = status;
        }

        static Prediction invalid(RegressionStatus status) {
            return new Prediction(false, Double.NaN, Double.NaN, status);
        }
    }

    /** Closed-loop velocity controller for the shooter flywheel. */
    private static class Loop {
        private final DcMotorEx motor;
        private double targetTPS;
        private double signedRawTPS;
        private double rawTPS;
        private double currentTPS;
        private double errorTPS;
        private double power;
        private boolean primed;
        private boolean controllerValid;

        Loop(DcMotorEx motor) {
            this.motor = motor;
        }

        void setTargetTPS(double target) {
            targetTPS = Double.isFinite(target) ? target : 0.0;
        }

        void update(double loopKF, double loopKP, double voltageScale, double filterAlpha) {
            controllerValid = false;
            signedRawTPS = motor.getVelocity();
            rawTPS = Math.abs(signedRawTPS);

            if (!Double.isFinite(rawTPS)
                    || !Double.isFinite(loopKF)
                    || !Double.isFinite(loopKP)
                    || !Double.isFinite(voltageScale)) {
                primed = false;
                currentTPS = 0.0;
                errorTPS = 0.0;
                commandZero();
                return;
            }

            if (!primed) {
                currentTPS = rawTPS;
                primed = true;
            } else {
                double alpha = Double.isFinite(filterAlpha)
                        ? clamp(filterAlpha, 0.0, 1.0) : 1.0;
                currentTPS += alpha * (rawTPS - currentTPS);
            }

            double wantedTPS = Math.abs(targetTPS);
            if (!Double.isFinite(wantedTPS) || wantedTPS <= 0.0) {
                errorTPS = 0.0;
                commandZero();
                return;
            }

            errorTPS = wantedTPS - currentTPS;
            double calculatedPower = loopKF * wantedTPS * voltageScale
                    + loopKP * errorTPS;

            if (!Double.isFinite(calculatedPower) || !Double.isFinite(MAX_POWER)) {
                commandZero();
                return;
            }

            calculatedPower = clamp(calculatedPower, 0.0, clamp(MAX_POWER, 0.0, 1.0));
            power = Math.signum(targetTPS) * calculatedPower;
            motor.setPower(power);
            controllerValid = true;
        }

        private void commandZero() {
            power = 0.0;
            motor.setPower(0.0);
        }

        void stop() {
            targetTPS = 0.0;
            signedRawTPS = 0.0;
            rawTPS = 0.0;
            currentTPS = 0.0;
            errorTPS = 0.0;
            power = 0.0;
            primed = false;
            controllerValid = false;
            motor.setPower(0.0);
        }

        boolean atSpeed(double toleranceTPS) {
            return primed
                    && controllerValid
                    && targetTPS != 0.0
                    && Double.isFinite(toleranceTPS)
                    && Math.abs(Math.abs(targetTPS) - currentTPS) <= Math.abs(toleranceTPS);
        }

        double getMotorPower() {
            return motor.getPower();
        }
    }

    private Loop shooter;
    private DcMotorEx feederMotor;
    private Servo hood;
    private VoltageSensor voltageSensor;

    // Feeder state: requested power and last power sent to the motor
    private double feederRequested;
    private double commandedFeederPower;

    private double filteredVoltage;
    private double voltageScale = 1.0;
    private boolean voltagePrimed;
    private long lastVoltageSampleNanos;

    private double hoodBase = HOOD_HOME;
    private double fixedHoodBase = HOOD_HOME;
    private double hoodCommanded = HOOD_HOME;
    private double hoodCompensation;

    private double fixedTargetTPS;
    private double goalDistance = Double.NaN;
    private long distanceUpdatedNanos;
    private boolean distanceReceived;
    private boolean usingDistanceRegression;

    public void init(HardwareMap hardwareMap) {
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, SHOOTER_MOTOR_NAME);
        feederMotor = hardwareMap.get(DcMotorEx.class, FEEDER_MOTOR_NAME);
        hood = hardwareMap.get(Servo.class, HOOD_SERVO_NAME);

        shooterMotor.setDirection(SHOOTER_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        feederMotor.setDirection(FEEDER_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        feederMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feederMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter = new Loop(shooterMotor);

        /*
         * Prefer the Control Hub's voltage sensor. The first entry in the
         * list can be the Expansion Hub, and if that hub drops out (it has
         * in the logs) every voltage read would wait on a dead hub.
         */
        voltageSensor = null;
        VoltageSensor firstSensor = null;
        for (Map.Entry<String, VoltageSensor> entry : hardwareMap.voltageSensor.entrySet()) {
            VoltageSensor sensor = entry.getValue();
            if (sensor == null) continue;
            if (firstSensor == null) firstSensor = sensor;
            String name = entry.getKey();
            if (name != null && name.toLowerCase().contains("control")) {
                voltageSensor = sensor;
                break;
            }
        }
        if (voltageSensor == null) voltageSensor = firstSensor;
        lastVoltageSampleNanos = 0L;

        double initialHood = Double.isFinite(HOOD_HOME) ? HOOD_HOME : 0.50;
        hoodBase = clampHood(initialHood);
        fixedHoodBase = hoodBase;
        hoodCommanded = hoodBase;
        hoodCompensation = 0.0;
        hood.setPosition(hoodCommanded);
        stopOuttake();
    }

    public void setTargetTPS(double targetTPS) {
        fixedTargetTPS = Double.isFinite(targetTPS) ? Math.max(0.0, targetTPS) : 0.0;
        usingDistanceRegression = false;
        if (shooter != null) shooter.setTargetTPS(fixedTargetTPS);
    }

    public void setGoalDistance(double distance) {
        goalDistance = distance;
        distanceUpdatedNanos = System.nanoTime();
        distanceReceived = Double.isFinite(distance) && distance >= 0.0;
    }

    // ---------------------------------------------------------------------
    // Distance model
    // ---------------------------------------------------------------------

    /**
     * Evaluates the active distance model (lookup table or straight line)
     * for a distance, without applying it. Safe to call for telemetry.
     */
    public Prediction predict(double distance) {
        if (!Double.isFinite(distance) || distance < 0.0) {
            return Prediction.invalid(RegressionStatus.INVALID_DISTANCE);
        }
        return USE_LOOKUP_TABLE ? predictFromTable(distance) : predictFromLine(distance);
    }

    private Prediction predictFromLine(double distance) {
        if (!Double.isFinite(SHOT_DISTANCE_MIN)
                || !Double.isFinite(SHOT_DISTANCE_MAX)
                || SHOT_DISTANCE_MIN < 0.0
                || SHOT_DISTANCE_MAX <= SHOT_DISTANCE_MIN) {
            return Prediction.invalid(RegressionStatus.INVALID_RANGE);
        }
        if (!Double.isFinite(TPS_INTERCEPT)
                || !Double.isFinite(TPS_SLOPE)
                || !Double.isFinite(HOOD_INTERCEPT)
                || !Double.isFinite(HOOD_SLOPE)) {
            return Prediction.invalid(RegressionStatus.INVALID_MODEL);
        }

        RegressionStatus status = RegressionStatus.ACTIVE;
        double d = distance;
        if (d < SHOT_DISTANCE_MIN) {
            if (!CLAMP_DISTANCE_TO_RANGE) return Prediction.invalid(RegressionStatus.BELOW_RANGE);
            d = SHOT_DISTANCE_MIN;
            status = RegressionStatus.ACTIVE_CLAMPED_LOW;
        } else if (d > SHOT_DISTANCE_MAX) {
            if (!CLAMP_DISTANCE_TO_RANGE) return Prediction.invalid(RegressionStatus.ABOVE_RANGE);
            d = SHOT_DISTANCE_MAX;
            status = RegressionStatus.ACTIVE_CLAMPED_HIGH;
        }

        return finishPrediction(
                TPS_INTERCEPT + TPS_SLOPE * d,
                HOOD_INTERCEPT + HOOD_SLOPE * d,
                status);
    }

    private Prediction predictFromTable(double distance) {
        double[] dist = {LUT_DIST_1, LUT_DIST_2, LUT_DIST_3, LUT_DIST_4,
                LUT_DIST_5, LUT_DIST_6, LUT_DIST_7};
        double[] tps = {LUT_TPS_1, LUT_TPS_2, LUT_TPS_3, LUT_TPS_4,
                LUT_TPS_5, LUT_TPS_6, LUT_TPS_7};
        double[] hoodPos = {LUT_HOOD_1, LUT_HOOD_2, LUT_HOOD_3, LUT_HOOD_4,
                LUT_HOOD_5, LUT_HOOD_6, LUT_HOOD_7};

        int n = LUT_POINTS;
        if (n < 2 || n > dist.length) {
            return Prediction.invalid(RegressionStatus.INVALID_MODEL);
        }

        for (int i = 0; i < n; i++) {
            if (!Double.isFinite(dist[i]) || !Double.isFinite(tps[i])
                    || !Double.isFinite(hoodPos[i]) || tps[i] <= 0.0) {
                return Prediction.invalid(RegressionStatus.INVALID_MODEL);
            }
            if (i > 0 && dist[i] <= dist[i - 1]) {
                return Prediction.invalid(RegressionStatus.INVALID_MODEL);
            }
        }

        if (distance <= dist[0]) {
            if (distance < dist[0] && !CLAMP_DISTANCE_TO_RANGE) {
                return Prediction.invalid(RegressionStatus.BELOW_RANGE);
            }
            return finishPrediction(tps[0], hoodPos[0],
                    distance < dist[0]
                            ? RegressionStatus.ACTIVE_CLAMPED_LOW
                            : RegressionStatus.ACTIVE);
        }

        if (distance >= dist[n - 1]) {
            if (distance > dist[n - 1] && !CLAMP_DISTANCE_TO_RANGE) {
                return Prediction.invalid(RegressionStatus.ABOVE_RANGE);
            }
            return finishPrediction(tps[n - 1], hoodPos[n - 1],
                    distance > dist[n - 1]
                            ? RegressionStatus.ACTIVE_CLAMPED_HIGH
                            : RegressionStatus.ACTIVE);
        }

        for (int i = 1; i < n; i++) {
            if (distance <= dist[i]) {
                double t = (distance - dist[i - 1]) / (dist[i] - dist[i - 1]);
                return finishPrediction(
                        tps[i - 1] + t * (tps[i] - tps[i - 1]),
                        hoodPos[i - 1] + t * (hoodPos[i] - hoodPos[i - 1]),
                        RegressionStatus.ACTIVE);
            }
        }

        return Prediction.invalid(RegressionStatus.INVALID_MODEL);
    }

    private Prediction finishPrediction(double tps, double hoodPos, RegressionStatus status) {
        if (!Double.isFinite(tps) || tps <= 0.0 || !Double.isFinite(hoodPos)
                || !Double.isFinite(REGRESSION_MIN_TPS)
                || !Double.isFinite(REGRESSION_MAX_TPS)
                || REGRESSION_MIN_TPS <= 0.0
                || REGRESSION_MAX_TPS < REGRESSION_MIN_TPS) {
            return Prediction.invalid(RegressionStatus.INVALID_MODEL);
        }

        if (tps < REGRESSION_MIN_TPS || tps > REGRESSION_MAX_TPS) {
            if (!CLAMP_DISTANCE_TO_RANGE) {
                return Prediction.invalid(RegressionStatus.TPS_OUT_OF_RANGE);
            }
            tps = clamp(tps, REGRESSION_MIN_TPS, REGRESSION_MAX_TPS);
        }

        return new Prediction(true, tps, clampHood(hoodPos), status);
    }

    private void updateShotTargets() {
        if (shooter == null) return;

        usingDistanceRegression = false;
        shooter.setTargetTPS(fixedTargetTPS);
        hoodBase = clampHood(fixedHoodBase);

        if (!DISTANCE_REGRESSION_ENABLED) {
            regressionStatus = RegressionStatus.DISABLED;
            return;
        }
        if (fixedTargetTPS <= 0.0) {
            regressionStatus = RegressionStatus.SHOOTER_STOPPED;
            return;
        }
        if (!distanceReceived) {
            regressionStatus = RegressionStatus.NO_DISTANCE;
            return;
        }
        if (!Double.isFinite(goalDistance) || goalDistance < 0.0) {
            regressionStatus = RegressionStatus.INVALID_DISTANCE;
            return;
        }

        double ageSeconds = (System.nanoTime() - distanceUpdatedNanos) / 1_000_000_000.0;
        if (!Double.isFinite(ageSeconds)
                || !Double.isFinite(DISTANCE_TIMEOUT_SECONDS)
                || DISTANCE_TIMEOUT_SECONDS <= 0.0
                || ageSeconds > DISTANCE_TIMEOUT_SECONDS) {
            regressionStatus = RegressionStatus.STALE_DISTANCE;
            return;
        }

        Prediction p = predict(goalDistance);
        regressionStatus = p.status;
        if (!p.valid) return;

        shooter.setTargetTPS(p.tps);
        hoodBase = p.hood;
        usingDistanceRegression = true;
    }

    // ---------------------------------------------------------------------
    // Feeder (open loop, full power when on)
    // ---------------------------------------------------------------------

    /** Turns the feeder on (forward, feederPower) or off. */
    public void setFeederOn(boolean on) {
        shooterPower(on ? feederOnPower() : 0.0);
    }

    /** Runs the feeder backwards at feederPower, e.g. to clear a jam. */
    public void setFeederReverse() {
        shooterPower(-feederOnPower());
    }

    public void stopFeeder() {
        shooterPower(0.0);
    }

    public void shooterPower(double power) {
        feederRequested = Double.isFinite(power) ? clamp(power, -1.0, 1.0) : 0.0;
        applyFeederPower();
    }

    private double feederOnPower() {
        return Double.isFinite(feederPower) ? clamp(Math.abs(feederPower), 0.0, 1.0) : 1.0;
    }

    private void applyFeederPower() {
        if (feederMotor == null) return;
        // Only write to the hub when the value changes, to save loop time
        if (feederRequested != commandedFeederPower) {
            commandedFeederPower = feederRequested;
            feederMotor.setPower(commandedFeederPower);
        }
    }

    public boolean isFeederOn() { return commandedFeederPower != 0.0; }

    // ---------------------------------------------------------------------
    // Hood and update
    // ---------------------------------------------------------------------

    public void setHoodPosition(double position) {
        if (!Double.isFinite(position)) return;
        fixedHoodBase = clampHood(position);
        hoodBase = fixedHoodBase;
        if (shooter == null || shooter.targetTPS <= 0.0) setHoodServoPosition(hoodBase);
    }

    public void setHoodHome() {
        setHoodPosition(HOOD_HOME);
    }

    /** Updates shot selection, voltage, shooter control and hood only. */
    public void updateShooter() {
        if (shooter == null) return;
        updateShotTargets();
        updateVoltageScale();
        shooter.update(kF, kP, voltageScale, VELOCITY_FILTER_ALPHA);
        updateHood();
    }

    /** Re-applies the requested feeder power. */
    public void updateFeeder() {
        applyFeederPower();
    }

    /** Compatibility wrapper for autonomous and existing OpModes. */
    public void update() {
        updateShooter();
        updateFeeder();
    }

    public void updatePF() {
        update();
    }

    public void stopOuttake() {
        fixedTargetTPS = 0.0;
        usingDistanceRegression = false;
        regressionStatus = DISTANCE_REGRESSION_ENABLED
                ? RegressionStatus.SHOOTER_STOPPED : RegressionStatus.DISABLED;
        goalDistance = Double.NaN;
        distanceReceived = false;
        distanceUpdatedNanos = 0L;
        if (shooter != null) shooter.stop();
        feederRequested = 0.0;
        commandedFeederPower = 0.0;
        if (feederMotor != null) feederMotor.setPower(0.0);
        voltagePrimed = false;
        voltageScale = 1.0;
        filteredVoltage = 0.0;
        hoodBase = hoodCommanded;
        hoodCompensation = 0.0;
    }

    private void updateHood() {
        if (hood == null) return;
        hoodCompensation = 0.0;

        if (HOOD_COMP_ENABLED
                && shooter != null
                && shooter.controllerValid
                && shooter.targetTPS > 0.0
                && Double.isFinite(HOOD_COMP_GAIN)
                && Double.isFinite(HOOD_COMP_DEADBAND_TPS)
                && Double.isFinite(HOOD_COMP_MAX)) {
            double errorTPS = shooter.currentTPS - shooter.targetTPS;
            double deadband = Math.abs(HOOD_COMP_DEADBAND_TPS);
            if (Math.abs(errorTPS) > deadband) {
                double beyondDeadband = Math.signum(errorTPS) * (Math.abs(errorTPS) - deadband);
                double normalizedError = beyondDeadband / Math.max(1.0, shooter.targetTPS);
                hoodCompensation = clamp(
                        HOOD_COMP_GAIN * normalizedError,
                        -Math.abs(HOOD_COMP_MAX),
                        Math.abs(HOOD_COMP_MAX));
            }
        }

        setHoodServoPosition(hoodBase + hoodCompensation);
    }

    private void setHoodServoPosition(double position) {
        if (hood == null || !Double.isFinite(position)) return;
        double clipped = clampHood(position);
        double minimumStep = Double.isFinite(HOOD_MIN_STEP) ? Math.abs(HOOD_MIN_STEP) : 0.0;
        if (Math.abs(clipped - hoodCommanded) >= minimumStep) {
            hood.setPosition(clipped);
            hoodCommanded = clipped;
        }
    }

    private double clampHood(double position) {
        double minimum = Double.isFinite(HOOD_MIN) ? clamp(HOOD_MIN, 0.0, 1.0) : 0.40;
        double maximum = Double.isFinite(HOOD_MAX) ? clamp(HOOD_MAX, 0.0, 1.0) : 0.80;
        return clamp(position, Math.min(minimum, maximum), Math.max(minimum, maximum));
    }

    private void updateVoltageScale() {
        if (!VOLTAGE_COMP_ENABLED || voltageSensor == null) {
            voltageScale = 1.0;
            voltagePrimed = false;
            return;
        }

        // Only sample the battery every voltageSampleIntervalMs; between
        // samples keep using the current voltageScale.
        long now = System.nanoTime();
        double intervalNanos = Double.isFinite(voltageSampleIntervalMs)
                ? Math.max(0.0, voltageSampleIntervalMs) * 1_000_000.0 : 0.0;
        if (voltagePrimed && (now - lastVoltageSampleNanos) < intervalNanos) {
            return;
        }
        lastVoltageSampleNanos = now;

        double measured = voltageSensor.getVoltage();
        if (!Double.isFinite(measured)
                || measured <= 0.0
                || !Double.isFinite(MIN_VOLTAGE)
                || !Double.isFinite(MAX_VOLTAGE)
                || MIN_VOLTAGE <= 0.0
                || MAX_VOLTAGE < MIN_VOLTAGE
                || !Double.isFinite(NOMINAL_VOLTAGE)
                || NOMINAL_VOLTAGE <= 0.0) {
            voltageScale = 1.0;
            voltagePrimed = false;
            return;
        }

        measured = clamp(measured, MIN_VOLTAGE, MAX_VOLTAGE);
        if (!voltagePrimed) {
            filteredVoltage = measured;
            voltagePrimed = true;
        } else {
            double alpha = Double.isFinite(VOLTAGE_FILTER_ALPHA)
                    ? clamp(VOLTAGE_FILTER_ALPHA, 0.0, 1.0) : 1.0;
            filteredVoltage += alpha * (measured - filteredVoltage);
        }

        double minimumScale = Double.isFinite(MIN_VOLTAGE_SCALE)
                ? Math.max(0.0, MIN_VOLTAGE_SCALE) : 0.80;
        double maximumScale = Double.isFinite(MAX_VOLTAGE_SCALE)
                ? Math.max(minimumScale, MAX_VOLTAGE_SCALE) : 1.20;
        voltageScale = clamp(NOMINAL_VOLTAGE / filteredVoltage, minimumScale, maximumScale);
    }

    // ---------------------------------------------------------------------
    // Getters
    // ---------------------------------------------------------------------

    /** True when the shooter is within READY_TOLERANCE_TPS of its target. */
    public boolean isReady() {
        return isAtSpeed(READY_TOLERANCE_TPS);
    }

    public boolean isUsingDistanceRegression() { return usingDistanceRegression; }
    public RegressionStatus getRegressionStatus() { return regressionStatus; }
    public double getGoalDistance() { return goalDistance; }
    public double getFixedTargetTPS() { return fixedTargetTPS; }

    public boolean isAtSpeed(double toleranceTPS) {
        return shooter != null && shooter.atSpeed(toleranceTPS);
    }
    public double getTargetTPS() { return shooter == null ? 0.0 : shooter.targetTPS; }
    public double getCurrentTPS() { return shooter == null ? 0.0 : shooter.currentTPS; }
    public double getRawTPS() { return shooter == null ? 0.0 : shooter.rawTPS; }
    public double getSignedRawTPS() { return shooter == null ? 0.0 : shooter.signedRawTPS; }
    public double getErrorTPS() { return shooter == null ? 0.0 : shooter.errorTPS; }
    public double getCommandedPower() { return shooter == null ? 0.0 : shooter.power; }

    public double getFeederCommandedPower() { return commandedFeederPower; }

    public double getFixedHoodBase() { return fixedHoodBase; }
    public double getHoodBase() { return hoodBase; }
    public double getHoodPosition() { return hoodCommanded; }
    public double getHoodCompensation() { return hoodCompensation; }
    public double getVoltage() { return filteredVoltage; }
    public double getVoltageScale() { return voltageScale; }

    public double getFrontMotorPower() { return shooter == null ? 0.0 : shooter.getMotorPower(); }
    public double getBackMotorPower() { return feederMotor == null ? 0.0 : feederMotor.getPower(); }
    public double getFrontVelocityTPS() { return getCurrentTPS(); }

    @Deprecated public boolean isFeederAtSpeed(double toleranceTPS) { return isFeederOn(); }
    @Deprecated public double getFeederTargetTPS() { return 0.0; }
    @Deprecated public double getFeederCurrentTPS() { return 0.0; }
    @Deprecated public double getFeederRawTPS() { return 0.0; }
    @Deprecated public double getFeederSignedRawTPS() { return 0.0; }
    @Deprecated public double getFeederErrorTPS() { return 0.0; }
    @Deprecated public double getBackVelocityTPS() { return 0.0; }

    private static double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }
}