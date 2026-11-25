// Pedro Pathing localizer backed by the BBR Digital Expander's onboard
// localizer (two dead-wheel pods + IMU, pose integrated at 1 kHz on the
// expander). Because the expander already returns a fused absolute pose,
// this class is a thin adapter — like Pedro's PinpointLocalizer and
// OTOSLocalizer — that unit-converts the expander's reading into a Pedro
// MotionState. No odometry math happens on the Control Hub.
//
// Written for Pedro Pathing 3.x. Wire it into a Follower with:
//     Follower follower = new Follower(
//             new BBRExpanderLocalizer(hardwareMap),
//             new Mecanum(hardwareMap, drivetrainConfig),
//             new Foresight(foresightConfig));
//
// UNITS / FRAME — the two things to check on the robot before trusting a path:
//   * The expander reports millimetres + radians; Pedro uses INCHES + radians.
//     This class converts (MM_PER_INCH).
//   * The expander and Pedro both use "X forward, Y left, heading CCW
//     positive", so the pose maps 1:1 after scaling. If forward, strafe or
//     turning reads backwards, fix it with the settings below (REVERSE_X,
//     REVERSE_Y, IMU_AXIS_UP) rather than negating values here.
//   * The expander reports velocity in the FIELD frame, which is what Pedro's
//     Velocity is; MotionState works out the robot-frame Twist from it.
package org.firstinspires.ftc.teamcode.pedro;

import com.buildingblockrobotics.expander.BBRDigitalExpander;
import com.buildingblockrobotics.expander.BBRRegMap;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BBRExpanderLocalizer implements Localizer {

    /** Name of the expander in your robot configuration. */
    public static final String HARDWARE_NAME = "expander";

    private static final double MM_PER_INCH = 25.4;

    // --- Pod tuning. MEASURE these on YOUR robot (push it a measured 2 m
    // along each axis and divide ticks by millimetres) rather than working
    // them out from gear ratios. The defaults are for goBILDA 4-bar pods.
    // They are sent to the expander when this class is constructed; the
    // expander's own localizer does the rest. ---
    public static double TICKS_PER_MM_X = 19.89;  // forward pod
    public static double TICKS_PER_MM_Y = 19.89;  // left/strafe pod
    public static int PORT_X = 0;                 // encoder channel: forward pod
    public static int PORT_Y = 1;                 // encoder channel: strafe pod

    // Where the pods sit relative to the point the robot turns about, in the
    // robot frame (X forward, Y left), in mm. Only each pod's SIDEWAYS offset
    // from that point matters:
    //   TCP_OFFSET_Y_MM = forward pod's left/right offset (left is +)
    //   TCP_OFFSET_X_MM = strafe pod's forward/back offset (forward is +)
    // To check them, spin the robot on the spot: the x and y of the pose
    // should stay put.
    //
    // Starting values converted from the old Pinpoint config in Constants
    // (24 Nov 2026), which used goBILDA's convention:
    //   Pinpoint xPodOffset 0.9063 in = forward pod's sideways offset
    //       -> TCP_OFFSET_Y_MM = 0.9063 * 25.4 = 23.02 mm
    //   Pinpoint yPodOffset 5.6224 in = strafe pod's forward offset
    //       -> TCP_OFFSET_X_MM = 5.6224 * 25.4 = 142.81 mm
    // CHECK with the spin-on-the-spot test: if x/y drift in a circle, try
    // flipping the sign of one or both.
    public static double TCP_OFFSET_X_MM = 142.81;
    public static double TCP_OFFSET_Y_MM = 23.02;

    // Which way up the expander is mounted (a BBRRegMap.AXIS_* value). Flat
    // and face up is AXIS_POS_Z; flat and face down is AXIS_NEG_Z; on its edge
    // it is one of the X or Y axes. Check it with the "BBR: IMU Heading"
    // OpMode: turning the robot 90 degrees to the left should read about +90.
    // Sent on every construction, so it holds even on a freshly flashed board.
    public static int IMU_AXIS_UP = BBRRegMap.AXIS_POS_Z;

    // Pod directions. Pushing the robot forward must raise "pose x", and
    // pushing it left must raise "pose y". If a pod counts backwards, set its
    // REVERSE flag to true.
    //
    // Starting guess from the old Pinpoint config (X FORWARD, Y REVERSED).
    // The expander may count the other way: CHECK with the push test.
    public static boolean REVERSE_X = false;
    public static boolean REVERSE_Y = true;

    // How long to wait for the expander's gyro calibration to finish.
    private static final long CALIBRATE_TIMEOUT_MS = 5000;

    private final BBRDigitalExpander exp;

    private MotionState motionState;

    // The last raw reading, kept so the TeleOp can check the localizer's
    // status and collision flag without a second I2C read.
    private BBRDigitalExpander.LocalizerState lastState;

    /** Starts at (0, 0, 0). */
    public BBRExpanderLocalizer(HardwareMap map) {
        this(map, Pose.zero());
    }

    public BBRExpanderLocalizer(HardwareMap map, Pose startPose) {
        exp = map.get(BBRDigitalExpander.class, HARDWARE_NAME);

        // Send the pod settings to the expander's localizer (RAM only).
        // Saving them to flash is the job of a setup OpMode such as
        // BBRLocalizerExample; sending them here means the localizer works
        // even if flash was never written.
        BBRDigitalExpander.LocalizerParams p = new BBRDigitalExpander.LocalizerParams();
        p.ticksPerMmX = TICKS_PER_MM_X;
        p.ticksPerMmY = TICKS_PER_MM_Y;
        p.portX = PORT_X;
        p.portY = PORT_Y;
        p.tcpOffsetXMm = TCP_OFFSET_X_MM;
        p.tcpOffsetYMm = TCP_OFFSET_Y_MM;
        exp.setLocalizerParams(p);

        // Tell the IMU which way is up before calibrating, and set the pod
        // directions.
        exp.setImuAxisUp(IMU_AXIS_UP);
        exp.setEncoderDirection(PORT_X, REVERSE_X
                ? BBRDigitalExpander.Direction.REVERSE : BBRDigitalExpander.Direction.FORWARD);
        exp.setEncoderDirection(PORT_Y, REVERSE_Y
                ? BBRDigitalExpander.Direction.REVERSE : BBRDigitalExpander.Direction.FORWARD);

        // Zero the pose and calibrate the gyro. The robot must be still
        // during init, as with Pinpoint and OTOS, which also calibrate when
        // they are constructed.
        reset();
        setPose(startPose);
        update();
    }

    @Override
    public void setPose(Pose pose) {
        exp.setPose(new BBRDigitalExpander.Pose2D(
                pose.x() * MM_PER_INCH,
                pose.y() * MM_PER_INCH,
                pose.heading()));

        if (motionState != null) {
            motionState = motionState.withPose(pose);
        } else {
            motionState = MotionState.ofVelocity(pose, Velocity.zero());
        }
    }

    /**
     * Reads a fresh pose from the expander. The expander tracks the pose
     * itself, so this only converts units. If a read fails as the OpMode is
     * stopping, the driver serves the last good pose (see isDataFresh()), so
     * this does not throw in the middle of a loop.
     */
    @Override
    public void update() {
        BBRDigitalExpander.LocalizerState s = exp.readLocalizerRaw();
        lastState = s;

        Pose pose = new Pose(
                s.xMm / MM_PER_INCH,
                s.yMm / MM_PER_INCH,
                s.headingRad);

        Velocity velocity = new Velocity(
                s.velXMmPerSec / MM_PER_INCH,
                s.velYMmPerSec / MM_PER_INCH,
                s.headingVelRadPerSec);

        motionState = MotionState.ofVelocity(pose, velocity);
    }

    @Override
    public MotionState state() {
        return motionState;
    }

    /** Zeros the pose and recalibrates the gyro. The robot must be still. */
    @Override
    public void reset() {
        exp.resetLocalizerAndCalibrateImu();
        if (!exp.waitForLocalizerReady(CALIBRATE_TIMEOUT_MS)) {
            throw new IllegalStateException(
                    "Expander localizer did not start within " + CALIBRATE_TIMEOUT_MS
                            + " ms. Keep the robot still during init and check the pod settings.");
        }
        motionState = MotionState.ofVelocity(Pose.zero(), Velocity.zero());
    }

    /**
     * The reading from the last update(), or null before the first one.
     * Use it for status (RUNNING or not) and hasGyroEverSaturated().
     */
    public BBRDigitalExpander.LocalizerState getLastState() {
        return lastState;
    }

    /** True if the last reading says the localizer is tracking. */
    public boolean isRunning() {
        return lastState != null
                && lastState.status == BBRDigitalExpander.LocalizerStatus.RUNNING;
    }

    /**
     * True once a collision has clipped the gyro since the last
     * resetLocalizerAndCalibrateImu(). The expander latches this, so it stays
     * true for the rest of the OpMode.
     */
    public boolean hasGyroEverSaturated() {
        return lastState != null && lastState.hasGyroEverSaturated();
    }

    /** The underlying driver, if you need its status or flags. */
    public BBRDigitalExpander getExpander() {
        return exp;
    }
}