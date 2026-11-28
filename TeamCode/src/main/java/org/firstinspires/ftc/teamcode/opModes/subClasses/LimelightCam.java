package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * Limelight 3A version of the ArduCam subsystem (BIOBUZZ HIVE tracking).
 *
 * Same job and nearly the same public methods as ArduCam, with these
 * differences:
 *
 * - The Limelight does the AprilTag work on its own processor, so none of
 *   this costs Control Hub CPU. Exposure, gain, resolution, decimation,
 *   lens calibration and tag size are set per pipeline in the Limelight
 *   web UI (http://limelight.local:5801 or 172.29.0.1:5801 over USB),
 *   not in code.
 * - The Limelight has no "AprilTag cluster" support. HIVE clusters are
 *   built here from the visible member tags (IDs 30-45): every member tag
 *   of the same HIVE cell is grouped into one HiveDetection. Bearing and
 *   range are averaged over the group; pitch/roll/yaw come from the
 *   largest (clearest) tag in the group.
 * - The getters return this class's own TagDetection / HiveDetection
 *   instead of the SDK's AprilTagSingleDetection / AprilTagClusterDetection.
 *   Field names match ftcPose: bearing, range, pitch, roll, yaw
 *   (degrees and inches, bearing positive = LEFT, as in the FTC SDK).
 *   So code that used "detection.ftcPose.bearing" becomes
 *   "detection.bearing".
 * - There is no Panels camera stream processor. The Limelight streams its
 *   own video at port 5800; Panels' Limelight proxy plugin can show it.
 *
 * Limelight pipeline setup (web UI):
 *   - Pipeline type: Fiducial / AprilTag, family 36h11.
 *   - Tag size: the BIOBUZZ member-tag size in mm.
 *   - "Full 3D" ON (needed for range, pitch, roll, yaw).
 *   - Optional ID filter: 30-45 to ignore every other tag.
 *   - Calibrate the camera (Calibration tab) for accurate range and angles.
 *
 * Usage: call camera.update() once at the top of every loop. If you never
 * call it, the class refreshes itself automatically on each getter.
 */
@Configurable
public class LimelightCam {

    // ---------------------------------------------------------------------
    // Configuration
    // ---------------------------------------------------------------------

    /** Name of the Limelight in the robot configuration. */
    public static String hardwareName = "limelight";

    /** Limelight pipeline to run (the AprilTag pipeline). */
    public static int pipelineIndex = 0;

    /** How often the SDK polls the Limelight for results. */
    public static int pollRateHz = 100;

    /** Results older than this are discarded (Limelight stalled or unplugged). */
    public static long detectionMaxAgeMs = 300;

    /** How often to fetch status (fps, temperature). It is an HTTP call, so keep it slow. */
    public static long statusIntervalMs = 1000;

    // ---------------------------------------------------------------------
    // HIVE state configuration (counts CAMERA FRAMES, not OpMode loops)
    // ---------------------------------------------------------------------

    public static int hiveStateFramesToConfirm = 5;

    public static int tipFramesToConfirm = 3;

    /*
     * RE-RECORD THESE for the Limelight. Its angles come from a different
     * camera and a different pose solver than the Arducam/SDK, so the
     * ArduCam thresholds may not carry over. Use getVisibleClusterDetails()
     * in telemetry with the HIVE in both states.
     */
    public static double upCellPitchThresholdDeg = 35.0;

    public static double tippedPitchThresholdDeg = 35.0;

    // ---------------------------------------------------------------------
    // Turret aiming
    // ---------------------------------------------------------------------

    /**
     * Which of the alliance's HIVE cells the turret aims at: "FAR" (the
     * scoring side, away from the audience) or "AUDIENCE". "ANY" uses
     * getTurretDetection()'s sticky active/nearest choice. CHECK which
     * cluster sits at your goal and set this to match.
     */
    public static String aimCellSide = "FAR";

    /** Minimum member tags visible before the cluster is used for aiming. */
    public static int aimMinTags = 2;   // was 1: one tag alone can't locate the cluster centre

    // ---------------------------------------------------------------------
    // Diagnostics for telemetry
    // ---------------------------------------------------------------------

    public static String cameraStatus = "NOT INITIALIZED";

    // ---------------------------------------------------------------------
    // HIVE definitions
    // ---------------------------------------------------------------------

    public enum Alliance {
        RED,
        BLUE
    }

    public enum HiveSide {
        AUDIENCE,
        FAR
    }

    public enum HiveState {
        UP,
        DOWN,
        UNKNOWN
    }

    public static class HiveCell {

        public final Alliance alliance;
        public final HiveSide side;

        public HiveCell(Alliance alliance, HiveSide side) {
            this.alliance = alliance;
            this.side = side;
        }

        @Override
        public boolean equals(Object o) {
            if (!(o instanceof HiveCell)) return false;
            HiveCell other = (HiveCell) o;
            return alliance == other.alliance && side == other.side;
        }

        @Override
        public int hashCode() {
            return alliance.hashCode() * 31 + side.hashCode();
        }

        @Override
        public String toString() {
            return alliance + "_" + side;
        }
    }

    /**
     * One AprilTag seen by the Limelight. Angles in degrees, range in
     * inches. bearing is positive to the LEFT (FTC SDK convention; the
     * Limelight's tx is positive to the right). hasPose is false if the
     * pipeline's Full 3D is off, in which case range/pitch/roll/yaw are NaN.
     */
    public static class TagDetection {

        public final int id;
        public final double bearing;
        public final double elevation;
        public final double range;
        public final double pitch;
        public final double roll;
        public final double yaw;
        public final double area;
        public final boolean hasPose;

        TagDetection(int id, double bearing, double elevation, double range,
                     double pitch, double roll, double yaw, double area,
                     boolean hasPose) {
            this.id = id;
            this.bearing = bearing;
            this.elevation = elevation;
            this.range = range;
            this.pitch = pitch;
            this.roll = roll;
            this.yaw = yaw;
            this.area = area;
            this.hasPose = hasPose;
        }
    }

    /**
     * One HIVE cell, built from its visible member tags. Replaces
     * AprilTagClusterDetection. percentClusterFound = visible member tags
     * out of 4.
     */
    public static class HiveDetection {

        public final HiveCell cell;
        public final String name;
        public final double bearing;
        public final double range;
        public final double pitch;
        public final double roll;
        public final double yaw;
        public final int tagCount;
        public final double percentClusterFound;
        public final List<TagDetection> tags;

        HiveDetection(HiveCell cell, double bearing, double range,
                      double pitch, double roll, double yaw,
                      List<TagDetection> tags) {
            this.cell = cell;
            this.name = cell.alliance + " " + cell.side;
            this.bearing = bearing;
            this.range = range;
            this.pitch = pitch;
            this.roll = roll;
            this.yaw = yaw;
            this.tags = Collections.unmodifiableList(tags);
            this.tagCount = tags.size();
            this.percentClusterFound = 100.0 * tagCount / TAGS_PER_CELL;
        }

        /** True if the angles and range are usable (Full 3D on). */
        public boolean hasPose() {
            return Double.isFinite(range) && Double.isFinite(pitch);
        }
    }

    // ---------------------------------------------------------------------
    // Official BIOBUZZ HIVE member-tag ranges
    //
    // Red FAR:      30, 31, 32, 33
    // Red Audience: 34, 35, 36, 37
    // Blue Audience:38, 39, 40, 41
    // Blue FAR:     42, 43, 44, 45
    // (Names kept from ArduCam so existing references still compile.)
    // ---------------------------------------------------------------------

    public static final int RED_FAR_TAG_MIN = 30;
    public static final int RED_FAR_TAG_MAX = 33;

    public static final int RED_AUDIENCE_TAG_MIN = 34;
    public static final int RED_AUDIENCE_TAG_MAX = 37;

    public static final int BLUE_AUDIENCE_TAG_MIN = 38;
    public static final int BLUE_AUDIENCE_TAG_MAX = 41;

    public static final int BLUE_FAR_TAG_MIN = 42;
    public static final int BLUE_FAR_TAG_MAX = 45;

    private static final int TAGS_PER_CELL = 4;

    private static final double METERS_TO_INCHES = 39.3700787;

    public static HiveCell getHiveCellFromTagId(int tagId) {
        if (tagId >= RED_FAR_TAG_MIN && tagId <= RED_FAR_TAG_MAX) {
            return new HiveCell(Alliance.RED, HiveSide.FAR);
        }
        if (tagId >= RED_AUDIENCE_TAG_MIN && tagId <= RED_AUDIENCE_TAG_MAX) {
            return new HiveCell(Alliance.RED, HiveSide.AUDIENCE);
        }
        if (tagId >= BLUE_AUDIENCE_TAG_MIN && tagId <= BLUE_AUDIENCE_TAG_MAX) {
            return new HiveCell(Alliance.BLUE, HiveSide.AUDIENCE);
        }
        if (tagId >= BLUE_FAR_TAG_MIN && tagId <= BLUE_FAR_TAG_MAX) {
            return new HiveCell(Alliance.BLUE, HiveSide.FAR);
        }
        return null;
    }

    public static boolean isHiveMemberTag(int tagId) {
        return getHiveCellFromTagId(tagId) != null;
    }

    // ---------------------------------------------------------------------
    // Per-frame snapshot
    // ---------------------------------------------------------------------

    private List<TagDetection> cachedTags = Collections.emptyList();
    private List<TagDetection> cachedMemberTags = Collections.emptyList();
    private List<HiveDetection> cachedHives = Collections.emptyList();
    private HiveCell cachedMemberCell = null;

    private LLResult lastResult = null;
    private double lastResultTimestamp = Double.NaN;
    private long cachedAtNs = 0L;
    private long frameId = 0L;

    private boolean manualUpdateUsed = false;

    // Status (fps etc.), fetched every statusIntervalMs
    private long lastStatusNs = 0L;
    private double lastFps = 0.0;
    private double lastTempC = Double.NaN;
    private double lastLatencyMs = Double.NaN;

    // ---------------------------------------------------------------------
    // HIVE state tracking
    // ---------------------------------------------------------------------

    private HiveState confirmedState = HiveState.UNKNOWN;
    private HiveState candidateState = HiveState.UNKNOWN;
    private int candidateStateFrames = 0;
    private long lastStateFrameId = -1L;

    private boolean hiveTipDetected = false;
    private int hiveTipCandidateFrames = 0;
    private long lastTipFrameId = -1L;
    private Alliance hiveTipAlliance = null;

    // Turret target selection (sticky)
    private String turretTargetKey = null;

    // ---------------------------------------------------------------------
    // Hardware
    // ---------------------------------------------------------------------

    private final HardwareMap hardwareMap;
    private final String limelightName;

    private Limelight3A limelight;

    private boolean enabled = true;

    // ---------------------------------------------------------------------
    // Construction and initialization
    // ---------------------------------------------------------------------

    /** Uses the configured hardwareName ("limelight"). */
    public LimelightCam(HardwareMap hardwareMap) {
        this(hardwareMap, hardwareName);
    }

    public LimelightCam(HardwareMap hardwareMap, String limelightName) {
        if (hardwareMap == null) {
            throw new IllegalArgumentException("HardwareMap cannot be null");
        }
        if (limelightName == null || limelightName.trim().isEmpty()) {
            throw new IllegalArgumentException("Limelight name cannot be empty");
        }
        this.hardwareMap = hardwareMap;
        this.limelightName = limelightName;
    }

    public void init() {
        if (limelight != null) {
            return;
        }

        clearCaches();
        cachedAtNs = 0L;
        lastResultTimestamp = Double.NaN;

        limelight = hardwareMap.get(Limelight3A.class, limelightName);
        limelight.setPollRateHz(Math.max(1, pollRateHz));
        limelight.pipelineSwitch(pipelineIndex);
        if (enabled) {
            limelight.start();
        }

        cameraStatus = enabled ? "STARTED (pipeline " + pipelineIndex + ")" : "PAUSED";

        resetHiveState();
    }

    // ---------------------------------------------------------------------
    // Per-loop update
    // ---------------------------------------------------------------------

    /**
     * Optional. Call once at the top of every OpMode loop. After this has
     * been called once, getters read the cached snapshot without
     * refreshing, so one loop always sees one consistent frame.
     */
    public void update() {
        manualUpdateUsed = true;
        refreshFromLimelight();
        updateStatus();
        expireStaleCache();
    }

    private void prepare() {
        if (!manualUpdateUsed) {
            refreshFromLimelight();
        }
        expireStaleCache();
    }

    private void refreshFromLimelight() {
        if (!enabled || limelight == null) {
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null) {
            return;
        }

        // Only rebuild on a NEW camera frame. The SDK polls faster than the
        // camera runs, so the same frame can be returned several times.
        double ts = result.getTimestamp();
        if (result == lastResult || ts == lastResultTimestamp) {
            return;
        }
        lastResult = result;
        lastResultTimestamp = ts;

        if (!result.isValid()) {
            // Frame processed, no targets.
            clearCaches();
            cachedAtNs = System.nanoTime();
            frameId++;
            return;
        }

        if (result.getPipelineIndex() != pipelineIndex) {
            // Still switching pipelines: don't trust this frame.
            return;
        }

        // Age the snapshot from when the frame was taken, not when it
        // arrived here.
        lastLatencyMs = result.getCaptureLatency() + result.getTargetingLatency();
        long stalenessMs = Math.max(0L, result.getStaleness());
        cachedAtNs = System.nanoTime() - stalenessMs * 1_000_000L;
        frameId++;

        rebuildCaches(result.getFiducialResults());
    }

    private void updateStatus() {
        if (limelight == null) {
            cameraStatus = "NOT INITIALIZED";
            return;
        }
        long now = System.nanoTime();
        if (now - lastStatusNs < statusIntervalMs * 1_000_000L) {
            return;
        }
        lastStatusNs = now;

        try {
            if (!limelight.isConnected()) {
                cameraStatus = "NOT CONNECTED (check USB cable)";
                lastFps = 0.0;
                return;
            }
            LLStatus status = limelight.getStatus();
            if (status != null) {
                lastFps = status.getFps();
                lastTempC = status.getTemp();
            }
            cameraStatus = !enabled ? "PAUSED"
                    : limelight.isRunning() ? "RUNNING" : "CONNECTED, NOT RUNNING";
        } catch (RuntimeException e) {
            cameraStatus = "STATUS ERROR: " + e.getClass().getSimpleName();
        }
    }

    private void expireStaleCache() {
        if (cachedAtNs == 0L || cachedTags.isEmpty()) {
            return;
        }
        if (getDetectionAgeMs() > Math.max(1L, detectionMaxAgeMs)) {
            clearCaches();
        }
    }

    private void clearCaches() {
        cachedTags = Collections.emptyList();
        cachedMemberTags = Collections.emptyList();
        cachedHives = Collections.emptyList();
        cachedMemberCell = null;
    }

    private void rebuildCaches(List<LLResultTypes.FiducialResult> fiducials) {
        List<TagDetection> tags = new ArrayList<>();
        List<TagDetection> members = new ArrayList<>();
        HiveCell memberCell = null;

        if (fiducials != null) {
            for (LLResultTypes.FiducialResult f : fiducials) {
                TagDetection tag = toTagDetection(f);
                tags.add(tag);

                HiveCell cell = getHiveCellFromTagId(tag.id);
                if (cell != null) {
                    members.add(tag);
                    if (memberCell == null) memberCell = cell;
                }
            }
        }

        // Group member tags into HIVE cells.
        List<HiveDetection> hives = new ArrayList<>();
        List<HiveCell> seen = new ArrayList<>();
        for (TagDetection tag : members) {
            HiveCell cell = getHiveCellFromTagId(tag.id);
            if (seen.contains(cell)) continue;
            seen.add(cell);

            List<TagDetection> group = new ArrayList<>();
            for (TagDetection other : members) {
                if (cell.equals(getHiveCellFromTagId(other.id))) group.add(other);
            }
            hives.add(buildHive(cell, group));
        }

        cachedTags = Collections.unmodifiableList(tags);
        cachedMemberTags = Collections.unmodifiableList(members);
        cachedHives = Collections.unmodifiableList(hives);
        cachedMemberCell = memberCell;
    }

    private static TagDetection toTagDetection(LLResultTypes.FiducialResult f) {
        double bearing = -f.getTargetXDegrees();   // Limelight +right -> FTC +left
        double elevation = f.getTargetYDegrees();

        double range = Double.NaN;
        double pitch = Double.NaN;
        double roll = Double.NaN;
        double yaw = Double.NaN;
        boolean hasPose = false;

        Pose3D p = f.getTargetPoseCameraSpace();
        if (p != null && p.getPosition() != null) {
            Position pos = p.getPosition();
            double xIn = pos.unit.toInches(pos.x);
            double yIn = pos.unit.toInches(pos.y);
            double zIn = pos.unit.toInches(pos.z);
            double r = Math.sqrt(xIn * xIn + yIn * yIn + zIn * zIn);

            // All zeros means Full 3D is off for this pipeline.
            if (r > 1e-6 && Double.isFinite(r)) {
                range = r;
                YawPitchRollAngles o = p.getOrientation();
                if (o != null) {
                    pitch = o.getPitch(AngleUnit.DEGREES);
                    roll = o.getRoll(AngleUnit.DEGREES);
                    yaw = o.getYaw(AngleUnit.DEGREES);
                }
                hasPose = true;
            }
        }

        return new TagDetection(f.getFiducialId(), bearing, elevation, range,
                pitch, roll, yaw, f.getTargetArea(), hasPose);
    }

    /**
     * Bearing to the CENTRE of the cluster, even when only some of its four
     * tags are visible. The tags sit in a row in ID order (e.g. 30-31-32-33),
     * so their bearings fall on a line against their position in the row
     * (0-3). A straight-line fit through the visible tags gives the bearing
     * at position 1.5, the middle. A plain average of, say, tags 30 and 31
     * would be pulled toward that end of the cluster and aim off to one side.
     * Works from either side of the cell (the fit finds the direction).
     * With a single tag there's nothing to fit, so its own bearing is used.
     */
    private static double clusterCentreBearing(List<TagDetection> group, double average) {
        int n = group.size();
        if (n < 2) return average;

        double sumX = 0.0, sumY = 0.0, sumXX = 0.0, sumXY = 0.0;
        for (TagDetection t : group) {
            double x = Math.floorMod(t.id - RED_FAR_TAG_MIN, TAGS_PER_CELL);  // 0-3 in the row
            sumX += x;
            sumY += t.bearing;
            sumXX += x * x;
            sumXY += x * t.bearing;
        }
        double denom = n * sumXX - sumX * sumX;
        if (Math.abs(denom) < 1e-9) return average;   // same position twice (shouldn't happen)

        double slope = (n * sumXY - sumX * sumY) / denom;
        double intercept = (sumY - slope * sumX) / n;
        double centre = intercept + slope * (TAGS_PER_CELL - 1) / 2.0;
        return Double.isFinite(centre) ? centre : average;
    }

    private static HiveDetection buildHive(HiveCell cell, List<TagDetection> group) {
        double bearingSum = 0.0;
        double rangeSum = 0.0;
        int rangeCount = 0;
        TagDetection largest = null;

        for (TagDetection t : group) {
            bearingSum += t.bearing;
            if (t.hasPose) {
                rangeSum += t.range;
                rangeCount++;
                if (largest == null || t.area > largest.area) largest = t;
            }
        }

        double bearing = clusterCentreBearing(group, bearingSum / group.size());
        double range = rangeCount > 0 ? rangeSum / rangeCount : Double.NaN;

        // Angles from the clearest tag rather than an average, so angle
        // wrap-around (e.g. roll near +/-180) can't produce nonsense.
        double pitch = largest != null ? largest.pitch : Double.NaN;
        double roll = largest != null ? largest.roll : Double.NaN;
        double yaw = largest != null ? largest.yaw : Double.NaN;

        return new HiveDetection(cell, bearing, range, pitch, roll, yaw, group);
    }

    /**
     * Increasing ID of the newest processed camera frame. Compare with the
     * last value you used to act only on new frames.
     */
    public long getFrameId() {
        prepare();
        return frameId;
    }

    public double getDetectionAgeMs() {
        if (cachedAtNs == 0L) {
            return Double.POSITIVE_INFINITY;
        }
        return (System.nanoTime() - cachedAtNs) / 1_000_000.0;
    }

    // ---------------------------------------------------------------------
    // Diagnostics
    // ---------------------------------------------------------------------

    /** Limelight pipeline fps (updated every statusIntervalMs by update()). */
    public double getFps() {
        return lastFps;
    }

    /** Capture + targeting latency of the last frame, ms. */
    public double getLatencyMs() {
        return lastLatencyMs;
    }

    /** Limelight temperature, deg C (NaN until the first status read). */
    public double getTemperatureC() {
        return lastTempC;
    }

    public String getStatusText() {
        return cameraStatus;
    }

    // ---------------------------------------------------------------------
    // MegaTag2 robot pose (for relocalizing)
    // ---------------------------------------------------------------------

    /**
     * Robot pose on the field from MegaTag2, or null if no valid fix.
     * MegaTag2 uses YOUR heading (from odometry) and the tags for X/Y, so
     * it is far steadier than a tag-only pose. Needs a BIOBUZZ field map
     * uploaded to the Limelight and the camera's position on the robot set
     * in the web UI.
     *
     * The returned pose is in the Limelight's field frame (metres, origin
     * at field centre), NOT Pedro's frame: convert before setPose().
     *
     * @param robotYawDeg robot heading in degrees, in the Limelight's field frame
     */
    public Pose3D getMegaTag2Pose(double robotYawDeg) {
        if (!enabled || limelight == null) {
            return null;
        }
        limelight.updateRobotOrientation(robotYawDeg);
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()
                || result.getStaleness() > detectionMaxAgeMs) {
            return null;
        }
        return result.getBotpose_MT2();
    }

    // ---------------------------------------------------------------------
    // General detection access (cached snapshot)
    // ---------------------------------------------------------------------

    /** Every tag in the current frame. */
    public List<TagDetection> getDetections() {
        prepare();
        return enabled ? cachedTags : Collections.<TagDetection>emptyList();
    }

    /** Same as getDetections() (kept for ArduCam compatibility). */
    public List<TagDetection> getSingleDetections() {
        return getDetections();
    }

    /** HIVE cells built from member tags (replaces SDK cluster detections). */
    public List<HiveDetection> getClusterDetections() {
        prepare();
        return enabled ? cachedHives : Collections.<HiveDetection>emptyList();
    }

    public TagDetection getDetection(int tagId) {
        for (TagDetection detection : getDetections()) {
            if (detection.id == tagId) {
                return detection;
            }
        }
        return null;
    }

    /** Looks up a HIVE cell by name, e.g. "RED FAR" or "BLUE AUDIENCE". */
    public HiveDetection getClusterDetection(String clusterName) {
        if (clusterName == null) {
            return null;
        }
        String wanted = clusterName.trim().toUpperCase(Locale.US).replace('_', ' ');
        for (HiveDetection detection : getClusterDetections()) {
            if (detection.name.equals(wanted)) {
                return detection;
            }
        }
        return null;
    }

    public HiveDetection getHiveDetection(HiveCell cell) {
        if (cell == null) {
            return null;
        }
        for (HiveDetection detection : getClusterDetections()) {
            if (detection.cell.equals(cell)) {
                return detection;
            }
        }
        return null;
    }

    public List<String> getVisibleClusterNames() {
        List<String> names = new ArrayList<>();
        for (HiveDetection detection : getClusterDetections()) {
            names.add(detection.name);
        }
        return names;
    }

    /** One line per visible HIVE cell. Use this to record the up/down thresholds. */
    public List<String> getVisibleClusterDetails() {
        List<String> details = new ArrayList<>();
        for (HiveDetection d : getClusterDetections()) {
            details.add(String.format(Locale.US,
                    "%s | tags %d (%.0f%%) | bearing=%.1f range=%.1f pitch=%.1f roll=%.1f yaw=%.1f",
                    d.name, d.tagCount, d.percentClusterFound,
                    d.bearing, d.range, d.pitch, d.roll, d.yaw));
        }
        return details;
    }

    // ---------------------------------------------------------------------
    // Visible HIVE member tags
    // ---------------------------------------------------------------------

    public List<TagDetection> getVisibleHiveMemberTags() {
        prepare();
        return enabled ? cachedMemberTags : Collections.<TagDetection>emptyList();
    }

    public HiveCell getVisibleHiveCellFromMemberTags() {
        prepare();
        return enabled ? cachedMemberCell : null;
    }

    /** All visible HIVE cells (same as getClusterDetections()). */
    public List<HiveDetection> getHiveDetections() {
        return getClusterDetections();
    }

    // ---------------------------------------------------------------------
    // Immediate HIVE visibility
    // ---------------------------------------------------------------------

    /** True if any member tag of this alliance's HIVE is visible. */
    public boolean isHiveVisible(Alliance alliance) {
        if (alliance == null) {
            return false;
        }
        for (HiveDetection d : getClusterDetections()) {
            if (d.cell.alliance == alliance) {
                return true;
            }
        }
        return false;
    }

    // ---------------------------------------------------------------------
    // Active/upward HIVE CELL
    // ---------------------------------------------------------------------

    public HiveCell getActiveCell(Alliance alliance) {
        HiveDetection d = getActiveCellDetection(alliance);
        return d == null ? null : d.cell;
    }

    /**
     * The upward alliance CELL with the smallest absolute pitch inside
     * upCellPitchThresholdDeg.
     */
    public HiveDetection getActiveCellDetection(Alliance alliance) {
        if (alliance == null) {
            return null;
        }
        double threshold = validPositiveThreshold(upCellPitchThresholdDeg, 35.0);
        HiveDetection best = null;
        double bestPitch = Double.MAX_VALUE;

        for (HiveDetection d : getClusterDetections()) {
            if (d.cell.alliance != alliance || !d.hasPose()) continue;
            double pitch = Math.abs(d.pitch);
            if (pitch < threshold && pitch < bestPitch) {
                bestPitch = pitch;
                best = d;
            }
        }
        return best;
    }

    /** The nearest visible HIVE cell for the alliance, up or down. */
    public HiveDetection getAnyHiveDetection(Alliance alliance) {
        if (alliance == null) {
            return null;
        }
        HiveDetection best = null;
        double bestRange = Double.MAX_VALUE;

        for (HiveDetection d : getClusterDetections()) {
            if (d.cell.alliance != alliance || !d.hasPose()) continue;
            if (d.range >= 0.0 && d.range < bestRange) {
                bestRange = d.range;
                best = d;
            }
        }
        return best;
    }

    // ---------------------------------------------------------------------
    // Turret target selection (sticky)
    // ---------------------------------------------------------------------

    /**
     * The single HIVE cell the turret should track. STICKY: once chosen,
     * the same cell is returned while it stays visible. Otherwise the
     * active (upward) cell is preferred, then the nearest.
     */
    public HiveDetection getTurretDetection(Alliance alliance) {
        if (alliance == null) {
            return null;
        }
        double threshold = validPositiveThreshold(upCellPitchThresholdDeg, 35.0);

        HiveDetection bestActive = null;
        double bestPitch = Double.MAX_VALUE;
        HiveDetection bestNearest = null;
        double bestRange = Double.MAX_VALUE;

        for (HiveDetection d : getClusterDetections()) {
            if (d.cell.alliance != alliance
                    || !Double.isFinite(d.bearing)
                    || !Double.isFinite(d.range)) {
                continue;
            }

            if (d.name.equals(turretTargetKey)) {
                return d;
            }

            double pitch = Math.abs(d.pitch);
            if (Double.isFinite(pitch) && pitch < threshold && pitch < bestPitch) {
                bestPitch = pitch;
                bestActive = d;
            }
            if (d.range >= 0.0 && d.range < bestRange) {
                bestRange = d.range;
                bestNearest = d;
            }
        }

        HiveDetection chosen = bestActive != null ? bestActive : bestNearest;
        if (chosen == null) {
            return null;
        }
        turretTargetKey = chosen.name;
        return chosen;
    }

    /**
     * The cluster the turret should aim at for this alliance (see
     * aimCellSide), or null if it isn't visible with enough tags.
     */
    public HiveDetection getAimDetection(Alliance alliance) {
        if (alliance == null) {
            return null;
        }
        String side = aimCellSide == null ? "FAR" : aimCellSide.trim().toUpperCase(Locale.US);

        HiveDetection d;
        if (side.equals("ANY")) {
            d = getTurretDetection(alliance);
        } else {
            HiveSide wanted = side.equals("AUDIENCE") ? HiveSide.AUDIENCE : HiveSide.FAR;
            d = getHiveDetection(new HiveCell(alliance, wanted));
        }

        if (d == null || !Double.isFinite(d.bearing) || d.tagCount < Math.max(1, aimMinTags)) {
            return null;
        }
        return d;
    }

    public String getTurretTargetKey() {
        return turretTargetKey;
    }

    public void resetTurretTarget() {
        turretTargetKey = null;
    }

    // ---------------------------------------------------------------------
    // HIVE up/down state
    // ---------------------------------------------------------------------

    private HiveState readRawState(Alliance alliance) {
        if (alliance == null) {
            return HiveState.UNKNOWN;
        }
        boolean sawAny = false;
        double threshold = validPositiveThreshold(upCellPitchThresholdDeg, 35.0);

        for (HiveDetection d : cachedHives) {
            if (d.cell.alliance != alliance || !d.hasPose()) continue;
            sawAny = true;
            if (Math.abs(d.pitch) < threshold) {
                return HiveState.UP;
            }
        }
        return sawAny ? HiveState.DOWN : HiveState.UNKNOWN;
    }

    /** Debounced HIVE state. Advances once per CAMERA FRAME. */
    public HiveState updateHiveState(Alliance alliance) {
        prepare();
        if (frameId == lastStateFrameId) {
            return confirmedState;
        }
        lastStateFrameId = frameId;

        HiveState raw = readRawState(alliance);
        if (raw == candidateState) {
            candidateStateFrames++;
        } else {
            candidateState = raw;
            candidateStateFrames = 1;
        }

        if (candidateStateFrames >= Math.max(1, hiveStateFramesToConfirm)
                && candidateState != HiveState.UNKNOWN) {
            confirmedState = candidateState;
        }
        return confirmedState;
    }

    public HiveState getHiveState() {
        return confirmedState;
    }

    // ---------------------------------------------------------------------
    // Latched generic HIVE-tip detection
    // ---------------------------------------------------------------------

    public boolean updateHiveTipDetected(Alliance alliance) {
        if (alliance == null || !enabled || limelight == null) {
            hiveTipCandidateFrames = 0;
            return hiveTipDetected;
        }
        prepare();
        if (frameId == lastTipFrameId) {
            return hiveTipDetected;
        }
        lastTipFrameId = frameId;

        boolean tippedThisFrame = false;
        double threshold = validPositiveThreshold(tippedPitchThresholdDeg, 35.0);

        for (HiveDetection d : cachedHives) {
            if (d.cell.alliance != alliance || !d.hasPose()) continue;
            if (Math.abs(d.pitch) < threshold) {
                tippedThisFrame = true;
                break;
            }
        }

        hiveTipCandidateFrames = tippedThisFrame ? hiveTipCandidateFrames + 1 : 0;

        if (hiveTipCandidateFrames >= Math.max(1, tipFramesToConfirm)) {
            hiveTipDetected = true;
            hiveTipAlliance = alliance;
        }
        return hiveTipDetected;
    }

    public boolean isHiveTipped() {
        return hiveTipDetected;
    }

    public boolean isHiveTipped(Alliance alliance) {
        return hiveTipDetected && alliance != null && hiveTipAlliance == alliance;
    }

    public void resetHiveTipDetection() {
        hiveTipDetected = false;
        hiveTipCandidateFrames = 0;
        lastTipFrameId = -1L;
        hiveTipAlliance = null;
    }

    public void updateHive(Alliance alliance) {
        updateHiveState(alliance);
        updateHiveTipDetected(alliance);
    }

    public void resetHiveState() {
        confirmedState = HiveState.UNKNOWN;
        candidateState = HiveState.UNKNOWN;
        candidateStateFrames = 0;
        lastStateFrameId = -1L;
        resetHiveTipDetection();
    }

    // ---------------------------------------------------------------------
    // Enable/disable
    // ---------------------------------------------------------------------

    /**
     * Pausing stops the Limelight's polling (it keeps running its pipeline
     * on its own processor, so this only saves Control Hub USB traffic).
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        clearCaches();
        resetHiveState();
        resetTurretTarget();

        if (limelight != null) {
            if (enabled) limelight.start();
            else limelight.pause();
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    /** Switches pipeline (e.g. a second pipeline for different lighting). */
    public void setPipeline(int index) {
        pipelineIndex = index;
        if (limelight != null) {
            limelight.pipelineSwitch(index);
        }
        clearCaches();
    }

    // ---------------------------------------------------------------------
    // Panels streaming (kept so existing calls still compile)
    // ---------------------------------------------------------------------

    /**
     * No-op. The Limelight streams its own video (port 5800). To see it in
     * Panels, enable Panels' Limelight proxy plugin.
     */
    public void streamToPanels() {
    }

    public void stopPanelsStream() {
    }

    // ---------------------------------------------------------------------
    // Status and control
    // ---------------------------------------------------------------------

    /** True if the Limelight is connected and running (replaces "camera STREAMING"). */
    public boolean isStreaming() {
        return limelight != null && limelight.isConnected() && limelight.isRunning();
    }

    public boolean isConnected() {
        return limelight != null && limelight.isConnected();
    }

    public void stopStreaming() {
        if (limelight != null) {
            limelight.pause();
        }
    }

    public void resumeStreaming() {
        if (limelight != null) {
            limelight.start();
        }
    }

    public void close() {
        if (limelight != null) {
            limelight.stop();
            limelight = null;
        }
        clearCaches();
        cachedAtNs = 0L;
        lastResult = null;
        lastResultTimestamp = Double.NaN;
        cameraStatus = "CLOSED";
        resetHiveState();
        resetTurretTarget();
    }

    /** The underlying SDK device, for anything not wrapped here. */
    public Limelight3A getLimelight() {
        return limelight;
    }

    // ---------------------------------------------------------------------
    // Utility
    // ---------------------------------------------------------------------

    private static double validPositiveThreshold(double configuredValue, double fallbackValue) {
        return Double.isFinite(configuredValue)
                ? Math.abs(configuredValue)
                : Math.abs(fallbackValue);
    }
}