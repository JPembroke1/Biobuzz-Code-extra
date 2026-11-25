package org.firstinspires.ftc.teamcode.opModes;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/**
 * Lists every AprilTag Cluster the camera sees, with its SDK name, how much
 * of it was found, and its roll (|roll| < 90 = upright / scorable).
 * Also shows the camera's actual frame rate.
 */
@Configurable
@TeleOp(name = "Hive Cluster Test", group = "Test")
public class HiveClusterTest extends LinearOpMode {

    public static String CAMERA_NAME = "Webcam 1";

    /*
     * Must be a size the camera supports. If you get "does not support
     * requested resolution", the error lists the valid sizes; use one.
     */
    public static int CAMERA_WIDTH = 1280;
    public static int CAMERA_HEIGHT = 800;

    /* MJPEG is needed for high frame rates on the mono Arducam. */
    public static boolean USE_MJPEG = true;

    /* Higher = faster processing of the large frame, shorter detection range. */
    public static double DECIMATION = 3.0;

    @Override
    public void runOpMode() {
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        aprilTag.setDecimation((float) Math.max(1.0, DECIMATION));

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .setStreamFormat(USE_MJPEG
                        ? VisionPortal.StreamFormat.MJPEG
                        : VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .addProcessor(aprilTag)
                .build();

        while (opModeInInit()) {
            telemetry.addData("Camera", portal.getCameraState());
            telemetry.addData("Resolution", "%d x %d %s",
                    CAMERA_WIDTH, CAMERA_HEIGHT, USE_MJPEG ? "MJPEG" : "YUY2");
            telemetry.addLine("Point the camera at a HIVE cell, then press Start");
            telemetry.update();
            sleep(50);
        }

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            telemetry.addData("Camera", "%s, %.0f fps",
                    portal.getCameraState(), portal.getFps());
            telemetry.addData("Detections", detections.size());

            for (AprilTagDetection d : detections) {
                if (d instanceof AprilTagClusterDetection) {
                    AprilTagClusterDetection c = (AprilTagClusterDetection) d;
                    telemetry.addLine("\n==== CLUSTER: " + c.metadata.name);
                    telemetry.addData("Found", "%s%%", c.percentClusterFound);
                    if (c.ftcPose != null) {
                        telemetry.addData("Roll", "%.1f  (%s)", c.ftcPose.roll,
                                Math.abs(c.ftcPose.roll) < 90 ? "UPRIGHT / scorable" : "upside down");
                        telemetry.addData("Range/Bearing", "%.1f in, %.1f deg",
                                c.ftcPose.range, c.ftcPose.bearing);
                        telemetry.addData("PRY", "%.1f %.1f %.1f",
                                c.ftcPose.pitch, c.ftcPose.roll, c.ftcPose.yaw);
                    }
                } else if (d instanceof AprilTagSingleDetection) {
                    AprilTagSingleDetection s = (AprilTagSingleDetection) d;
                    telemetry.addLine(String.format("\n==== SINGLE: ID %d %s", s.id,
                            s.metadata != null ? s.metadata.name : "(unknown)"));
                }
            }
            telemetry.update();
            sleep(20);
        }
        portal.close();
    }
}