package org.firstinspires.ftc.teamcode.sensor.vision;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;
import java.util.stream.Stream;

@Config
public class AprilTagSensor {
    public static final int STEP_SIZE = 100;
    public static final AprilTagLibrary library = getCenterStageTagLibrary();

    /**
     * Common calibrations can be found under <a href="https://github.com/marvelsatjiar/ftc-24064-2024/blob/75de9c200b0a6623cd4f7505753be39556633b36/TeamCode/src/main/res/xml/teamwebcamcalibrations.xml">teamwebcamcalibrations.xml</a>. This calibration was done with mrcal at 640x480 on a C920. More information about common calibrating tools can be found here: <a href="https://ftc-docs.firstinspires.org/en/latest/programming_resources/vision/camera_calibration/camera-calibration.html">Camera Calibration for FIRST Tech Challenge</a>
     */
    public static final double
            fx = 622.001,
            fy = 622.001,
            cx = 326.6183680,
            cy = 243.0307655;

    public final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTagProcessor;

    private int loops = STEP_SIZE;

    private final boolean isBackFacing;
    private final Vector2d offset;

    public AprilTagSensor(HardwareMap hardwareMap, boolean isBackFacing, Vector2d offset, String name) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(library)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(fx, fy, cx, cy)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, name))
                .enableLiveView(true)
                .addProcessor(aprilTagProcessor)
                .build();

        this.isBackFacing = isBackFacing;
        this.offset = offset;
    }

    public List<AprilTagDetection> getRawDetections() {
        return aprilTagProcessor.getDetections();
    }

    // Faze out goofy flips and ternaries and use camera orientation and tag orientation as to work
    // with non-CenterStage games
    public Pose2d getPoseEstimate(AprilTagDetection detection, double heading) {
        Pose2d estimate = null;
        if (detection.metadata != null) {
            boolean isAudienceSideTag = detection.metadata.id >= 7;
            // All headings will be normalized to [90, -90] degrees. Anything outside that range will
            // simply have 180 degrees added to it.
            heading = AngleUnit.normalizeRadians(heading);
            heading = PI / 2 < heading || -PI / 2 > heading ? PI + heading : heading;
            int audienceSideFlip = isAudienceSideTag ? -1 : 1;
            int facingFlip = isBackFacing ? -1 : 1;
            double x = -detection.ftcPose.y - abs(facingFlip * offset.y * sin(PI / 2 - heading));
            double y = detection.ftcPose.x - facingFlip * offset.y * sin(heading);
            double tagFacing = isAudienceSideTag ? (isBackFacing ? 0 : PI) : (isBackFacing ? PI : 0);

            VectorF tagVec = library.lookupTag(detection.id).fieldPosition;
            estimate = new Pose2d(
                    tagVec.get(0) + x * audienceSideFlip,
                    tagVec.get(1) + y * audienceSideFlip,
                    tagFacing + detection.ftcPose.yaw
            );
        }

        return estimate;
    }

    public Pose2d getPoseEstimate(double heading) {
        if (loops >= STEP_SIZE) {
            loops = 0;

            Supplier<Stream<Pose2d>> estimates = () -> getRawDetections().stream()
                    .map(detection -> getPoseEstimate(detection, heading))
                    .filter(Objects::nonNull);

            long count = estimates.get().count();

            if (count == 0) return null;

            Vector2d avgPosition = estimates.get()
                    .map(estimate -> estimate.position)
                    .reduce(new Vector2d(0, 0), Vector2d::plus)
                    .div(count);

            double avgHeading = estimates.get()
                    .map(estimate -> estimate.heading.toDouble())
                    .reduce(0.0, Double::sum) / count;

            return new Pose2d(avgPosition, avgHeading);
        }  else {
            loops++;
            return null;
        }
    }

    // Credit to @overkil of team 14343
    public static AprilTagLibrary getCenterStageTagLibrary() {
        return new AprilTagLibrary.Builder()
                .addTag(1, "BlueAllianceLeft",
                        2, new VectorF(61.75f, 41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(2, "BlueAllianceCenter",
                        2, new VectorF(61.75f, 35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(3, "BlueAllianceRight",
                        2, new VectorF(61.75f, 29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(4, "RedAllianceLeft",
                        2, new VectorF(61.75f, -29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(5, "RedAllianceCenter",
                        2, new VectorF(61.75f, -35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(6, "RedAllianceRight",
                        2, new VectorF(61.75f, -41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(7, "RedAudienceWallLarge",
                        5, new VectorF(-70.25f, -40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(8, "RedAudienceWallSmall",
                        2, new VectorF(-70.25f, -35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(9, "BlueAudienceWallSmall",
                        2, new VectorF(-70.25f, 35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(10, "BlueAudienceWallLarge",
                        5, new VectorF(-70.25f, 40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .build();
    }
}