package org.firstinspires.ftc.teamcode.teamcode.mechanism;

import static android.os.SystemClock.sleep;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;


public class AprilTagsWebCam {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    private Telemetry telemetry;

    private ExposureControl exposureControl;
    private GainControl gainControl;

    private static int RED_TAG_ID = 24;
    private static int BLUE_TAG_ID = 20;
    private static int PPG_TAG_ID = 23;

    private static int KNOWN_APRILTAG_ID_1 = RED_TAG_ID;
    private static final int KNOWN_APRILTAG_ID_2 = BLUE_TAG_ID;

    private static final long MIN_EXPOSURE_MS = 5;
    private static final long INITIAL_EXPOSURE_MS = 20;
    private static final long EXPOSURE_STEP_MS = 2;
    private static final int MAX_GAIN = 255;
    private static final int GAIN_STEP = 10;

    public void initialize(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        aprilTagProcessor.setDecimation(1);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                sleep(20);
            }

            exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            gainControl = visionPortal.getCameraControl(GainControl.class);

            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
        //init_loop();
    }

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
        if (detectedTags == null || detectedTags.isEmpty()) {
            refreshDetections();
        }
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public List<AprilTagDetection> getFreshDetections() {
        return aprilTagProcessor.getFreshDetections();
    }

    public void refreshDetections() {
        List<AprilTagDetection> tags = aprilTagProcessor.getDetections();
        if (tags == null) {
            // set detectedTags to empty list
            detectedTags = new ArrayList<>();
        } else {
            detectedTags = tags;
        }
    }


    public void displayDetectionTelemetry(AprilTagDetection detection) {
        if (detection == null) {
            return;
        }

        if (detection.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
        }
    }

    public AprilTagDetection getTagBySpecificId(int id) {
        for (AprilTagDetection tag : getDetectedTags()) {
            telemetry.addData("tag.id: ", tag.id);
            if (tag.id == id) {
                return tag;
            }
        }
        return null;
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    public void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }
        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
        sleep(20);
        gainControl.setGain(gain);
        sleep(20);
    }

    public void autoSetExposure(int timeoutMs) {
        boolean tagsFound = false;
        long currentExposure = exposureControl.getExposure(TimeUnit.MILLISECONDS);
        int currentGain = gainControl.getGain();

        ElapsedTime timer = new ElapsedTime();

        // Loop to adjust until tag is found
        while (!tagsFound) {
            // set a default exposure if we
            if (timer.milliseconds() > timeoutMs) {
                setManualExposure(6, 250);
                return;
            }

            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

            if (!currentDetections.isEmpty()) {
                for (AprilTagDetection detection : currentDetections) {
                    // Check if a known AprilTag ID is detected
                    if (detection.id == KNOWN_APRILTAG_ID_1 || detection.id == KNOWN_APRILTAG_ID_2) { // Replace with your known IDs
                        tagsFound = true;
                        telemetry.addData("AprilTag Found", "ID: %d", detection.id);
                        break; // Exit inner loop
                    }
                }
            }

            if (!tagsFound) {
                // Adjust exposure and/or gain
                // Example: Decrease exposure if no tag found, or increase gain
                // Implement a strategy to sweep through a range of values
                if (currentExposure > MIN_EXPOSURE_MS) { // Define MIN_EXPOSURE_MS
                    currentExposure -= EXPOSURE_STEP_MS; // Define EXPOSURE_STEP_MS
                    exposureControl.setExposure(currentExposure, TimeUnit.MILLISECONDS);
                } else if (currentGain < MAX_GAIN) { // Define MAX_GAIN
                    currentGain += GAIN_STEP; // Define GAIN_STEP
                    gainControl.setGain(currentGain);
                    currentExposure = INITIAL_EXPOSURE_MS; // Reset exposure if gain is adjusted
                    exposureControl.setExposure(currentExposure, TimeUnit.MILLISECONDS);
                } else {
                    // Exhausted adjustment range, break or log error
                    telemetry.addData("Warning", "Could not find AprilTag after adjusting camera settings.");
                    break;
                }
                sleep(50); // Short delay to allow camera to adjust
            }
            telemetry.update();
        }
    }


}
