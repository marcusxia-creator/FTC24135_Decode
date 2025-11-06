package org.firstinspires.ftc.teamcode.TeleOps;

import android.graphics.Color;

import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Manages the detection and stabilization of ball colors.
 * This class uses a stability counter and timeout to provide reliable, debounced color readings.
 * It requires being called repeatedly in a loop via the updateDetection() method.
 */
public class ColorDetection {
    // Public-facing variable for the final detected color
    private BallColor stableColor = BallColor.UNKNOWN;

    // Internal state variables
    private BallColor lastReadColor = BallColor.UNKNOWN;
    private int stableCount = 0;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean isDetectionRunning = false;

    // Hardware and Configuration
    private final RobotHardware robot;
    private static final int SENSOR_GAIN = 8;
    private final int REQUIRED_STABLE_COUNT = 5; // number of consistent readings to lock a color
    private final double TIMEOUT_S = 1.0;        // maximum time to wait before giving up

    public ColorDetection(RobotHardware robot) {
        this.robot = robot;
    }

    //================================================================================
    //                                  Main Methods
    //================================================================================

    /**
     * Resets the detector and starts the detection process.
     */
    public void startDetection() {
        this.isDetectionRunning = true;
        this.stableColor = BallColor.UNKNOWN;
        this.lastReadColor = BallColor.UNKNOWN;
        this.stableCount = 0;
        this.timer.reset();
    }

    /**
     * Call this method in a loop to run the color detection logic.
     * It will update the stableColor based on consistent sensor readings.
     */
    public void updateDetection() {
        // Only run the logic if detection has been started
        if (!isDetectionRunning) {
            return;
        }

        // --- Timeout Check ---
        // If it takes too long to find a stable color, stop the process.
        if (timer.seconds() > TIMEOUT_S) {
            this.stableColor = BallColor.UNKNOWN;
            this.isDetectionRunning = false; // Stop trying
            return;
        }

        // --- Ball Presence Check ---
        // Only proceed if a ball is physically present.
        if (!isBallPresent()) {
            // If ball is removed mid-detection, reset the stability counter but keep trying until timeout.
            this.stableCount = 0;
            this.lastReadColor = BallColor.UNKNOWN;
            return;
        }

        // --- Read and Stabilize Color ---
        BallColor currentColor = getSensorColor();

        if (currentColor == lastReadColor && currentColor.isKnown()) {
            // If the color reading is the same as the last one and is a known color...
            stableCount++;
        } else {
            // If the color changes or is UNKNOWN, reset the stability counter.
            stableCount = 0;
        }

        // Update the last seen color for the next loop iteration.
        this.lastReadColor = currentColor;

        // --- Lock-in Logic ---
        if (stableCount >= REQUIRED_STABLE_COUNT) {
            // We have seen the same color enough times consecutively. Lock it in.
            this.stableColor = currentColor;
            this.isDetectionRunning = false; // Detection is complete, stop the process.
        }
    }

    //================================================================================
    //                                  Getters & Helpers
    //================================================================================

    /**
     * Returns true if a stable color has been successfully locked in.
     */
    public boolean isColorStable() {
        return stableColor.isKnown();
    }

    /**
     * Returns the last known stable color.
     */
    public BallColor getStableColor() {
        return stableColor;
    }

    /**
     * Checks if a ball is physically present in front of the sensor.
     */
    public boolean isBallPresent() {
        double distance = robot.distanceSensor.getDistance(DistanceUnit.MM);
        return distance < RobotActionConfig.BALL_PRESENT_THRESHOLD_MM;
    }

    /**
     * Reads the RGB values, applies gain, and converts to HSV to determine the color.
     * @return The immediate BallColor seen by the sensor.
     */
    private BallColor getSensorColor() {
        float[] hsv = getHsvValues();
        float hue = hsv[0];
        float value = hsv[2]; // Brightness
        return BallColor.fromHue(hue);
    }

    /**
     * Helper method to get the raw HSV values from the sensor.
     */
    private float[] getHsvValues() {
        float[] hsv = new float[3];
        Color.RGBToHSV(
                robot.colorSensor.red() * SENSOR_GAIN,
                robot.colorSensor.green() * SENSOR_GAIN,
                robot.colorSensor.blue() * SENSOR_GAIN,
                hsv
        );
        return hsv;
    }

    /** Returns the raw hue value (useful for telemetry or tuning). */
    public float getHue() {
        return getHsvValues()[0];
    }

    /**Returns raw RGB readings for diagnostics.*/
    public String getRawRGB() {
        return "R:" + robot.colorSensor.red() + " G:" + robot.colorSensor.green() + " B:" + robot.colorSensor.blue();
    }

    /** Gets the raw distance reading from the sensor. */
    public double getDistance() {
        return robot.distanceSensor.getDistance(DistanceUnit.MM);
    }
}

