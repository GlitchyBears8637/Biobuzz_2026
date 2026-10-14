// In RobotHdw/ColorSensorSubsystem.java
package org.firstinspires.ftc.teamcode.RobotHdw;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.graphics.Color;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.LinkedList;
import java.util.List;

/**
 * A subsystem to manage multiple color sensors and their corresponding indicator lamps.
 * This class handles sensor initialization, reading HSV values, identifying specific colors (Purple, Green),
 * and setting lamp positions accordingly.
 */
public class ColorSensorSubsystem {

    // --- Member Variables ---
    // Physical hardware devices
    private ColorSensor sensor1, sensor2, sensor3;
    private Servo lamp1, lamp2, lamp3, lamp4;
    // A queue to store the sequence of detected balls
    private LinkedList<DetectedColor> ballQueue = new LinkedList<>();
    // Timers for managing flashing animations
    private ElapsedTime flashTimer = new ElapsedTime();
    private boolean lampOn = false;
    // Timer and state for Lamp 4 flashing
    private ElapsedTime lamp4FlashTimer = new ElapsedTime();
    private boolean lamp4FlashState = false;

    // --- Debounce logic variables ---
    // Timers to track how long a sensor has been seeing NONE
    private final ElapsedTime[] debounceTimers = {new ElapsedTime(), new ElapsedTime(), new ElapsedTime()};
    // The last non-NONE color detected by each sensor
    private final DetectedColor[] lastDetectedColors = {DetectedColor.NONE, DetectedColor.NONE, DetectedColor.NONE};
    // How long to wait before changing a lamp from a color to NONE (in milliseconds)
    private static final long DEBOUNCE_DELAY_MS = 250;

    // HSV value arrays for each sensor, used for color conversion
    private float[] hsvValues1 = {0F, 0F, 0F};
    private float[] hsvValues2 = {0F, 0F, 0F};
    private float[] hsvValues3 = {0F, 0F, 0F};

    // Constant to scale RGB values from 0-1 to 0-255
    private final double SCALE_FACTOR = 255;

    // Enum to represent the possible detected colors for clarity and type safety
    public enum DetectedColor {
        PURPLE, GREEN, NONE
    }

    // Enum to represent the state of Lamps 1-3 and their corresponding servo positions
    private enum LampState {
        OFF(0),
        PURPLE(0.722),
        GREEN(0.5);

        public final double position;

        LampState(double position) {
            this.position = position;
        }
    }

    // Enum to represent the state of Lamp 4 and its corresponding servo positions
    private enum Lamp4State {
        OFF(0),
        RED(0.277),
        GREEN(0.5),
        BLUE(0.611); // For auto-aim without a target

        public final double position;

        Lamp4State(double position) {
            this.position = position;
        }
    }

    // --- Constructor ---
    public ColorSensorSubsystem(HardwareRobot robot) {
        // Initialize sensors from the HardwareRobot instance
        sensor1 = robot.sensorColor1;
        sensor2 = robot.sensorColor2;
        sensor3 = robot.sensorColor3;

        // Initialize lamps from the HardwareRobot instance
        lamp1 = robot.lamp1;
        lamp2 = robot.lamp2;
        lamp3 = robot.lamp3;
        lamp4 = robot.lamp4;

        // Reset timers on initialization
        flashTimer.reset();
        lamp4FlashTimer.reset();
        for (ElapsedTime timer : debounceTimers) {
            timer.reset();
        }
    }

    // --- Queue Management ---

    /**
     * Adds a detected ball color to the queue.
     * If the queue is full (size 3), it removes the oldest element before adding the new one.
     * @param color The detected color of the ball.
     */
    public void enqueueBall(DetectedColor color) {
        // We don't want to add "NONE" to our queue, as it's not a valid ball color
        if (color == DetectedColor.NONE) {
            return; // Do not add NONE to the queue
        }
        // If the queue is at capacity, remove the first (oldest) item
        if (ballQueue.size() >= 3) {
            ballQueue.removeFirst();
        }
        // Add the new color to the end of the queue
        ballQueue.add(color);
    }

    /**
     * Checks if the current sequence of balls in the queue matches the expected pattern.
     * The comparison is case-insensitive.
     * @param pattern The expected color pattern from PatternManager.
     * @return true if the sequence is correct, false otherwise.
     */
    public boolean isSequenceCorrect(List<String> pattern) {
        // If the number of balls in the queue doesn't match the pattern length, it can't be correct.
        if (ballQueue.size() != pattern.size()) {
            return false;
        }
        // Iterate through the queue and pattern to check for a match at each position
        for (int i = 0; i < ballQueue.size(); i++) {
            String patternColor = pattern.get(i).toUpperCase();
            String queueColor = ballQueue.get(i).toString();
            // If any color is mismatched, the sequence is incorrect
            if (!patternColor.equals(queueColor)) {
                return false;
            }
        }
        // If the loop completes without finding any mismatches, the sequence is correct
        return true;
    }

    /**
     * Clears the ball queue.
     */
    public void clearQueue() {
        ballQueue.clear();
    }

    // --- Core Methods ---

    /**
     * Reads the latest values from all sensors and converts them to the HSV color model.
     * This method should be called once per loop cycle.
     */
    public void updateSensorReadings() {
        // Convert the RGB sensor readings to HSV for more reliable color detection
        Color.RGBToHSV((int) (sensor1.red() * SCALE_FACTOR), (int) (sensor1.green() * SCALE_FACTOR), (int) (sensor1.blue() * SCALE_FACTOR), hsvValues1);
        Color.RGBToHSV((int) (sensor2.red() * SCALE_FACTOR), (int) (sensor2.green() * SCALE_FACTOR), (int) (sensor2.blue() * SCALE_FACTOR), hsvValues2);
        Color.RGBToHSV((int) (sensor3.red() * SCALE_FACTOR), (int) (sensor3.green() * SCALE_FACTOR), (int) (sensor3.blue() * SCALE_FACTOR), hsvValues3);
    }

    /**
     * Analyzes the HSV values from a specific sensor to determine the detected color.
     *
     * @param sensorNumber The sensor to check (1, 2, or 3).
     * @return The DetectedColor enum value (PURPLE, GREEN, or NONE).
     */
    public DetectedColor getDetectedColor(int sensorNumber) {
        float[] hsv;
        // Select the correct HSV values based on the sensor number
        switch (sensorNumber) {
            case 1: hsv = hsvValues1; break;
            case 2: hsv = hsvValues2; break;
            case 3: hsv = hsvValues3; break;
            default: return DetectedColor.NONE; // Return NONE for an invalid sensor number
        }

        float hue = hsv[0]; // Hue represents the color itself
        float value = hsv[2]; // Value represents the brightness

        // A high brightness value helps distinguish a color from black or shadow
        if (value > 300) { // This threshold may need tuning based on lighting conditions
            // Check if the hue falls within the range for Green
            if (hue >= 90 && hue <= 180) {
                return DetectedColor.GREEN;
                // Check if the hue falls within the range for Purple
            } else if (hue >= 220 && hue <= 290) {
                return DetectedColor.PURPLE;
            }
        }
        // If no color is detected or brightness is too low, return NONE
        return DetectedColor.NONE;
    }

    /**
     * Updates all lamp positions based on the current detected color of each sensor.
     * This method includes debounce logic to prevent flickering.
     * If no ball is detected, the lamp will flash the color of the expected pattern.
     * @param pattern The expected color pattern from PatternManager.
     */
    public void updateLamps(List<String> pattern) {
        // This timer creates a 500ms flashing effect for the lamps when no ball is present
        if (flashTimer.milliseconds() > 500) {
            lampOn = !lampOn; // Toggle the lamp state
            flashTimer.reset();
        }

        // --- Debounce Logic ---
        // Loop through each sensor to apply debounce logic to its readings.
        for (int i = 0; i < 3; i++) {
            // Get the real-time color from the sensor
            DetectedColor currentColor = getDetectedColor(i + 1);

            // If a color is detected, update the last seen color and reset its timer.
            if (currentColor != DetectedColor.NONE) {
                lastDetectedColors[i] = currentColor;
                debounceTimers[i].reset();
            } else {
                // If no color is seen, check if the debounce delay has passed.
                // This prevents flickering if the sensor briefly loses sight of the ball.
                if (debounceTimers[i].milliseconds() > DEBOUNCE_DELAY_MS) {
                    lastDetectedColors[i] = DetectedColor.NONE;
                }
            }
        }

        // Update each of the three primary lamps with the debounced color value.
        updateLamp(lamp1, lastDetectedColors[0], pattern, 0);
        updateLamp(lamp2, lastDetectedColors[1], pattern, 1);
        updateLamp(lamp3, lastDetectedColors[2], pattern, 2);
    }

    /**
     * Manages the state of the 4th lamp, which provides driver feedback based on a priority system.
     *
     * Lamp 4 Signal Priority:
     * 1. Auto-Aim Status (Highest)         - Solid GREEN (lock) or BLUE (searching).
     * 2. Queue Full (when Auto-Aim is off) - Flashes RED to alert the driver.
     * 3. Default (Lowest)                  - OFF.
     *
     * @param isQueueFull True if the intake queue is full.
     * @param isAutoAim   True if the robot is in auto-aim mode.
     * @param hasTarget   True if the Limelight has a valid target.
     */
    public void updateLamp4(boolean isQueueFull, boolean isAutoAim, boolean hasTarget) {
        // This timer creates a 500ms flashing effect for indicators.
        if (lamp4FlashTimer.milliseconds() > 500) {
            lamp4FlashState = !lamp4FlashState; // Toggle the flash state
            lamp4FlashTimer.reset();
        }

        Lamp4State desiredState;

        // 1. Highest Priority: Auto-Aim Status
        if (isAutoAim) {
            if (hasTarget) {
                // Solid GREEN indicates that auto-aim is active and has a lock on the target.
                desiredState = Lamp4State.GREEN;
            } else {
                // Solid BLUE indicates that auto-aim is active but currently has no target.
                desiredState = Lamp4State.BLUE;
            }
        }
        // 2. Second Priority: Queue Full (only when auto-aim is off)
        else if (isQueueFull) {
            // Flash RED to signal that the ball queue is full.
            // During the "on" cycle of the flash, the lamp is RED. Otherwise, it's OFF.
            desiredState = lamp4FlashState ? Lamp4State.RED : Lamp4State.OFF;
        }
        // 3. Default State
        else {
            // If no other conditions are met, the lamp is turned OFF.
            desiredState = Lamp4State.OFF;
        }

        // Set the physical lamp servo to the position determined by the desired state.
        lamp4.setPosition(desiredState.position);
    }

    /**
     * Helper method to determine the state of a single lamp (1-3) and set its position.
     * - Solid color for a detected ball.
     * - Flashing color for the expected pattern if no ball is present.
     * @param lamp The servo object for the lamp.
     * @param detectedColor The detected color from the sensor.
     * @param pattern The expected color pattern.
     * @param patternIndex The index in the pattern for this lamp.
     */
    private void updateLamp(Servo lamp, DetectedColor detectedColor, List<String> pattern, int patternIndex) {
        LampState desiredState = LampState.OFF;

        // If a ball is detected, show its color. This is the highest priority.
        if (detectedColor != DetectedColor.NONE) {
            if (detectedColor == DetectedColor.PURPLE) {
                desiredState = LampState.PURPLE;
            } else if (detectedColor == DetectedColor.GREEN) {
                desiredState = LampState.GREEN;
            }
        }
        // If no ball is detected, flash the expected pattern color.
        // This only happens during the "on" cycle of the main lamp flash timer.
        else if (lampOn && pattern != null && pattern.size() > patternIndex) {
            String expectedColor = pattern.get(patternIndex);
            if (expectedColor.equalsIgnoreCase("PURPLE")) {
                desiredState = LampState.PURPLE;
            } else if (expectedColor.equalsIgnoreCase("GREEN")) {
                desiredState = LampState.GREEN;
            }
        }
        // Otherwise, the lamp is off (either in the "off" cycle of the flash or if there's no pattern).

        // Set the physical lamp servo to the position determined by the desired state.
        setLampState(lamp, desiredState);
    }


    /**
     * Helper method to set a single lamp's position based on its determined state.
     * @param lamp The servo object for the lamp.
     * @param state The desired LampState (e.g., PURPLE, GREEN, OFF).
     */
    private void setLampState(Servo lamp, LampState state) {
        lamp.setPosition(state.position);
    }


    /**
     * Provides telemetry data for all sensors.
     * @param telemetry The Telemetry object from your OpMode.
     */
    public void addTelemetry(Telemetry telemetry) {
        // Add sensor and queue data to the driver station telemetry
        telemetry.addData("Sensor 1 Color", getDetectedColor(1));
        telemetry.addData("Sensor 2 Color", getDetectedColor(2));
        telemetry.addData("Sensor 3 Color", getDetectedColor(3));
        telemetry.addData("Ball Queue", ballQueue.toString());
    }
}