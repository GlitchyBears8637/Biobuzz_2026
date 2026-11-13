package org.firstinspires.ftc.teamcode.RobotHdw;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.graphics.Color;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.LinkedList;
import java.util.List;

/**
 * ColorSensorSubsystem
 * 
 * EDUCATIONAL OVERVIEW FOR ROOKIE PROGRAMMERS:
 * This subsystem manages 4 color sensors and 4 indicator lamps for our 4-section ball queue.
 * 
 * Key concepts:
 * 1. RGB to HSV Conversion: Color sensors measure Red, Green, and Blue (RGB). Converting RGB to 
 *    Hue, Saturation, Value (HSV) makes color detection robust against lighting changes and shadows.
 * 2. Debounce Filtering: Prevents flickering when balls pass sensors by requiring a stable reading over time.
 * 3. Queue Management: Maintains a 4-item list (LinkedList) tracking the sequence of balls in the queue.
 */
public class ColorSensorSubsystem {

    // Physical hardware devices for 4 sensors and 4 lamps
    private ColorSensor sensor1, sensor2, sensor3, sensor4;
    private Servo lamp1, lamp2, lamp3, lamp4;
    
    // A LinkedList queue storing up to 4 detected ball colors in order
    private LinkedList<DetectedColor> ballQueue = new LinkedList<>();
    
    // Timer for flashing indicator lights when waiting for game pieces
    private ElapsedTime flashTimer = new ElapsedTime();
    private boolean lampOn = false;

    // --- Debounce Logic Variables ---
    // Timers to track how long a sensor has been seeing NONE (prevents rapid flickering)
    private final ElapsedTime[] debounceTimers = {new ElapsedTime(), new ElapsedTime(), new ElapsedTime(), new ElapsedTime()};
    private final DetectedColor[] lastDetectedColors = {DetectedColor.NONE, DetectedColor.NONE, DetectedColor.NONE, DetectedColor.NONE};
    private static final long DEBOUNCE_DELAY_MS = 250; // 250 milliseconds debounce threshold

    // HSV value storage arrays for all 4 sensors
    private float[] hsvValues1 = {0F, 0F, 0F};
    private float[] hsvValues2 = {0F, 0F, 0F};
    private float[] hsvValues3 = {0F, 0F, 0F};
    private float[] hsvValues4 = {0F, 0F, 0F};

    private final double SCALE_FACTOR = 255;

    public enum DetectedColor {
        PURPLE, GREEN, NONE
    }

    // Lamp servo positions representing different colors
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
    public ColorSensorSubsystem(RobotHdw robot) {
        // Initialize sensors from the RobotHdw instance
        sensor1 = robot.sensorColor1;
        sensor2 = robot.sensorColor2;
        sensor3 = robot.sensorColor3;
        sensor4 = robot.sensorColor4;

        // Initialize lamps from the RobotHdw instance
        lamp1 = robot.lamp1;
        lamp2 = robot.lamp2;
        lamp3 = robot.lamp3;
        lamp4 = robot.lamp4;

        // Reset timers on initialization
        flashTimer.reset();
        for (ElapsedTime timer : debounceTimers) {
            timer.reset();
        }
    }

    // --- Queue Management ---

    /**
     * Adds a detected ball color to the 4-section ball queue.
     * If the queue reaches 4 items, the oldest (first) item is removed.
     */
    public void enqueueBall(DetectedColor color) {
        if (color == DetectedColor.NONE) {
            return; // Ignore empty readings
        }
        if (ballQueue.size() >= 4) {
            ballQueue.removeFirst(); // FIFO queue behavior (First In, First Out)
        }
        ballQueue.add(color);
    }

    /**
     * Checks if the queue sequence matches the expected strategic game pattern.
     */
    public boolean isSequenceCorrect(List<String> pattern) {
        if (ballQueue.size() != pattern.size()) {
            return false;
        }
        for (int i = 0; i < ballQueue.size(); i++) {
            String patternColor = pattern.get(i).toUpperCase();
            String queueColor = ballQueue.get(i).toString();
            if (!patternColor.equals(queueColor)) {
                return false;
            }
        }
        return true;
    }

    public void clearQueue() {
        ballQueue.clear();
    }

    /**
     * Reads raw RGB from all 4 sensors and converts them to HSV color space.
     */
    public void updateSensorReadings() {
        Color.RGBToHSV((int) (sensor1.red() * SCALE_FACTOR), (int) (sensor1.green() * SCALE_FACTOR), (int) (sensor1.blue() * SCALE_FACTOR), hsvValues1);
        Color.RGBToHSV((int) (sensor2.red() * SCALE_FACTOR), (int) (sensor2.green() * SCALE_FACTOR), (int) (sensor2.blue() * SCALE_FACTOR), hsvValues2);
        Color.RGBToHSV((int) (sensor3.red() * SCALE_FACTOR), (int) (sensor3.green() * SCALE_FACTOR), (int) (sensor3.blue() * SCALE_FACTOR), hsvValues3);
        Color.RGBToHSV((int) (sensor4.red() * SCALE_FACTOR), (int) (sensor4.green() * SCALE_FACTOR), (int) (sensor4.blue() * SCALE_FACTOR), hsvValues4);
    }

    /**
     * Analyzes HSV values from a specific sensor to determine if a ball is PURPLE, GREEN, or NONE.
     */
    public DetectedColor getDetectedColor(int sensorNumber) {
        float[] hsv;
        switch (sensorNumber) {
            case 1: hsv = hsvValues1; break;
            case 2: hsv = hsvValues2; break;
            case 3: hsv = hsvValues3; break;
            case 4: hsv = hsvValues4; break;
            default: return DetectedColor.NONE;
        }

        float hue = hsv[0];   // Hue (0-360 degrees on color wheel)
        float value = hsv[2]; // Brightness value

        // Minimum brightness threshold to ensure a ball is present rather than a shadow
        if (value > 300) {
            if (hue >= 90 && hue <= 180) {
                return DetectedColor.GREEN;
            } else if (hue >= 220 && hue <= 290) {
                return DetectedColor.PURPLE;
            }
        }
        return DetectedColor.NONE;
    }

    /**
     * Updates all 4 indicator lamps with debounce filtering to show detected ball colors.
     */
    public void updateLamps(List<String> pattern) {
        // Flash timer for empty slots
        if (flashTimer.milliseconds() > 500) {
            lampOn = !lampOn;
            flashTimer.reset();
        }

        // Apply debounce filtering across all 4 sensors
        for (int i = 0; i < 4; i++) {
            DetectedColor currentColor = getDetectedColor(i + 1);

            if (currentColor != DetectedColor.NONE) {
                lastDetectedColors[i] = currentColor;
                debounceTimers[i].reset();
            } else {
                if (debounceTimers[i].milliseconds() > DEBOUNCE_DELAY_MS) {
                    lastDetectedColors[i] = DetectedColor.NONE;
                }
            }
        }

        // Update lamps 1 through 4 for each queue section
        updateLamp(lamp1, lastDetectedColors[0], pattern, 0);
        updateLamp(lamp2, lastDetectedColors[1], pattern, 1);
        updateLamp(lamp3, lastDetectedColors[2], pattern, 2);
        updateLamp(lamp4, lastDetectedColors[3], pattern, 3);
    }

    private void updateLamp(Servo lamp, DetectedColor detectedColor, List<String> pattern, int patternIndex) {
        LampState desiredState = LampState.OFF;

        if (detectedColor != DetectedColor.NONE) {
            if (detectedColor == DetectedColor.PURPLE) {
                desiredState = LampState.PURPLE;
            } else if (detectedColor == DetectedColor.GREEN) {
                desiredState = LampState.GREEN;
            }
        } else if (lampOn && pattern != null && pattern.size() > patternIndex) {
            String expectedColor = pattern.get(patternIndex);
            if (expectedColor.equalsIgnoreCase("PURPLE")) {
                desiredState = LampState.PURPLE;
            } else if (expectedColor.equalsIgnoreCase("GREEN")) {
                desiredState = LampState.GREEN;
            }
        }
        lamp.setPosition(desiredState.position);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Sensor 1, 2", "%s, %s", getDetectedColor(1), getDetectedColor(2));
        telemetry.addData("Sensor 3, 4", "%s, %s", getDetectedColor(3), getDetectedColor(4));
        telemetry.addData("4-Section Ball Queue", ballQueue.toString());
    }
}
