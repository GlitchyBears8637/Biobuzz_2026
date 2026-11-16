package org.firstinspires.ftc.teamcode.RobotHdw;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TubeOfDespairSubsystem
 * 
 * EDUCATIONAL OVERVIEW FOR ROOKIE PROGRAMMERS:
 * This subsystem manages the 4-section flighted rotary ball holder (the "Tube of Despair") 
 * driven by a 5-turn servo, mapped so that all 4 pockets span 1 full rotation (360° total = 0.00 to 0.20 position).
 * 
 * Key features:
 * 1. 4-Step Forward Indexing & 5th Press Rewind:
 *    - Starting State (0): Home (STARTING_OFFSET + 0.00)
 *    - Press 1 (Step 1): Moves forward to Pocket 2 (STARTING_OFFSET + 0.05)
 *    - Press 2 (Step 2): Moves forward to Pocket 3 (STARTING_OFFSET + 0.10)
 *    - Press 3 (Step 3): Moves forward to Pocket 4 (STARTING_OFFSET + 0.15)
 *    - Press 4 (Step 4): Moves forward to End of 4th Position (STARTING_OFFSET + 0.20)
 *    - Press 5 (Step 5): Rewinds back to Home (STARTING_OFFSET + 0.00) and resets count.
 * 2. Manual & Automatic Pocket Indexing: Supports both Gamepad 2 manual button steps (`dpad_up`) and auto-loading.
 * 3. Ball Presence & Count Tracking: Uses the 4 color sensors to track ball occupancy.
 */
public class TubeOfDespairSubsystem {

    private final RobotHdw robot;
    private final LauncherSubsystem launcherSubsystem;
    private final ColorSensorSubsystem colorSensorSubsystem;

    // --- Configurable Starting Position Offset ---
    public static double STARTING_OFFSET = 0.00; // Baseline offset for physical alignment

    // --- 5-Turn Servo Pocket Step (90° = 0.05 on 5-turn scale) ---
    private static final double POCKET_STEP = 0.05;

    private int stepCount = 0; // Tracks manual/automatic step count (0 through 4)
    private boolean dpadUpWasPressed = false;
    private boolean dpadDownWasPressed = false;
    private boolean queueIsFull = false;
    private final ElapsedTime queueFullTimer = new ElapsedTime();
    private boolean lastBallDetectedState = false;

    public TubeOfDespairSubsystem(RobotHdw robot, LauncherSubsystem launcherSubsystem, ColorSensorSubsystem colorSensorSubsystem) {
        this.robot = robot;
        this.launcherSubsystem = launcherSubsystem;
        this.colorSensorSubsystem = colorSensorSubsystem;
        queueFullTimer.reset();

        // Initialize servo to home position (starting offset)
        if (robot.servoTubeWheel != null) {
            robot.servoTubeWheel.setPosition(STARTING_OFFSET);
        }
    }

    /**
     * Calculates the target position for a given step (0 to 4) built on top of the starting offset.
     *
     * @param step Step index (0 to 4)
     * @return Target servo position
     */
    public static double getStepPosition(int step) {
        if (step <= 0) return STARTING_OFFSET;
        if (step == 1) return STARTING_OFFSET + POCKET_STEP;        // 0.05 (90°)
        if (step == 2) return STARTING_OFFSET + (POCKET_STEP * 2); // 0.10 (180°)
        if (step == 3) return STARTING_OFFSET + (POCKET_STEP * 3); // 0.15 (270°)
        if (step == 4) return STARTING_OFFSET + (POCKET_STEP * 4); // 0.20 (360°)
        return STARTING_OFFSET;
    }

    /**
     * Returns the exact number of balls currently detected in the 4-section tube (0 to 4).
     */
    public int getBallCount() {
        int ballCount = 0;
        for (int i = 1; i <= 4; i++) {
            if (hasBallAt(i)) {
                ballCount++;
            }
        }
        return ballCount;
    }

    /**
     * Checks if a specific position/slot (1 through 4) currently has a ball.
     */
    public boolean hasBallAt(int slot) {
        if (slot < 1 || slot > 4) return false;
        return colorSensorSubsystem.getDetectedColor(slot) != ColorSensorSubsystem.DetectedColor.NONE;
    }

    /**
     * Returns true if all 4 slots in the tube are occupied.
     */
    public boolean isQueueFull() {
        return queueIsFull;
    }

    /**
     * Returns true if the tube is completely empty (0 balls).
     */
    public boolean isQueueEmpty() {
        return getBallCount() == 0;
    }

    /**
     * Rewinds the multi-turn servo back to home position (STARTING_OFFSET) and resets step count.
     */
    public void rewind() {
        stepCount = 0;
        if (robot.servoTubeWheel != null) {
            robot.servoTubeWheel.setPosition(STARTING_OFFSET);
        }
    }

    /**
     * Advances to the next step in sequence.
     * Steps 1 through 4 move forward (0.05, 0.10, 0.15, 0.20).
     * On the 5th call (stepCount > 4), it rewinds back to home (STARTING_OFFSET / 0.00) and resets count to 0.
     */
    public void nextPocket() {
        stepCount++;
        if (stepCount > 4) {
            rewind(); // 5th press goes back to home (STARTING_OFFSET)
        } else {
            double targetPosition = getStepPosition(stepCount);
            if (robot.servoTubeWheel != null) {
                robot.servoTubeWheel.setPosition(targetPosition);
            }
        }
    }

    /**
     * LOADING ROUTINE: Monitors sensors and intake, indexing to next pocket when a ball enters.
     */
    public void updateLoading() {
        int ballCount = getBallCount();
        boolean allSensorsDetecting = (ballCount == 4);

        if (allSensorsDetecting) {
            if (queueFullTimer.seconds() > 0.5) {
                queueIsFull = true;
            }
        } else {
            queueFullTimer.reset();
            queueIsFull = false;
        }

        if (queueIsFull) {
            return;
        }

        boolean isIntakeRunning = (robot.motorIntake.getPower() != 0);

        boolean currentBallAt1 = hasBallAt(1);
        boolean ballJustEntered = currentBallAt1 && !lastBallDetectedState;
        lastBallDetectedState = currentBallAt1;

        // If intake is running and a ball enters, index to next pocket
        if (isIntakeRunning && ballJustEntered && !isQueueFull() && stepCount < 4) {
            nextPocket();
        }
    }

    /**
     * FIRING / UNLOADING ROUTINE: Steps through pockets to unload balls when launcher is up to speed.
     */
    public void updateUnloading() {
        if (launcherSubsystem.isRpmWithinTolerance() && !isQueueEmpty()) {
            nextPocket();
        }
    }

    /**
     * Main update loop called every cycle.
     */
    public void update() {
        if (launcherSubsystem.isRpmWithinTolerance() && !isQueueEmpty()) {
            updateUnloading();
        } else {
            updateLoading();
        }
    }

    /**
     * Overloaded update method taking Gamepad 2 input for manual pocket stepping (Dpad Up = Next Step, Dpad Down = Rewind).
     */
    public void update(Gamepad gamepad) {
        if (gamepad != null) {
            // Dpad Up: Advances to next step (Steps 1-4 move forward, Step 5 rewinds)
            if (gamepad.dpad_up && !dpadUpWasPressed) {
                nextPocket();
            }
            dpadUpWasPressed = gamepad.dpad_up;

            // Dpad Down: Explicit Manual Rewind back to home
            if (gamepad.dpad_down && !dpadDownWasPressed) {
                rewind();
            }
            dpadDownWasPressed = gamepad.dpad_down;
        }

        update();
    }
}
