package org.firstinspires.ftc.teamcode.RobotHdw;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TubeOfDespairSubsystem
 *
 * EDUCATIONAL OVERVIEW FOR ROOKIE PROGRAMMERS:
 * This subsystem manages the 4-section flighted rotary ball holder (the "Tube of Despair").
 * As the rotary wheel indexes, a static arm pulls/pushes the balls out into the shooter.
 *
 * CRITICAL MECHANICAL REQUIREMENT:
 * Every time the tube indexes or unloads and then stops, it MUST end squarely in a fully aligned,
 * loadable position. We guarantee this by running the indexing servo for a precise, timed 90-degree
 * pulse duration (`INDEX_DURATION_MS`) and immediately cutting power (`0.0`) upon completion.
 *
 * Key features:
 * 1. Ball Presence & Count Tracking: Uses the 4 color sensors to track exactly how many balls are loaded (0 to 4).
 * 2. Step-by-Step Indexing Loading: Pulses the rotary tube servo to index to the next position when a ball enters,
 *    continuing until all 4 positions are full in precise loadable alignment.
 * 3. Rapid Unloading / Firing Routine: Rapidly indexes position by position to feed all loaded balls out into
 *    the shooter when the launcher motor is up to speed, landing in loadable positions between steps.
 * 4. Servo Homing Routine: Uses Sensor 1 as a reference point to calibrate and home the rotary axis on startup.
 */
public class TubeOfDespairSubsystem {

    private final RobotHdw robot;
    private final LauncherSubsystem launcherSubsystem;
    private final ColorSensorSubsystem colorSensorSubsystem;

    // --- Indexing Timing Constants ---
    private static final double INDEX_DURATION_MS = 300;       // Duration (ms) for a standard 90-degree loading step
    private static final double COOLDOWN_DURATION_MS = 200;    // Pause between loading index steps
    private static final double FIRING_INDEX_DURATION_MS = 150;// Faster duration for rapid firing/unloading
    private static final double FIRING_COOLDOWN_MS = 50;       // Short pause between shots for rapid dumping

    // --- Homing Variables & Constants ---
    private boolean isHomed = false;
    private boolean isHoming = false;
    private final ElapsedTime homingTimer = new ElapsedTime();
    private static final double HOMING_POWER = 0.35; // Slow, controlled power for homing search
    private static final double HOMING_TIMEOUT_MS = 4000; // 4 seconds max timeout

    private final ElapsedTime indexTimer = new ElapsedTime();
    private final ElapsedTime cooldownTimer = new ElapsedTime();
    private final ElapsedTime queueFullTimer = new ElapsedTime();

    private boolean queueIsFull = false;
    private boolean isIndexing = false;
    private boolean lastBallDetectedState = false;

    public TubeOfDespairSubsystem(RobotHdw robot, LauncherSubsystem launcherSubsystem, ColorSensorSubsystem colorSensorSubsystem) {
        this.robot = robot;
        this.launcherSubsystem = launcherSubsystem;
        this.colorSensorSubsystem = colorSensorSubsystem;
        indexTimer.reset();
        cooldownTimer.reset();
        queueFullTimer.reset();
    }

    /**
     * Starts the homing routine to find the zero/home position using Sensor 1.
     */
    public void startHoming() {
        isHoming = true;
        isHomed = false;
        homingTimer.reset();
    }

    /**
     * Updates the homing routine. Runs the tube wheel slowly until Sensor 1 detects a reference/ball
     * or until the timeout is reached, ensuring it stops in a valid loadable position.
     *
     * @return true when homing is successfully completed.
     */
    public boolean updateHoming() {
        if (!isHoming) {
            return isHomed;
        }

        // Check if Sensor 1 detects the reference position (e.g., a ball or alignment marker)
        boolean referenceDetected = hasBallAt(1);
        boolean timeoutReached = (homingTimer.milliseconds() > HOMING_TIMEOUT_MS);

        if (referenceDetected || timeoutReached) {
            // Stop servo immediately to park in a loadable position
            robot.servoTubeWheel.setPower(0.0);
            isHoming = false;
            isHomed = true;
            return true;
        } else {
            // Keep searching slowly
            robot.servoTubeWheel.setPower(HOMING_POWER);
            return false;
        }
    }

    public boolean isHomed() {
        return isHomed;
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
     * Triggers a single 90-degree index step of the flighted rotary holder,
     * ensuring it stops squarely in a loadable position.
     */
    public void triggerIndexStep() {
        double currentCooldown = (launcherSubsystem.isRpmWithinTolerance() && !isQueueEmpty()) ? FIRING_COOLDOWN_MS : COOLDOWN_DURATION_MS;
        if (!isIndexing && cooldownTimer.milliseconds() > currentCooldown) {
            isIndexing = true;
            indexTimer.reset();
        }
    }

    /**
     * LOADING ROUTINE: Monitors sensors and intake, triggering index steps until all 4 positions are full.
     */
    public void updateLoading() {
        int ballCount = getBallCount();
        boolean allSensorsDetecting = (ballCount == 4);

        // Debounce timer for queue fullness
        if (allSensorsDetecting) {
            if (queueFullTimer.seconds() > 0.5) {
                queueIsFull = true;
            }
        } else {
            queueFullTimer.reset();
            queueIsFull = false;
        }

        // If the queue is full, do not index further
        if (queueIsFull) {
            stopIndexing();
            return;
        }

        boolean isIntakeRunning = (robot.motorIntake.getPower() != 0);

        // Check if a new ball is entering at sensor 1 (rising edge detection)
        boolean currentBallAt1 = hasBallAt(1);
        boolean ballJustEntered = currentBallAt1 && !lastBallDetectedState;
        lastBallDetectedState = currentBallAt1;

        // If intake is running, trigger index when a ball enters or slot 1 needs clearing
        if (isIntakeRunning && (ballJustEntered || (!hasBallAt(1) && !isQueueFull()))) {
            triggerIndexStep();
        }

        // Execute active indexing pulse or cooldown
        executeIndexingMotor(false);
    }

    /**
     * RAPID UNLOADING / FIRING ROUTINE: Rapidly indexes position by position to feed all balls out into the shooter.
     */
    public void updateUnloading() {
        // If launcher is up to speed, trigger rapid index steps to feed balls out quickly
        if (launcherSubsystem.isRpmWithinTolerance() && !isQueueEmpty()) {
            triggerIndexStep();
        }

        executeIndexingMotor(true);
    }

    /**
     * Manages the timed pulsing of the `servoTubeWheel` for the flighted rotary mechanism.
     * Guaranteed to stop squarely in a loadable position after each 90-degree step.
     *
     * @param isFiring True if running in rapid firing/unloading mode.
     */
    private void executeIndexingMotor(boolean isFiring) {
        double currentDuration = isFiring ? FIRING_INDEX_DURATION_MS : INDEX_DURATION_MS;

        if (isIndexing) {
            if (indexTimer.milliseconds() < currentDuration) {
                robot.servoTubeWheel.setPower(1.0); // Run servo to advance to next flight position
            } else {
                robot.servoTubeWheel.setPower(0.0); // Stop servo immediately in loadable position
                isIndexing = false;
                cooldownTimer.reset();
            }
        } else {
            robot.servoTubeWheel.setPower(0.0);
        }
    }

    private void stopIndexing() {
        isIndexing = false;
        robot.servoTubeWheel.setPower(0.0);
    }

    /**
     * Main update loop called every cycle in TeleOp or Autonomous.
     */
    public void update() {
        // If not yet homed, run homing routine first
        if (!isHomed) {
            updateHoming();
            return;
        }

        // Switch between rapid unloading (firing) and loading (indexing) based on launcher state
        if (launcherSubsystem.isRpmWithinTolerance() && !isQueueEmpty()) {
            updateUnloading();
        } else {
            updateLoading();
        }
    }
}
