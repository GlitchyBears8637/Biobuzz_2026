package org.firstinspires/ftc.teamcode.RobotHdw;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TubeWheelSubsystem
 * 
 * EDUCATIONAL OVERVIEW FOR ROOKIE PROGRAMMERS:
 * This subsystem controls the indexing mechanism (tube wheel) that pulls game pieces (balls)
 * into the robot's 4-section storage queue. 
 * 
 * It checks all 4 color sensors simultaneously. When all 4 slots are occupied by balls, 
 * a timer ensures the balls have settled, and then automatically stops the tube wheel motor 
 * to prevent jamming or overloading the intake mechanism.
 */
public class TubeWheelSubsystem {

    private final RobotHdw robot;
    private final LauncherSubsystem launcherSubsystem;
    private final ColorSensorSubsystem colorSensorSubsystem;
    private final double LIFT_RESTED_POSITION = 0.89; // Rest position for the lift mechanism
    
    private ElapsedTime queueFullTimer = new ElapsedTime();
    private boolean queueIsFull = false;

    public TubeWheelSubsystem(RobotHdw robot, LauncherSubsystem launcherSubsystem, ColorSensorSubsystem colorSensorSubsystem) {
        this.robot = robot;
        this.launcherSubsystem = launcherSubsystem;
        this.colorSensorSubsystem = colorSensorSubsystem;
        queueFullTimer.reset();
    }

    /**
     * Called every loop cycle to check queue fullness and manage the tube wheel motor power.
     */
    public void update() {
        // Check if all 4 color sensors detect a game piece (i.e. all 4 queue slots are full)
        boolean allSensorsDetecting = colorSensorSubsystem.getDetectedColor(1) != ColorSensorSubsystem.DetectedColor.NONE &&
                colorSensorSubsystem.getDetectedColor(2) != ColorSensorSubsystem.DetectedColor.NONE &&
                colorSensorSubsystem.getDetectedColor(3) != ColorSensorSubsystem.DetectedColor.NONE &&
                colorSensorSubsystem.getDetectedColor(4) != ColorSensorSubsystem.DetectedColor.NONE;

        // Debounce timer: make sure all 4 balls are stably detected for at least 1 full second
        if (allSensorsDetecting) {
            if (queueFullTimer.seconds() > 1) {
                queueIsFull = true;
            }
        } else {
            queueFullTimer.reset();
            queueIsFull = false;
        }

        // If the queue is full, stop the tube wheel immediately to prevent jams
        if (queueIsFull) {
            robot.servoTubeWheel.setPower(0.0);
            return; 
        }

        // Check if intake motor is running
        boolean isIntakeRunning = (robot.motorIntake.getPower() != 0);

        // Check if launcher motor is running
        boolean isLauncherRunning = (launcherSubsystem.getTargetRPM() > 0);

        // Check if the lift mechanism is at rest
        boolean isLiftRested = (Math.abs(robot.servoLift.getPosition() - LIFT_RESTED_POSITION) < 0.1);

        // Run the tube wheel continuous rotation servo if the lift is rested AND either intake or launcher is active
        if (isLiftRested && (isIntakeRunning || isLauncherRunning)) {
            robot.servoTubeWheel.setPower(1.0);
        } else {
            robot.servoTubeWheel.setPower(0.0);
        }
    }

    /**
     * Returns true if all 4 queue slots are full.
     */
    public boolean isQueueFull() {
        return queueIsFull;
    }
}
