package org.firstinspires.ftc.teamcode.RobotHdw;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHdw.HardwareRobot;

public class TubeWheelSubsystem {

    private final HardwareRobot robot;
    private final LauncherSubsystem launcherSubsystem;
    private final ColorSensorSubsystem colorSensorSubsystem;
    private final double LIFT_RESTED_POSITION = 0.89;
    private ElapsedTime queueFullTimer = new ElapsedTime();
    private boolean queueIsFull = false;


    public TubeWheelSubsystem(HardwareRobot robot, LauncherSubsystem launcherSubsystem, ColorSensorSubsystem colorSensorSubsystem) {
        this.robot = robot;
        this.launcherSubsystem = launcherSubsystem;
        this.colorSensorSubsystem = colorSensorSubsystem;
        queueFullTimer.reset();
    }

    public void update() {
        // Check if all 3 sensors see a color
        boolean allSensorsDetecting = colorSensorSubsystem.getDetectedColor(1) != ColorSensorSubsystem.DetectedColor.NONE &&
                colorSensorSubsystem.getDetectedColor(2) != ColorSensorSubsystem.DetectedColor.NONE &&
                colorSensorSubsystem.getDetectedColor(3) != ColorSensorSubsystem.DetectedColor.NONE;

        if (allSensorsDetecting) {
            if (queueFullTimer.seconds() > 1) {
                queueIsFull = true;
            }
        } else {
            queueFullTimer.reset();
            queueIsFull = false;
        }

        if (queueIsFull) {
            robot.servoTubeWheel.setPower(0.0);
            return; // Exit early, queue is full.
        }

        // Check if the intake is running
        boolean isIntakeRunning = (robot.motorIntake.getPower() != 0);

        // Check if the Launcher is running
        boolean isLauncherRunning = (launcherSubsystem.getTargetRPM() > 0);

        // Check if the lift servo is at its rested position
        boolean isLiftRested = (Math.abs(robot.servoLift.getPosition() - LIFT_RESTED_POSITION) < 0.1);

        // The tubewheel should run if the lift is rested AND (the intake is running OR the Launcher is running).
        if (isLiftRested && (isIntakeRunning || isLauncherRunning)) {
            robot.servoTubeWheel.setPower(1.0);
        } else {
            robot.servoTubeWheel.setPower(0.0);
        }
    }

    public boolean isQueueFull() {
        return queueIsFull;
    }
}
