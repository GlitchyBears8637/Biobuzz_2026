package org.firstinspires.ftc.teamcode.RobotHdw;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class TurretControl {

    private final HardwareRobot robot;
    private final LimelightSubsystem limelight;

    // Constants for target positions (adjust as needed)
    private static final double RED_TARGET_X = 127;
    private static final double RED_TARGET_Y = 131;
    private static final double BLUE_TARGET_X = 16;
    private static final double BLUE_TARGET_Y = 131;

    // Turret servo constants - NOW MATCHES LAUNCHER SUBSYSTEM
    private static final double SERVO_AIM_MIN = 0.48;
    private static final double SERVO_AIM_MAX = 0.66;

    // This value maps the physical rotation of the turret to the servo's range.
    // We've set this to represent a 180-degree total sweep (+/- 90 degrees).
    private static final double MAX_TURRET_ANGLE_RAD = Math.PI / 2.0;

    public TurretControl(HardwareRobot robot, LimelightSubsystem limelightSubsystem) {
        this.robot = robot;
        this.limelight = limelightSubsystem; // Save it for later
    }

    /**
     * Calculates the required turret angle to aim at the target.
     * It now gets the alliance color automatically from the Limelight subsystem.
     * @param robotPose The current pose of the robot (x, y, heading).
     * @return The target servo position for the turret.
     */
    public double calculateTurretAngle(Pose robotPose) {
        double targetX;
        double targetY;

        if (limelight.getAlliance() == LimelightSubsystem.Alliance.RED) {
            targetX = RED_TARGET_X;
            targetY = RED_TARGET_Y;
        } else { // Assumes BLUE or null (defaults to BLUE)
            targetX = BLUE_TARGET_X;
            targetY = BLUE_TARGET_Y;
        }

        double dx = targetX - robotPose.getX();
        double dy = targetY - robotPose.getY();

        // Calculate the angle to the target relative to the field's coordinate system
        double angleToTarget = Math.atan2(dy, dx);

        // Get the robot's current heading from the Pose
        double robotHeading = robotPose.getHeading();

        // Calculate the angle the turret needs to be relative to the robot
        double turretAngle = angleToTarget - robotHeading;

        // Normalize the angle to be within -PI to PI
        turretAngle = AngleUnit.normalizeRadians(turretAngle);

        // Convert the angle to a servo position by mapping the angle range [-MAX_TURRET_ANGLE_RAD, MAX_TURRET_ANGLE_RAD]
        // to the servo range [SERVO_AIM_MAX, SERVO_AIM_MIN].
        double servoRange = SERVO_AIM_MAX - SERVO_AIM_MIN;
        double angleRatio = turretAngle / MAX_TURRET_ANGLE_RAD; // Should be between -1 and 1
        double servoCenter = SERVO_AIM_MIN + servoRange / 2.0;
        double servoPosition = servoCenter - (servoRange / 2.0) * angleRatio;

        return servoPosition;
    }
}