package org.firstinspires.ftc.teamcode.RobotHdw;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * RobotAimSubsystem
 * 
 * EDUCATIONAL OVERVIEW FOR ROOKIE PROGRAMMERS:
 * This subsystem calculates how the *entire robot* should rotate to face 
 * the closest scoring bucket on the field based on odometry pose and alliance color.
 * 
 * Key concepts used here:
 * 1. Cartesian Coordinates & Pythagorean Distance: Determining which bucket is closest to the robot.
 * 2. Trigonometry (Math.atan2): Calculating the exact angle from the robot's current (x, y) position 
 *    to the target bucket's (x, y) position on the 144" x 144" FTC field.
 * 3. Angle Normalization: Ensuring rotation calculations stay between -180 and +180 degrees (-PI to PI radians).
 * 4. Proportional Control (kP): Turning power proportional to how far off the robot's heading is.
 */
public class RobotAimSubsystem {

    private final LimelightSubsystem limelight;

    /**
     * Represents a scoring bucket on the field with coordinates and alliance assignment.
     */
    public static class Bucket {
        public final double x;
        public final double y;
        public final LimelightSubsystem.Alliance alliance;
        public final String name;

        public Bucket(String name, double x, double y, LimelightSubsystem.Alliance alliance) {
            this.name = name;
            this.x = x;
            this.y = y;
            this.alliance = alliance;
        }
    }

    // Define all 4 field buckets (2 RED, 2 BLUE) in field inches
    private final Bucket[] buckets = {
        new Bucket("Red Bucket 1", 36.0, 120.0, LimelightSubsystem.Alliance.RED),
        new Bucket("Red Bucket 2", 108.0, 120.0, LimelightSubsystem.Alliance.RED),
        new Bucket("Blue Bucket 1", 36.0, 24.0, LimelightSubsystem.Alliance.BLUE),
        new Bucket("Blue Bucket 2", 108.0, 24.0, LimelightSubsystem.Alliance.BLUE)
    };

    private Bucket closestBucket = null;

    public RobotAimSubsystem(RobotHdw robot, LimelightSubsystem limelightSubsystem) {
        this.limelight = limelightSubsystem;
    }

    /**
     * Calculates the heading error (in radians) between the robot's current orientation
     * and the direction of the closest bucket matching our alliance color.
     * 
     * @param robotPose The current pose (x position, y position, heading angle) of the robot from odometry.
     * @return The heading error in radians (-PI to PI).
     */
    public double calculateRobotAimHeadingError(Pose robotPose) {
        LimelightSubsystem.Alliance currentAlliance = limelight.getAlliance();
        if (currentAlliance == null || currentAlliance == LimelightSubsystem.Alliance.DETECT) {
            currentAlliance = LimelightSubsystem.Alliance.BLUE; // Safe default if alliance isn't set
        }

        double minDistanceSq = Double.MAX_VALUE;
        closestBucket = null;

        // Loop through all 4 buckets to find the closest one belonging to our alliance
        for (Bucket bucket : buckets) {
            if (bucket.alliance == currentAlliance) {
                double dx = bucket.x - robotPose.x();
                double dy = bucket.y - robotPose.y();
                double distSq = dx * dx + dy * dy; // Squared distance
                if (distSq < minDistanceSq) {
                    minDistanceSq = distSq;
                    closestBucket = bucket;
                }
            }
        }

        if (closestBucket == null && buckets.length > 0) {
            closestBucket = buckets[0];
        }

        double dx = closestBucket.x - robotPose.x();
        double dy = closestBucket.y - robotPose.y();
        double angleToBucket = Math.atan2(dy, dx);
        double headingError = angleToBucket - robotPose.heading();

        return AngleUnit.normalizeRadians(headingError);
    }

    /**
     * Returns the closest bucket object that the robot is currently targeting.
     */
    public Bucket getClosestBucket() {
        return closestBucket;
    }

    /**
     * Calculates proportional turning correction power to rotate the robot toward the bucket.
     * 
     * @param robotPose The current pose of the robot.
     * @return Turning power value to feed into drivetrain manual controls.
     */
    public double getTurnCorrection(Pose robotPose) {
        double headingError = calculateRobotAimHeadingError(robotPose);
        double kP = 1.5;
        return headingError * kP;
    }
}
