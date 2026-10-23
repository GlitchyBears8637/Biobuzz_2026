package org.firstinspires.ftc.teamcode.RobotHdw;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.RobotHdw.PIDController;

import java.util.List;

/**
 * LimelightSubsystem
 * 
 * EDUCATIONAL OVERVIEW FOR ROOKIE PROGRAMMERS:
 * This class interfaces with the Limelight 3A vision camera. It performs three major tasks:
 * 1. Alliance Tag Filtering: Ignoring AprilTags belonging to the opposing alliance.
 * 2. Vision Target Tracking: Measuring horizontal offset (tx) and area (ta) to aim and adjust launcher speed.
 * 3. Sensor Fusion (Position Reaffirmation): Blending wheel odometry with camera vision (Botpose) 
 *    to eliminate drifting over time.
 */
public class LimelightSubsystem {

    private Limelight3A limelight;
    private PIDController aimPidController;
    private Alliance alliance;

    public enum Alliance {
        RED,
        BLUE,
        DETECT
    }

    // Class variables to store the latest camera data each loop cycle
    private LLResult latestResult;
    private boolean hasValidTarget = false;
    private double targetX = 0.0;
    private double targetY = 0.0;
    private double targetArea = 0.0;
    private int targetId = -1;
    private int pipelineIndex = 0;
    private Pose3D botpose = null;
    private boolean hasValidBotpose = false;

    /**
     * Constructor: Initializes the Limelight camera hardware and configures the aiming PID controller.
     */
    public LimelightSubsystem(HardwareMap hardwareMap) {
        // Retrieve the Limelight camera from the robot configuration
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // --- PID Controller Initialization ---
        // PID (Proportional, Integral, Derivative) helps smooth out movements.
        // kP handles proportional correction, kI eliminates steady-state error, kD dampens overshoot.
        aimPidController = new PIDController(0.0004, 0.003, 0); 
        aimPidController.setTarget(0); // We want the target horizontal offset (tx) to be 0 (centered)
    }

    /**
     * Switches the camera pipeline (e.g., Tag detection vs. color blob detection).
     */
    public void setPipeline(int pipelineIndex) {
        if (pipelineIndex < 0 || pipelineIndex > 9) {
            return; 
        }
        this.pipelineIndex = pipelineIndex;
        limelight.pipelineSwitch(pipelineIndex);
    }
    
    public int getPipeline() {
        return pipelineIndex;
    }

    /**
     * Sets the team alliance and automatically switches to the correct camera pipeline.
     */
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
        if (alliance == Alliance.RED) {
            setPipeline(1); // Pipeline 1: Red Alliance tags/goals
        } else if (alliance == Alliance.BLUE) {
            setPipeline(2); // Pipeline 2: Blue Alliance tags/goals
        } else {
            setPipeline(0); // Pipeline 0: General scan
        }
    }

    public Alliance getAlliance() {
        return alliance;
    }

    /**
     * Called once per control loop cycle to grab fresh results from the Limelight camera,
     * filter out wrong alliance tags, and store target/botpose information.
     */
    public void update() {
        latestResult = limelight.getLatestResult();

        // Reset tracking flags before checking new frame data
        hasValidTarget = false;
        hasValidBotpose = false;
        targetX = 0.0;
        targetY = 0.0;
        targetArea = 0.0;
        targetId = -1;
        botpose = null;

        // Ensure the camera has a valid result containing AprilTag fiducial results
        if (latestResult != null && latestResult.isValid() && !latestResult.getFiducialResults().isEmpty()) {
            
            // Loop through all detected tags in the camera frame
            for (LLResultTypes.FiducialResult fr : latestResult.getFiducialResults()) {
                int id = fr.getFiducialId();
                
                // ALLIANCE FILTERING: Ignore tags that do not belong to our alliance color!
                if (isTagForAlliance(id, alliance)) {
                    hasValidTarget = true;
                    targetX = latestResult.getTx(); // Horizontal offset from crosshair
                    targetY = latestResult.getTy(); // Vertical offset from crosshair
                    targetArea = latestResult.getTa(); // Target area (size / distance proxy)
                    targetId = id;
                    
                    // Grab camera estimated field position (Botpose)
                    botpose = latestResult.getBotpose();
                    hasValidBotpose = (botpose != null);
                    break; // Stop searching once we find a valid tag for our alliance
                }
            }
        }
    }

    /**
     * Educational Note on Alliance Filtering:
     * In FTC games, AprilTags have unique ID numbers across the field. 
     * This method ensures your robot only interacts with tags belonging to your alliance.
     */
    public boolean isTagForAlliance(int tagId, Alliance targetAlliance) {
        if (targetAlliance == Alliance.RED) {
            // Example: Red alliance tags are IDs 1 through 3 (adjust based on Game Manual)
            return tagId >= 1 && tagId <= 3;
        } else if (targetAlliance == Alliance.BLUE) {
            // Example: Blue alliance tags are IDs 4 through 6 (adjust based on Game Manual)
            return tagId >= 4 && tagId <= 6;
        }
        return true; // Default accept if alliance is unassigned
    }

    // --- Public Getter Methods ---

    public boolean hasValidTarget() {
        return hasValidTarget;
    }

    public Pose3D getBotpose() {
        return botpose;
    }

    public boolean hasValidBotpose() {
        return hasValidBotpose;
    }

    /**
     * SENSOR FUSION / POSITION REAFFIRMATION:
     * Odometry wheels can slip and drift over time. The Limelight camera sees AprilTags 
     * in the real world and knows our exact global field position. 
     * 
     * This method blends wheel odometry (95%) with camera vision (5%) to constantly 
     * correct drift smoothly without causing sudden jump glitches on the screen.
     * 
     * @param follower The Pedro Pathing Follower instance managing robot position.
     */
    public void reaffirmPosition(Follower follower) {
        if (hasValidBotpose && botpose != null && follower != null) {
            // Limelight botpose positions are measured in METERS.
            // Pedro Pathing uses INCHES. We convert meters to inches (1 meter = 39.3701 inches).
            double limeXInches = botpose.getPosition().x * 39.3701;
            double limeYInches = botpose.getPosition().y * 39.3701;

            Pose currentOdometryPose = follower.closestPose();

            // Blend odometry with camera vision
            double fusedX = currentOdometryPose.x() * 0.95 + limeXInches * 0.05;
            double fusedY = currentOdometryPose.y() * 0.95 + limeYInches * 0.05;

            // Update follower position while keeping smooth heading
            follower.setPose(new Pose(fusedX, fusedY, currentOdometryPose.heading()));
        }
    }

    public double getTargetX() {
        return targetX;
    }

    public double getTargetY() {
        return targetY;
    }

    public double getTargetArea() {
        return targetArea;
    }

    public double getTargetId() {
        return targetId;
    }

    // --- Control Methods ---

    public double getAimCorrection() {
        return -aimPidController.update(getTargetX());
    }

    public void resetAimPID() {
        aimPidController.reset();
    }

    public void start() {
        limelight.start();
    }

    public void switchPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    public void addTelemetry(Telemetry telemetry) {
        if (hasValidTarget()) {
            telemetry.addData("Limelight", "Alliance Tag Found (ID: " + getTargetId() + ")");
            telemetry.addData("Target X, Y", "%.2f, %.2f", getTargetX(), getTargetY());
        } else {
            telemetry.addData("Limelight", "Searching for Alliance Tags...");
        }
    }
}
