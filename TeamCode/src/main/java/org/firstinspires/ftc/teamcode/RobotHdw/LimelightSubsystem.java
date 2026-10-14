package org.firstinspires.ftc.teamcode.RobotHdw;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHdw.PIDController;

import java.util.List;

public class LimelightSubsystem {

    private Limelight3A limelight;
    private PIDController aimPidController;
    private Alliance alliance;

    public enum Alliance {
        RED,
        BLUE,
        DETECT
    }

    // Class variables to store the latest data
    private LLResult latestResult;
    private boolean hasValidTarget = false;
    private double targetX = 0.0;
    private double targetY = 0.0;
    private double targetArea = 0.0;
    private int targetId = -1;
    private int pipelineIndex = 0;


    // Constructor: This is called when you create an instance of the class
    public LimelightSubsystem(HardwareMap hardwareMap) {
        // Initialize the hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // --- PID Controller Initialization ---
        // These are STARTING values. You WILL need to tune them.
        // kP: Start with a small value (e.g., 0.01) and increase until it oscillates. Then back off.
        // If it is oscillating to begin with, decrease the value by 0.01.
        // kI: Start at 0. Add a tiny value (e.g., 0.001) if it never quite reaches the target.
        // kD: Start at 0. Add a small value (e.g., 0.005) to dampen oscillations and reduce overshoot.
        aimPidController = new PIDController(0.0004, 0.003, 0); // Starting Point was P=0.0004, I=0.003, D=0
        aimPidController.setTarget(0); // We want the tx value to be 0 (centered)
    }

    public void setPipeline(int pipelineIndex) {
        // Ensure the pipeline index is within the valid range
        if (pipelineIndex < 0 || pipelineIndex > 9) {
            return; // Or throw an error
        }
        this.pipelineIndex = pipelineIndex;

        // The Limelight3A class has a built-in method for this.
        // You don't need LimelightHelpers.
        limelight.pipelineSwitch(pipelineIndex);
    }
    public int getPipeline () {
        return pipelineIndex;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
        if (alliance == Alliance.RED) {
            setPipeline(1); // Red Alliance pipeline
        } else if (alliance == Alliance.BLUE) {
            setPipeline(2); // Blue Alliance pipeline
        } else {
            setPipeline(0); // Initial Pattern Scan Pipeline
        }
    }

    public Alliance getAlliance() {
        return alliance;
    }


    // Call this method once per loop to update the data from the Limelight
    public void update() {
        latestResult = limelight.getLatestResult();

        // Add a check to ensure the detector results list is not empty
        hasValidTarget = latestResult != null && latestResult.isValid() && !latestResult.getFiducialResults().isEmpty();

        if (hasValidTarget) {
            targetX = latestResult.getTx();
            targetY = latestResult.getTy();
            targetArea = latestResult.getTa();
            // This line is now safe because of the check above
            targetId = latestResult.getFiducialResults().get(0).getFiducialId();
        } else {
            // If no target is found, reset all values to a known safe state
            targetX = 0.0;
            targetY = 0.0;
            targetArea = 0.0;
            targetId = -1; // Use -1 to indicate no valid ID
        }
    }

    // --- Public "Getter" Methods to Access Data ---

    public boolean hasValidTarget() {
        return hasValidTarget;
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

    /**
     * Calculates the PID correction value needed to center the robot on the target.
     * @return The calculated power/servo adjustment.
     */
    public double getAimCorrection() {
        // The PID controller calculates how much we need to correct based on the horizontal error (tx)
        return -aimPidController.update(getTargetX());
    }

    /**
     * Resets the PID controller's internal state. Call this when switching out of auto-aim.
     */
    public void resetAimPID() {
        aimPidController.reset();
    }

    /**
     * Starts the Limelight polling for data.
     */
    public void start() {
        limelight.start();
    }



    /**
     * Switches the Limelight pipeline.
     * @param pipeline The pipeline number (0-9).
     */
    public void switchPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    // --- Telemetry Method ---

    /**
     * Adds Limelight-specific data to the telemetry object for debugging.
     * @param telemetry The OpMode's telemetry object.
     */
    public void addTelemetry(Telemetry telemetry) {
        if (hasValidTarget()) {
            telemetry.addData("Limelight", "Target Found!");
            telemetry.addData("Target ID", getTargetId());
            telemetry.addData("Target X", getTargetX());
            telemetry.addData("Target Y", getTargetY());
            telemetry.addData("Target Area", getTargetArea());
        } else {
            telemetry.addData("Limelight", "No Targets");
        }
    }
}