package org.firstinspires.ftc.teamcode.RobotHdw; // Make sure this package matches your project structure

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    // PID gains (tuning parameters)
    private double kP, kI, kD;

    // PID state variables
    private double integralSum = 0;
    private double lastError = 0;

    // The target value the controller is trying to reach
    private double target = 0;

    // Timer to calculate the change in time (delta time)
    private ElapsedTime timer = new ElapsedTime();

    // Output clamping
    private double minOutput = -1.0;
    private double maxOutput = 1.0;

    /**
     * Constructor for the PID controller.
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * Sets the target value for the PID controller. For aiming, this will be 0.
     * @param target The desired value.
     */
    public void setTarget(double target) {
        this.target = target;
    }

    /**
     * Sets the minimum and maximum output values for the PID controller.
     * @param min The minimum output value.
     * @param max The maximum output value.
     */
    public void setOutputLimits(double min, double max) {
        if (min >= max) {
            throw new IllegalArgumentException("Min output must be less than max output.");
        }
        this.minOutput = min;
        this.maxOutput = max;
    }

    /**
     * Calculates the PID output based on the current measured value.
     * @param current The current value from a sensor (e.g., Limelight's tx).
     * @return The calculated correction value (e.g., the power or speed to apply).
     */
    public double update(double current) {
        // Calculate the error
        double error = target - current;

        // Get the time elapsed since the last update
        double deltaTime = timer.seconds();

        // Handle the first loop iteration or a reset timer
        // If deltaTime is zero, we can't calculate the derivative.
        if (deltaTime == 0) {
            timer.reset();
            // Return the proportional part for a quick response
            double p_output = kP * error;
            return Math.max(minOutput, Math.min(p_output, maxOutput));
        }

        // Calculate the integral term
        integralSum += error * deltaTime;

        // Calculate the derivative term
        double derivative = (error - lastError) / deltaTime;

        // Update lastError and reset the timer for the *next* iteration
        lastError = error;
        timer.reset();

        // Calculate the final PID output
        double output = (kP * error) + (kI * integralSum) + (kD * derivative);

        // Clamp the output
        return Math.max(minOutput, Math.min(output, maxOutput));
    }

    /**
     * Resets the integral sum and last error, useful when changing targets or modes.
     */
    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }
}