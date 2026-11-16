package org.firstinspires.ftc.teamcode.RobotHdw;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * LauncherSubsystem
 * 
 * EDUCATIONAL OVERVIEW FOR ROOKIE PROGRAMMERS:
 * This subsystem manages the game piece shooter mechanism. 
 * 
 * Key concepts:
 * 1. Closed-Loop Velocity Control (PIDF): Using encoder feedback on `DcMotorEx` to maintain 
 *    consistent flywheel RPM under the load of launching game pieces.
 * 2. Vision Distance Scaling: Using the Limelight target area (ta) to dynamically scale 
 *    launcher RPM based on how far away the robot is from the bucket.
 * 3. Tolerance Checking: Ensuring the flywheel has reached target speed before allowing a shot.
 */
public class LauncherSubsystem {

    private final DcMotorEx motorLauncher;
    private final LimelightSubsystem limelight;
    private final RobotAimSubsystem robotAimSubsystem;
    private final Follower follower;

    // --- Tuning Constants ---
    private static final double MIN_TARGET_AREA = 0.23; // Minimum visible target size
    private static final double MAX_TARGET_AREA = 2.5;  // Maximum visible target size
    private static final double MIN_RPM = 2750;
    private static final double MAX_RPM = 3500;
    private static final double TELEOP_MAX_RPM = 3400;
    private static final double RPM_TOLERANCE = 100;    // RPM speed window required before launching
    private static final double TICKS_PER_REVOLUTION = 28; // Encoder resolution for GoBilda / REV motors

    // PIDF Coefficients for maintaining constant flywheel speed under load
    public static double MOTOR_VELO_PID_P = 25;
    public static double MOTOR_VELO_PID_I = 0;
    public static double MOTOR_VELO_PID_D = 10;
    public static double MOTOR_VELO_PID_F = 13;

    private double targetRPM = 0;
    private boolean autoshootaim = false;
    private boolean leftBumperWasPressed = false;
    private boolean isTeleOp = false;

    public void setTeleOpMode(boolean teleOp) {
        this.isTeleOp = teleOp;
    }

    public LauncherSubsystem(HardwareMap hardwareMap, LimelightSubsystem limelight, RobotAimSubsystem robotAimSubsystem, Follower follower) {
        this.limelight = limelight;
        this.robotAimSubsystem = robotAimSubsystem;
        this.follower = follower;

        // Initialize shooter motor as a DcMotorEx to utilize advanced velocity PIDF control
        motorLauncher = hardwareMap.get(DcMotorEx.class, "motorLauncher");
        motorLauncher.setDirection(DcMotor.Direction.REVERSE);
        motorLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLauncher.setVelocityPIDFCoefficients(MOTOR_VELO_PID_P, MOTOR_VELO_PID_I, MOTOR_VELO_PID_D, MOTOR_VELO_PID_F);
    }

    /**
     * Called every loop cycle to handle trigger controls (auto-aim/auto-shoot disabled temporarily).
     */
    public void update(Gamepad gamepad, Telemetry telemetry) {
        // Auto-shoot and auto-aim disabled temporarily to prevent follower/pose null errors
        autoshootaim = false;
        
        telemetry.addData("Auto Aim & Shoot", "Disabled");

        // Manual trigger control for shooter RPM
        if (gamepad.right_trigger > 0.1) {
            double normalizedTrigger = (gamepad.right_trigger - 0.1) / 0.9;
            double manualRPM = MIN_RPM + normalizedTrigger * (MAX_RPM - MIN_RPM);
            setRPM(manualRPM);
        } else {
            setRPM(0);
        }
    }

    public void setAutoShootAim(boolean autoAim) {
        this.autoshootaim = autoAim;
    }

    /**
     * Dynamically scales launcher RPM based on camera target size (distance proxy).
     */
    public void aimAndPowerForDistance(double targetArea) {
        double currentMax = isTeleOp ? TELEOP_MAX_RPM : MAX_RPM;
        if (targetArea < MIN_TARGET_AREA || targetArea > MAX_TARGET_AREA) {
            setRPM(0);
            return;
        }

        // Linear interpolation between target area and RPM
        double range = MAX_TARGET_AREA - MIN_TARGET_AREA;
        double fraction = (targetArea - MIN_TARGET_AREA) / range;
        double rpm = currentMax + fraction * (MIN_RPM - currentMax);
        double clampedRpm = Math.max(MIN_RPM, Math.min(rpm, currentMax));

        setRPM(clampedRpm);
    }

    public void aimAndPowerFromPose() {
        Pose robotPose = follower.closestPose();
        double targetX = limelight.getAlliance() == LimelightSubsystem.Alliance.RED ? 127 : 16;
        double targetY = 131;
        double dx = targetX - robotPose.x();
        double dy = targetY - robotPose.y();
        double distance = Math.sqrt(dx * dx + dy * dy);

        final double MIN_DISTANCE = 50.0;
        final double MAX_DISTANCE = 150.0;

        if (distance < MIN_DISTANCE || distance > MAX_DISTANCE) {
            setRPM(0);
            return;
        }
        double currentMax = isTeleOp ? TELEOP_MAX_RPM : MAX_RPM;
        double range = MAX_DISTANCE - MIN_DISTANCE;
        double fraction = (distance - MIN_DISTANCE) / range;

        double rpm = MIN_RPM + fraction * (currentMax - MIN_RPM);
        double clampedRpm = Math.max(MIN_RPM, Math.min(rpm, currentMax));

        setRPM(clampedRpm);
    }

    /**
     * Sets the target RPM using ticks per second converted from rotations per minute.
     */
    public void setRPM(double rpm) {
        this.targetRPM = rpm;
        double ticksPerSecond = (rpm * TICKS_PER_REVOLUTION) / 60.0;
        motorLauncher.setVelocity(ticksPerSecond);
    }

    public double getRPM() {
        return (motorLauncher.getVelocity() * 60.0) / TICKS_PER_REVOLUTION;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * Returns true when the flywheel has reached target speed within tolerance (safe to shoot).
     */
    public boolean isRpmWithinTolerance() {
        return Math.abs(getRPM() - targetRPM) < RPM_TOLERANCE && targetRPM != 0;
    }

    public boolean isAutoShootAim() {
        return autoshootaim;
    }
}
