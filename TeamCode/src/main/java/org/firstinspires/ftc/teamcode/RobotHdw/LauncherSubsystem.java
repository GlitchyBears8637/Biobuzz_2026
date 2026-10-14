package org.firstinspires.ftc.teamcode.RobotHdw;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LauncherSubsystem {

    private final DcMotorEx motorLauncher;
    private final Servo servoHood;
    private final Servo servoAim;
    private final LimelightSubsystem limelight;
    private final TurretControl turretControl;
    private final Follower follower;

    // --- TUNING CONSTANTS ---
    private static final double MIN_TARGET_AREA = 0.23;
    private static final double MAX_TARGET_AREA = 2.5;
    private static final double MIN_RPM = 2750;
    private static final double MAX_RPM = 3500;
    private static final double TELEOP_MAX_RPM = 3400;
    private static final double RPM_TOLERANCE = 100; // The range at which the motor has to be close enough in speed to launch!
    private static final double TICKS_PER_REVOLUTION = 28;
    public static double MOTOR_VELO_PID_P = 25;
    public static double MOTOR_VELO_PID_I = 0;
    public static double MOTOR_VELO_PID_D = 10;
    public static double MOTOR_VELO_PID_F = 13;
    private static final double MIN_HOOD_ANGLE = 0.30; // Hood farthest up, aiming more straight forward
    private static final double MAX_HOOD_ANGLE = 0.57; // Hood farthest down, aiming more up
    private static final double SERVO_AIM_MIN = 0.48; // Servo limits are just under our hard limits, hard limit is 0.47
    private static final double SERVO_AIM_MAX = 0.66; // Servo limits are just under our hard limits, hard limit is 0.67

    private double targetRPM = 0;
    private double aimPosition = 0.5;
    private double hoodPosition = 0.56;
    private boolean autoshootaim = false;
    private boolean leftBumperWasPressed = false;
    private boolean isTeleOp = false;
    public void setTeleOpMode(boolean teleOp) {
        this.isTeleOp = teleOp;
    }


    public LauncherSubsystem(HardwareMap hardwareMap, LimelightSubsystem limelight, TurretControl turretControl, Follower follower) {
        this.limelight = limelight;
        this.turretControl = turretControl;
        this.follower = follower;

        motorLauncher = hardwareMap.get(DcMotorEx.class, "motorLauncher");
        motorLauncher.setDirection(DcMotor.Direction.REVERSE);
        motorLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLauncher.setVelocityPIDFCoefficients(MOTOR_VELO_PID_P, MOTOR_VELO_PID_I, MOTOR_VELO_PID_D, MOTOR_VELO_PID_F);

        servoHood = hardwareMap.get(Servo.class, "servoHood");
        servoAim = hardwareMap.get(Servo.class, "servoAim");
    }

    public void update(Gamepad gamepad, Telemetry telemetry) {
        if (gamepad.left_bumper && !leftBumperWasPressed) {
            autoshootaim = !autoshootaim;
        }
        leftBumperWasPressed = gamepad.left_bumper;

        double newAimPosition = this.aimPosition;

        if (autoshootaim) {
            telemetry.addData("Auto Turret", "Enabled");

            Pose robotPose = follower.getPose();
            // <<< FIXED: Removed the second argument to match the new method in TurretControl >>>
            double baseAimPosition = turretControl.calculateTurretAngle(robotPose);

            if (limelight.hasValidTarget()) {
                // Clamp the base position to see if we are at a physical limit.
                double clampedBaseAim = Math.max(SERVO_AIM_MIN, Math.min(SERVO_AIM_MAX, baseAimPosition));
                double pidCorrection = 0.0;

                // Only apply PID correction if the base aim is NOT at a limit.
                // This prevents the PID from fighting the physical stops.
                if (clampedBaseAim > SERVO_AIM_MIN && clampedBaseAim < SERVO_AIM_MAX) {
                    pidCorrection = limelight.getAimCorrection();
                }

                newAimPosition = clampedBaseAim + pidCorrection;

                aimAndPowerForDistance(limelight.getTargetArea());
            } else {
                newAimPosition = baseAimPosition;
                limelight.resetAimPID();
                setRPM(0);
            }
        } else {
            limelight.resetAimPID();
            telemetry.addData("Auto Turret", "Disabled");

            // Use player 2's left joystick to manually aim the turret
            double manualAimInput = gamepad.left_stick_x;
            if (Math.abs(manualAimInput) > 0.1) { // Apply a deadzone
                // This increment value scales the joystick input for finer control.
                // You can adjust this value for faster or slower manual aiming.
                newAimPosition += manualAimInput * 0.005;
            }

            // Use player 2's right joystick to manually control the hood
            double manualHoodInput = gamepad.right_stick_y;
            if (Math.abs(manualHoodInput) > 0.1) { // Apply a deadzone
                // Pushing up on the stick (negative value) decreases position, moving hood up.
                // Pushing down on the stick (positive value) increases position, moving hood down.
                hoodPosition += manualHoodInput * 0.005;
            }
            hoodPosition = Math.max(MIN_HOOD_ANGLE, Math.min(MAX_HOOD_ANGLE, hoodPosition));
            setHoodPosition(hoodPosition);

            if (gamepad.right_trigger > 0.1) {
                // Scale the trigger value from 0.1-1.0 to a 0.0-1.0 range
                double normalizedTrigger = (gamepad.right_trigger - 0.1) / 0.9;
                double manualRPM = MIN_RPM + normalizedTrigger * (MAX_RPM - MIN_RPM);
                setRPM(manualRPM);
            } else {
                setRPM(0);
            }
        }
        // Use the centralized, safe method to set the turret's position
        setAimPosition(newAimPosition);
    }

    public void setAutoShootAim(boolean autoAim) {
        this.autoshootaim = autoAim;
    }

    /**
     * Sets the turret's aim position while enforcing physical hardware limits.
     * @param position The desired, unclamped servo position.
     */
    public void setAimPosition(double position) {
        // Clamp the value to the servo's physical limits
        double clampedPosition = Math.max(SERVO_AIM_MIN, Math.min(SERVO_AIM_MAX, position));
        servoAim.setPosition(clampedPosition);
        this.aimPosition = clampedPosition; // Also update the internal state
    }

    public void aimAndPowerForDistance(double targetArea) {
        double currentMax = isTeleOp ? TELEOP_MAX_RPM : MAX_RPM;
        if (targetArea < MIN_TARGET_AREA || targetArea > MAX_TARGET_AREA) {
            setRPM(0);
            return;
        }

        double range = MAX_TARGET_AREA - MIN_TARGET_AREA;
        double fraction = (targetArea - MIN_TARGET_AREA) / range;

        double rpm = currentMax + fraction * (MIN_RPM - currentMax);
        double hoodAngle = MIN_HOOD_ANGLE + fraction * (MAX_HOOD_ANGLE - MIN_HOOD_ANGLE);

        double clampedRpm = Math.max(MIN_RPM, Math.min(rpm, currentMax));
        double clampedHoodAngle = Math.max(MIN_HOOD_ANGLE, Math.min(hoodAngle, MAX_HOOD_ANGLE));

        setRPM(clampedRpm);
        setHoodPosition(clampedHoodAngle);
        this.hoodPosition = clampedHoodAngle;
    }

    public void aimAndPowerFromPose() {
        Pose robotPose = follower.getPose();
        double targetX = limelight.getAlliance() == LimelightSubsystem.Alliance.RED ? 127 : 16;
        double targetY = 131;
        double dx = targetX - robotPose.getX();
        double dy = targetY - robotPose.getY();
        double distance = Math.sqrt(dx*dx + dy*dy);

        // This is a guess, you may need to tune these values
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
        double hoodAngle = MIN_HOOD_ANGLE + fraction * (MAX_HOOD_ANGLE - MIN_HOOD_ANGLE);


        double clampedRpm = Math.max(MIN_RPM, Math.min(rpm, currentMax));
        double clampedHoodAngle = Math.max(MIN_HOOD_ANGLE, Math.min(hoodAngle, MAX_HOOD_ANGLE));

        setRPM(clampedRpm);
        setHoodPosition(clampedHoodAngle);
        this.hoodPosition = clampedHoodAngle;
    }

    public void setRPM(double rpm) {
        this.targetRPM = rpm;
        double ticksPerSecond = (rpm * TICKS_PER_REVOLUTION) / 60.0;
        motorLauncher.setVelocity(ticksPerSecond);
    }

    public void setHoodPosition(double position) {
        servoHood.setPosition(position);
        this.hoodPosition = position;
    }

    public double getRPM() {
        return (motorLauncher.getVelocity() * 60.0) / TICKS_PER_REVOLUTION;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getHoodPosition() {
        return hoodPosition;
    }

    public double getAimPosition() {
        return aimPosition;
    }

    public boolean isRpmWithinTolerance() {
        return Math.abs(getRPM() - targetRPM) < RPM_TOLERANCE && targetRPM != 0;
    }

    public boolean isAutoShootAim() {
        return autoshootaim;
    }
}
