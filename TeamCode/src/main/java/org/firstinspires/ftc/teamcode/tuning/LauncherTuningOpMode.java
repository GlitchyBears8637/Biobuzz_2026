package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * LauncherTuningOpMode
 * 
 * Standalone OpMode to test and calibrate launcher flywheel speeds.
 * 
 * Controls:
 * - Dpad Up: Increase target RPM (+50 RPM)
 * - Dpad Down: Decrease target RPM (-50 RPM)
 * - Dpad Right: Fine increase (+10 RPM)
 * - Dpad Left: Fine decrease (-10 RPM)
 * - Y button: Jump to preset high (3400 RPM)
 * - X button: Jump to preset mid (3000 RPM)
 * - A button: Jump to preset low (2750 RPM)
 * - B button: Stop launcher (0 RPM)
 * - Right Bumper: Toggle motor ON / OFF at current target RPM
 */
@TeleOp(name = "Tuning: Launcher Speed", group = "Tuning")
public class LauncherTuningOpMode extends LinearOpMode {

    private static final double TICKS_PER_REVOLUTION = 28.0; // Standard REV / GoBilda HD Hex
    private static final double RPM_TOLERANCE = 50.0;

    // Velocity PIDF coefficients (matches LauncherSubsystem defaults)
    public static double MOTOR_VELO_PID_P = 25.0;
    public static double MOTOR_VELO_PID_I = 0.0;
    public static double MOTOR_VELO_PID_D = 10.0;
    public static double MOTOR_VELO_PID_F = 13.0;

    private double targetRPM = 2800.0;
    private boolean motorRunning = false;

    // Edge detection flags for button presses
    private boolean dpadUpWasPressed = false;
    private boolean dpadDownWasPressed = false;
    private boolean dpadRightWasPressed = false;
    private boolean dpadLeftWasPressed = false;
    private boolean rightBumperWasPressed = false;
    private boolean aWasPressed = false;
    private boolean bWasPressed = false;
    private boolean xWasPressed = false;
    private boolean yWasPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing launcher motor...");
        telemetry.update();

        DcMotorEx motorLauncher = hardwareMap.get(DcMotorEx.class, "motorLauncher");
        motorLauncher.setDirection(DcMotor.Direction.REVERSE);
        motorLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLauncher.setVelocityPIDFCoefficients(MOTOR_VELO_PID_P, MOTOR_VELO_PID_I, MOTOR_VELO_PID_D, MOTOR_VELO_PID_F);

        telemetry.addData("Status", "Ready. Press START to tune launcher.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Target RPM Adjustments ---
            // Coarse adjustments (+/- 50 RPM)
            if (gamepad1.dpad_up && !dpadUpWasPressed) {
                targetRPM = Math.min(4000, targetRPM + 50);
            }
            if (gamepad1.dpad_down && !dpadDownWasPressed) {
                targetRPM = Math.max(0, targetRPM - 50);
            }

            // Fine adjustments (+/- 10 RPM)
            if (gamepad1.dpad_right && !dpadRightWasPressed) {
                targetRPM = Math.min(4000, targetRPM + 10);
            }
            if (gamepad1.dpad_left && !dpadLeftWasPressed) {
                targetRPM = Math.max(0, targetRPM - 10);
            }

            // Presets
            if (gamepad1.y && !yWasPressed) targetRPM = 3400;
            if (gamepad1.x && !xWasPressed) targetRPM = 3000;
            if (gamepad1.a && !aWasPressed) targetRPM = 2750;

            // Stop immediately
            if (gamepad1.b && !bWasPressed) {
                motorRunning = false;
            }

            // Toggle running state
            if (gamepad1.right_bumper && !rightBumperWasPressed) {
                motorRunning = !motorRunning;
            }

            // Update edge detection states
            dpadUpWasPressed = gamepad1.dpad_up;
            dpadDownWasPressed = gamepad1.dpad_down;
            dpadRightWasPressed = gamepad1.dpad_right;
            dpadLeftWasPressed = gamepad1.dpad_left;
            rightBumperWasPressed = gamepad1.right_bumper;
            aWasPressed = gamepad1.a;
            bWasPressed = gamepad1.b;
            xWasPressed = gamepad1.x;
            yWasPressed = gamepad1.y;

            // --- Apply to Motor ---
            double currentRPM = (motorLauncher.getVelocity() * 60.0) / TICKS_PER_REVOLUTION;
            double effectiveTarget = motorRunning ? targetRPM : 0.0;
            double targetTicksPerSec = (effectiveTarget * TICKS_PER_REVOLUTION) / 60.0;
            motorLauncher.setVelocity(targetTicksPerSec);

            double rpmError = currentRPM - effectiveTarget;
            boolean atSpeed = motorRunning && Math.abs(rpmError) <= RPM_TOLERANCE;

            // --- Telemetry ---
            telemetry.addData("=== LAUNCHER TUNING ===", motorRunning ? "RUNNING" : "STOPPED");
            telemetry.addData("Motor State", motorRunning ? "[ON] (RB to toggle)" : "[OFF] (RB to toggle)");
            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("Current RPM", "%.1f", currentRPM);
            telemetry.addData("RPM Error", "%.1f", rpmError);
            telemetry.addData("At Target Speed?", atSpeed ? "YES (READY TO SHOOT)" : "NO");
            telemetry.addLine();
            telemetry.addLine("Controls:");
            telemetry.addLine("  Dpad Up/Down : +/- 50 RPM");
            telemetry.addLine("  Dpad Left/Rt : +/- 10 RPM");
            telemetry.addLine("  Y / X / A    : 3400 / 3000 / 2750 RPM presets");
            telemetry.addLine("  B            : Stop");
            telemetry.addLine("  RB           : Toggle Motor ON/OFF");
            telemetry.update();
        }

        motorLauncher.setVelocity(0);
        motorLauncher.setPower(0);
    }
}
