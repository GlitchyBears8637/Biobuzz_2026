package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * IntakeTuningOpMode
 * 
 * Standalone OpMode to test and find optimal intake and tube-feed speeds.
 * 
 * Controls:
 * - Dpad Up: Increase intake power (+0.05)
 * - Dpad Down: Decrease intake power (-0.05)
 * - Dpad Right: Fine increase (+0.01)
 * - Dpad Left: Fine decrease (-0.01)
 * - A button: Toggle Intake ON / OFF
 * - Y button: Reverse intake (clearing jams)
 * - Right Bumper: Toggle Tube Wheel continuous servo ON / OFF
 * - Left Bumper: Reverse Tube Wheel
 * - B button: Stop everything
 */
@TeleOp(name = "Tuning: Intake & Tube Speed", group = "Tuning")
public class IntakeTuningOpMode extends LinearOpMode {

    private double intakePowerTarget = 0.50; // Default test power
    private boolean intakeRunning = false;
    private boolean intakeReversed = false;

    private boolean tubeWheelRunning = false;
    private boolean tubeWheelReversed = false;

    // Edge detection flags
    private boolean dpadUpWasPressed = false;
    private boolean dpadDownWasPressed = false;
    private boolean dpadRightWasPressed = false;
    private boolean dpadLeftWasPressed = false;
    private boolean aWasPressed = false;
    private boolean bWasPressed = false;
    private boolean yWasPressed = false;
    private boolean rbWasPressed = false;
    private boolean lbWasPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing intake motor and tube servo...");
        telemetry.update();

        DcMotor motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");
        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        CRServo servoTubeWheel = null;
        try {
            servoTubeWheel = hardwareMap.get(CRServo.class, "servoTubeWheel");
        } catch (Exception e) {
            telemetry.addData("Warning", "servoTubeWheel not found in hardware map");
        }

        telemetry.addData("Status", "Ready. Press START to tune intake.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Power level adjustments
            if (gamepad1.dpad_up && !dpadUpWasPressed) {
                intakePowerTarget = Math.min(1.0, intakePowerTarget + 0.05);
            }
            if (gamepad1.dpad_down && !dpadDownWasPressed) {
                intakePowerTarget = Math.max(0.0, intakePowerTarget - 0.05);
            }
            if (gamepad1.dpad_right && !dpadRightWasPressed) {
                intakePowerTarget = Math.min(1.0, intakePowerTarget + 0.01);
            }
            if (gamepad1.dpad_left && !dpadLeftWasPressed) {
                intakePowerTarget = Math.max(0.0, intakePowerTarget - 0.01);
            }

            // Intake toggle
            if (gamepad1.a && !aWasPressed) {
                intakeRunning = !intakeRunning;
                intakeReversed = false;
            }

            // Reverse intake (for jam testing)
            if (gamepad1.y && !yWasPressed) {
                intakeReversed = !intakeReversed;
                if (intakeReversed) intakeRunning = true;
            }

            // Tube wheel toggles
            if (gamepad1.right_bumper && !rbWasPressed) {
                tubeWheelRunning = !tubeWheelRunning;
                tubeWheelReversed = false;
            }
            if (gamepad1.left_bumper && !lbWasPressed) {
                tubeWheelReversed = !tubeWheelReversed;
                if (tubeWheelReversed) tubeWheelRunning = true;
            }

            // Stop all
            if (gamepad1.b && !bWasPressed) {
                intakeRunning = false;
                intakeReversed = false;
                tubeWheelRunning = false;
                tubeWheelReversed = false;
            }

            // Update edge detection
            dpadUpWasPressed = gamepad1.dpad_up;
            dpadDownWasPressed = gamepad1.dpad_down;
            dpadRightWasPressed = gamepad1.dpad_right;
            dpadLeftWasPressed = gamepad1.dpad_left;
            aWasPressed = gamepad1.a;
            bWasPressed = gamepad1.b;
            yWasPressed = gamepad1.y;
            rbWasPressed = gamepad1.right_bumper;
            lbWasPressed = gamepad1.left_bumper;

            // Apply motor power
            double appliedPower = 0.0;
            if (intakeRunning) {
                appliedPower = intakeReversed ? -intakePowerTarget : intakePowerTarget;
            }
            motorIntake.setPower(appliedPower);

            // Apply tube wheel power
            if (servoTubeWheel != null) {
                double tubePower = 0.0;
                if (tubeWheelRunning) {
                    tubePower = tubeWheelReversed ? -1.0 : 1.0;
                }
                servoTubeWheel.setPower(tubePower);
            }

            // Telemetry
            telemetry.addData("=== INTAKE TUNING ===", intakeRunning ? "RUNNING" : "STOPPED");
            telemetry.addData("Intake Power Target", "%.2f (%.0f%%)", intakePowerTarget, intakePowerTarget * 100);
            telemetry.addData("Intake Applied Power", "%.2f", appliedPower);
            telemetry.addData("Intake Direction", intakeReversed ? "REVERSE (OUTTAKE)" : "FORWARD (INTAKE)");
            telemetry.addData("Tube Wheel Servo", tubeWheelRunning ? (tubeWheelReversed ? "REVERSE (-1.0)" : "FORWARD (1.0)") : "OFF (0.0)");
            telemetry.addLine();
            telemetry.addLine("Controls:");
            telemetry.addLine("  Dpad Up/Down : +/- 5% power");
            telemetry.addLine("  Dpad Left/Rt : +/- 1% power");
            telemetry.addLine("  A            : Toggle Intake ON/OFF");
            telemetry.addLine("  Y            : Toggle Intake REVERSE");
            telemetry.addLine("  RB           : Toggle Tube Wheel ON/OFF");
            telemetry.addLine("  LB           : Toggle Tube Wheel REVERSE");
            telemetry.addLine("  B            : Stop Everything");
            telemetry.update();
        }

        motorIntake.setPower(0);
        if (servoTubeWheel != null) {
            servoTubeWheel.setPower(0);
        }
    }
}
