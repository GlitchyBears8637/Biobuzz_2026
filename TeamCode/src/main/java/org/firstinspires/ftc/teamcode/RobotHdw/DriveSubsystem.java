package org.firstinspires.ftc.teamcode.RobotHdw;

import com.qualcomm.robotcore.hardware.Gamepad;

public class DriveSubsystem {

    private final HardwareRobot robot;
    private float overdrive = 0.35F;

    public DriveSubsystem(HardwareRobot robot) {
        this.robot = robot;
    }

    public void update(Gamepad gamepad) {
        if (gamepad.right_bumper) {
            overdrive = 0.8F; // Set to faster speed when bumper is pressed
        } else {
            overdrive = 0.35F;
        }

        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x * 2;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator * overdrive;
        double backLeftPower = (y - x + rx) / denominator * overdrive;
        double frontRightPower = (y - x - rx) / denominator * overdrive;
        double backRightPower = (y + x - rx) / denominator * overdrive;

        robot.motorFrontLeft.setPower(frontLeftPower);
        robot.motorBackLeft.setPower(backLeftPower);
        robot.motorFrontRight.setPower(frontRightPower);
        robot.motorBackRight.setPower(backRightPower);
    }

    public float getOverdrive() {
        return overdrive;
    }
}