package org.firstinspires.ftc.teamcode.RobotHdw;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * DriveSubsystems
 *
 * EDUCATIONAL OVERVIEW FOR ROOKIE PROGRAMMERS:
 * This subsystem handles driver-controlled (TeleOp) movement of the robot using Pedro Pathing.
 *
 * Key concepts:
 * 1. Mecanum Drive Kinematics: Allows omnidirectional movement (moving forward/backward,
 *    strafing left/right, and rotating in place simultaneously).
 * 2. Gamepad Inputs:
 *    - Left Stick Y: Controls forward / reverse driving.
 *    - Left Stick X: Controls strafing (sliding left / right).
 *    - Right Stick X: Controls turning (rotation).
 */
public class DriveSubsystems {

    private final Follower follower;

    /**
     * Constructor: Connects the drive subsystem to the Pedro Pathing Follower instance.
     */
    public DriveSubsystems(Follower follower) {
        this.follower = follower;
    }

    /**
     * Called every loop cycle in TeleOp to read gamepad joysticks and drive the robot.
     */
    public void loop(Gamepad gamepad) {
        if (follower == null || gamepad == null) return;

        // Feed driver joystick values into Pedro Pathing's manual drive function
        follower.manual(
                gamepad.left_stick_y,    // Forward / Reverse
                gamepad.left_stick_x,    // Strafe Left / Right
                gamepad.right_stick_x    // Turn Left / Right
        );

        // Update the follower's internal position and motor outputs
        follower.update();
    }
}
