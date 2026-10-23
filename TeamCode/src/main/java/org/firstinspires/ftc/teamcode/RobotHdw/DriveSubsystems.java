package org.firstinspires.ftc.teamcode.RobotHdw;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * DriveSubsystems
 * 
 * EDUCATIONAL OVERVIEW FOR ROOKIE PROGRAMMERS:
 * This subsystem handles manual driver control of the robot using Pedro Pathing.
 * It reads the left and right analog joysticks on Gamepad 1 and passes them into 
 * the Follower's manual drive method.
 * 
 * SPEED SCALING FEATURE:
 * - Regular speed is set to 30% (0.3) for precise driving and control.
 * - Holding the Right Bumper on Gamepad 1 boosts the speed to 80% (0.8) for fast movement.
 */
public class DriveSubsystems {

    private final Follower follower;

    public DriveSubsystems(Follower follower) {
        this.follower = follower;
    }

    /**
     * Reads joystick inputs every loop cycle, applies speed scaling based on the right bumper,
     * and commands the drivetrain.
     * 
     * @param gamepad Gamepad 1 controller.
     */
    public void loop(Gamepad gamepad) {
        if (follower == null || gamepad == null) return;

        // Determine speed multiplier: 80% (0.8) if right bumper is held, otherwise 30% (0.3)
        double speedMultiplier = gamepad.right_bumper ? 0.8 : 0.3;

        // Multiply joystick inputs by the speed factor to scale robot velocity
        follower.manual(
                -gamepad.left_stick_y * speedMultiplier,
                gamepad.left_stick_x * speedMultiplier,
                gamepad.right_stick_x * speedMultiplier
        );

        follower.update();
    }
}
