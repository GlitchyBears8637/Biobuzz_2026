package org.firstinspires.ftc.teamcode.RobotHdw;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DriveSubsystems {

    private final Follower follower;

    public DriveSubsystems(Follower follower) {
        this.follower = follower;
    }

    public void loop(Gamepad gamepad) {
        if (follower == null || gamepad == null) return;

        follower.manual(
                -gamepad.left_stick_y,
                gamepad.left_stick_x,
                gamepad.right_stick_x
        );

        follower.update();
    }
}
