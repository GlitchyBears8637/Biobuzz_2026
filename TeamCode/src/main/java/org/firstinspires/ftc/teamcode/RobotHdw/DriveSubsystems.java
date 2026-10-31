package org.firstinspires.ftc.teamcode.RobotHdw;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;


public class DriveSubsystems {

    private Follower follower;

    public DriveSubsystems(HardwareRobot robot) {
    };

    public void init(HardwareMap HardwareRobot) {
        follower = Constants.create(HardwareRobot);
    }
    
    public void loop() {
        follower.manual(
                -gamepad.left_stick_y * speedMultiplier,
                gamepad.left_stick_x * speedMultiplier,
                gamepad.right_stick_x * speedMultiplier
        );

        follower.update();
    }
}
