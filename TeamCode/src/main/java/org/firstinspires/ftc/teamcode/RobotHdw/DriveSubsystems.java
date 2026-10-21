package org.firstinspires.ftc.teamcode.RobotHdw;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

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
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        follower.update();
    }
}