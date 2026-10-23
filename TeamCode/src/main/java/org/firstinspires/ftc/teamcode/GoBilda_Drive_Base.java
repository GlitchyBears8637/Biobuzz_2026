package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHdw.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.DriveSubsystems;
import org.firstinspires.ftc.teamcode.RobotHdw.RobotHdw;
import org.firstinspires.ftc.teamcode.RobotHdw.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.KonamiCodeManager;
import org.firstinspires.ftc.teamcode.RobotHdw.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.TubeWheelSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.TurretControl;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp
public class GoBilda_Drive_Base extends LinearOpMode {

    // Create an instance of the hardware class
    RobotHdw robot = new RobotHdw();
    Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware from the HardwareRobot class
        robot.init(hardwareMap);

        // Subsystem Initialization
        follower = Constants.create(hardwareMap);


        /*
        LimelightSubsystem limelightSubsystem = new LimelightSubsystem(hardwareMap);
        sleep(500);

        TurretControl turretControl = new TurretControl(robot, limelightSubsystem);
        LauncherSubsystem launcherSubsystem = new LauncherSubsystem(hardwareMap, limelightSubsystem, turretControl, follower);
        ColorSensorSubsystem colorSensorSubsystem = new ColorSensorSubsystem(robot);
        TubeWheelSubsystem tubeWheelSubsystem = new TubeWheelSubsystem(robot, launcherSubsystem, colorSensorSubsystem);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(robot);
        */
        KonamiCodeManager konamiCodeManager = new KonamiCodeManager(hardwareMap, telemetry);
        DriveSubsystems driveSubsystem = new DriveSubsystems(follower);

        telemetry.addData("Status", "Initialization Complete. Ready for START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // === DRIVE & FOLLOWER UPDATE ===
            driveSubsystem.loop(gamepad1);
            konamiCodeManager.update(gamepad1);

            /*
            limelightSubsystem.update();
            colorSensorSubsystem.updateSensorReadings();
            tubeWheelSubsystem.update();
            intakeSubsystem.update(tubeWheelSubsystem.isQueueFull(), gamepad2);
            launcherSubsystem.update(gamepad2, telemetry);
            colorSensorSubsystem.updateLamp4(tubeWheelSubsystem.isQueueFull(), launcherSubsystem.isAutoShootAim(), limelightSubsystem.hasValidTarget());
            */

            // === TELEMETRY ===
            telemetry.addData("Drive", "Active");
            telemetry.addData("Status", "RUN BOB RUN!");
            telemetry.update();
        }
    }
}

