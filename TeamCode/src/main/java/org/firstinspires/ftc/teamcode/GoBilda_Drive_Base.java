package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHdw.DriveSubsystems;
import org.firstinspires.ftc.teamcode.RobotHdw.RobotHdw;
import org.firstinspires.ftc.teamcode.RobotHdw.KonamiCodeManager;
import org.firstinspires.ftc.teamcode.RobotHdw.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * GoBilda TeleOp (Driver Controlled OpMode)
 * 
 * EDUCATIONAL OVERVIEW FOR ROOKIE PROGRAMMERS:
 * This is the main TeleOp OpMode for driving the robot and running mechanisms during matches.
 * 
 * ALLIANCE SELECTOR PROMPT:
 * During the INIT phase (before pressing START), drivers can press:
 * - [X] on Gamepad 1 for Red Alliance
 * - [B] on Gamepad 1 for Blue Alliance
 */
@TeleOp(name = "GoBilda TeleOp", group = "TeleOp")
public class GoBilda_Drive_Base extends LinearOpMode {

    RobotHdw robot = new RobotHdw();
    Follower follower;
    LimelightSubsystem limelightSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize hardware and Pedro Pathing follower
        robot.init(hardwareMap);
        follower = Constants.create(hardwareMap);

        // Initialize Limelight camera
        limelightSubsystem = new LimelightSubsystem(hardwareMap);
        limelightSubsystem.start();

        // Default to BLUE alliance if not changed during INIT
        LimelightSubsystem.Alliance selectedAlliance = LimelightSubsystem.Alliance.BLUE;
        limelightSubsystem.setAlliance(selectedAlliance);

        KonamiCodeManager konamiCodeManager = new KonamiCodeManager(hardwareMap, telemetry);
        DriveSubsystems driveSubsystem = new DriveSubsystems(follower);

        // --- INIT PHASE ALLIANCE SELECTOR PROMPT ---
        // Runs repeatedly while waiting for the match to START on the Driver Station
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) {
                selectedAlliance = LimelightSubsystem.Alliance.RED;
                limelightSubsystem.setAlliance(selectedAlliance);
            } else if (gamepad1.b) {
                selectedAlliance = LimelightSubsystem.Alliance.BLUE;
                limelightSubsystem.setAlliance(selectedAlliance);
            }

            telemetry.addData("=== ALLIANCE SELECTOR ===", "");
            telemetry.addData("Press [X] on Gamepad 1 for", "RED ALLIANCE");
            telemetry.addData("Press [B] on Gamepad 1 for", "BLUE ALLIANCE");
            telemetry.addData("Current Selected Alliance", selectedAlliance);
            telemetry.addData("Status", "Initialization Complete. Ready for START.");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            // === Drive & Sensor Updates ===
            driveSubsystem.loop(gamepad1);
            konamiCodeManager.update(gamepad1);
            
            // Update Limelight camera and reaffirm robot position with AprilTags
            limelightSubsystem.update();
            limelightSubsystem.reaffirmPosition(follower);

            // === Telemetry Readouts ===
            telemetry.addData("Alliance", limelightSubsystem.getAlliance());
            telemetry.addData("Drive Speed", "30% (Hold Right Bumper for 80% Turbo)");
            telemetry.addData("Status", "RUN BOB RUN!");
            telemetry.update();
        }
    }
}
