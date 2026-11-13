package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHdw.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.RobotHdw;
import org.firstinspires.ftc.teamcode.RobotHdw.TubeOfDespairSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.RobotAimSubsystem;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * RedAutonomous (Biobuzz Season Autonomous Program)
 * 
 * EDUCATIONAL TEMPLATE FOR ROOKIE PROGRAMMERS:
 * This file outlines a complete, step-by-step example workflow for an FTC Autonomous period on the Red Alliance.
 * For the Biobuzz game, a typical autonomous routine follows this timeline:
 * 
 * 1. INIT PHASE: Set alliance color (RED), reset odometry pose, and initialize mechanisms.
 * 2. START / PRELOAD: Drive from start tile to scoring position, aim at the closest Red bucket, and launch preloaded game pieces.
 * 3. INTAKE CYCLE: Drive to game piece collection zone, run intake and 4-section ball queue until full.
 * 4. RETURN & SCORE: Drive back to scoring bucket, re-aim using Limelight AprilTags, and score second batch.
 * 5. PARK: Navigate to the designated parking/observation zone before time expires.
 */
@Autonomous(name = "Red Alliance Autonomous (Template)", group = "Autonomous")
public class RedAutonomous extends LinearOpMode {

    // Robot hardware and subsystems
    RobotHdw robot = new RobotHdw();
    Follower follower;
    LimelightSubsystem limelightSubsystem;
    RobotAimSubsystem robotAimSubsystem;
    LauncherSubsystem launcherSubsystem;
    ColorSensorSubsystem colorSensorSubsystem;
    TubeOfDespairSubsystem tubeOfDespairSubsystem;
    IntakeSubsystem intakeSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- STEP 1: INITIALIZATION PHASE ---
        // Map hardware and create Pedro Pathing follower
        robot.init(hardwareMap);
        follower = Constants.create(hardwareMap);

        // Initialize vision, targeting, and scoring subsystems
        limelightSubsystem = new LimelightSubsystem(hardwareMap);
        limelightSubsystem.start();
        limelightSubsystem.setAlliance(LimelightSubsystem.Alliance.RED); // Lock camera to Red Alliance

        robotAimSubsystem = new RobotAimSubsystem(robot, limelightSubsystem);
        launcherSubsystem = new LauncherSubsystem(hardwareMap, limelightSubsystem, robotAimSubsystem, follower);
        colorSensorSubsystem = new ColorSensorSubsystem(robot);
        tubeOfDespairSubsystem = new TubeOfDespairSubsystem(robot, launcherSubsystem, colorSensorSubsystem);
        intakeSubsystem = new IntakeSubsystem(robot);

        // Set starting robot pose (X, Y in inches, Heading in radians) on the Red starting tile
        // TODO: Adjust these starting coordinates to match your robot's physical placement on the field
        Pose startPose = new Pose(58.6, 12.0, Math.toRadians(0.0));
        follower.setPose(startPose);

        telemetry.addData("Status", "Red Autonomous Initialized. Ready for START.");
        telemetry.update();

        // Wait here until the driver station presses START
        waitForStart();

        if (opModeIsActive()) {

            // --- STEP 2: SCORING PRELOADED GAME PIECES ---
            telemetry.addData("Autonomous", "Scoring Preload...");
            telemetry.update();

            // EXAMPLE A: Drive to scoring position using Pedro Pathing (uncomment when paths are defined)
            /*
            Path scorePath = new Path(new BezierCurve(new Point(12.0, 60.0), new Point(36.0, 60.0), new Point(36.0, 24.0)));
            scorePath.setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(90.0));
            follower.followPath(scorePath);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                limelightSubsystem.update();
                limelightSubsystem.reaffirmPosition(follower); // Reaffirm position with AprilTags while driving
            }
            */

            // EXAMPLE B: Aim robot toward the closest Red bucket using RobotAimSubsystem
            /*
            double headingError = robotAimSubsystem.calculateRobotAimHeadingError(follower.closestPose());
            // Turn robot until heading error is close to 0
            */

            // EXAMPLE C: Spin up launcher and launch game pieces from the 4-section queue
            /*
            launcherSubsystem.setRPM(3200); // Set launcher flywheel speed
            while (opModeIsActive() && !launcherSubsystem.isRpmWithinTolerance()) {
                // Wait until flywheel reaches target speed
            }
            // Run tube wheel to push balls into shooter
            robot.servoTubeWheel.setPower(1.0);
            sleep(1500); // 1.5 seconds to launch
            robot.servoTubeWheel.setPower(0.0);
            launcherSubsystem.setRPM(0); // Stop launcher
            */


            // --- STEP 3: COLLECTING MORE GAME PIECES (INTAKE CYCLE) ---
            telemetry.addData("Autonomous", "Intaking Game Pieces...");
            telemetry.update();

            /*
            Path intakePath = new Path(new BezierLine(new Point(36.0, 24.0), new Point(60.0, 24.0)));
            follower.followPath(intakePath);
            intakeSubsystem.turnOn(); // Turn on intake

            while (opModeIsActive() && follower.isBusy() && !tubeOfDespairSubsystem.isQueueFull()) {
                follower.update();
                limelightSubsystem.update();
                limelightSubsystem.reaffirmPosition(follower);
                colorSensorSubsystem.updateSensorReadings();
                tubeOfDespairSubsystem.update();
                intakeSubsystem.update(tubeOfDespairSubsystem.isQueueFull());
            }
            intakeSubsystem.turnOff();
            */


            // --- STEP 4: RETURN AND SCORE SECOND BATCH ---
            telemetry.addData("Autonomous", "Returning to Score...");
            telemetry.update();

            /*
            // Drive back to scoring bucket, re-aim using Limelight, and launch again
            */


            // --- STEP 5: PARK IN OBSERVATION ZONE ---
            telemetry.addData("Autonomous", "Parking...");
            telemetry.update();

            /*
            Path parkPath = new Path(new BezierLine(follower.closestPose().toPoint(), new Point(16.0, 12.0)));
            follower.followPath(parkPath);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
            }
            */

            // End of Autonomous loop
            while (opModeIsActive()) {
                follower.update();
                telemetry.addData("Autonomous", "Complete. Pausing.");
                telemetry.update();
            }
        }
    }
}
