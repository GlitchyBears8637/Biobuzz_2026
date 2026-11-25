package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHdw.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.RobotAimSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.RobotHdw;
import org.firstinspires.ftc.teamcode.RobotHdw.TubeOfDespairSubsystem;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PathsRed;

/**
 * RedAutonomous (Biobuzz Season Autonomous Program)
 * 
 * Executes the Red Alliance autonomous sequence driven by PathsRed:
 * 1. Score preloaded game pieces at start.
 * 2. Move forward, turn toward garden.
 * 3. Drive into garden while running intake & tube wheel to collect game pieces.
 * 4. Move back from garden and return to shooting position.
 * 5. Spin up launcher and score collected pieces.
 * 6. Follow curved path to park.
 */
@Autonomous(name = "Red Alliance Autonomous", group = "Autonomous")
public class RedAutonomous extends LinearOpMode {

    // Robot hardware and subsystems
    RobotHdw robot = new RobotHdw();
    Follower follower;
    RobotAimSubsystem robotAimSubsystem;
    LauncherSubsystem launcherSubsystem;
    ColorSensorSubsystem colorSensorSubsystem;
    TubeOfDespairSubsystem tubeOfDespairSubsystem;
    IntakeSubsystem intakeSubsystem;
    PathsRed pathsRed;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- STEP 1: INITIALIZATION PHASE ---
        robot.init(hardwareMap);
        follower = Constants.create(hardwareMap);
        pathsRed = new PathsRed();

        robotAimSubsystem = new RobotAimSubsystem(robot, null);
        launcherSubsystem = new LauncherSubsystem(hardwareMap, null, robotAimSubsystem, follower);
        colorSensorSubsystem = new ColorSensorSubsystem(robot);
        tubeOfDespairSubsystem = new TubeOfDespairSubsystem(robot, launcherSubsystem, colorSensorSubsystem);
        intakeSubsystem = new IntakeSubsystem(robot);

        // Set starting robot pose from PathsRed
        follower.setPose(pathsRed.start);

        telemetry.addData("Status", "Red Autonomous Initialized. Ready for START.");
        telemetry.addData("Start Pose", "X: %.1f, Y: %.1f, Heading: %.1f deg", 
                pathsRed.start.x(), pathsRed.start.y(), Math.toDegrees(pathsRed.start.heading()));
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // --- STEP 2: SHOOT PRELOAD ---
            telemetry.addData("Auto Phase", "1/6: Shooting Preload");
            telemetry.update();

            shootGamePieces(3000, 2.5); // Spin up to 3000 RPM and launch

            // --- STEP 3: MOVE FORWARD AFTER SHOOT ---
            telemetry.addData("Auto Phase", "2/6: Moving Forward After Shoot");
            telemetry.update();
            followPathBlocking(pathsRed.moveForwardAfterShoot());

            // --- STEP 4: TURN TOWARD GARDEN ---
            telemetry.addData("Auto Phase", "3/6: Turn Toward Garden");
            telemetry.update();
            followPathBlocking(pathsRed.moveTurnTowardsGarden());

            // --- STEP 5: INTAKE GARDEN BALLS ---
            telemetry.addData("Auto Phase", "4/6: Collecting Garden Balls");
            telemetry.update();

            // Turn intake motor on
            intakeSubsystem.turnOn();

            followPathBlocking(pathsRed.grabGardenBalls());

            // --- STEP 6: RETURN TO SHOOTING POSITION ---
            telemetry.addData("Auto Phase", "5/6: Returning To Shoot");
            telemetry.update();

            followPathBlocking(pathsRed.moveBackFromGarden());
            followPathBlocking(pathsRed.moveFurtherFromGarden());

            // Stop intake once leaving collection zone
            intakeSubsystem.turnOff();

            followPathBlocking(pathsRed.moveBackToShoot());

            // Score collected game pieces
            shootGamePieces(3000, 3.0);

            // --- STEP 7: PARK ---
            telemetry.addData("Auto Phase", "6/6: Parking");
            telemetry.update();
            followPathBlocking(pathsRed.park());

            telemetry.addData("Auto Phase", "Complete! Parked.");
            telemetry.update();
        }
    }

    /**
     * Follows a Pedro Pathing path until completion while updating odometry.
     */
    private void followPathBlocking(Path path) {
        follower.follow(path);
        while (opModeIsActive() && !follower.atParametricEnd()) {
            follower.update();
        }
    }

    /**
     * Spins up the flywheel and indexes the tube pockets to launch game pieces.
     */
    private void shootGamePieces(double targetRpm, double durationSeconds) {
        launcherSubsystem.setRPM(targetRpm);
        ElapsedTime timer = new ElapsedTime();

        // Wait up to 1.5s for flywheel to reach target speed
        while (opModeIsActive() && timer.seconds() < 1.5 && !launcherSubsystem.isRpmWithinTolerance()) {
            follower.update();
        }

        // Cycle through pockets to feed balls into launcher
        ElapsedTime shootTimer = new ElapsedTime();
        double stepInterval = durationSeconds / 4.0;
        int step = 0;
        while (opModeIsActive() && shootTimer.seconds() < durationSeconds) {
            int currentStep = (int) (shootTimer.seconds() / stepInterval);
            if (currentStep > step) {
                tubeOfDespairSubsystem.nextPocket();
                step = currentStep;
            }
            follower.update();
        }

        // Stop launcher and rewind tube
        tubeOfDespairSubsystem.rewind();
        launcherSubsystem.setRPM(0);
    }
}
