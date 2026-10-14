package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHdw.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.DriveSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.HardwareRobot;
import org.firstinspires.ftc.teamcode.RobotHdw.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.KonamiCodeManager;
import org.firstinspires.ftc.teamcode.RobotHdw.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.TubeWheelSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.TurretControl;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Paths;

@TeleOp
public class GoBilda_Drive_Base_TWOPLAYER extends LinearOpMode {

    // Create an instance of the hardware class
    HardwareRobot robot = new HardwareRobot();
    Follower follower;
    private LimelightSubsystem.Alliance alliance;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware from the HardwareRobot class
        robot.init(hardwareMap);

        // Subsystem Initialization
        follower = Constants.create(hardwareMap);
    }
        LimelightSubsystem limelightSubsystem = new LimelightSubsystem(hardwareMap);
        Paths paths = new Paths();

        // Add a delay to allow the Limelight to fully power on and initialize.
        // This helps prevent race conditions where we try to set the pipeline
        // before the Limelight is ready to accept commands.
        sleep(500); // 1-second delay

        TurretControl turretControl = new TurretControl(robot, limelightSubsystem); // Corrected
        LauncherSubsystem launcherSubsystem = new LauncherSubsystem(hardwareMap, limelightSubsystem, turretControl, follower); // Corrected
        launcherSubsystem.setTeleOpMode(true);
        ColorSensorSubsystem colorSensorSubsystem = new ColorSensorSubsystem(robot);
        TubeWheelSubsystem tubeWheelSubsystem = new TubeWheelSubsystem(robot, launcherSubsystem, colorSensorSubsystem);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(robot);
        KonamiCodeManager konamiCodeManager = new KonamiCodeManager(hardwareMap, telemetry);
        DriveSubsystem driveSubsystem = new DriveSubsystem(robot);
        BallLifterSubsystem ballLifterSubsystem = new BallLifterSubsystem(robot, launcherSubsystem);

        // === Read Autonomous State ===
        Pose autonomousPose = GameState.readPose();
        follower.setPose(autonomousPose);
        alliance = GameState.readAlliance();
        PatternManager.detectedPosition = GameState.readPattern();

        // After reading the alliance, clear it so it's not used again on the next TeleOp run
        GameState.saveAlliance(null);

        // === ALLIANCE & POSE SELECTION (if not set by Autonomous) ===
        if (alliance == null) {
            telemetry.addLine("Select Alliance: Press X for Blue, B for Red");
            telemetry.update();

            // Alliance selection loop
            while (!isStarted() && !isStopRequested()) {
                if (gamepad1.x) {
                    alliance = LimelightSubsystem.Alliance.BLUE;
                    telemetry.addData("Alliance Selected", "Blue");
                    telemetry.update();
                    break;
                } else if (gamepad1.b) {
                    alliance = LimelightSubsystem.Alliance.RED;
                    telemetry.addData("Alliance Selected", "Red");
                    telemetry.update();
                    break;
                }
            }

            // Pose selection, only if an alliance was chosen by the driver
            if (alliance != null) {
                telemetry.addLine("Select Start Side: D-Pad Up for FAR, D-Pad Down for CLOSE");
                telemetry.update();

                Pose startPose = null;
                // Pose selection loop
                while (!isStarted() && !isStopRequested()) {
                    if (gamepad1.dpad_up) {
                        startPose = (alliance == LimelightSubsystem.Alliance.BLUE) ? paths.blueFarstartPose : paths.redFarstartPose;
                        telemetry.addData("Start Side Selected", "FAR");
                        telemetry.update();
                        break;
                    } else if (gamepad1.dpad_down) {
                        startPose = (alliance == LimelightSubsystem.Alliance.BLUE) ? paths.blueCloseStartPose : paths.redCloseStartPose;
                        telemetry.addData("Start Side Selected", "CLOSE");
                        telemetry.update();
                        break;
                    }
                }

                // If start is pressed before a pose is selected, default to FAR for the chosen alliance
                if (startPose == null) {
                    startPose = (alliance == LimelightSubsystem.Alliance.BLUE) ? paths.blueFarstartPose : paths.redFarstartPose;
                }
                follower.setPose(startPose);

            } else {
                // If start is pressed before an alliance is selected, default to RED alliance and FAR start pose
                alliance = LimelightSubsystem.Alliance.RED;
                follower.setPose(paths.redFarstartPose);
            }

        } else {
            telemetry.addData("Alliance Loaded from Auto:", alliance);
            telemetry.update();
        }

        // Start the Limelight polling before setting the alliance
        // to ensure the pipeline switch command is received.
        limelightSubsystem.start();

        // === Set Alliance for Subsystems (ONCE) ===
        limelightSubsystem.setAlliance(alliance);


        // === Reduce Telemetry and File I/O rates to prevent lag ===
        telemetry.setMsTransmissionInterval(100); // Send telemetry 10 times a second
        int loopCounter = 0;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        telemetry.addData("Status", "Initialization Complete. Ready for START.");
        telemetry.update();


        waitForStart();

        if (isStopRequested()) {
            // Clear the pose and pattern files on an early exit to ensure a clean start for the next run
            GameState.savePose(new Pose(0, 0, 0));
            GameState.savePattern(null);
            return;
        }

        try {
            while (opModeIsActive()) {
                // === UPDATE SUBSYSTEMS FIRST ===
                follower.update(); // This is critical for TeleOp position tracking!

                // Only save the pose every 10th loop cycle to reduce lag
                if (loopCounter % 10 == 0) {
                    GameState.savePose(follower.getPose());
                }
                loopCounter++;

                limelightSubsystem.update();
                colorSensorSubsystem.updateSensorReadings();
                colorSensorSubsystem.updateLamps(PatternManager.colorPattern);
                tubeWheelSubsystem.update();
                intakeSubsystem.update(tubeWheelSubsystem.isQueueFull(), gamepad2);
                konamiCodeManager.update(gamepad1);
                driveSubsystem.update(gamepad1);
                ballLifterSubsystem.update(gamepad2);
                launcherSubsystem.update(gamepad2, telemetry);

                // Update lamp 4 based on system state
                colorSensorSubsystem.updateLamp4(tubeWheelSubsystem.isQueueFull(), launcherSubsystem.isAutoShootAim(), limelightSubsystem.hasValidTarget());

                // === TELEMETRY ===
                telemetry.addData("Alliance", alliance);
                limelightSubsystem.addTelemetry(telemetry);
                telemetry.addData("Launcher RPM", launcherSubsystem.getRPM());
                telemetry.addData("Launcher Target RPM", launcherSubsystem.getTargetRPM());
                telemetry.addData("Aim Position", launcherSubsystem.getAimPosition());
                telemetry.addData("Hood Position", launcherSubsystem.getHoodPosition());
                telemetry.addData("Lift Position", ballLifterSubsystem.getPosition());
                telemetry.addData("Tube Wheel Power", robot.servoTubeWheel.getPower());
                colorSensorSubsystem.addTelemetry(telemetry);
                telemetry.addData("Overdrive", driveSubsystem.getOverdrive());
                telemetry.addData("Status", "RUN BOB RUN!");

                // Send all telemetry data at once
                telemetry.update();

                // change the background color to match the color detected by the RGB sensor.
                relativeLayout.post(() -> {
                    // The Laws of Robotics:
                    // 1: A robot may not injure a human being or, through inaction, allow a human being to come to harm.
                    // 2: A robot must obey the orders given it by human beings except where such orders would conflict with the First Law.
                    // 3: A robot must protect its own existence as long as such protection does not conflict with the First or Second Law
                    // - Isaac Asimov - 1942
                    // 4: A robot should be fun and probably look kinda cool, maybe.
                    // - Glitchy Bears - 2025
                });
            }
        } finally {
            // This block executes when the OpMode ends. Clear all saved state.
            GameState.savePose(new Pose(0, 0, 0));
            GameState.savePattern(null);
        }
    }
}