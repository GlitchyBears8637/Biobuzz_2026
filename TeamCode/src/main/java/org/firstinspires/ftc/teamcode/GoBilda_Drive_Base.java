package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHdw.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.DriveSubsystems;
import org.firstinspires.ftc.teamcode.RobotHdw.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.KonamiCodeManager;
import org.firstinspires.ftc.teamcode.RobotHdw.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.RobotAimSubsystem;
import org.firstinspires.ftc.teamcode.RobotHdw.RobotHdw;
import org.firstinspires.ftc.teamcode.RobotHdw.TubeOfDespairSubsystem;
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

    // Create an instance of the hardware class
    RobotHdw robot = new RobotHdw();
    Follower follower;
    private LimelightSubsystem.Alliance alliance = LimelightSubsystem.Alliance.BLUE;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize hardware and Pedro Pathing follower
        robot.init(hardwareMap);
        follower = Constants.create(hardwareMap);

        LimelightSubsystem limelightSubsystem = new LimelightSubsystem(hardwareMap);
        limelightSubsystem.start();

        // Add a delay to allow the Limelight to fully power on and initialize.
        sleep(500);

        RobotAimSubsystem robotAimSubsystem = new RobotAimSubsystem(robot, limelightSubsystem);
        LauncherSubsystem launcherSubsystem = new LauncherSubsystem(hardwareMap, limelightSubsystem, robotAimSubsystem, follower);
        launcherSubsystem.setTeleOpMode(true);

        ColorSensorSubsystem colorSensorSubsystem = new ColorSensorSubsystem(robot);
        TubeOfDespairSubsystem tubeOfDespairSubsystem = new TubeOfDespairSubsystem(robot, launcherSubsystem, colorSensorSubsystem);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(robot);
        KonamiCodeManager konamiCodeManager = new KonamiCodeManager(hardwareMap, telemetry);
        DriveSubsystems driveSubsystem = new DriveSubsystems(follower);

        tubeOfDespairSubsystem.startHoming();

        // Alliance selection and servo homing during INIT
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) {
                alliance = LimelightSubsystem.Alliance.RED;
            } else if (gamepad1.b) {
                alliance = LimelightSubsystem.Alliance.BLUE;
            }
            limelightSubsystem.setAlliance(alliance);

            colorSensorSubsystem.updateSensorReadings();
            tubeOfDespairSubsystem.updateHoming();

            telemetry.addData("=== ALLIANCE SELECTOR ===", "");
            telemetry.addData("Press [X] for", "RED ALLIANCE");
            telemetry.addData("Press [B] for", "BLUE ALLIANCE");
            telemetry.addData("Selected Alliance", alliance);
            telemetry.addData("Tube Homed", tubeOfDespairSubsystem.isHomed());
            telemetry.addData("Status", "Initialization Complete. Ready for START.");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            // === DRIVE & FOLLOWER UPDATE ===
            driveSubsystem.loop(gamepad1);
            konamiCodeManager.update(gamepad1);

            // Update Limelight camera
            limelightSubsystem.update();

            // Color sensor & tube feed mechanism
            colorSensorSubsystem.updateSensorReadings();
            tubeOfDespairSubsystem.update();
            intakeSubsystem.update(tubeOfDespairSubsystem.isQueueFull(), gamepad2);

            // Launcher controls
            launcherSubsystem.update(gamepad2, telemetry);

            // Telemetry
            telemetry.addData("Alliance", alliance);
            limelightSubsystem.addTelemetry(telemetry);
            telemetry.addData("Launcher RPM", "%.0f", launcherSubsystem.getRPM());
            telemetry.addData("Launcher Target RPM", "%.0f", launcherSubsystem.getTargetRPM());
            telemetry.addData("Tube Ball Count", "%d / 4 (Full: %b)", tubeOfDespairSubsystem.getBallCount(), tubeOfDespairSubsystem.isQueueFull());
            telemetry.addData("Tube Wheel Power", robot.servoTubeWheel.getPower());
            colorSensorSubsystem.addTelemetry(telemetry);
            telemetry.addData("Status", "RUN BOB RUN!");
            telemetry.update();
        }
    }
}
