package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHdw.DriveSubsystems;
import org.firstinspires.ftc.teamcode.RobotHdw.HardwareRobot;
import org.firstinspires.ftc.teamcode.RobotHdw.IntakeSubsystem;
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

    // Create an instance of the hardware class
    HardwareRobot robot = new HardwareRobot();
    Follower follower;
    private LimelightSubsystem.Alliance alliance;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize hardware and Pedro Pathing follower
        robot.init(hardwareMap);
        follower = Constants.create(hardwareMap);


        LimelightSubsystem limelightSubsystem = new LimelightSubsystem(hardwareMap);

        // Add a delay to allow the Limelight to fully power on and initialize.
        // This helps prevent race conditions where we try to set the pipeline
        // before the Limelight is ready to accept commands.
        sleep(500); // 1-second delay
        {

        }

        TurretControl turretControl = new TurretControl(robot, limelightSubsystem); // Corrected
        LauncherSubsystem launcherSubsystem = new LauncherSubsystem(hardwareMap, limelightSubsystem, turretControl, follower); // Corrected
        // launcherSubsystem.setTeleOpMode(true);
        ColorSensorSubsystem colorSensorSubsystem = new ColorSensorSubsystem(robot);
        TubeWheelSubsystem tubeWheelSubsystem = new TubeWheelSubsystem(robot, launcherSubsystem, colorSensorSubsystem);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(robot);
        KonamiCodeManager konamiCodeManager = new KonamiCodeManager(hardwareMap, telemetry);
        DriveSubsystems driveSubsystem = new DriveSubsystems(robot);


        int loopCounter = 0;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        telemetry.addData("Status", "Initialization Complete. Ready for START.");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
                // === UPDATE SUBSYSTEMS FIRST ===
                follower.update(); // This is critical for TeleOp position tracking!

                // Only save the pose every 10th loop cycle to reduce lag

                limelightSubsystem.update();
                colorSensorSubsystem.updateSensorReadings();
                tubeWheelSubsystem.update();
                intakeSubsystem.update(tubeWheelSubsystem.isQueueFull(), gamepad2);
                konamiCodeManager.update(gamepad1);
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
                telemetry.addData("Tube Wheel Power", robot.servoTubeWheel.getPower());
                colorSensorSubsystem.addTelemetry(telemetry);
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
        }
    }
