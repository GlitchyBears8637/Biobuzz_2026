package org.firstinspires.ftc.teamcode.RobotHdw;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.rev.RevColorSensorV3;

/**
 * RobotHdw (Robot Hardware Definition Class)
 *
 * EDUCATIONAL OVERVIEW FOR ROOKIE PROGRAMMERS:
 * This is NOT an OpMode. Instead, this class acts as a centralized blueprint for all physical
 * hardware components on the robot (Motors, Servos, Sensors, and the IMU).
 *
 * Key concepts:
 * 1. Hardware Mapping (`hwMap.get`): Connects code variables to the exact device names configured
 *    in the FTC Driver Station app.
 * 2. Motor Directions: Setting `FORWARD` or `REVERSE` so positive power moves the robot forward.
 * 3. Run Modes: `RUN_WITHOUT_ENCODER` for drive motors (since Pedro Pathing handles odometry tracking),
 *    and `RUN_USING_ENCODER` for velocity-regulated mechanisms.
 * 4. IMU Initialization: Configures the Control Hub's inertial measurement unit to track rotation and heading.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Motors:
 *  - "motorFrontLeft" (Control Hub Port 0)
 *  - "motorBackLeft" (Control Hub Port 1)
 *  - "motorFrontRight" (Control Hub Port 2)
 *  - "motorBackRight" (Control Hub Port 3)
 *  - "motorIntake" (Expansion Hub Port 0)
 *  - "motorLauncher" (Expansion Hub Port 1)
 *
 * Servos:
 *  - "servoTubeWheel" (5-Turn Servo, Expansion Hub Port 1)
 *  - (Lamps commented out until installed)
 *
 * Sensors:
 *  - "imu" (Control Hub I2C Port 0)
 *  - "limelight" (Configured via Limelight3A class)
 */
public class RobotHdw {

    // --- Public Hardware Members ---
    // Drivetrain & Mechanism Motors
    public DcMotorEx motorFrontLeft   = null;
    public DcMotorEx motorBackLeft    = null;
    public DcMotorEx motorFrontRight  = null;
    public DcMotorEx motorBackRight   = null;
    public DcMotor   motorIntake      = null;
    public DcMotor   motorLauncher    = null;

    // Servos and Continuous Rotation Servos (CRServos)
    public Servo   servoTubeWheel   = null;
    // public Servo   lamp1            = null;
    // public Servo   lamp2            = null;
    // public Servo   lamp3            = null;
    // public Servo   lamp4            = null;

    // Sensors
    public RevColorSensorV3 sensorColor1;
    public RevColorSensorV3 sensorColor2;
    public RevColorSensorV3 sensorColor3;
    public RevColorSensorV3 sensorColor4;
    public IMU imu = null;

    HardwareMap hwMap = null;

    public RobotHdw() {
        // Constructor left empty intentionally
    }

    /**
     * Initializes all hardware devices by fetching their names from the Driver Station configuration.
     *
     * @param ahwMap The HardwareMap passed from the active OpMode (Autonomous or TeleOp).
     */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // --- Step 1: Map Drivetrain and Mechanism Motors ---
        motorFrontLeft  = hwMap.get(DcMotorEx.class, "motorFrontLeft");
        motorBackLeft   = hwMap.get(DcMotorEx.class, "motorBackLeft");
        motorFrontRight = hwMap.get(DcMotorEx.class, "motorFrontRight");
        motorBackRight  = hwMap.get(DcMotorEx.class, "motorBackRight");
        motorIntake     = hwMap.get(DcMotor.class, "motorIntake");
        motorLauncher   = hwMap.get(DcMotor.class, "motorLauncher");

        // --- Step 2: Set Motor Directions ---
        // Right side motors are typically reversed so positive power drives the robot forward
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        // --- Step 3: Stop All Motors Upon Initialization for Safety ---
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorIntake.setPower(0);
        motorLauncher.setPower(0);

        // --- Step 4: Configure Motor Run Modes ---
        // Run drivetrain motors without built-in encoders (Pedro Pathing handles odometry tracking)
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- Step 5: Map Servos ---
        servoTubeWheel = hwMap.get(Servo.class, "servoTubeWheel");
        // lamp1          = hwMap.get(Servo.class, "lamp1");
        // lamp2          = hwMap.get(Servo.class, "lamp2");
        // lamp3          = hwMap.get(Servo.class, "lamp3");
        // lamp4          = hwMap.get(Servo.class, "lamp4");

        // --- Step 6: Initialize the Inertial Measurement Unit (IMU) ---
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        imu.resetYaw();
    }
}
