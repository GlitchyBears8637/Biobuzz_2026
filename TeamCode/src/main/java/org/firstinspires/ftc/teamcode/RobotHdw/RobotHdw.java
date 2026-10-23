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
 * RobotHdw (Hardware Map Subsystem)
 * 
 * EDUCATIONAL OVERVIEW FOR ROOKIE PROGRAMMERS:
 * This class serves as the central hardware mapping definition for the robot. 
 * Instead of writing hardware declarations inside every OpMode, all motors, servos, 
 * and sensors are declared and initialized here in one clean, reusable place.
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

    // Servos
    public CRServo servoTubeWheel   = null;
    public Servo   servoLift        = null;
    public Servo   lamp1            = null;
    public Servo   lamp2            = null;
    public Servo   lamp3            = null;
    public Servo   lamp4            = null;

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
     */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Map drivetrain and mechanism motors
        motorFrontLeft  = hwMap.get(DcMotorEx.class, "motorFrontLeft");
        motorBackLeft   = hwMap.get(DcMotorEx.class, "motorBackLeft");
        motorFrontRight = hwMap.get(DcMotorEx.class, "motorFrontRight");
        motorBackRight  = hwMap.get(DcMotorEx.class, "motorBackRight");
        motorIntake     = hwMap.get(DcMotor.class, "motorIntake");
        motorLauncher   = hwMap.get(DcMotor.class, "motorLauncher");

        // Set motor directions so positive power drives the robot forward
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        // Stop all motors upon initialization
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorIntake.setPower(0);
        motorLauncher.setPower(0);

        // Run drivetrain motors without encoders by default (Pedro Pathing manages its own tracking)
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize IMU (Inertial Measurement Unit / Gyro)
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        imu.resetYaw();
    }
}
