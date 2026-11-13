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
 * This is NOT an OpMode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is the GoBilda Drive Base.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 *
 * Motors:
 *  "motorFrontLeft" ControlHub - 0
 *  "motorBackLeft" ControlHub - 1
 *  "motorFrontRight" ControlHub - 2
 *  "motorBackRight" ControlHub - 3
 *  "motorIntake" ExpansionHub - 0
 *  "motorLauncher" ExpansionHub - 1
 *
 * Servos:
 *  "servoLift" ExpansionHub - 0
 *  "servoTubeWheel" (CRServo) ExpansionHub - 1
 *  "servoAim" ControlHub - 0
 *  "servoHood" ControlHub - 1
 *  "lamp1" ControlHub - 2
 *  "lamp2" ControlHub - 3
 *  "lamp3" ControlHub - 4
 *  "lamp4" ControlHub -5
 *
 * Sensors:
 * "imu" ControlHub I2C - 0
 *  "sensorColor1" ControlHub I2C - 1
 *  "sensorColor2" ControlHub I2C - 2
 *  "sensorColor3" ControlHub I2C - 3
 *  "otos" ExpansionHu I2C - 0
 *  //"limelight" is initialized in limelight class routine
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
    public Servo   servoAim         = null;
    public Servo   servoHood        = null;
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
        motorLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        servoTubeWheel = hwMap.get(CRServo.class, "servoTubeWheel");
        servoAim       = hwMap.get(Servo.class, "servoAim");
        servoHood      = hwMap.get(Servo.class, "servoHood");
        servoLift      = hwMap.get(Servo.class, "servoLift");
        lamp1          = hwMap.get(Servo.class, "lamp1");
        lamp2          = hwMap.get(Servo.class, "lamp2");
        lamp3          = hwMap.get(Servo.class, "lamp3");
        lamp4          = hwMap.get(Servo.class, "lamp4");

        // Define and initialize ALL installed sensors.
        // sensorColor1 = (RevColorSensorV3) hwMap.get(ColorSensor.class, "colorSense1");
        // sensorColor2 = (RevColorSensorV3) hwMap.get(ColorSensor.class, "colorSense2");
        // sensorColor3 = (RevColorSensorV3) hwMap.get(ColorSensor.class, "colorSense3");

        // if (sensorColor1 != null) sensorColor1.setGain(4);
        // if (sensorColor2 != null) sensorColor2.setGain(4);
        // if (sensorColor3 != null) sensorColor3.setGain(4);

        // Initialize the IMU
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        imu.resetYaw();
    }
}
