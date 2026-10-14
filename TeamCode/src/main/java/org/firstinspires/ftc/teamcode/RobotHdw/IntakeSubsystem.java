package org.firstinspires.ftc.teamcode.RobotHdw;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class IntakeSubsystem {

    private final DcMotor motorIntake;
    private boolean intakeOn = false;
    private boolean aWasPressed = false;

    public IntakeSubsystem(HardwareRobot robot) {
        this.motorIntake = robot.motorIntake;
    }

    public void toggle() {
        intakeOn = !intakeOn;
    }

    public void turnOn() {
        intakeOn = true;
    }

    public void turnOff() {
        intakeOn = false;
    }

    public boolean isIntakeOn() {
        return intakeOn;
    }

    public void update(boolean queueFull) {
        if (queueFull) {
            motorIntake.setPower(0);
        } else {
            motorIntake.setPower(intakeOn ? 1.0 : 0.0);
        }
    }

    public void update(boolean queueFull, Gamepad gamepad) {
        if (gamepad.a && !aWasPressed) {
            toggle();
        }
        aWasPressed = gamepad.a;

        update(queueFull);
    }
}
