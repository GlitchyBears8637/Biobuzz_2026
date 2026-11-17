package org.firstinspires.ftc.teamcode.RobotHdw;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * IntakeSubsystem
 * 
 * EDUCATIONAL OVERVIEW FOR ROOKIE PROGRAMMERS:
 * This subsystem controls the intake motor that pulls game pieces (balls) into the robot.
 * It includes safety logic: if the 4-section ball queue is full, the intake automatically 
 * stops to prevent motor burnout or jamming.
 */
public class IntakeSubsystem {

    private final DcMotor motorIntake;
    private boolean intakeOn = false;
    private boolean aWasPressed = false;

    public IntakeSubsystem(RobotHdw robot) {
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

    /**
     * Updates intake motor power based on queue fullness.
     * 
     * @param queueFull True if all 4 slots in the ball queue are occupied.
     */
    public void update(boolean queueFull) {
        if (queueFull) {
            motorIntake.setPower(0); // Safety stop when queue is full
        } else {
            motorIntake.setPower(intakeOn ? 1.0 : 0.0);
        }
    }

    /**
     * Handles gamepad button toggles (Button A) with edge detection to prevent rapid toggling.
     */
    public void update(boolean queueFull, Gamepad gamepad) {
        if (gamepad.a && !aWasPressed) {
            toggle();
        }
        aWasPressed = gamepad.a;

        update(queueFull);
    }
}
