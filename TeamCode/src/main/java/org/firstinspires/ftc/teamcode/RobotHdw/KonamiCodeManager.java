package org.firstinspires.ftc.teamcode.RobotHdw;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class KonamiCodeManager {

    private final ArrayList<String> inputHistory = new ArrayList<>();
    private long lastInputTime = 0;
    private boolean konamiCodeActivated = false;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepad previousGamepadState = new Gamepad();

    public KonamiCodeManager(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public void update(Gamepad gamepad) {
        long currentTime = System.currentTimeMillis();

        String pressedButton = getPressedButton(gamepad);

        if (!pressedButton.isEmpty()) {
            inputHistory.add(pressedButton);
            lastInputTime = currentTime;

            if (inputHistory.size() > 10) {
                inputHistory.remove(0);
            }

            if (inputHistory.size() == 10) {
                String recentInputs = String.join("", inputHistory);
                if (recentInputs.equals("upupdowndownleftrightleftrightba") && !konamiCodeActivated) {
                    activateKonami();
                }
            }
        }

        if (shouldReset(gamepad, currentTime) && !inputHistory.isEmpty()) {
            resetKonami();
        }

        previousGamepadState.copy(gamepad);
    }

    private String getPressedButton(Gamepad gamepad) {
        if (!previousGamepadState.dpad_up && gamepad.dpad_up) return "up";
        if (!previousGamepadState.dpad_down && gamepad.dpad_down) return "down";
        if (!previousGamepadState.dpad_left && gamepad.dpad_left) return "left";
        if (!previousGamepadState.dpad_right && gamepad.dpad_right) return "right";
        if (!previousGamepadState.b && gamepad.b) return "b";
        if (!previousGamepadState.a && gamepad.a) return "a";
        return "";
    }

    private void activateKonami() {
        konamiCodeActivated = true;
        telemetry.addData("Easter Egg", "Konami Code Activated!");
        int soundID = hardwareMap.appContext.getResources().getIdentifier("zeldapuzzle", "raw", hardwareMap.appContext.getPackageName());
        if (soundID != 0) {
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);
        } else {
            telemetry.addData("Easter Egg", "Sound file not found!");
        }
    }

    private boolean shouldReset(Gamepad gamepad, long currentTime) {
        boolean resetButtonJustPressed = (!previousGamepadState.x && gamepad.x) || (!previousGamepadState.y && gamepad.y);
        boolean timeout = currentTime - lastInputTime > 3000;
        return resetButtonJustPressed || timeout;
    }

    private void resetKonami() {
        inputHistory.clear();
        konamiCodeActivated = false;
        telemetry.addData("Easter Egg", "Sequence Reset.");
    }
}
