package org.firstinspires.ftc.teamcode.samples.components;

import static org.firstinspires.ftc.teamcode.utilities.Functions.Clamp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utilities.Component;
import org.firstinspires.ftc.teamcode.utilities.ReceiveMessage;

public class ClawV1 extends Component {

    private LimitedServo clawYaw, clawPitch, clawLeft, clawRight;
    private GamepadEx gamepad2;

    private double currentPitch = 0.5, lowerPitchLimit = 0;
    private final double pitchIncrement = 0.25;

    private boolean yawLock = false, clawStateMultiplier = false;

    private ToggleButtonReader clawButtonReader;

    @Override
    public void init() {
        HardwareMap hardwareMap = getDependency("HardwareMap", HardwareMap.class);
        gamepad2 = getDependency("Gamepad2", GamepadEx.class);
        armForward();

        SimpleServo clawYawServo = new SimpleServo(hardwareMap, "claw yaw", 0, 300);
        clawYawServo.setInverted(true);
        SimpleServo clawPitchServo = new SimpleServo(hardwareMap, "claw pitch", 0, 300);
        clawPitchServo.setInverted(true);
        SimpleServo clawLeftServo = new SimpleServo(hardwareMap, "claw left", 0, 300);
        SimpleServo clawRightServo = new SimpleServo(hardwareMap, "claw right", 0, 300);
        clawRightServo.setInverted(true);

        clawYaw = new LimitedServo(clawYawServo, 0.2, 0.85);
        clawPitch = new LimitedServo(clawPitchServo, 0.15, 0.8);
        clawLeft = new LimitedServo(clawLeftServo, 0, 0.2, 0);
        clawRight = new LimitedServo(clawRightServo, 0, 0.45, 0);

        clawButtonReader = new ToggleButtonReader(gamepad2, GamepadKeys.Button.A);
    }

    @Override
    public void loop() {
        gamepad2.readButtons();
        if (gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_UP))
        {
            currentPitch += pitchIncrement;
        }
        if (gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
        {
            currentPitch -= pitchIncrement;
        }
        currentPitch = Clamp(lowerPitchLimit, 1, currentPitch);
        clawPitch.setPosition(currentPitch);

        if (yawLock) {
            clawYaw.setPosition(0.5);
        } else {
            double yawInput = (gamepad2.getLeftX() + 1) / 2;
            clawYaw.setPosition(yawInput);
        }

        clawButtonReader.readValue();
        if (clawButtonReader.getState() == !clawStateMultiplier)
        {
            // Open
            clawLeft.setPosition(1);
            clawRight.setPosition(1);
        }
        else
        {
            // Closed
            clawLeft.setPosition(0);
            clawRight.setPosition(0);
        }
    }

    @ReceiveMessage(message = "Arm Forward")
    public void armForward() {
        lowerPitchLimit = 0.5;
        currentPitch = 0.5;
        yawLock = true;
    }

    @ReceiveMessage(message = "Arm Pre Grab")
    public void armPreGrab() {
        if (clawButtonReader.getState() == clawStateMultiplier) {
            clawStateMultiplier = !clawStateMultiplier;
        }
        lowerPitchLimit = 0;
        currentPitch = 1;
        yawLock = false;
    }

    @ReceiveMessage(message = "Arm Grab")
    public void armGrab() {
        if (clawButtonReader.getState() != clawStateMultiplier) {
            clawStateMultiplier = !clawStateMultiplier;
        }
        yawLock = false;
    }

    @ReceiveMessage(message = "Arm Grabbed")
    public void armGrabbed() {
        yawLock = false;
    }

    @ReceiveMessage(message = "Arm Hang")
    public void armHang() {
        currentPitch = 1;
        yawLock = false;
    }

    @ReceiveMessage(message = "Arm Hang Release")
    public void armHangRelease() {
        if (clawButtonReader.getState() == clawStateMultiplier) {
            clawStateMultiplier = !clawStateMultiplier;
        }
        yawLock = true;
    }
}
