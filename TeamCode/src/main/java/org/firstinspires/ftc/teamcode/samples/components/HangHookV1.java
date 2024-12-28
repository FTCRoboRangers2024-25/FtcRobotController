package org.firstinspires.ftc.teamcode.samples.components;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utilities.Component;

public class HangHookV1 extends Component {
    private GamepadEx gamepad1;
    private Motor motorLeft, motorRight;

    @Override
    public void init() {
        HardwareMap hardwareMap = getDependency("HardwareMap", HardwareMap.class);
        gamepad1 = getDependency("Gamepad1", GamepadEx.class);

        motorLeft = new Motor(hardwareMap, "hang left", Motor.GoBILDA.RPM_435);
        motorRight = new Motor(hardwareMap, "hang right", Motor.GoBILDA.RPM_435);
        motorRight.setInverted(true);
    }

    @Override
    public void loop() {
        if (gamepad1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            motorLeft.set(-0.8);
        } else if (gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {
            motorLeft.set(gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        } else motorLeft.set(0);

        if (gamepad1.isDown(GamepadKeys.Button.Y)) {
            motorRight.set(-0.8);
        } else if (gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
            motorRight.set(gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        } else motorRight.set(0);
    }
}
