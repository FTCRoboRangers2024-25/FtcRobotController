package org.firstinspires.ftc.teamcode.utilities;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Constructor;
import java.lang.reflect.Type;
import java.util.HashSet;

public abstract class OpModeBase extends LinearOpMode {
    private final HashSet<Component> components = new HashSet<>();

    private final IMessageBroadcaster messageBroadcaster = new StandardMessageBroadcaster(telemetry);

    private GamepadEx firstGamepad, secondGamepad;

    public abstract void startup();

    @Override
    public void runOpMode() {
        firstGamepad = new GamepadEx(gamepad1);
        secondGamepad = new GamepadEx(gamepad2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        startup();

        for (Component component : components) {
            component.addDependency("Telemetry", telemetry);
            component.addDependency("HardwareMap", hardwareMap);
            component.addDependency("MessageBroadcaster", messageBroadcaster);
            component.addDependency("Gamepad1", firstGamepad);
            component.addDependency("Gamepad2", secondGamepad);
            component.init();
        }

        messageBroadcaster.addReceiverRange(new HashSet<>(components));

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                for (Component component : components) {
                    component.loop();
                }

                telemetry.update();
            }
        }
    }

    protected <T> void addComponent(Class<T> componentClass) {
        try {
            Constructor<?> constructor = componentClass.getConstructor();
            Object createdComponent = constructor.newInstance();

            Component castedComponent = (Component)createdComponent;
            components.add(castedComponent);
        } catch (Exception e) {
            telemetry.addLine("Something went wrong when instantiating " + componentClass.getName());
            telemetry.update();

            requestOpModeStop();
        }
    }
}
