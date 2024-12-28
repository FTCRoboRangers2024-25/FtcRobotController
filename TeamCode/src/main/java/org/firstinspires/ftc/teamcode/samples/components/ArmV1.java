package org.firstinspires.ftc.teamcode.samples.components;

import static org.firstinspires.ftc.teamcode.utilities.Functions.Clamp01;

import com.acmerobotics.dashboard.canvas.Circle;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.Component;

import java.util.concurrent.CompletableFuture;

@Config
public class ArmV1 extends Component {
    public static PIDCoefficients armCoefficients = new PIDCoefficients(0.001, 0.05, 0.0001);

    private Motor armMotor;
    private AnalogInput potentiometer;
    private TouchSensor magLimit;
    private GamepadEx gamepad2;
    private Telemetry telemetry;

    private PIDController armPID = new PIDController(0.001, 0.05, 0.0001);

    private final double upVoltage = 1.1, ninetyDegDelta = 1.2;
    private final double feedforwardOut = 0.1;

    private int targetPosition, prevPos;

    private boolean rawPowerControl = true, cancelFlag = false, finishedHang = true;

    @Override
    public void init() {
        HardwareMap hardwareMap = getDependency("HardwareMap", HardwareMap.class);
        gamepad2 = getDependency("Gamepad2", GamepadEx.class);
        telemetry = getDependency("Telemetry", Telemetry.class);

        armMotor = new Motor(hardwareMap, "arm", Motor.GoBILDA.RPM_117);
        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armMotor.resetEncoder();
        potentiometer = hardwareMap.get(AnalogInput.class, "arm pot");
        magLimit = hardwareMap.get(TouchSensor.class, "arm mag");

        // For FTC Dashboard
        //armPID.setPID(armCoefficients.p, armCoefficients.i, armCoefficients.d);
    }

    @Override
    public void loop() {
        // For FTC Dashboard
        //armPID.setPID(armCoefficients.p, armCoefficients.i, armCoefficients.d);
        int currentPosition = armMotor.getCurrentPosition();

        readPositions();
        calculateEvents(prevPos, currentPosition);

        double output = 0;
        if (Math.abs(gamepad2.getRightY()) > 0.1 || rawPowerControl) {
            output = gamepad2.getRightY();
            targetPosition = -100;
            rawPowerControl = true;
        } else {
            output = armPID.calculate(currentPosition, targetPosition);
        }

        if (magLimit.isPressed()) {
            output = Clamp01(output);
            armMotor.resetEncoder();
        }

        armMotor.set(output + calculateFeedForward());

        prevPos = currentPosition;

        telemetry.addData("Mag", magLimit.isPressed());
        telemetry.addData("Target position", targetPosition);
        telemetry.addData("Current position", currentPosition);
    }

    private void readPositions() {
        if (gamepad2.isDown(GamepadKeys.Button.X)) {
            targetPosition = -100;
            rawPowerControl = false;
        } else if (gamepad2.isDown(GamepadKeys.Button.Y)) {
            targetPosition = 3700;
            broadcastMessage("Arm Pre Grab");
            rawPowerControl = false;
        } else if (gamepad2.isDown(GamepadKeys.Button.B)) {
            targetPosition = 2050;
            rawPowerControl = false;
        } else if (gamepad2.isDown(GamepadKeys.Button.RIGHT_BUMPER) && targetPosition == 3700) {
            targetPosition = 4100;
            CompletableFuture.runAsync(() -> {
                try {
                    Thread.sleep(200);
                    broadcastMessage("Arm Grab");
                    Thread.sleep(150);
                    broadcastMessage("Arm Grabbed");
                    targetPosition = 3700;
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            });
            rawPowerControl = false;
        } else if (gamepad2.isDown(GamepadKeys.Button.LEFT_BUMPER) && finishedHang) {
            targetPosition = 2050;
            cancelFlag = false;
            finishedHang = false;
            CompletableFuture<Void> hang = CompletableFuture.runAsync(() -> {
                try {
                    if (armMotor.getCurrentPosition() < 1850) Thread.sleep(1200);
                    else Thread.sleep(400);
                    if (cancelFlag) return;

                    broadcastMessage("Arm Hang");
                    Thread.sleep(500);
                    if (cancelFlag) return;

                    targetPosition = 1000;
                    Thread.sleep(200);
                    if (cancelFlag) return;

                    broadcastMessage("Arm Hang Release");
                    Thread.sleep(100);
                    if (cancelFlag) return;

                    targetPosition = -100;
                    finishedHang = true;
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            });
            CompletableFuture<Void> cancel = CompletableFuture.runAsync(() -> {
               while (!gamepad2.isDown(GamepadKeys.Button.B)) {
                   if (finishedHang) return;
               }
               if (!finishedHang) cancelFlag = true;
               finishedHang = true;
            });
            CompletableFuture.anyOf(hang, cancel);
            rawPowerControl = false;
        }
    }

    private void calculateEvents(int prevPos, int curPos) {
        if (prevPos > 1000 && curPos <= 1000) {
            broadcastMessage("Arm Forward");
        } else if (prevPos < 3000 && curPos >= 3000) {
            broadcastMessage("Arm Pre Grab");
        }
    }

    private double calculateFeedForward() {
        double voltage = potentiometer.getVoltage();
        telemetry.addData("Arm potentiometer", voltage);

        double angle = (voltage - upVoltage) * 90 / ninetyDegDelta;
        angle = Math.toRadians(angle);
        double sin = Math.sin(angle);

        if (magLimit.isPressed()) {
            sin = 0;
        }

        return feedforwardOut * sin;
    }
}
