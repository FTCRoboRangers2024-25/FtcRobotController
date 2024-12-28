package org.firstinspires.ftc.teamcode.samples.opmodes;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.samples.components.LimitedServo;

import java.util.concurrent.TimeUnit;



@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous")
public class Autonomous extends LinearOpMode {
    private Motor armMotor;
    private AnalogInput potentiometer;
    private TouchSensor magLimit;
    private MecanumDrive mecanumDrive;

    private final double upVoltage = 1.1, ninetyDegDelta = 1.2;
    private final double feedforwardOut = 0.1;

    private PIDController armPID = new PIDController(0.001, 0.05, 0.0001);

    @Override
    public void runOpMode() {
        Motor frontLeft = new Motor(hardwareMap, "front left", Motor.GoBILDA.RPM_312);
        Motor frontRight = new Motor(hardwareMap, "front right", Motor.GoBILDA.RPM_312);
        Motor backLeft = new Motor(hardwareMap, "back left", Motor.GoBILDA.RPM_312);
        Motor backRight = new Motor(hardwareMap, "back right", Motor.GoBILDA.RPM_312);

        MotorGroup group = new MotorGroup(frontLeft, frontRight, backLeft, backRight);
        group.setInverted(true);
        group.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        mecanumDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        armMotor = new Motor(hardwareMap, "arm", Motor.GoBILDA.RPM_117);
        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armMotor.resetEncoder();
        potentiometer = hardwareMap.get(AnalogInput.class, "arm pot");
        magLimit = hardwareMap.get(TouchSensor.class, "arm mag");

        moveArmMillis(1050, 1000);
        armMotor.set(0.07);

        SimpleServo clawPitchServo = new SimpleServo(hardwareMap, "claw pitch", 0, 300);
        clawPitchServo.setInverted(true);
        LimitedServo clawPitch = new LimitedServo(clawPitchServo, 0.15, 0.8, true);

        waitForStart();
        clawPitch.setPosition(0.5);
        frontLeft.resetEncoder();
        mecanumDrive.driveRobotCentric(0.2, 0, 0);
        while (frontLeft.getCurrentPosition() < 450) {}
        mecanumDrive.stop();
        for (int i = 0; i < 3; i++) {
            moveMillis(0, 0.4, 0, 2000);
            moveMillis(0.2, 0, 0, 1400);
            moveMillis(0, -0.4, 0, 2000);
            waitMillis(500);
        }
    }

    private void waitMillis(long millis) {
        try {
            TimeUnit.MILLISECONDS.sleep(millis);
        } catch (InterruptedException e) {
            // Prosta catch ca sa nu dea eroare
        }
    }

    private void moveMillis(double strafeSpeed, double forwardSpeed, double turnSpeed, long millis) {
        mecanumDrive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
        waitMillis(millis);
        mecanumDrive.stop();
    }

    private void moveArmMillis(int targetPosition, long millis) {
        double start = System.currentTimeMillis();
        while (System.currentTimeMillis() < start + millis) {
            int currentPosition = armMotor.getCurrentPosition();
            double output = armPID.calculate(currentPosition, targetPosition);
            armMotor.set(output + calculateFeedForward());
        }
        armMotor.stopMotor();
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
