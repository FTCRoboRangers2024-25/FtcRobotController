package org.firstinspires.ftc.teamcode.samples.components;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utilities.Component;

public class MecanumBase extends Component {
    private MecanumDrive mecanumDrive;
    private GamepadEx gamepad1;

    private double slowModeCoef = 0.6;

    @Override
    public void init() {
        HardwareMap hardwareMap = getDependency("HardwareMap", HardwareMap.class);
        gamepad1 = getDependency("Gamepad1", GamepadEx.class);

        Motor frontLeft = new Motor(hardwareMap, "front left", Motor.GoBILDA.RPM_312);
        Motor frontRight = new Motor(hardwareMap, "front right", Motor.GoBILDA.RPM_312);
        Motor backLeft = new Motor(hardwareMap, "back left", Motor.GoBILDA.RPM_312);
        Motor backRight = new Motor(hardwareMap, "back right", Motor.GoBILDA.RPM_312);

        MotorGroup group = new MotorGroup(frontLeft, frontRight, backLeft, backRight);
        group.setInverted(true);
        group.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        mecanumDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
    }

    @Override
    public void loop() {
        double coef = gamepad1.isDown(GamepadKeys.Button.RIGHT_BUMPER) ? slowModeCoef : 1;
        if (Math.abs(gamepad1.getLeftX()) > 0.1 || Math.abs(gamepad1.getLeftY()) > 0.1 || Math.abs(gamepad1.getRightX()) > 0.1) {
            mecanumDrive.driveRobotCentric(gamepad1.getLeftX() * coef, gamepad1.getLeftY() * coef, gamepad1.getRightX() * coef - 0.1, true);
        } else {
            mecanumDrive.stop();
        }
    }
}
