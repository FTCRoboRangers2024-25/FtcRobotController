package org.firstinspires.ftc.teamcode.samples.components;

import static org.firstinspires.ftc.teamcode.utilities.Functions.Clamp01;
import static org.firstinspires.ftc.teamcode.utilities.Functions.LinearInterpolation;

import com.arcrobotics.ftclib.hardware.ServoEx;

public class LimitedServo {
    private ServoEx servo;
    private double posZero;
    private double posOne;

    public LimitedServo(ServoEx servo, double posZero, double posOne) {
        this.servo = servo;
        this.posZero = posZero;
        this.posOne = posOne;

        setPosition((posOne + posZero) / 2);
    }

    public LimitedServo(ServoEx servo, double posZero, double posOne, double initPos) {
        this.servo = servo;
        this.posZero = posZero;
        this.posOne = posOne;

        setPosition(initPos);
    }

    public LimitedServo(ServoEx servo, double posZero, double posOne, boolean noInit) {
        this.servo = servo;
        this.posZero = posZero;
        this.posOne = posOne;

        if (!noInit) {
            setPosition((posOne + posZero) / 2);
        }
    }

    public void setPosition(double pos) {
        servo.setPosition(LinearInterpolation(posZero, posOne, Clamp01(pos)));
    }
}
