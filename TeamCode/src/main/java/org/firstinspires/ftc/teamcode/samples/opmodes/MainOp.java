package org.firstinspires.ftc.teamcode.samples.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.samples.components.ArmV1;
import org.firstinspires.ftc.teamcode.samples.components.ClawV1;
import org.firstinspires.ftc.teamcode.samples.components.HangHookV1;
import org.firstinspires.ftc.teamcode.samples.components.MecanumBase;
import org.firstinspires.ftc.teamcode.utilities.OpModeBase;

@TeleOp(name = "Main Op")
public class MainOp extends OpModeBase {
    @Override
    public void startup() {
        addComponent(MecanumBase.class);
        addComponent(ArmV1.class);
        addComponent(ClawV1.class);
        addComponent(HangHookV1.class);
    }
}
