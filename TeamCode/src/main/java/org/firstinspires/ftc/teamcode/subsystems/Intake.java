package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Bot;

@Config
public class Intake implements Subsystem {
    private final Bot bot;
    private DcMotorEx intake;

    public Intake(Bot bot) {
        this.bot = bot;

        intake = bot.hMap.get(DcMotorEx.class,"intake");

        intake.setDirection(DcMotorEx.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intake.setPower(0);

        bot.telem.update();
    }

    public void setPower(double power){
        intake.setPower(power);

    }
}
