package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Bot;

public class Shooter implements Subsystem {
    private final Bot bot;
    private final DcMotorEx shooter;


    public Shooter(Bot bot){
        this.bot = bot;

        shooter = bot.hMap.get(DcMotorEx.class,"shooter");

        shooter.setPower(0);

        shooter.setDirection(DcMotor.Direction.FORWARD);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void shootOn(){
        shooter.setPower(1);
    }
    public void shootOff(){
        shooter.setPower(0);
    }
}
