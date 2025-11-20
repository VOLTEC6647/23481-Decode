package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Bot;

public class Shooter implements Subsystem {
    private final Bot bot;
    private final DcMotorEx shooter;
    private final CRServo indexer;


    public Shooter(Bot bot){
        this.bot = bot;

        shooter = bot.hMap.get(DcMotorEx.class,"shooter");
        indexer = bot.hMap.get(CRServo.class,"indexer");

        shooter.setPower(0);
        indexer.setPower(0);

        shooter.setDirection(DcMotor.Direction.REVERSE);
        indexer.setDirection(CRServo.Direction.REVERSE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void shootOn(){
        shooter.setPower(1);
    }
    public void shootOff(){
        shooter.setPower(0);
    }
    public void indexOn(){
        indexer.setPower(1);
    }
    public void indexOff(){
        indexer.setPower(0);
    }
}
