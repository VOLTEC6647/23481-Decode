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
    //private final DcMotorEx indexer;


    public Shooter(Bot bot){
        this.bot = bot;

        shooter = bot.hMap.get(DcMotorEx.class,"shooter");
        indexer = bot.hMap.get(CRServo.class,"indexer");
        //indexer = bot.hMap.get(DcMotorEx.class,"indexer");

        shooter.setPower(0);
        indexer.setPower(0);

        shooter.setDirection(DcMotor.Direction.FORWARD);
        indexer.setDirection(CRServo.Direction.FORWARD);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bot.telem.addData("Status", "Initialized");
        bot.telem.update();
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
