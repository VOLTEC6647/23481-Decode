package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Bot;

@Config
public class Indexer implements Subsystem {
    private final DcMotorEx indexer;
    public double setPower = 0;

    public Indexer(Bot bot){

        indexer = bot.hMap.get(DcMotorEx.class,"indexer");

        indexer.setPower(0);

        indexer.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void indexOn(){
        indexer.setPower(.45);
    }
    public void indexOff(){
        indexer.setPower(0);
    }

    public void setPower(double power){
        setPower = power;
        indexer.setPower(setPower);


    }
}
