package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Bot;

@Config
public class Indexer implements Subsystem {
    private final Bot bot;
    private final CRServo indexer;

    public Indexer(Bot bot){
        this.bot = bot;

        indexer = bot.hMap.get(CRServo.class,"indexer");

        indexer.setPower(0);

        indexer.setDirection(CRServo.Direction.FORWARD);
    }

    public void indexOn(){
        indexer.setPower(1);
    }
    public void indexOff(){
        indexer.setPower(0);
    }
}
