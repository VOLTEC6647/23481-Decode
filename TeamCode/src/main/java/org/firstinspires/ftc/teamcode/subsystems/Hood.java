package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Bot;

public class Hood implements Subsystem {
    private final Bot bot;
    private Servo hood;
    boolean shootsToFront = true;

    public Hood(Bot bot){
        this.bot = bot;

        hood = bot.hMap.get(Servo.class,"hood");
        hood.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void periodic(){
        if(bot.opertator.gamepad.bWasPressed()){
            if(shootsToFront){
                hood.setPosition(0);
                bot.telem.addData("Direction","Front");
            } else {
                hood.setPosition(1);
                bot.telem.addData("Direction","Back");
            }
        }
    }
}
