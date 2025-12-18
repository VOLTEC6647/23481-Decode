package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Bot;

public class Pivot implements Subsystem {
    private final Bot bot;
    private final Servo pivot;

    public Pivot(Bot bot){
        this.bot = bot;

        pivot = bot.hMap.get(Servo.class,"pivot");
        pivot.setDirection(Servo.Direction.REVERSE);
        pivot.setPosition(0.5);
    }
    public void zero(){
        pivot.setPosition(0.5);
    }
    public void one(){
        pivot.setPosition(0.8);
    }
}
