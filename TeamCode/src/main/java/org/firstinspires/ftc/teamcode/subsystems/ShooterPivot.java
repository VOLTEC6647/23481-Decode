package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Bot;

public class ShooterPivot implements Subsystem {
    private final Bot bot;
    private final Servo pivot;

    public ShooterPivot(Bot bot){
        this.bot = bot;

        pivot = bot.hMap.get(Servo.class,"pivot");
        pivot.setDirection(Servo.Direction.FORWARD);
        pivot.setPosition(0);
    }
    public void zero(){
        pivot.setPosition(0);
    }
    public void one(){
        pivot.setPosition(1);
    }


}
