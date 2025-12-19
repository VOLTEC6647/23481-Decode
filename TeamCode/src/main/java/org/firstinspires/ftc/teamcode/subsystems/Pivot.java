package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Bot;

public class Pivot implements Subsystem {
    private final Bot bot;
    private final Servo pivot;
    public double setPoint = 0;

    public Pivot(Bot bot){
        this.bot = bot;

        pivot = bot.hMap.get(Servo.class,"pivot");
        pivot.setPosition(1);
    }
    public void zero(){
        pivot.setPosition(0.23);
    }
    public void one(){
        pivot.setPosition(0.3);
    }
    public void setPosition(double setpoint){
        setPoint = setpoint;
        pivot.setPosition(setPoint);


    }
}
