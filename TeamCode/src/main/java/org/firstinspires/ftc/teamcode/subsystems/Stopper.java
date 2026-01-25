package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Bot;

public class Stopper implements Subsystem {
    private final Servo stopper;
    public double setPoint = 0;

    public Stopper(Bot bot){

        stopper = bot.hMap.get(Servo.class,"stopper");
        stopper.setDirection(Servo.Direction.FORWARD);
    }
    public void setPosition(double setpoint){
        setPoint = setpoint;
        stopper.setPosition(setPoint);
    }
}
