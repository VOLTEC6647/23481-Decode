package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.Bot;

public class ClawUp implements Subsystem {
    //private Servo clawServo;

    private Servo clawServo;
    private Bot bot;
    public double setPoint = 0;

    public ClawUp(Bot bot) {
        this.bot = bot;

        clawServo = bot.hMap.get(Servo.class,"clawUp");

    }
    public void setSetpoint (double setPoint) {
        this.setPoint = setPoint;
        clawServo.setPosition(setPoint);
    }


    @Override
    public void periodic(){
        //double currentPosition = clawServo.getPosition();







    }
    public void setPosition(double setpoint){
        setPoint = setpoint;
        clawServo.setPosition(setPoint);


    }
}
