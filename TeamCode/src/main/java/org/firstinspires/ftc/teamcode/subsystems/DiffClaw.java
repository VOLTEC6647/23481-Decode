package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.Vision.Vision;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

public class DiffClaw implements Subsystem {
    private Servo sI;
    private Servo sD;

    private Bot bot;
    private Vision vision;
    public double setPoint = 0;
    double intakeRotatePerDegree = 0.000555555556;


    public DiffClaw(Bot bot) {
        this.bot = bot;

        sI = bot.hMap.get(Servo.class,"sI");
        sD = bot.hMap.get(Servo.class,"sD");

        sI.setDirection(Servo.Direction.REVERSE);
        sD.setDirection(Servo.Direction.FORWARD);

      //  sI.setPosition(0.56);
       // sD.setPosition(0.56);




    }
    public void setSetpoint (double setPoint) {
        this.setPoint = setPoint;
        sI.setPosition(setPoint);
        sD.setPosition(setPoint);

    }


    @Override
    public void periodic(){


        /*if(bot.opertator.gamepad.left_bumper) {
            sD.setPosition(0.52 -(0 * intakeRotatePerDegree));
            sI.setPosition(0.52 +(0 * intakeRotatePerDegree));
        }
        else if (bot.opertator.gamepad.x){
            sD.setPosition(0.52 -(45 * intakeRotatePerDegree));
            sI.setPosition(0.52 +(45 * intakeRotatePerDegree));
        }*/




    }
    public void setPositionI(double setpoint){
        setPoint = setpoint;
        sI.setPosition(setPoint);


    }
    public void setPositionD(double setpoint){
        setPoint = setpoint;
        sD.setPosition(setPoint);


    }
    public void resultDegrees() {
        Double sampleDegrees = vision.getTurnServoDegree();
        if (sampleDegrees == null) return;

        double mappedDegrees = sampleDegrees;
        if (mappedDegrees > 180) {
            mappedDegrees = 360 - mappedDegrees;
        }
        mappedDegrees = 180 - mappedDegrees;
        double resultDegrees = MathUtils.linear(mappedDegrees, 0, 180, 0.2, 1);
        bot.telem.addData("Result Degrees", resultDegrees);
    }

    public double rotateDegrees() {
        Double sampleDegrees = vision.getTurnServoDegree();
        if (sampleDegrees == null){
            return 0;
        }

        double mappedDegrees = sampleDegrees;
        if (mappedDegrees > 180) {
            mappedDegrees = 360 - mappedDegrees;
        }
        mappedDegrees = 180 - mappedDegrees;
        double resultDegrees = MathUtils.linear(mappedDegrees, 0, 180, 0.2, 1);

        return resultDegrees;



    }

}
