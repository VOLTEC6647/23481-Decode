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
        //indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bot.telem.addData("Status", "Initialized");
        bot.telem.update();
    }

    @Override
    public void periodic(){
        if (bot.opertator.gamepad.left_trigger > 0.6) {
            shooter.setPower(1.0);
            bot.telem.addData("Shooter", "Full Power");
        } else if(bot.opertator.gamepad.left_trigger <= 0.6 && bot.driver.gamepad.left_trigger >= 0.1){
            shooter.setPower(0.5);
            bot.telem.addData("Shooter", "Half Power");
        } else if(bot.opertator.gamepad.left_trigger <= 0.09){
            shooter.setPower(0);
            bot.telem.addData("Shooter", "Dead");
        }

        if (bot.opertator.gamepad.left_bumper) {
            indexer.setPower(1);
            bot.telem.addData("Indexer", "Running");
        } else {
            indexer.setPower(0);
            bot.telem.addData("Indexer", "Stopped");
        }
        bot.telem.update();
    }
}
