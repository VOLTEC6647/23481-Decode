package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Bot;

public class ServoTest implements Subsystem {
    private final Bot bot;
    private final CRServo indexer;

    public ServoTest(Bot bot) {
        this.bot = bot;

        indexer = bot.hMap.get(CRServo.class, "indexer");
    }

    @Override
    public void periodic(){
        indexer.setPower(1);
    }
}
