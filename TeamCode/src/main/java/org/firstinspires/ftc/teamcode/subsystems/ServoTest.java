package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Servo Test", group = "TeleOp")
public class ServoTest implements Subsystem {
    private final CRServo indexer;

    public ServoTest(CRServo indexer) {
        this.indexer = indexer;
    }

    @Override
    public void periodic(){
        indexer.setPower(1);
    }
}
