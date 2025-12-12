package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Bot;

@TeleOp
public class ShootIndexTest extends OpMode {
    private Bot bot;
    private Indexer index;
    private Shooter shoot;

    @Override
    public void init() {
        index = new Indexer(bot);
        index.register();

        shoot = new Shooter(bot);
        shoot.register();
    }

    @Override
    public void loop() {
        index.indexOn();
        index.index2On();
        shoot.shootOn();
    }
}
