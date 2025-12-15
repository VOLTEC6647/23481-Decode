package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ClawPivot;
import org.firstinspires.ftc.teamcode.subsystems.ClawUp;
import org.firstinspires.ftc.teamcode.subsystems.DiffClaw;
import org.firstinspires.ftc.teamcode.subsystems.DiffClawUp;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import static org.firstinspires.ftc.teamcode.tests.setMotor.setMecanisms.shootPower;
import static org.firstinspires.ftc.teamcode.tests.setMotor.setMecanisms.indexPower;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "SetMotor", group = "Tools")
public class setMotor extends CommandOpMode {
    private Bot bot;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    private Shooter shoot;
    private Indexer index;

    public void initialize() {

        CommandScheduler.getInstance().reset();

        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        bot = new Bot(telem, hardwareMap, driverGamepad, operatorGamepad);

        shoot = new Shooter(bot);
        shoot.register();

        index = new Indexer(bot);
        index.register();




        telem.addData("status","init");
        telem.update();

    }

    @Config
    public static class setMecanisms{
        public static double shootPower = 1;
        public static double indexPower = 1;

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        index.setPower(indexPower);
        shoot.setPower(shootPower);




        telem.addData("status","start");
    }
}