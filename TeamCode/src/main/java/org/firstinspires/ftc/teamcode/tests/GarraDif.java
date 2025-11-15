package org.firstinspires.ftc.teamcode.tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DiffClaw;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Dif")
public class GarraDif extends CommandOpMode {
    private Bot bot;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    private DiffClaw claw;

    public void initialize() {

        CommandScheduler.getInstance().reset();

        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        bot = new Bot(telem, hardwareMap, driverGamepad, operatorGamepad);

        claw = new DiffClaw(bot);
        claw.register();


        telem.addData("status","init");
        telem.update();

    }

    /*@Config
    public static class targetServo{
        public static double servoPosition = 0;
    }*/

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telem.addData("status","start");
    }
}

/*if(bot.opertator.gamepad.dpad_up) {
            sD.setPosition(1);
            sI.setPosition(1);
        }
        else if (bot.opertator.gamepad.dpad_down){
            sD.setPosition(0);
            sI.setPosition(0);
        }
        if(bot.driver.gamepad.dpad_left) {
            sD.setPosition(0);
            sI.setPosition(1);
        }
        else if (bot.driver.gamepad.dpad_right){
            sD.setPosition(1);
            sI.setPosition(0);
        }*/