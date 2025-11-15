package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Bot;

public class Turret extends SubsystemBase {
    private final Bot bot;
    private final DcMotor turret;
    public static final int MAX_ENCODER_LIMIT = 2000; //missing tuning
    public static final int MIN_ENCODER_LIMIT = -2000; //missing tuning
    public static final double MAX_TURRET_SPEED = 0.8;

    public Turret(Bot bot) {
        this.bot = bot;

        turret = bot.hMap.get(DcMotor.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.FORWARD);

        // IMPORTANT: Use RUN_USING_ENCODER or RUN_WITHOUT_ENCODER but you need
        // to set a mode that allows reading of the current position.
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        resetEncoder();
    }
    public void resetEncoder() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTurretPower(double power) {
        int currentPosition = turret.getCurrentPosition();

        if (power > 0) {
            if (currentPosition >= MAX_ENCODER_LIMIT) {
                turret.setPower(0.0);
            } else {
                turret.setPower(power);
            }
            if (currentPosition <= MIN_ENCODER_LIMIT) {
                turret.setPower(0.0);
            } else {
                turret.setPower(power);
            }
        } else {
            turret.setPower(0.0);
        }

        bot.telem.addData("Turret | Position", currentPosition);
    }

    public int getCurrentPosition() {
        return turret.getCurrentPosition();
    }
    @Override
    public void periodic() {
        bot.telem.addData("Turret | Position (Ticks)", getCurrentPosition());
        bot.telem.addData("Turret | Limits (Min/Max)", MIN_ENCODER_LIMIT + " / " + MAX_ENCODER_LIMIT);
    }
}