package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Bot;

public class ScissorElevator extends SubsystemBase {
    private final Bot bot;
    private DcMotorEx elevator;
    private final int low = 0;
    private final int high = 4500; //missing tuning
    private final double speed = 0.85;
    private final double powerLimit = 0.30;
    public ScissorElevator(Bot bot) {
        this.bot = bot;
        elevator = bot.hMap.get(DcMotorEx.class, "elevator");

        elevator.setDirection(DcMotorEx.Direction.FORWARD);
        elevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void goToPosition(int targetPos) {
        elevator.setTargetPosition(targetPos);
        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        elevator.setPower(speed);
    }

    public void goToLow() {
        goToPosition(low);
    }

    public void goToHigh() {
        goToPosition(high);
    }

    public void setManualPower(double power) {
        if (elevator.getMode() == DcMotorEx.RunMode.RUN_TO_POSITION) {
            elevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        int currentPos = elevator.getCurrentPosition();
        double limitedPower = power * powerLimit;

        if (limitedPower > 0 && currentPos >= high) {
            limitedPower = 0;
        }
        else if (limitedPower < 0 && currentPos <= low) {
            limitedPower = 0;
        }
        elevator.setPower(limitedPower);
    }
    public int getCurrentPosition() {
        return elevator.getCurrentPosition();
    }

    public boolean isBusy() {
        return elevator.isBusy();
    }
}