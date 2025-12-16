package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Bot;

public class NewElevator extends SubsystemBase {
    private final Bot bot;
    private final DcMotorEx rightElevator;
    private final DcMotorEx leftElevator;
    private final int low = 0;
    private final int high = 5000;
    private final double speed = 0.5;
    public NewElevator(Bot bot) {
        this.bot = bot;
        rightElevator = bot.hMap.get(DcMotorEx.class, "re");
        leftElevator = bot.hMap.get(DcMotorEx.class, "le");

        rightElevator.setDirection(DcMotorEx.Direction.FORWARD);
        rightElevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftElevator.setDirection(DcMotorEx.Direction.REVERSE);
        leftElevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rightElevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftElevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftElevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void goToPosition(int targetPos) {
        rightElevator.setTargetPosition(targetPos);
        rightElevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightElevator.setPower(speed);

        leftElevator.setTargetPosition(targetPos);
        leftElevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftElevator.setPower(speed);
    }

    @Override
    public void periodic(){
    }

    public void goToLow() {
        goToPosition(low);
    }

    public void goToHigh() {
        goToPosition(high);
    }

    /*public int getCurrentPositionRight() {
        return rightElevator.getCurrentPosition();
    }
    public int getCurrentPositionLeft() {
        return leftElevator.getCurrentPosition();
    }

    public boolean isBusy() {
        return elevator.isBusy();
    }*/
}