package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Bot;

public class MotorTest implements Subsystem {
    private final Bot bot;
    private final DcMotorEx rightElevator, leftElevator;
    //private final DcMotorEx frontLeft;
    //private final DcMotorEx frontRight, backLeft, backRight;

    public MotorTest(Bot bot){
        this.bot = bot;

        rightElevator = bot.hMap.get(DcMotorEx.class, "re");
        leftElevator = bot.hMap.get(DcMotorEx.class, "le");
        //frontLeft = bot.hMap.get(DcMotorEx.class, "M0");
        //frontRight = bot.hMap.get(DcMotorEx.class, "M1");
        //backLeft = bot.hMap.get(DcMotorEx.class, "M2");
        //backRight = bot.hMap.get(DcMotorEx.class, "M3");

        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);//ESTE VA EN FORWARD PARA MOVER CHASSIS
        //backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        //frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightElevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftElevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftElevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //frontLeft.setPower(0);
        //backLeft.setPower(0);
        //frontRight.setPower(0);
        //backRight.setPower(0);
    }

    @Override
    public void periodic() {
        bot.telem.addData("rightPos",rightElevator.getCurrentPosition());
        bot.telem.addData("leftPos",leftElevator.getCurrentPosition());
        //frontLeft.setPower(1);
        //backLeft.setPower(1);
        //frontRight.setPower(1);
        //backRight.setPower(1);

        bot.telem.update();
    }
}
