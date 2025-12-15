package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Bot;

@TeleOp
public class MotorTest extends OpMode {
    private final Bot bot;
    //private final DcMotorEx rightElevator, leftElevator;
    private final DcMotorEx shooter;
    //private final DcMotorEx frontLeft;
    //private final DcMotorEx frontRight, backLeft, backRight;
    private final CRServo index;

    public MotorTest(Bot bot){
        this.bot = bot;

        index = bot.hMap.get(CRServo.class,"indexer");
        //rightElevator = bot.hMap.get(DcMotorEx.class, "re");
        //leftElevator = bot.hMap.get(DcMotorEx.class, "le");
        shooter = bot.hMap.get(DcMotorEx.class,"shooter");
        //frontLeft = bot.hMap.get(DcMotorEx.class, "M0");
        //frontRight = bot.hMap.get(DcMotorEx.class, "M1");
        //backLeft = bot.hMap.get(DcMotorEx.class, "M2");
        //backRight = bot.hMap.get(DcMotorEx.class, "M3");

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);//ESTE VA EN FORWARD PARA MOVER CHASSIS
        //backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        //frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        index.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //rightElevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //rightElevator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //leftElevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //leftElevator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setPower(0);
        //frontLeft.setPower(0);
        //backLeft.setPower(0);
        //frontRight.setPower(0);
        //backRight.setPower(0);
        index.setPower(0);
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        //bot.telem.addData("rightPos",rightElevator.getCurrentPosition());
        //bot.telem.addData("leftPos",leftElevator.getCurrentPosition());
        shooter.setPower(1);
        //frontLeft.setPower(1);
        //backLeft.setPower(1);
        //frontRight.setPower(1);
        //backRight.setPower(1);
        index.setPower(1);

        bot.telem.update();
    }
}
