package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MotorTest extends OpMode {
    //private DcMotorEx rightElevator, leftElevator;
    private DcMotorEx shooter;
    //private DcMotorEx frontLeft;
    //private DcMotorEx frontRight, backLeft, backRight;
    private CRServo index;
    private DcMotorEx intake;
    private Servo pivot;
    private GamepadEx driverGamepad;
    private final int low = 0;
    private final int high = 5000;
    private final double speed = 0.3;



    @Override
    public void init() {
        driverGamepad = new GamepadEx(gamepad1);

        intake = hardwareMap.get(DcMotorEx.class,"intake");
        index = hardwareMap.get(CRServo.class,"indexer");
        shooter = hardwareMap.get(DcMotorEx.class,"shooter");
        pivot = hardwareMap.get(Servo.class,"pivot");
        //rightElevator = hardwareMap.get(DcMotorEx.class, "re");
        //leftElevator = hardwareMap.get(DcMotorEx.class, "le");
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
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        pivot.setDirection(Servo.Direction.FORWARD);

        //rightElevator.setDirection(DcMotorEx.Direction.FORWARD);
        //rightElevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //leftElevator.setDirection(DcMotorEx.Direction.REVERSE);
        //leftElevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //rightElevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //rightElevator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //leftElevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //leftElevator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        intake.setPower(0);
        index.setPower(0);
        shooter.setPower(0);
        pivot.setPosition(0);
        //frontLeft.setPower(0);
        //backLeft.setPower(0);
        //frontRight.setPower(0);
        //backRight.setPower(0);

        //rightElevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //leftElevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        /*telemetry.addData("rightPos",rightElevator.getCurrentPosition());
        telemetry.addData("leftPos",leftElevator.getCurrentPosition());
        goToHigh();
        telemetry.update();*/

        intake.setPower(1);
        shooter.setPower(1);
        //frontLeft.setPower(1);
        //backLeft.setPower(1);
        //frontRight.setPower(1);
        //backRight.setPower(1);
        if(driverGamepad.gamepad.left_bumper){
            pivot.setPosition(0.3);
        }
        pivot.setPosition(0);
        if(driverGamepad.gamepad.b){
            index.setPower(1);
        }
        index.setPower(0);
    }
    /*public void goToPosition(int targetPos) {
        rightElevator.setTargetPosition(targetPos);
        leftElevator.setTargetPosition(targetPos);
        leftElevator.setPower(speed);
        rightElevator.setPower(speed);
    }
    public void goToLow() {
        goToPosition(low);
    }

    public void goToHigh() {
        goToPosition(high);
    }*/
}