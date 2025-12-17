package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Bot;

public class Shooter implements Subsystem {
    private DcMotorEx shooter;
    @Config
    public static class ShooterPIDF{
        public static double kp = 15;
        public static double ki = 0;
        public static double kd = 0.;
        public static double kf = 20;
    }
    public static double targetVelocity = 2500;
    private MultipleTelemetry telemetry;



    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        shooter.setVelocityPIDFCoefficients(
                ShooterPIDF.kp,
                ShooterPIDF.ki,
                ShooterPIDF.kd,
                ShooterPIDF.kf
        );


    }

    @Override
    public void periodic(){
        double currentVelocity = shooter.getVelocity();

        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", currentVelocity);


    }
    public void setVelocity(){
        shooter.setVelocity(targetVelocity);
    }
    public void shootOn(){
        shooter.setVelocity(targetVelocity);
    }
    public void shootOff(){
        shooter.setVelocity(0);
    }
}