package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter extends SubsystemBase {
    public DcMotorEx shooter;
    @Config
    public static class ShooterPIDF{
        public static double kp = 400;
        public static double ki = 0;
        public static double kd = 2.5;
        public static double kf = 15;
    }
    private MultipleTelemetry telemetry;



    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        updatePIDF();
    }
    public void updatePIDF() {
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        telemetry.addData("Shooter/Current", currentVelocity);
    }
    public void setVelocity(double velocity){
        shooter.setVelocity(velocity);
    }
}