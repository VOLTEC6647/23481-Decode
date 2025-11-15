package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Bot;


@Config
public class Limelight extends SubsystemBase {
    private Bot bot;
    private Limelight3A limelight;
    public static double TURRET_HEADING_KP = 0.02;
    public static double MAX_TURRET_ROTATION_POWER = 0.5;
    public static double CHASSIS_HEADING_KP = 0.02;
    public static double MAX_CHASSIS_ROTATION_POWER = 0.5;
    private double turretCorrectionRotation = 0.0;


    public Limelight(Bot bot){
        this.bot = bot;

        limelight = bot.hMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }
    public double getTurretCorrectionPower() {
        return turretCorrectionRotation;
    }
    public boolean isTargetVisible() {
        LLResult result = limelight.getLatestResult();
        return (result != null && result.isValid());
    }

    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();

        turretCorrectionRotation = 0.0;
        MecanumDrive.targetCorrectionRotation = 0.0;

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (MecanumDrive.odo != null && botpose != null) {
                MecanumDrive.odo.setPosition(new Pose2D(DistanceUnit.INCH,
                        botpose.getPosition().x,
                        botpose.getPosition().y,
                        AngleUnit.RADIANS,
                        botpose.getOrientation().getYaw(AngleUnit.RADIANS)));
            }

            double tx = result.getTx();

            double chassisPower = tx * CHASSIS_HEADING_KP;
            chassisPower = Math.min(Math.max(chassisPower, -MAX_CHASSIS_ROTATION_POWER), MAX_CHASSIS_ROTATION_POWER);
            MecanumDrive.targetCorrectionRotation = -chassisPower;

            double turretPower = tx * TURRET_HEADING_KP;

            turretPower = Math.min(Math.max(turretPower, -MAX_TURRET_ROTATION_POWER), MAX_TURRET_ROTATION_POWER);

            turretCorrectionRotation = -turretPower;

            bot.telem.addData("Limelight | Target Visible", true);
            bot.telem.addData("Limelight | tx (Error Grados)", tx);
            bot.telem.addData("Limelight | Turret Correction", turretCorrectionRotation);
            bot.telem.addData("Limelight | Chassis Correction", MecanumDrive.targetCorrectionRotation);

        } else {
            bot.telem.addData("Limelight | Target Visible", false);
        }
    }
}