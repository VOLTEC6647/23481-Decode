package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Bot;

import java.util.List;


@Config
public class Limelight extends SubsystemBase {
    private final Bot bot;
    private final Limelight3A limelight;
    public static double TURRET_HEADING_KP = 0.02;
    public static double MAX_TURRET_ROTATION_POWER = 0.5;
    public static double CHASSIS_HEADING_KP = 0.02;
    private double turretCorrectionRotation = 0.0;

    private double tx = 0;
    private double ty = 0;
    private boolean hasTarget = false;
    private int tagID = -1;

    public Limelight(Bot bot){
        this.bot = bot;

        limelight = bot.hMap.get(Limelight3A.class,"limelight");
        bot.telem.setMsTransmissionInterval(11);
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
    public int getTagID(){
        return tagID;
    }

    @Override
    public void periodic() {LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            hasTarget = true;
            tx = result.getTx();
            ty = result.getTy();
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (!fiducials.isEmpty()) {
                LLResultTypes.FiducialResult primaryTag = fiducials.get(0);
                this.tagID = primaryTag.getFiducialId();
            } else {
                this.tagID = -1;
            }

            Pose3D botpose = result.getBotpose();
            if (MecanumDrive.odo != null && botpose != null) {
                MecanumDrive.odo.setPosition(new Pose2D(DistanceUnit.METER, botpose.getPosition().x, botpose.getPosition().y, AngleUnit.RADIANS, botpose.getOrientation().getYaw(AngleUnit.RADIANS)));
            }
        } else {
            hasTarget = false;
            tx = 0;
            ty = 0;
            tagID = -1;
        }

        bot.telem.addData("LL | Target", hasTarget);
        bot.telem.addData("LL | tx", tx);
        bot.telem.addData("LL | Tag ID", tagID);
    }
    public boolean hasTarget() {
        return hasTarget;
    }

    public double getTurnPower() {
        if (!hasTarget) return 0;

        double power = -tx * CHASSIS_HEADING_KP;

        double MAX_POWER = 0.6;
        return Math.min(Math.max(power, -MAX_POWER), MAX_POWER);
    }
}

/*previous periodic:

        LLResult result = limelight.getLatestResult();

        turretCorrectionRotation = 0.0;

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
            targetCorrectionRotation = -chassisPower;

            double turretPower = tx * TURRET_HEADING_KP;

            turretPower = Math.min(Math.max(turretPower, -MAX_TURRET_ROTATION_POWER), MAX_TURRET_ROTATION_POWER);

            turretCorrectionRotation = -turretPower;

            bot.telem.addData("Limelight | Target Visible", true);
            bot.telem.addData("Limelight | tx (Error Grados)", tx);
            bot.telem.addData("Limelight | Turret Correction", turretCorrectionRotation);
            bot.telem.addData("Limelight | Chassis Correction", targetCorrectionRotation);

        } else {
            bot.telem.addData("Limelight | Target Visible", false);
        }
 */