package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class TurretAutoAlignCommand extends CommandBase {
    private final Turret turret;
    private final Limelight limelight;
    public static final double ALIGNMENT_TOLERANCE_POWER = 0.05;

    public TurretAutoAlignCommand(Turret turret, Limelight limelight) {
        this.turret = turret;
        this.limelight = limelight;
        addRequirements(turret, limelight);
    }

    @Override
    public void initialize() {
        // Optional: Perform any setup (e.g., set pipeline, check initial status)
    }

    @Override
    public void execute() {
        double correctionPower = limelight.getTurretCorrectionPower();

        if (limelight.isTargetVisible()) {
            turret.setTurretPower(correctionPower);
        } else {
            turret.setTurretPower(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.setTurretPower(0.0);
    }

    @Override
    public boolean isFinished() {
        double correctionPower = limelight.getTurretCorrectionPower();
        int currentPosition = turret.getCurrentPosition();

        // Condition 1: Turret is successfully aligned (correction power is small)
        boolean aligned = limelight.isTargetVisible() &&
                Math.abs(correctionPower) < ALIGNMENT_TOLERANCE_POWER;

        // Condition 2: Turret has hit a limit and is trying to move past it
        boolean atLimitAndStuck =
                (currentPosition >= Turret.MAX_ENCODER_LIMIT && correctionPower > 0) ||
                        (currentPosition <= Turret.MIN_ENCODER_LIMIT && correctionPower < 0);

        // The command finishes if it's aligned OR if it's stuck against a limit.
        return aligned || atLimitAndStuck;
    }
}
