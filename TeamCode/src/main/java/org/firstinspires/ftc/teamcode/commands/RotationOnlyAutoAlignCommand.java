package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

/**
 * A command that uses PID control to hold only the robot's rotation at a specific heading.
 * Unlike PositionHoldCommand, this does NOT hold X/Y position - the robot can drift.
 * This is useful for auto-aligning to specific headings while maintaining manual control.
 */
@Config
public class RotationOnlyAutoAlignCommand extends CommandBase {
    private final Bot bot;
    private final double targetHeading;

    // --- PID COEFFICIENTS (Tune these values!) ---
    // Heading PID - reused from PositionHoldCommand
    public static double kPH = 1.2;
    public static double kIH = 0.0;
    public static double kDH = 0.01;

    private final PIDController hController = new PIDController(kPH, kIH, kDH);

    /**
     * Creates a command to auto-align the robot to a specific heading.
     * @param bot The main robot object.
     * @param targetHeading The target heading in radians.
     */
    public RotationOnlyAutoAlignCommand(Bot bot, double targetHeading) {
        this.bot = bot;
        this.targetHeading = targetHeading;
        addRequirements(MecanumDrive.getInstance());
    }

    @Override
    public void initialize() {
        hController.reset();
    }

    @Override
    public void execute() {
        double currentHeading = MecanumDrive.odo.getPosition().getHeading(AngleUnit.RADIANS);

        // Calculate heading error with proper angle wrapping
        double errorH = Math.atan2(
                Math.sin(targetHeading - currentHeading),
                Math.cos(targetHeading - currentHeading)
        );

        // Calculate rotation power from PID
        double pH = hController.calculate(0, errorH);

        // Pass through X/Y controls from left stick, override rotation with PID
        MecanumDrive.getInstance().drive(
                -bot.driver.getLeftY() * bot.speed,

                -bot.driver.getLeftX() * bot.speed,
                pH
        );
        bot.telem.addData("CurrentAngle", Math.toDegrees(currentHeading));
        bot.telem.addData("AngleTarget", Math.toDegrees(targetHeading));
    }

    @Override
    public void end(boolean interrupted) {
        MecanumDrive.getInstance().drive(0, 0, 0);
    }
}