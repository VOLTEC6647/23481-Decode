package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

/**
 * A command that uses PID control to hold the robot at a specific target Pose.
 * This is useful for maintaining stability during shooting or for endgame parking.
 */
@Config
public class PositionHoldCommand extends CommandBase {
    private final Bot bot;
    private final Follower follower;
    private final Pose targetPose;

    // --- PID COEFFICIENTS (Tune these values!) ---
    // Translational PID (X and Y)
    public static double kPX = 0.08;
    public static double kIX = 0.0;
    public static double kDX = 0.005;

    public static double kPH = 1.2;
    public static double kIH = 0.0;
    public static double kDH = 0.005;

    private static final double MAX_POWER = 0.8;

    private final PIDController xController = new PIDController(kPX, kIX, kDX);
    private final PIDController yController = new PIDController(kPX, kIX, kDX);
    private final PIDController hController = new PIDController(kPH, kIH, kDH);
    /**
     * Creates a command to hold the robot at its current pose.
     * @param bot The main robot object.
     * @param follower The PedroPathing follower instance.
     */
    public PositionHoldCommand(Bot bot, Follower follower) {
        this(bot, follower, follower.getPose());
    }

    /**
     * Creates a command to hold the robot at a specific pose.
     * @param bot The main robot object.
     * @param follower The PedroPathing follower instance.
     * @param targetPose The specific pose (x, y, heading) to hold.
     */
    public PositionHoldCommand(Bot bot, Follower follower, Pose targetPose) {
        this.bot = bot;
        this.follower = follower;
        this.targetPose = targetPose;
        addRequirements(MecanumDrive.getInstance());
    }

    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        hController.reset();
    }

    @Override
    public void execute() {
        Pose currentPose = follower.getPose();
        double heading = currentPose.getHeading();

        // Calculate Errors in Field Coordinates
        double errorX = targetPose.getX() - currentPose.getX();
        double errorY = targetPose.getY() - currentPose.getY();
        double errorH = Math.atan2(Math.sin(targetPose.getHeading() - heading), Math.cos(targetPose.getHeading() - heading));

        // Field-centric power demands
        double pX = xController.calculate(0, errorX);
        double pY = yController.calculate(0, errorY);
        double pH = hController.calculate(0, errorH);

        // Convert Field Power to Robot-Relative Power
        // This is the Inverse Rotation Matrix
        double rotX = pX * Math.cos(-heading) - pY * Math.sin(-heading);
        double rotY = pX * Math.sin(-heading) + pY * Math.cos(-heading);

        double fl = rotY + rotX + pH;
        double bl = rotY - rotX + pH;
        double fr = rotY - rotX - pH;
        double br = rotY + rotX - pH;

        MecanumDrive.getInstance().setRawMotorPowers(fl, fr, bl, br);
    }

    @Override
    public void end(boolean interrupted) {
        MecanumDrive.getInstance().setRawMotorPowers(0,0,0,0);}
}