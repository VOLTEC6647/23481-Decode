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
    public static double kPX = 0.05;
    public static double kIX = 0.001;
    public static double kDX = 0.001;

    public static double kPH = 0.8;
    public static double kIH = 0.001;
    public static double kDH = 0.001;

    private static final double MAX_POWER = 0.7;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController hController;

    /**
     * Creates a command to hold the robot at its current pose.
     * @param bot The main robot object.
     * @param follower The PedroPathing follower instance.
     */
    public PositionHoldCommand(Bot bot, Follower follower) {
        this.bot = bot;
        this.follower = follower;
        this.targetPose = follower.getPose();

        xController = new PIDController(kPX, kIX, kDX);
        yController = new PIDController(kPX, kIX, kDX);
        hController = new PIDController(kPH, kIH, kDH);
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

        xController = new PIDController(kPX, kIX, kDX);
        yController = new PIDController(kPX, kIX, kDX);
        hController = new PIDController(kPH, kIH, kDH);
    }

    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        hController.reset();

        xController.setPID(kPX, kIX, kDX);
        yController.setPID(kPX, kIX, kDX);
        hController.setPID(kPH, kIH, kDH);
    }

    @Override
    public void execute() {
        Pose currentPose = follower.getPose();

        double errorX = targetPose.getX() - currentPose.getX(); // Error along field X (Strafe)
        double errorY = targetPose.getY() - currentPose.getY(); // Error along field Y (Forward)

        double errorH = targetPose.getHeading() - currentPose.getHeading();
        errorH = Math.atan2(Math.sin(errorH), Math.cos(errorH));

        double powerFieldX = xController.calculate(0, errorX);
        double powerFieldY = yController.calculate(0, errorY);
        double powerH = hController.calculate(0, errorH);

        double heading = currentPose.getHeading();
        double sin = Math.sin(heading);
        double cos = Math.cos(heading);

        // robotCentricPowerY = Forward/Backward power correction
        // robotCentricPowerX = Strafe power correction
        double robotCentricPowerX = powerFieldX * cos + powerFieldY * sin;
        double robotCentricPowerY = powerFieldX * sin + powerFieldY * cos;

        robotCentricPowerX = com.arcrobotics.ftclib.util.MathUtils.clamp(robotCentricPowerX, -MAX_POWER, MAX_POWER);
        robotCentricPowerY = com.arcrobotics.ftclib.util.MathUtils.clamp(robotCentricPowerY, -MAX_POWER, MAX_POWER);
        powerH = com.arcrobotics.ftclib.util.MathUtils.clamp(powerH, -MAX_POWER, MAX_POWER);

        double frontLeftPower = robotCentricPowerY + robotCentricPowerX + powerH;
        double backLeftPower = robotCentricPowerY - robotCentricPowerX + powerH;
        double frontRightPower = robotCentricPowerY - robotCentricPowerX - powerH;
        double backRightPower = robotCentricPowerY + robotCentricPowerX - powerH;

        bot.setRawMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    @Override
    public void end(boolean interrupted) {
        MecanumDrive.getInstance().setRawMotorPowers(0,0,0,0);}
}