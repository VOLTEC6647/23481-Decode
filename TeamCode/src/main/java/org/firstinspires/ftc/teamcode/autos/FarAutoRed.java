package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.commands.Drawing;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;


@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Far Auto Red")
public class FarAutoRed extends LinearOpMode {

    // --- RED TEAM POSES ---
    public static Pose start = new Pose(81, 12.5, Math.toRadians(-90));
    public static Pose score = new Pose(81, 20.5, Math.toRadians(-113));
    public static Pose preGrab = new Pose(89, 34, Math.toRadians(0));
    public static Pose grab = new Pose(124, 34, Math.toRadians(0));
    public static Pose preGrab2  = new Pose(89, 58.5, Math.toRadians(0));
    public static Pose grab2  = new Pose(124, 58.5, Math.toRadians(0));
    public static Pose end = new Pose(108,72,Math.toRadians(-90));
    private Bot bot;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    private Intake intake;
    private Shooter shooter,shooter2;
    private Pivot pivot;
    private Stopper stopper;

    private SequentialCommandGroup getFireSequence(Stopper stopper) {
        return new SequentialCommandGroup(
                new InstantCommand(stopper::pass,stopper),
                new WaitCommand(3000),
                new InstantCommand(stopper::stop,stopper)
        );
    }
    private RunCommand shoot(Shooter shooter){
        return new RunCommand(() -> {
            double currentVel = shooter.shooter.getVelocity();
            if (Math.abs(currentVel - 1350) < 40) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }
        }, intake);
    }
    private SequentialCommandGroup getScoringPath(Follower f){
        return new SequentialCommandGroup(
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(new BezierLine(score, preGrab))
                        .setLinearHeadingInterpolation(score.getHeading(), preGrab.getHeading())
                        .addPath(new BezierLine(preGrab, grab))
                        .setLinearHeadingInterpolation(preGrab.getHeading(), grab.getHeading())
                        .addPath(new BezierLine(grab, preGrab))
                        .setLinearHeadingInterpolation(grab.getHeading(), preGrab.getHeading())
                        .addPath(new BezierLine(preGrab, score))
                        .setLinearHeadingInterpolation(preGrab.getHeading(), score.getHeading())
                        .build()
                )
        );
    }


    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);
        bot = new Bot(telem, hardwareMap, driverGamepad, operatorGamepad);

        VoltageSensor vs = bot.hMap.voltageSensor.iterator().next();

        Follower f = Constants.createFollower(bot.hMap);
        f.setStartingPose(start);
        f.update();

        intake = new Intake(bot);
        intake.register();

        stopper = new Stopper(bot);
        stopper.register();
        stopper.stop();

        shooter = new Shooter(hardwareMap,telemetry);
        shooter.register();
        shooter2 = new Shooter(hardwareMap,telemetry);
        shooter2.register();

        pivot = new Pivot(bot);
        pivot.register();
        pivot.setPosition(.35);


        ParallelDeadlineGroup auto = new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                        //Initial score
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(new BezierLine(start, score))
                                        .setLinearHeadingInterpolation(start.getHeading(), score.getHeading())
                                        .build()
                                ),
                                new WaitCommand(2000),
                                getFireSequence(stopper),
                                getScoringPath(f),
                                new WaitCommand(500),
                                getFireSequence(stopper),
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(new BezierLine(score, preGrab2))
                                        .setLinearHeadingInterpolation(score.getHeading(), preGrab2.getHeading())
                                        .build()
                                ),
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(new BezierLine(preGrab2, grab2))
                                        .setLinearHeadingInterpolation(preGrab2.getHeading(), grab2.getHeading())
                                        .build()
                                ),
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(new BezierLine(grab2, preGrab2))
                                        .setLinearHeadingInterpolation(grab2.getHeading(), preGrab2.getHeading())
                                        .build()
                                ),
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(new BezierLine(preGrab2, score))
                                        .setLinearHeadingInterpolation(preGrab2.getHeading(), score.getHeading())
                                        .build()
                                ),
                                new WaitCommand(500),
                                getFireSequence(stopper),
                                new FollowPathCommand(f,f.pathBuilder()
                                        .addPath(new BezierLine(score,end))
                                        .setLinearHeadingInterpolation(score.getHeading(),end.getHeading())
                                        .build()
                                )
                        )
                )
        );
        auto.addCommands(
                shoot(shooter),
                new RunCommand(()->shooter.setVelocity(1350),shooter),
                new RunCommand(()->shooter2.setVelocity2(1350),shooter2)
        );

        waitForStart();

        f.setMaxPower(10.0 / vs.getVoltage());
        CommandScheduler.getInstance().schedule(auto);

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            f.update();
            telem.addData("current pose", f.getPose());
            telem.addData("velocity",shooter.shooter.getVelocity());
            telem.addData("Follower Status", f.isBusy() ? "Running Path" : "Finished");
            telem.update();
            Drawing.drawDebug(f);
            Drawing.sendPacket();
        }
        CommandScheduler.getInstance().reset();

        File gyr = AppUtil.getInstance().getSettingsFile("gyropending.txt");
        ReadWriteFile.writeFile(gyr, "1");
    }
}