package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
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
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.PedroMirror;


@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Close Blue Auto")
public class CloseBlueAuto extends LinearOpMode {

    // --- BLUE TEAM POSES ---
    public static Pose score = new Pose(60, 90, Math.toRadians(-45));
    public static Pose start = new Pose(24, 121.5, Math.toRadians(-90));
    public static Pose preGrab = new Pose(50, 84, Math.toRadians(180));
    public static Pose grab = new Pose(17, 84, Math.toRadians(180));
    public static Pose preGrab2  = new Pose(50, 59, Math.toRadians(180));
    public static Pose grab2  = new Pose(20, 59, Math.toRadians(180));
    private Bot bot;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    private Intake intake;
    private Indexer indexer;
    private Shooter shooter;
    private Pivot pivot;

    private SequentialCommandGroup getFireSequence(Indexer indexer) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(indexer::indexOn, indexer)
                ),
                new WaitCommand(3000),
                new ParallelCommandGroup(
                        new InstantCommand(indexer::indexOff, indexer)
                )
        );
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

        indexer = new Indexer(bot);
        indexer.register();

        shooter = new Shooter(hardwareMap,telemetry);
        shooter.register();

        pivot = new Pivot(bot);
        pivot.register();
        pivot.close();


        ParallelDeadlineGroup auto = new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                        //Initial score
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(new BezierLine(start, score))
                                        .setLinearHeadingInterpolation(start.getHeading(), score.getHeading())
                                        .build()
                                ),
                                new WaitCommand(1500),
                                getFireSequence(indexer),
                                getScoringPath(f),
                                new WaitCommand(500),
                                getFireSequence(indexer),
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
                                new FollowPathCommand(f,f.pathBuilder()
                                        .addPath(new BezierLine(preGrab2, score))
                                        .setLinearHeadingInterpolation(grab2.getHeading(), score.getHeading())
                                        .build()
                        ),
                                new WaitCommand(500),
                                getFireSequence(indexer)
                        )
                )
        );
        auto.addCommands(
                new RunCommand(() -> intake.setPower(1), intake),
                new RunCommand(() -> shooter.setVelocity(1190), shooter)
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