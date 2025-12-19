package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
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
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blue Point Auto")
public class PointAutoBlue extends LinearOpMode {

    // --- BLUE TEAM POSES ---
    public static Pose score = new Pose(55, 85, Math.toRadians(315)); // Target
    public static Pose start = new Pose(55, 9.5, Math.toRadians(270)); // Wall Start
    public static Pose preGrab1  = new Pose(60, 80, Math.toRadians(180));
    public static Pose grab1  = new Pose(33.5, 80, Math.toRadians(180));
    public static Pose preGrab2  = new Pose(60, 60, Math.toRadians(180));
    public static Pose grab2  = new Pose(33.5, 60, Math.toRadians(180));
    public static Pose preGrab3  = new Pose(60, 35, Math.toRadians(180));
    public static Pose grab3  = new Pose(33.5, 35, Math.toRadians(180));

    private Bot bot;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    private Intake intake;
    private Indexer indexer;
    private Shooter shooter;

    private SequentialCommandGroup getFireSequence(Shooter shooter, Indexer indexer) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(shooter::shootOn, shooter),
                        new InstantCommand(indexer::indexOn, indexer)
                ),
                new WaitCommand(2000),
                new ParallelCommandGroup(
                        new InstantCommand(shooter::shootOff, shooter),
                        new InstantCommand(indexer::indexOff, indexer)
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


        SequentialCommandGroup auto =
                new SequentialCommandGroup(
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(new BezierLine(start, score))
                                .setLinearHeadingInterpolation(start.getHeading(), score.getHeading())
                                .build()
                        ),
                        getFireSequence(shooter, indexer),
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(new BezierLine(score, preGrab1))
                                .setLinearHeadingInterpolation(score.getHeading(),preGrab1.getHeading())
                                .build()
                        ),
                        new ParallelRaceGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(new BezierLine(preGrab1, grab1))
                                        .setLinearHeadingInterpolation(preGrab1.getHeading(),grab1.getHeading())
                                        .build()
                                ),
                                new RunCommand(() -> intake.setPower(1.0), intake),
                                new RunCommand(()-> indexer.indexOn(),indexer)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new InstantCommand(()-> intake.setPower(0), intake)
                        ),
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(new BezierLine(grab1, score))
                                .setLinearHeadingInterpolation(grab1.getHeading(),score.getHeading())
                                .build()
                        ),
                        getFireSequence(shooter, indexer),
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(new BezierLine(score, preGrab2))
                                .setLinearHeadingInterpolation(score.getHeading(),preGrab2.getHeading())
                                .build()
                        ),
                        new ParallelRaceGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(new BezierLine(preGrab2, grab2))
                                        .setLinearHeadingInterpolation(preGrab2.getHeading(),grab2.getHeading())
                                        .build()
                                ),
                                new RunCommand(() -> intake.setPower(1.0), intake)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new InstantCommand(()-> intake.setPower(0), intake)
                        ),
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(new BezierLine(grab2, score))
                                .setLinearHeadingInterpolation(grab2.getHeading(),score.getHeading())
                                .build()
                        ),
                        getFireSequence(shooter, indexer),
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(new BezierLine(score, preGrab3))
                                .setLinearHeadingInterpolation(score.getHeading(),preGrab3.getHeading())
                                .build()
                        ),
                        new ParallelRaceGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(new BezierLine(preGrab3, grab3))
                                        .setLinearHeadingInterpolation(preGrab3.getHeading(),grab3.getHeading())
                                        .build()
                                ),
                                new RunCommand(()-> intake.setPower(1), intake)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new InstantCommand(()-> intake.setPower(0), intake)
                        ),
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(new BezierLine(grab3, score))
                                .setLinearHeadingInterpolation(grab3.getHeading(),score.getHeading())
                                .build()
                        ),
                        getFireSequence(shooter, indexer)
                );

        waitForStart();

        f.setMaxPower(10.0 / vs.getVoltage());
        CommandScheduler.getInstance().schedule(auto);

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            f.update();
            telem.addData("current pose", f.getPose());

            telem.addData("Follower Status", f.isBusy() ? "Running Path" : "Finished");
            telem.update();
        }
        CommandScheduler.getInstance().reset();
    }
}