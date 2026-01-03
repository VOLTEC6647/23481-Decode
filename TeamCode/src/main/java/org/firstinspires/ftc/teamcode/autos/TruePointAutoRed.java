package org.firstinspires.ftc.teamcode.autos;

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
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blue Point Auto")
public class TruePointAutoRed extends LinearOpMode {

    // --- RED TEAM POSES ---
    public static Pose score = new Pose(88, 15, Math.toRadians(-111));
    public static Pose start = new Pose(88, 9.5, Math.toRadians(-90));
    public static Pose preGrab  = new Pose(114, 15, Math.toRadians(0));
    public static Pose grab  = new Pose(134, 15, Math.toRadians(0));
    private Bot bot;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    private Intake intake;
    private Indexer indexer;
    private Shooter shooter;

    private SequentialCommandGroup getFireSequence(Indexer indexer) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(indexer::indexOn, indexer)
                ),
                new WaitCommand(2000),
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
                        .addPath(new BezierLine(grab, score))
                        .setLinearHeadingInterpolation(grab.getHeading(), score.getHeading())
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


        ParallelDeadlineGroup auto = new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                        //Initial score
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, f.pathBuilder()
                                        .addPath(new BezierLine(start, score))
                                        .setLinearHeadingInterpolation(start.getHeading(), score.getHeading())
                                        .build()
                                ),
                                getFireSequence(indexer),
                                //Cycle 1
                                getScoringPath(f),
                                getFireSequence(indexer),
                                //Cycle 2
                                getScoringPath(f),
                                getFireSequence(indexer),
                                //Cycle 3
                                getScoringPath(f),
                                getFireSequence(indexer)
                        )
                )
        );
        auto.addCommands(
                new RunCommand(() -> intake.setPower(1), intake),
                new RunCommand(() -> shooter.setVelocity(1900), shooter)
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
            Drawing.drawDebug(f);
            Drawing.sendPacket();
        }
        CommandScheduler.getInstance().reset();
    }
}