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
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class CombinedAuto extends LinearOpMode {

    // Scoring Poses
    public static Pose score = new Pose(55, 85, Math.toRadians(315));
    public static Pose start = new Pose(55, 9.5, Math.toRadians(270));
    public static Pose checkID = new Pose(72, 108, Math.toRadians(270));
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
    private Limelight ll;
    private Intake intake;
    private Indexer indexer;
    private Shooter shooter;

    private FollowPathCommand initialMovementCommand;

    /**
     * Creates a command group for the firing sequence (Shooter ON, Indexer ON, wait, then OFF).
     */
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

    /**
     * Helper method to get the final path based on the detected tag ID.
     * This path starts after detection is complete.
     */
    private SequentialCommandGroup getFinalPath(int tagID, Follower f) {
        SequentialCommandGroup preStart = new SequentialCommandGroup(
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(new BezierLine(checkID, score))
                        .setLinearHeadingInterpolation(checkID.getHeading(), score.getHeading())
                        .build()
                ),
                getFireSequence(shooter,indexer)
        );
        SequentialCommandGroup autoPath1 = new SequentialCommandGroup(
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
                        new RunCommand(() -> intake.setPower(1.0), intake)
                ),
                new SequentialCommandGroup(
                        new WaitCommand(200),
                        new InstantCommand(()-> intake.setPower(0), intake)
                ),
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(new BezierLine(grab1, score))
                        .setLinearHeadingInterpolation(grab1.getHeading(),score.getHeading())
                        .build()
                )
        );

        SequentialCommandGroup autoPath2 = new SequentialCommandGroup(
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
                )
        );

        SequentialCommandGroup autoPath3 = new SequentialCommandGroup(
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
                        new RunCommand(() -> intake.setPower(1.0), intake)
                ),
                new SequentialCommandGroup(
                        new WaitCommand(200),
                        new InstantCommand(()-> intake.setPower(0), intake)
                ),
                new FollowPathCommand(f, f.pathBuilder()
                        .addPath(new BezierLine(grab3, score))
                        .setLinearHeadingInterpolation(grab3.getHeading(),score.getHeading())
                        .build()
                )
        );
        SequentialCommandGroup specificGrabSequence;
        switch (tagID) {
            case 22:
                specificGrabSequence = autoPath2;
                break;
            case 23:
                specificGrabSequence = autoPath3;
                break;
            case 21:
            default:
                specificGrabSequence = autoPath1;
                break;
        }
        return new SequentialCommandGroup(preStart, specificGrabSequence);
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

        ll = new Limelight(bot);
        ll.register();

        initialMovementCommand = new FollowPathCommand(f, f.pathBuilder()
                .addPath(new BezierLine(start, checkID))
                .setLinearHeadingInterpolation(start.getHeading(), checkID.getHeading())
                .build()
        );

        int preStartTag = 21;
        while (!isStarted() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            preStartTag = ll.getTagID();

            if (preStartTag != 21 && preStartTag != 22 && preStartTag != 23) {
                preStartTag = 21;
            }

            telemetry.addData("LL Status", ll.isTargetVisible() ? "TARGET VISIBLE" : "NO TARGET");
            telemetry.addData("Detection Pre-Start (For Reference)", preStartTag);
            telemetry.addData("Auto Path", "Initial Path: Start -> Detection Zone");
            telemetry.addData("Waiting for", "OpMode to Start...");
            telemetry.update();

            sleep(20);
        }


        waitForStart();

        f.setMaxPower(10.0 / vs.getVoltage());
        initialMovementCommand.schedule();

        while (initialMovementCommand.isScheduled() && opModeIsActive()) {
            CommandScheduler.getInstance().run();
            f.update();
            telem.addData("current pose", f.getPose());
            telem.addData("Follower Status", "Running Initial Path");
            telem.update();
        }

        CommandScheduler.getInstance().run();
        int finalDetectedTag = ll.getTagID();

        if (finalDetectedTag != 21 && finalDetectedTag != 22 && finalDetectedTag != 23) {
            finalDetectedTag = 21;
        }

        SequentialCommandGroup finalAuto = getFinalPath(finalDetectedTag, f);
        CommandScheduler.getInstance().schedule(finalAuto);

        telem.addData("Selected Auto Path", finalDetectedTag);


        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            f.update();
            telem.addData("current pose", f.getPose());

            telem.addData("Follower Status", f.isBusy() ? "Running Final Path" : "Finished");
            telem.update();
        }
        CommandScheduler.getInstance().reset();
    }
}