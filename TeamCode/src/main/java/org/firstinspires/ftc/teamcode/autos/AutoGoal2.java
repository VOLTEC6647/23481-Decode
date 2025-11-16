package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;


@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutoGoal2 extends LinearOpMode {

    // Scoring Poses
    public static Pose score = new Pose(55, 85, Math.toRadians(315));
    public static Pose start = new Pose(20, 120, Math.toRadians(325));
    public static Pose preGrab2  = new Pose(60, 60, Math.toRadians(180));
    public static Pose grab2  = new Pose(33.5, 60, Math.toRadians(180));
    private Bot bot;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    private Intake intake;



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


        SequentialCommandGroup auto =
                new SequentialCommandGroup(
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(new BezierLine(start, score))
                                .setLinearHeadingInterpolation(start.getHeading(), score.getHeading())
                                .build()
                        ),
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
                                new RunCommand(()->intake.setPower(1))
                        ),
                        new InstantCommand(()->intake.setPower(0)),
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(new BezierLine(grab2, score))
                                .setLinearHeadingInterpolation(grab2.getHeading(),score.getHeading())
                                .build()
                        )
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
