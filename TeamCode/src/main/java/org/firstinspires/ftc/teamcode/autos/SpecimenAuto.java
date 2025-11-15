/*package org.firstinspires.ftc.teamcode.autos;
import static org.firstinspires.ftc.teamcode.utils.RobotConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedropathing.Constants.FConstants;
import org.firstinspires.ftc.teamcode.pedropathing.Constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ClawPivot;
import org.firstinspires.ftc.teamcode.subsystems.ClawUp;
import org.firstinspires.ftc.teamcode.subsystems.DiffClaw;
import org.firstinspires.ftc.teamcode.subsystems.DiffClawUp;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Slides;


@Autonomous
public class SpecimenAuto extends LinearOpMode {

    //region Poses
    public static Pose startingPose = new Pose(10, 71, 0);
    public static Pose score1 = new Pose(44.5, 71, 0);
    public static Pose pushIntermediate = new Pose(31, 39, 0);

    private Bot bot;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    private MecanumDrive drive;
    private Slides slides;
    private DiffClaw dClaw;
    private DiffClawUp diffClawUp;
    private ClawUp clawUp;

    private Claw claw;
    private ClawPivot clawPivot;
    private Arm arm;



    @Override
    public void runOpMode() {

        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        Follower f = new Follower(hardwareMap, FConstants.class, LConstants.class);

        bot = new Bot(telem, hardwareMap, driverGamepad, operatorGamepad);


        f.setPose(startingPose);
        f.setMaxPower(0.75);

        slides = new Slides(bot);
        slides.register();

        dClaw = new DiffClaw(bot);
        dClaw.register();

        claw = new Claw(bot);
        claw.register();

        clawPivot = new ClawPivot(bot);
        clawPivot.register();

        arm = new Arm(bot);
        arm.register();

        diffClawUp = new DiffClawUp(bot);
        diffClawUp.register();

        clawUp = new ClawUp(bot);
        clawUp.register();


        //endregion

        SequentialCommandGroup auto = new SequentialCommandGroup(
                new SequentialCommandGroup(
                        new InstantCommand(()-> clawUp.setPosition(outakeClose)),
                        new WaitCommand(50),
                        new InstantCommand(()-> arm.setPosition(armScore)),
                        new WaitCommand(500),
                        new InstantCommand(()-> diffClawUp.setPositionD(placeDiff-(90*outTakeRotatePerDegree))),
                        new InstantCommand(()-> diffClawUp.setPositionI(placeDiff+(90*outTakeRotatePerDegree)))

                ),
                //region Intermediate
                new ParallelCommandGroup(
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Point(startingPose),
                                                new Point(score1)
                                        )
                                )
                                .setLinearHeadingInterpolation(startingPose.getHeading(), score1.getHeading())
                                .build()
                        )

                ),
                new SequentialCommandGroup(
                        new InstantCommand(()-> clawUp.setPosition(outakeOpen)),
                        new InstantCommand(()-> arm.setPosition(armAfterScore))

                ),

                new SequentialCommandGroup(
                        new FollowPathCommand(f, f.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Point(score1),
                                                new Point(pushIntermediate)
                                        )
                                )
                                .setLinearHeadingInterpolation(
                                        score1.getHeading(), pushIntermediate.getHeading())
                                .build()
                        ),

                        new SequentialCommandGroup(
                                new InstantCommand(()-> arm.setPosition(armGrabWall)),
                                new InstantCommand(()-> clawUp.setSetpoint(outakeOpen)),
                                new WaitCommand(50),
                                new InstantCommand(()-> diffClawUp.setPositionD(grabWall-(0*outTakeRotatePerDegree))),
                                new InstantCommand(()-> diffClawUp.setPositionI(grabWall+(0*outTakeRotatePerDegree)))
                        )



                )
        );

        waitForStart();
        CommandScheduler.getInstance().schedule(auto);

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            f.update();
            f.telemetryDebug(telem);
        }
    }
}*/