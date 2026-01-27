package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.RotationOnlyAutoAlignCommand;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;

import java.io.File;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class teleop extends CommandOpMode {
    private Bot bot;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    private MecanumDrive drive;
    //public static Limelight limelight;
    private Pivot pivot;
    private Shooter shooter,shooter2;
    private Intake intake;
    //private Indexer indexer;
    private Stopper stopper;
    private Follower follower;
    public boolean redTeam = false;
    private boolean smartShooting = false;
    private final double  VELOCITY_TOLERANCE = 40;
    public static Pose closeBlue = new Pose(60, 90, Math.toRadians(-45));
    public static Pose closeRed = new Pose(84, 90, Math.toRadians(-135));
    public static Pose farRed = new Pose(81, 20.5, Math.toRadians(-113));
    public static Pose farBlue = new Pose(55, 16, Math.toRadians(-67));

    public void initialize() {

        CommandScheduler.getInstance().reset();
        FtcDashboard dashboard = FtcDashboard.getInstance();


        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        // drive region

        bot = new Bot(telem, hardwareMap, driverGamepad, operatorGamepad);

        follower = Constants.createFollower(hardwareMap);

        //limelight = new Limelight(bot);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    limelight.register();

        drive = new MecanumDrive(bot);
        drive.register();

        File gyr = AppUtil.getInstance().getSettingsFile("gyropending.txt");
        String pending = "";
        try {
            pending = ReadWriteFile.readFile(gyr);
        } catch (Exception e) {
        }

        if (pending.trim().equals("1")) {
            ReadWriteFile.writeFile(gyr, "0");
            File teamFile = AppUtil.getInstance().getSettingsFile("team.txt");
            String team = "";
            try {
                team = ReadWriteFile.readFile(teamFile).trim();
            } catch (Exception e) {}

            /*double currentHeading = MecanumDrive.odo.getPosition().getHeading(AngleUnit.DEGREES);
            double newHeading = currentHeading;


            Pose2D currentPos = MecanumDrive.odo.getPosition();
            MecanumDrive.odo.setPosition(new Pose2D(DistanceUnit.MM, currentPos.getX(DistanceUnit.MM), currentPos.getY(DistanceUnit.MM), AngleUnit.DEGREES, newHeading));
            MecanumDrive.odo.update();*/
        }

        shooter = new Shooter(hardwareMap,telemetry);
        shooter.register();
        shooter2 = new Shooter(hardwareMap,telemetry);
        shooter2.register();

        intake = new Intake(bot);
        intake.register();

        pivot = new Pivot(bot);
        pivot.register();

        stopper = new Stopper(bot);
        stopper.register();


        register(drive);

        //chasis default command
        drive.setDefaultCommand(new RunCommand(
                () -> drive.drive(
                        -driverGamepad.getLeftY() * bot.speed,
                        -driverGamepad.getLeftX() * bot.speed,
                        -driverGamepad.getRightX() * 0.8
                ),
                drive
        ));

        //shooter command
        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(()->shooter.setVelocity(1500),shooter));
        new GamepadButton(driverGamepad, GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(()->shooter.setVelocity(1190),shooter));
        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(()->shooter2.setVelocity2(1500),shooter2));
        new GamepadButton(driverGamepad, GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(()->shooter2.setVelocity2(1190),shooter2));

        //intake and indexer command
        new Trigger(() -> (driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1))
                .whenActive(new ConditionalCommand(
                        new RunCommand(() -> {
                            double currentVel = shooter.shooter.getVelocity();
                            if (Math.abs(currentVel - 1500) < VELOCITY_TOLERANCE) {
                                intake.setPower(1);
                            } else {
                                intake.setPower(0);
                            }
                        }, intake),
                        new RunCommand(() ->{
                            intake.setPower(1);
                        },intake),
                        ()->smartShooting
                ));
        new Trigger(() -> (driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.1))
                .whenActive(new InstantCommand(()->intake.setPower(0),intake));
        new Trigger(() -> (driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1))
                .whenActive(new ConditionalCommand(
                        new RunCommand(() -> {
                            double currentVel = shooter.shooter.getVelocity();
                            if (Math.abs(currentVel - 1190) < VELOCITY_TOLERANCE) {
                                intake.setPower(1);
                            } else {
                                intake.setPower(0);
                            }
                        }, intake),
                        new RunCommand(() ->{
                            intake.setPower(1);
                        },intake),
                        ()->smartShooting
                ));
        new Trigger(() -> (driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1))
                .whenActive(new InstantCommand(()->intake.setPower(0),intake));

        //rotation auto-align command
        /*new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new ConditionalCommand(
                        new RotationOnlyAutoAlignCommand(bot, Math.toRadians(-50)),  // If true (Red)
                        new RotationOnlyAutoAlignCommand(bot, Math.toRadians(-130)), // If false (Blue)
                        () -> redTeam
                ));
        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new ConditionalCommand(
                        new RotationOnlyAutoAlignCommand(bot, Math.toRadians(-24.5)),  // If true (Red)
                        new RotationOnlyAutoAlignCommand(bot, Math.toRadians(-154.5)), // If false (Blue)
                        () -> redTeam
                ));*/
        //pivot toggle
        new GamepadButton(driverGamepad, GamepadKeys.Button.B)
                .toggleWhenPressed(new InstantCommand(()->pivot.setPosition(.31), pivot), new InstantCommand(()->pivot.close(), pivot));

        //stopper toggle
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
                .toggleWhenPressed(new InstantCommand(()->stopper.setPosition(1),stopper),new InstantCommand(()->stopper.setPosition(0.75),stopper));

        //position move toggle
        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(new ConditionalCommand(
                        new RunCommand(() -> {
                            Pose currentPose = follower.getPose();
                            FollowPathCommand toScore = new FollowPathCommand(follower, follower.pathBuilder()
                                    .addPath(new BezierLine(currentPose, farRed))
                                    .setLinearHeadingInterpolation(currentPose.getHeading(), farRed.getHeading())
                                    .build()
                            );
                            toScore.addRequirements(drive);
                            toScore.schedule();
                        }),
                        new RunCommand(() -> {
                            Pose currentPose = follower.getPose();
                            FollowPathCommand toScore = new FollowPathCommand(follower, follower.pathBuilder()
                                    .addPath(new BezierLine(currentPose, closeBlue))
                                    .setLinearHeadingInterpolation(currentPose.getHeading(), closeBlue.getHeading())
                                    .build()
                            );
                            toScore.addRequirements(drive);
                            toScore.schedule();
                        }),
                        ()->redTeam
                ));
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(new ConditionalCommand(
                        new RunCommand(() -> {
                            Pose currentPose = follower.getPose();
                            FollowPathCommand toScore = new FollowPathCommand(follower, follower.pathBuilder()
                                    .addPath(new BezierLine(currentPose, closeRed))
                                    .setLinearHeadingInterpolation(currentPose.getHeading(), closeRed.getHeading())
                                    .build()
                            );
                            toScore.addRequirements(drive);
                            toScore.schedule();
                        }),
                        new RunCommand(() -> {
                            Pose currentPose = follower.getPose();
                            FollowPathCommand toScore = new FollowPathCommand(follower, follower.pathBuilder()
                                    .addPath(new BezierLine(currentPose, farBlue))
                                    .setLinearHeadingInterpolation(currentPose.getHeading(), farBlue.getHeading())
                                    .build()
                            );
                            toScore.addRequirements(drive);
                            toScore.schedule();
                        }),
                        ()->redTeam
                ));

        //reset IMU command
        new GamepadButton(driverGamepad, GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> {
                    MecanumDrive.odo.recalibrateIMU();
                }));

        //switch teams toggle
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> {
                    redTeam = !redTeam;
                }));

        //smart shooting toggle
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(() -> {
                    smartShooting = !smartShooting;
                }));

        //intake + indexer failsafe
        new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new ParallelCommandGroup(
                        new RunCommand(()->intake.setPower(-1))
                        //new RunCommand(()->indexer.setPower(-1))
                ));
        /*
        while (opModeInInit()){
            telem.update();
        }
        //endregion
        */
    }
    @Override
    public void run() {
        //periodicBindings();
        follower.update();
        CommandScheduler.getInstance().run();
        bot.telem.addData("CurrentOdoAngle", MecanumDrive.odo.getPosition().getHeading(AngleUnit.DEGREES));
        bot.telem.addData("Velocity",shooter.shooter.getVelocity());
        bot.telem.addData("power",shooter.shooter.getPower());
        bot.telem.addData("x",follower.getPose().getX());
        bot.telem.addData("y",follower.getPose().getY());

        telem.update();
    }
}