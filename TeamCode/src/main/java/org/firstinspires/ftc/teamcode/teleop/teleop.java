package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MotorTest;
import org.firstinspires.ftc.teamcode.subsystems.ScissorElevator;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPivot;

import java.io.File;
import java.util.function.DoubleSupplier;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class teleop extends CommandOpMode {
    private Bot bot;
    private Follower follower;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    private MecanumDrive drive;
    private MotorTest test;
    private Limelight limelight;
    private ShooterPivot pivot;
    //private Turret turret;
    private Shooter shooter;
    private Intake intake;
    private ScissorElevator elevator;
    /*private DiffClaw dClaw;
    private DiffClawUp diffClawUp;
    private ClawUp clawUp;
    private Claw claw;
    private ClawPivot clawPivot;
    private Arm arm;*/
    /*private final Pose start = new Pose(33, 9.5, Math.toRadians(0));
    private final Pose blue = new Pose(33, 125, Math.toRadians(0));
    private final Pose red = new Pose(110, 125, Math.toRadians(0));
    private final Pose white  = new Pose(110, 9.5, Math.toRadians(0));*/


    public void initialize() {

        CommandScheduler.getInstance().reset();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        /*follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);
        follower.update();*/


        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        // drive region

        bot = new Bot(telem, hardwareMap, driverGamepad, operatorGamepad);
        //bot.getImu().resetYaw();

        File myFileName = AppUtil.getInstance().getSettingsFile("team.txt");
        String team = ReadWriteFile.readFile(myFileName);


        /*if (team.equals("blue")){
            bot.setRotationOffset(Rotation2d.fromDegrees(0));
        }
        if (team.equals("red")){
            bot.setRotationOffset(Rotation2d.fromDegrees(0));
        }*/

        //test = new MotorTest(bot);
        //test.register();

        drive = new MecanumDrive(bot,follower);
        drive.register();

        //limelight = new Limelight(bot);
        //limelight.register();

        //shooter = new Shooter(bot);
        //shooter.register();

        intake = new Intake(bot);
        intake.register();

        //pivot = new ShooterPivot(bot);
        //pivot.register();

        /*elevator = new ScissorElevator(bot);
        elevator.register();
        elevator.goToLow();*/

        //turret = new Turret(bot);
        //turret.register();

        /*new Trigger(()-> operatorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.1)
                .whenActive(()-> elevator.goToHigh())
                .whenInactive(()-> elevator.goToLow());*/


        register(drive);

        //chasis default command
        drive.setDefaultCommand(new RunCommand(
                () -> drive.drive(
                        driverGamepad.getLeftX() * bot.speed,
                        -driverGamepad.getLeftY() * bot.speed,
                        -driverGamepad.getRightX() * 0.8
                ),
                drive
        ));

        //chassis target-locked command
        /*new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenPressed(()-> {
                    MecanumDrive.isTargetLocked = !MecanumDrive.isTargetLocked;
                });*/

        //turret default command
        /*turret.setDefaultCommand(new RunCommand(
                () -> turret.setTurretPower(-operatorGamepad.getLeftX() * Turret.MAX_TURRET_SPEED * 0.5),
                turret
        ));*/

        //turret auto-align
        /*new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
                .whileHeld(new TurretAutoAlignCommand(turret, limelight));*/

        //reset turret position
        /*new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(turret::resetEncoder, turret));*/

        //intake command
        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new RunCommand(() -> intake.setPower(1.0), intake))
                .whenReleased(new InstantCommand(() -> intake.setPower(0), intake));

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
        CommandScheduler.getInstance().run();
        telem.update();
    }

}