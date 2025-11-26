package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MotorTest;
import org.firstinspires.ftc.teamcode.subsystems.NewElevator;
import org.firstinspires.ftc.teamcode.subsystems.ServoTest;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPivot;

import java.io.File;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class teleop extends CommandOpMode {
    private Bot bot;
    private Follower follower;
    private MultipleTelemetry telem;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
    private MecanumDrive drive;
    private ServoTest sTest;
    public static Limelight limelight;
    private ShooterPivot pivot;
    //private Turret turret;
    private Shooter shooter;
    private Intake intake;
    private Indexer indexer;
    private NewElevator elevator;
    /*private DiffClaw dClaw;
    private DiffClawUp diffClawUp;
    private ClawUp clawUp;
    private Claw claw;
    private ClawPivot clawPivot;
    private Arm arm;*/


    public void initialize() {

        CommandScheduler.getInstance().reset();
        FtcDashboard dashboard = FtcDashboard.getInstance();


        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        // drive region

        bot = new Bot(telem, hardwareMap, driverGamepad, operatorGamepad);

        File myFileName = AppUtil.getInstance().getSettingsFile("team.txt");
        String team = ReadWriteFile.readFile(myFileName);


        /*if (team.equals("blue")){
            bot.setRotationOffset(Rotation2d.fromDegrees(0));
        }
        if (team.equals("red")){
            bot.setRotationOffset(Rotation2d.fromDegrees(0));
        }*/

        //sTest = new ServoTest(bot);
        //sTest.register();

        limelight = new Limelight(bot);
        limelight.register();

        drive = new MecanumDrive(bot);
        drive.register();

        //shooter = new Shooter(bot);
        //shooter.register();

        intake = new Intake(bot);
        intake.register();

        indexer = new Indexer(bot);
        indexer.register();

        //pivot = new ShooterPivot(bot);
        //pivot.register();

        //elevator = new NewElevator(bot);
        //elevator.register();
        //elevator.goToLow();

        //turret = new Turret(bot);
        //turret.register();


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
        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenPressed(()-> { MecanumDrive.isTargetLocked = !MecanumDrive.isTargetLocked; });

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
        new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new RunCommand(() -> intake.setPower(1), intake))
                .whenReleased(new InstantCommand(() -> intake.setPower(0), intake));

        //shooter command
        new Trigger(()-> operatorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.1)
                .whenActive(new InstantCommand(()-> shooter.shootOn()))
                .whenInactive(new InstantCommand(()-> shooter.shootOff()));

        //indexer command
        new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new RunCommand(()->indexer.indexOn(), indexer))
                .whenReleased(new InstantCommand(()->indexer.indexOff(), indexer));

        //elevator command
        new Trigger(()-> driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.1)
                .whenActive(new InstantCommand(()-> elevator.goToHigh(), elevator));
        new Trigger(()-> driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)<0.1)
                .whenActive(new InstantCommand(()-> elevator.goToLow(), elevator));

        //pivot command
        new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
                .toggleWhenPressed(new InstantCommand(()->pivot.one(), pivot), new InstantCommand(()->pivot.zero(), pivot));

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