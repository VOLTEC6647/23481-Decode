package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.commands.RotationOnlyAutoAlignCommand;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.NewElevator;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

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
    private Shooter shooter;
    private Intake intake;
    private Indexer indexer;
    private NewElevator elevator;
    public boolean redTeam = false;
    //private Turret turret;
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



        //limelight = new Limelight(bot);
        //fff                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                limelight.register();

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

            double currentHeading = MecanumDrive.odo.getPosition().getHeading(AngleUnit.DEGREES);
            double newHeading = currentHeading;


            Pose2D currentPos = MecanumDrive.odo.getPosition();
            MecanumDrive.odo.setPosition(new Pose2D(DistanceUnit.MM, currentPos.getX(DistanceUnit.MM), currentPos.getY(DistanceUnit.MM), AngleUnit.DEGREES, newHeading));
            MecanumDrive.odo.update();
        }

        shooter = new Shooter(hardwareMap,telemetry);
        shooter.register();

        intake = new Intake(bot);
        intake.register();

        indexer = new Indexer(bot);
        indexer.register();

        pivot = new Pivot(bot);
        pivot.register();

        elevator = new NewElevator(bot);
        elevator.register();

        //turret = new Turret(bot);
        //turret.register();


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

        //chassis target-locked command
        /*new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenPressed(()-> { MecanumDrive.isTargetLocked = !MecanumDrive.isTargetLocked; });*/

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
        new Trigger(()-> driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.1)
                .whenActive(new RunCommand(() -> intake.setPower(1), intake));
        new Trigger(()-> driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)<0.1)
                .whenActive(new InstantCommand(() -> intake.setPower(0), intake));
        new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new RunCommand(()->intake.setPower(-1), intake))
                .whenReleased(new InstantCommand(()->intake.setPower(0), intake));

        //shooter command
        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(()->shooter.setVelocity(1400),shooter));//,new InstantCommand(()->shooter.shootOff(),shooter)
        new GamepadButton(driverGamepad, GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(()->shooter.setVelocity(1190),shooter));
        //indexer command
        new Trigger(()-> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.1)
                .whenActive(new RunCommand(()->indexer.setPower(0.4), indexer));
        new Trigger(()-> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)<0.1)
                .whenActive(new RunCommand(()->indexer.indexOff(), indexer));
        new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new RunCommand(()->indexer.indexOut(), indexer))
                .whenReleased(new InstantCommand(()->indexer.indexOff(), indexer));
        //elevator command
        new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
        .toggleWhenPressed(new InstantCommand(()-> elevator.goToHigh(), elevator), new InstantCommand(()-> elevator.goToLow(), elevator));

        //reset IMU
        new GamepadButton(driverGamepad, GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(()-> MecanumDrive.odo.resetPosAndIMU()));

        //heading lock
        //new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
        //        .whileHeld(new RotationOnlyAutoAlignCommand(bot,follower,Math.toRadians(225)));
        //new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
        //        .whileHeld(new RotationOnlyAutoAlignCommand(bot,follower,Math.toRadians(-225)));

        //switch teams
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> {
                    redTeam = !redTeam;
                }));
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new ConditionalCommand(
                        new RotationOnlyAutoAlignCommand(bot, Math.toRadians(-45)),  // If true (Red)
                        new RotationOnlyAutoAlignCommand(bot, Math.toRadians(45)), // If false (Blue)
                        () -> redTeam
                ));
        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new ConditionalCommand(
                        new RotationOnlyAutoAlignCommand(bot, Math.toRadians(-22)),  // If true (Red)
                        new RotationOnlyAutoAlignCommand(bot, Math.toRadians(22)), // If false (Blue)
                        () -> redTeam
                ));
        //pivot command
        new GamepadButton(driverGamepad, GamepadKeys.Button.B)
                .toggleWhenPressed(new InstantCommand(()->pivot.far(), pivot), new InstantCommand(()->pivot.close(), pivot));

        //hold current position command
        /*new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
                .whileHeld(new PositionHoldCommand(bot, follower),true);*/

        //hold score position command
        /*new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT)
                .whileHeld(new PositionHoldCommand(bot, follower, score),true);*/


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
        bot.telem.addData("CurrentOdoAngle", MecanumDrive.odo.getPosition().getHeading(AngleUnit.DEGREES));

        telem.update();
    }
    public void changeToRed() {
        redTeam = true;
    }
    public void changeToBlue() {
        redTeam = false;
    }
}