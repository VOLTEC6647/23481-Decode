package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.teleop.teleop;

@Config
public class MecanumDrive extends SubsystemBase {
    private final Bot bot;
    //private final Limelight limelight;
    private IMU imu = null;

    public final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public static GoBildaPinpointDriver odo;
    public static Pose pose = new Pose(0, 0, 0);

    private boolean isEncoderMode = false;
    public static boolean isTargetLocked = false;


    public Rotation2d getRobotOrientation() {
        return new Rotation2d(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    public static MecanumDrive instance;
    public static MecanumDrive getInstance(){

        return instance;
    }

    public MecanumDrive(Bot bot) {
        this.bot = bot;
        instance = this;
        //this.limelight = teleop.limelight;

        odo = bot.hMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-82.66924000028, 110.830759999962, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        frontLeft = bot.hMap.get(DcMotorEx.class, "M0");
        frontRight = bot.hMap.get(DcMotorEx.class, "M1");
        backLeft = bot.hMap.get(DcMotorEx.class, "M2");
        backRight = bot.hMap.get(DcMotorEx.class, "M3");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {
        odo.update();

        Pose2D currentPos = odo.getPosition();
        pose = new Pose(
                currentPos.getX(DistanceUnit.INCH),
                currentPos.getY(DistanceUnit.INCH),
                currentPos.getHeading(AngleUnit.RADIANS)
        );

        bot.telem.addData("TargetLocked", isTargetLocked);
        bot.telem.addData("FieldCentric Heading", Math.toDegrees(pose.getHeading()));
        bot.telem.addData("X (Inches)", pose.getX());
        bot.telem.addData("Y (Inches)", pose.getY());
        bot.telem.update();
    }


    public void drive(double xPower, double yPower, double rxInput) {

        double rotationPower = -rxInput * bot.rotMultiplier;
        double x = -xPower;
        double botHeading = pose.getHeading();

        double rotX = x * Math.cos(-botHeading) - yPower * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + yPower * Math.cos(-botHeading);

        rotX *= 1.1; // counteract imperfect strafe

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotationPower), 1);
        double fl = (rotY + rotX + rotationPower) / denominator;
        double bl = (rotY - rotX + rotationPower) / denominator;
        double fr = (rotY - rotX - rotationPower) / denominator;
        double br = (rotY + rotX - rotationPower) / denominator;

        setRawMotorPowers(fl,fr,bl,br);
    }

    /*public void resetEncoders() {
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }*/

    public void setRawMotorPowers(double fl, double fr, double bl, double br) {
        double max = Math.abs(fl);
        max = Math.max(max, Math.abs(fr));
        max = Math.max(max, Math.abs(bl));
        max = Math.max(max, Math.abs(br));

        if (max > 1.0) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    /*private double[] normalizeWheelSpeeds(double[] speeds) {
        if (largestAbsolute(speeds) > 1) {
            double max = largestAbsolute(speeds);
            for (int i = 0; i < speeds.length; i++){
                speeds[i] /= max;
            }
        }
        return speeds;
    }*/

    private double largestAbsolute(double[] arr) {
        double largestAbsolute = 0;
        for (double d : arr) {
            double absoluteValue = Math.abs(d);
            if (absoluteValue > largestAbsolute) {
                largestAbsolute = absoluteValue;
            }
        }
        return largestAbsolute;
    }
}