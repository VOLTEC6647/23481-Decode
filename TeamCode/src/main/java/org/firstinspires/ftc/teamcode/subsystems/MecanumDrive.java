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

@Config
public class MecanumDrive extends SubsystemBase {
    private final Bot bot;

    private IMU imu = null;

    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public static GoBildaPinpointDriver odo;
    public static boolean fieldCentric = true;
    public static Pose pose;
    public static Pose2D pose2D;

    private boolean isEncoderMode = false;

    private final Follower follower;
    public static boolean isTargetLocked = false;


    public Rotation2d getRobotOrientation() {
        return new Rotation2d(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    public MecanumDrive(Bot bot, Follower follower) {
        this.bot = bot;
        this.follower = follower;

        odo = bot.hMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-82.66924000028, 110.830759999962, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        if (pose == null) {
            pose2D = new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.RADIANS, 0);
        }

        odo.setPosition(pose2D);

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

        Pose position = new Pose(odo.getEncoderX(), odo.getEncoderY(), odo.getHeading(AngleUnit.RADIANS));
        odo.update();
        pose = position;

        bot.telem.addData("EncoderMode",isEncoderMode);
        bot.telem.addData("FieldCentric",fieldCentric);
        bot.telem.addData("TargetLocked", isTargetLocked);
        bot.telem.addData("TargetCorrection", Limelight.targetCorrectionRotation);
        bot.telem.addData("Heading", bot.getYaw());
        bot.telem.update();
    }


    public void drive(double xPower, double yPower, double rxInput) {

        double rotationPower;
        if (isTargetLocked) {
            rotationPower = Limelight.targetCorrectionRotation;
        } else {
            rotationPower = rxInput * bot.rotMultiplier;
        }
        double x = -xPower;
        double botHeading = pose.getHeading();

        double rotX = x * Math.cos(-botHeading) - yPower * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + yPower * Math.cos(-botHeading);

        rotX *= 1.1; // counteract imperfect strafe

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotationPower), 1);
        double frontLeftPower = (rotY + rotX + rotationPower) / denominator;
        double backLeftPower = (rotY - rotX + rotationPower) / denominator;
        double frontRightPower = (rotY - rotX - rotationPower) / denominator;
        double backRightPower = (rotY + rotX - rotationPower) / denominator;

        double[] powers = {frontLeftPower, frontRightPower, backLeftPower, backRightPower};
        double[] normalizedPowers = normalizeWheelSpeeds(powers);

        frontLeft.setPower(normalizedPowers[0]);
        frontRight.setPower(normalizedPowers[1]);
        backLeft.setPower(normalizedPowers[2]);
        backRight.setPower(normalizedPowers[3]);
    }

    public void resetEncoders() {
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }


    private double[] normalizeWheelSpeeds(double[] speeds) {
        if (largestAbsolute(speeds) > 1) {
            double max = largestAbsolute(speeds);
            for (int i = 0; i < speeds.length; i++){
                speeds[i] /= max;
            }
        }
        return speeds;
    }

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