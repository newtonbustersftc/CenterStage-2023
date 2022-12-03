package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Logger;
import org.firstinspires.ftc.teamcode.RobotFactory;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotProfile;
import org.firstinspires.ftc.teamcode.drive.FourStandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;

import java.io.File;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;
    RobotProfile profile;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        NBMecanumDrive drive;
        RobotProfile robotProfile = null;
        try{
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
        }

        Logger.init();
        RobotHardware robotHardware = RobotFactory.getRobotHardware(hardwareMap, robotProfile);
        robotHardware.resetDriveAndEncoders();
        robotHardware.resetImu();
        drive = (NBMecanumDrive)robotHardware.getMecanumDrive();
        drive.setPoseEstimate(new Pose2d(0,0,0));

        waitForStart();
        drive.setPoseEstimate(new Pose2d(0,0,0));

        while (!isStopRequested()) {
            robotHardware.clearBulkCache();
            Pose2d baseVel = new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );

            Pose2d vel;
            if (Math.abs(baseVel.getX()) + Math.abs(baseVel.getY()) + Math.abs(baseVel.getHeading()) > 1) {
                // re-normalize the powers according to the weights
                double denom = VX_WEIGHT * Math.abs(baseVel.getX())
                    + VY_WEIGHT * Math.abs(baseVel.getY())
                    + OMEGA_WEIGHT * Math.abs(baseVel.getHeading());
                vel = new Pose2d(
                    VX_WEIGHT * baseVel.getX(),
                    VY_WEIGHT * baseVel.getY(),
                    OMEGA_WEIGHT * baseVel.getHeading()
                ).div(denom);
            } else {
                vel = baseVel;
            }

            drive.setDrivePower(vel);

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            Pose2d velo = drive.getPoseVelocity();
            FourStandardTrackingWheelLocalizer ttl = (FourStandardTrackingWheelLocalizer) drive.getLocalizer();
            telemetry.addData("Right", ttl.rightEncoder.getCurrentPosition());
            telemetry.addData("Left:", ttl.leftEncoder.getCurrentPosition());
            telemetry.addData("Front", ttl.frontEncoder.getCurrentPosition());
            telemetry.addData("Back", ttl.backEncoder.getCurrentPosition());
//            telemetry.addData("Right: ", robotHardware.getEncoderCounts(RobotHardware.EncoderType.RIGHT));
//            telemetry.addData("Left: ", robotHardware.getEncoderCounts(RobotHardware.EncoderType.LEFT));
//            telemetry.addData("Horiz: ", robotHardware.getEncoderCounts(RobotHardware.EncoderType.HORIZONTAL));
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("velo", velo);
            telemetry.update();
        }
    }
}
