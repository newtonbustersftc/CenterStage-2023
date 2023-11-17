package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Logger;
import org.firstinspires.ftc.teamcode.RobotFactory;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotProfile;
import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.io.File;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        NBMecanumDrive drive;
        RobotProfile robotProfile = null;
        try{
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profileA.json"));
        } catch (Exception e) {
        }

        Logger.init();
        RobotHardware robotHardware = RobotFactory.getRobotHardware(hardwareMap, robotProfile);
        drive = (NBMecanumDrive)robotHardware.getMecanumDrive();
        robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));

        double imuOffset = drive.getRawExternalHeading();

        waitForStart();
        telemetry.clearAll();
        while (!isStopRequested()) {
            if (gamepad1.a) {
                while (gamepad1.a) {
                    idle();
                }
                drive.turn(Math.toRadians(ANGLE));
            }
            if (gamepad1.b) {
                while (gamepad1.a) {
                    idle();
                }
                drive.turn(Math.toRadians(-ANGLE));
            }
            telemetry.addLine("Use A/B to turn left or right");
            telemetry.addData("Pose2d:", drive.getPoseEstimate());
            telemetry.addData("IMU:", Math.toDegrees(drive.getRawExternalHeading() - imuOffset));
            telemetry.update();
        }
    }
}
