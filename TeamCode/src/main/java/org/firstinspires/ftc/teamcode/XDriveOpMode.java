package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;

@TeleOp(name="XDrive Test", group="Test")
//@Disabled
public class XDriveOpMode extends OpMode {
    RobotHardware robotHardware;
    RobotProfile robotProfile;
    double targetHeading;
    double headingOffset;

    @Override
    public void init() {
        try{
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
            Logger.logFile("Exception " + e);
            e.printStackTrace();
        }
        Logger.init();
        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap, robotProfile);
        targetHeading = 0;
        headingOffset = robotHardware.getGyroHeading();
    }

    @Override
    public void loop() {
        robotHardware.clearBulkCache();
        handleMovement();
    }

    @Override
    public void stop() {
        // open the clamp to relief the grabber servo
        try {
            //robotVision.stopWebcam("Webcam");
            robotHardware.stopAll();
            Logger.logFile("XDriveOpMode stop() called");
            Logger.flushToFile();
        } catch (Exception e) {
            Log.e("XDriveOpMode", Log.getStackTraceString(e));
        }

    }

    private void handleMovement() {
        double turn = gamepad1.right_stick_x/4; // negative when point left
        double power = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double padAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) + Math.PI/2;
        double currHeading = robotHardware.getGyroHeading();
        double pidP = 2;
        double corr = currHeading * pidP * Math.max(power, 0.2);
        if (corr>0.5) corr=0.5; else if (corr<-0.5) corr=-0.5;

        double movAngle;
        movAngle = padAngle;
        if (gamepad1.left_bumper) {
            power = power/3;
            turn = turn/3;
        }
        robotHardware.mecanumDrive2(power, movAngle, corr + turn);
        telemetry.addData("Move",  "a:" + Math.toDegrees(padAngle));
        telemetry.addData("Stick", "x:" + gamepad1.left_stick_x + " y:" + gamepad1.left_stick_y);
        telemetry.addData("Turn:", turn);
        telemetry.addData("IMU:", Math.toDegrees(currHeading));
    }


}