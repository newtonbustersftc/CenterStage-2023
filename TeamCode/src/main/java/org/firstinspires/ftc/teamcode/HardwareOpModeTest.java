package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.File;

@TeleOp(name="Hardware Test", group="Test")
//@Disabled
public class HardwareOpModeTest extends OpMode {
    RobotHardware robotHardware;
    RobotProfile robotProfile;

    Pose2d currPose;
    double grabberPos = 0.5;

    boolean grabberChange =  false;
    boolean blocking = false;
    boolean turretTurnTest = false;
    RobotControl currentTask = null;

    @Override
    public void init() {
        try{
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profileA.json"));
        } catch (Exception e) {
            Logger.logFile("Exception " + e);
            e.printStackTrace();
        }
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);
        String startPosStr = prefs.getString("starting position", "none");
        Logger.init();
        Logger.logFile("---- Profile ----\n" + robotProfile + "--------\n");
        Logger.logFile("StartPos:" + startPosStr);


        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap, robotProfile);
        robotHardware.grabberOpen();
        robotHardware.resetImu();
        robotHardware.enableManualCaching(true);

        try {
            Thread.sleep(100);
        }
        catch (Exception e) {
        }

        //robotVision.startWebcam("Webcam", null);
    }

    @Override
    public void loop() {
        robotHardware.clearBulkCache();
        robotHardware.getLocalizer().update();
        currPose = robotHardware.getLocalizer().getPoseEstimate();
        testHardware();

        telemetry.addData("Heading", Math.toDegrees(robotHardware.getGyroHeading()));
        telemetry.addData("Grabber", grabberPos);
        telemetry.addLine().addData("FL", robotHardware.flMotor.getCurrentPosition())
                .addData("RL - right ", robotHardware.rlMotor.getCurrentPosition())
                .addData("RR -  left ", robotHardware.rrMotor.getCurrentPosition())
                .addData("FR - center", robotHardware.frMotor.getCurrentPosition())
                .addData("Lift0", robotHardware.getLiftMotors()[0].getCurrentPosition())
                .addData("Lift1", robotHardware.getLiftMotors()[1].getCurrentPosition())
                .addData("Intake", robotHardware.intakeMotor.getCurrentPosition())
                .addData("Distance Sensor Left = ", robotHardware.distanceSensorLeft.getDistance(DistanceUnit.MM))
                .addData("Distane Sensor Right=", robotHardware.distanceSensorRight.getDistance(DistanceUnit.MM));
        telemetry.update();

        if (gamepad1.x) { // Square
            robotHardware.grabberClose();
        } else if (gamepad1.y) { // Triangle
            robotHardware.grabberOpen();
        }

        //if (gamepad1.left_bumper) {
        //    robotHardware.startIntake();
        //}
        //else if (gamepad1.right_bumper) {
        //    robotHardware.reverseIntake();
        //}
        //else {
        //    robotHardware.stopIntake();
        //}

        if (gamepad1.dpad_down) {
//            robotHardware.setMotorPower(0.5,0,0,0);
            double distance_left = robotHardware.distanceSensorLeft.getDistance(DistanceUnit.MM);
            double distance_right = robotHardware.distanceSensorRight.getDistance(DistanceUnit.MM);
            telemetry.addData("distance left=", distance_left);
            telemetry.addData("distance right=", distance_right);
            Logger.logFile("distance left="+distance_left);
            Logger.logFile("distance right="+distance_right);
        }

/*        if(gamepad1.dpad_up){
              robotHardware.setMotorPower(0,0.5,0,0);
        }

        if(gamepad1.dpad_left){
              robotHardware.setMotorPower(0,0,0.5,0);
        }

        if(gamepad1.dpad_right){
              robotHardware.setMotorPower(0,0,0,0.5);
        }
 */

        if (!grabberChange) {
            if (gamepad1.right_bumper) {
                grabberPos += 0.01;
                robotHardware.gripperServo.setPosition(grabberPos);
            }
            if (gamepad1.left_bumper) {
                grabberPos -= 0.01;
                robotHardware.gripperServo.setPosition(grabberPos);
            }
        }
        grabberChange = (gamepad1.left_bumper || gamepad1.right_bumper);

//        robotHardware.turnOnLight(gamepad1.left_trigger>0.3);

//        if (robotHardware.isLiftTouched()) {robotHardware.resetLiftPos();}

        //if (robotHardware.isMagneticTouched()) {robotHardware.resetTurretPos();}

    }

    @Override
    public void stop() {
        // open the clamp to relief the grabber servo
        try {
            //robotVision.stopWebcam("Webcam");
            robotHardware.stopAll();
            Logger.logFile("DriverOpMode Test stop() called");
            Logger.flushToFile();
        } catch (Exception e) {
            Log.e("DriverOpMode", Log.getStackTraceString(e));
        }
    }

    private void testHardware() {
    }
}