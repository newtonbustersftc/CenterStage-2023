package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.io.File;

import static org.firstinspires.ftc.teamcode.AutonomousOptions.START_POS_MODES_PREF;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name="Newton DriverOpMode", group="Main")
public class DriverOpMode extends OpMode {
    RobotHardware robotHardware;
    RobotProfile robotProfile;

    Pose2d currPose;
    double fieldHeadingOffset;

    boolean fieldMode;
    int fieldModeSign = -1;  // RED side = 1, BLUE side = -1
    boolean isRedTeam;
    boolean dpadRightPressed = false;
    boolean dpadLeftPressed = false;
    boolean leftBumperPressed = false;
    boolean rightBumperPressed = false;
    boolean aPressed = false;
    boolean yPressed = false;
    boolean xPressed = false;
    boolean dpadUpPressed = false;
    boolean dpadDownPressed = false;
    boolean leftTriggerPressed = false;
    boolean rightTriggerPressed = false;
    double imuAngleOffset = 0;
    double driveAngle = 0;
    boolean amCorrecting = true;
    int openLiftPos = 1;
    int closedLiftPos = 1;
    boolean liftCanChange = true;
    boolean gripperOpen = true;
    boolean gripperCanChange = true;

    // DriveThru combos
    SequentialComboTask intakeAndLift, deliverTask,  sharedHubTask;
    RobotControl currentTask = null;
    String startPosStr ;

    @Override
    public void init() {
        try {
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        }
        catch (Exception e) {
            System.out.println(e.getStackTrace());
        }
        fieldMode = true; //robot starts in field orientation

        Logger.init();
        //Obtain the RobotHardware object from factory
        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap, robotProfile);
        robotHardware.resetLiftPos();
        //robotHardware.initRobotVision();
        //robotVision = robotHardware.getRobotVision();
        //robotVision.activateNavigationTarget();
        //robotHardware.initLeds();   // need to init everytime
       // robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));
        //ensure lift is reset at the beginning and the end
        // Based on the Autonomous mode starting position, define the heading offset for field mode
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);
        if (prefs.getString(START_POS_MODES_PREF, "NONE").startsWith("RED")) {
            fieldModeSign = 1;
            isRedTeam = true;
        }
        else {
            fieldModeSign = -1;
        }
        if (prefs.getString(START_POS_MODES_PREF, "NONE").contains("DUCK")) {
            imuAngleOffset = Math.PI;
        }
        Logger.logFile("DriverOpMode: " + prefs.getString(START_POS_MODES_PREF, "NONE"));
        Logger.logFile("IMU Offset is " + Math.toDegrees(imuAngleOffset));
        Logger.logFile("Current IMU Angle " + Math.toDegrees(robotHardware.getImuHeading()));

        //robotHardware.getRobotVision().initRearCamera(isRedTeam);

        startPosStr = prefs.getString(START_POS_MODES_PREF, "NONE").contains("BLUE") ? "BLUE" : "RED";
        setupCombos();

    }

    /**
     * Main loop for the code
     */
    @Override
    public void loop() {
        //Read values from the control hub
        robotHardware.clearBulkCache();

        //currPose = new Pose2d(0,0,0);   // for now
        //Handling autonomous task loop
        if (currentTask != null) {
            //robotHardware.setLed1(true);
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                currentTask.cleanUp();
                currentTask = null;
            }
            else {
                currentTask.execute();
                if (currentTask.isDone()) {
                    currentTask.cleanUp();
                    Logger.logFile("TaskComplete: " + currentTask);
                    currentTask = null;
                }
            }
        }
        else {
            //robotHardware.setLed1(false);
        }

        //handleMovement();
        handleExtension();
        handleGripper();
        handleLift();
        handleTurret();

        telemetry.addData("Heading", Math.toDegrees(robotHardware.getImuHeading()));


    }

    @Override
    public void stop() {
        robotHardware.stopAll();
        try {
            Logger.logFile("DriverOpMode stop() called");
            //robotVision.deactivateNavigationTarget();
            Logger.flushToFile();
        }
        catch (Exception e) {
            System.out.println(e.getStackTrace());
        }
    }

    /**
     * Joystick Driving Controls
     * Left bumper for slow motion
     * Left trigger to enable field mode, right trigger to enable robot-oriented mode
     */
    private void handleMovement() {
        double turn = gamepad1.right_stick_x / 2;
        double power = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double padAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) + Math.PI / 2;
        double currHeading = robotHardware.getImuHeading() + driveAngle;
        double pidP = 2;
        double corr = currHeading * pidP * Math.max(power, 0.2);
        if (corr>0.5) corr=0.5; else if (corr<-0.5) corr=-0.5;

        double movAngle;

        if(gamepad1.right_stick_x > 0.3 || gamepad1.right_stick_x < -0.3){
            corr = 0;
            amCorrecting = false;
        } else if(amCorrecting == false){
            driveAngle = -robotHardware.getImuHeading();
            amCorrecting = true;
        }
        movAngle = padAngle;
        if (gamepad1.left_bumper) {
            power = power/3;
            turn = turn/3;
        }
        robotHardware.mecanumDrive2(power, movAngle + driveAngle, corr + turn);
        telemetry.addData("Move",  "a:" + Math.toDegrees(padAngle));
        telemetry.addData("Stick", "x:" + gamepad1.left_stick_x + " y:" + gamepad1.left_stick_y);
        telemetry.addData("Turn:", turn);
        telemetry.addData("DriveAngle: ", driveAngle);
        telemetry.addData("IMU:", Math.toDegrees(currHeading));

        //robotHardware.setLed2(fieldMode);
        if (fieldMode) {
            movAngle = padAngle + ((isRedTeam) ? Math.PI / 2 : -Math.PI / 2) - robotHardware.getImuHeading()-imuAngleOffset;
        } else {
            movAngle = padAngle;
        }
        if (gamepad1.left_trigger > 0) {
            power = power / 3;
            turn = turn / 3;
        }

        // toggle field mode on/off.
        // Driver 1: dpad down - enable; dpad right - disable
        if (gamepad1.dpad_down) {
            fieldMode = true;
        } else if (gamepad1.dpad_right) {
            fieldMode = false;  //good luck driving
        }
        if(gamepad1.share){
            robotHardware.resetImu();
            imuAngleOffset = 0;
            fieldMode = true;
        }

    }

    public void handleTurret() {
        if (gamepad2.left_stick_x < -.3) {
            robotHardware.turretMotor.setPower(-.5);
        } else if (gamepad2.left_stick_x > .3) {
            robotHardware.turretMotor.setPower(.5);
        } else {
            robotHardware.turretMotor.setPower(0);
        }
    }

    public void handleExtension() {
        double extensionTempPos = Math.max(0,gamepad2.left_stick_y) * (robotHardware.profile.hardwareSpec.extensionFullOutPos - robotHardware.profile.hardwareSpec.extensionDriverMin) + robotHardware.profile.hardwareSpec.extensionDriverMin;
        robotHardware.extensionServo.setPosition(extensionTempPos);
        telemetry.addData("Extension Pos", extensionTempPos);
    }

    public void handleLift() {
        // robotHardware.isMagneticTouched() liftMax
        /**
         * Two sets of height positions exist:
         * one for the open gripper and one for the closed gripper.
         *
         * Open Gripper (toggle between cone stack heights 1 to 5)
         * 1 - 0
         * 2 - 130
         * 3 - 260
         * 4 - 390
         * 5 - 520
         *
         * Closed Gripper (toggle between junction positions):
         * 0 - Floor (0) <-- Excluded, don't need
         * 1 - Ground (65)
         * 2 - Low (1619)
         * 3 - Medium (2682)
         * 4 - High (3781)
         */

        robotHardware.liftMotor.setPower(.4);
        if (liftCanChange) {
            if (gripperOpen) {
                if (gamepad2.right_bumper && openLiftPos < 5) {
                    openLiftPos++;
                    liftCanChange = false;
                    if (openLiftPos == 1) {
                        robotHardware.liftMotor.setTargetPosition(0);
                    } else if (openLiftPos == 2) {
                        robotHardware.liftMotor.setTargetPosition(130);
                    } else if (openLiftPos == 3) {
                        robotHardware.liftMotor.setTargetPosition(260);
                    } else if (openLiftPos == 4) {
                        robotHardware.liftMotor.setTargetPosition(390);
                    } else {
                        robotHardware.liftMotor.setTargetPosition(520);
                    }
                    robotHardware.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else if (gamepad2.left_bumper && openLiftPos > 1) {
                    openLiftPos--;
                    liftCanChange = false;
                    if (openLiftPos == 1) {
                        robotHardware.liftMotor.setTargetPosition(0);
                    } else if (openLiftPos == 2) {
                        robotHardware.liftMotor.setTargetPosition(130);
                    } else if (openLiftPos == 3) {
                        robotHardware.liftMotor.setTargetPosition(260);
                    } else if (openLiftPos == 4) {
                        robotHardware.liftMotor.setTargetPosition(390);
                    } else {
                        robotHardware.liftMotor.setTargetPosition(520);
                    }
                    robotHardware.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            } else {
                if (gamepad2.right_bumper && closedLiftPos < 4) {
                    closedLiftPos++;
                    liftCanChange = false;
                    if (closedLiftPos == 1) {
                        robotHardware.liftMotor.setTargetPosition(65);
                    } else if (closedLiftPos == 2) {
                        robotHardware.liftMotor.setTargetPosition(1619);
                    } else if (closedLiftPos == 3) {
                        robotHardware.liftMotor.setTargetPosition(2682);
                    } else {
                        robotHardware.liftMotor.setTargetPosition(3781);
                    }
                    robotHardware.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else if (gamepad2.left_bumper && closedLiftPos > 1) {
                    closedLiftPos--;
                    liftCanChange = false;
                    if (closedLiftPos == 1) {
                        robotHardware.liftMotor.setTargetPosition(65);
                    } else if (closedLiftPos == 2) {
                        robotHardware.liftMotor.setTargetPosition(1619);
                    } else if (closedLiftPos == 3) {
                        robotHardware.liftMotor.setTargetPosition(2682);
                    } else {
                        robotHardware.liftMotor.setTargetPosition(3781);
                    }
                    robotHardware.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }
        } else if (!liftCanChange && !gamepad2.right_bumper && !gamepad2.left_bumper) {
            liftCanChange = true;
        }
        telemetry.addData("Open Pos", openLiftPos);
        telemetry.addData("Closed Pos", closedLiftPos);
        telemetry.addData("Gripper Open", gripperOpen);
    }

    public void handleGripper() {
        if (gripperCanChange && (gamepad2.left_trigger > 0.3 || gamepad2.right_trigger > 0.3)) {
            if (gripperOpen) {
                gripperOpen = false;
                robotHardware.grabberClose();
                closedLiftPos = 2;
            } else {
                gripperOpen = true;
                robotHardware.grabberOpen();
            }
            gripperCanChange = false;
        } else if (gamepad2.left_trigger < 0.2 && gamepad2.right_trigger < 0.2) {
            gripperCanChange = true;
        }
    }

    /**
     * Define combo task for driver op mode
     */
    void setupCombos() {

    }
}
