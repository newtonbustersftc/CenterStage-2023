package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

import java.io.File;


@TeleOp(name="SOLO DriverOpMode", group="Main")
public class SoloDriverOpMode extends OpMode {
    RobotHardware robotHardware;
    RobotProfile robotProfile;

    boolean fieldMode;
    boolean isRedTeam;   // always true for current autonomous set up
    double imuAngleOffset = Math.PI/2;
    long loopCnt;
    double currHeading;

    RobotControl currentTask = null;
    boolean intakePressed;
    boolean liftPressed;
    boolean launchPressed;
    int launchStage = 0;


    @Override
    public void init() {
        try {
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profileA.json"));
        }
        catch (Exception e) {
            System.out.println(e.getStackTrace());
        }
        fieldMode = true; //robot starts in field orientation

        Logger.init();
        Logger.logFile("OpMode - DriverOpMode");

        //Obtain the RobotHardware object from factory
        robotHardware = RobotFactory.getRobotHardware(hardwareMap, robotProfile);
        robotHardware.resetDriveAndEncoders();
//        robotHardware.turnUpSignalBlocker();
        robotHardware.enableManualCaching(true);
        robotHardware.clearBulkCache();
        robotHardware.getLocalizer().update();
        currHeading = robotHardware.getGyroHeading();
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(robotHardware.getHardwareMap());
        String startPosMode = prefs.getString(AutonomousOptions.START_POS_MODES_PREF, AutonomousOptions.START_POS_MODES[0]);
        isRedTeam = startPosMode.startsWith("RED");
        imuAngleOffset = -Math.PI/2;

        Logger.logFile("IMU Offset is " + Math.toDegrees(imuAngleOffset));
        Logger.logFile("Current IMU Angle " + Math.toDegrees(currHeading));
        Logger.logFile("StartPos: " + startPosMode);
        setupCombos();
        //robotHardware.grabberOpen();

        robotHardware.droneInitPosition();

        loopCnt = 0;

        telemetry.addLine("Init Complete!");
        telemetry.update();
    }

    /**
     * Main loop for the code
     */
    @Override
    public void loop() {
        loopCnt++;
        robotHardware.clearBulkCache();
        if (loopCnt % 10==0) {
            currHeading = robotHardware.getGyroHeading();   // only use Gyro heading every 10 times
        }
        if (loopCnt % 1000==0) {
            Logger.logFile(robotHardware.getLiftMotorPos());
        }
        //Handling autonomous task loop
        if (currentTask != null) {
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
            if (gamepad1.options && gamepad1.dpad_down) {
                currentTask = new LiftResetTask(robotHardware, robotProfile);
                currentTask.prepare();
            }
        }
        if (currentTask==null || !(currentTask instanceof DropPixelTask)){
            // when doing dropPixelTask, no manual movement
            handleMovement();
        }
        handleGripper();
        handleLift();
        handleIntake();
        handleLauncher();
        handleHanger();
        telemetry.addData("Heading", Math.toDegrees(currHeading));
        telemetry.update();
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

        double movAngle;
        if (fieldMode) {
            movAngle = padAngle + Math.PI / 2 - currHeading -imuAngleOffset;
        } else {
            movAngle = padAngle;
        }
        if (gamepad1.left_trigger > 0) {
            power = power / 3;
            turn = turn / 3;
        }
//        telemetry.addData("Power:", power);
//        telemetry.addData("Move Angle:", movAngle);
//        telemetry.addData("Turn:", turn);
        if (power > 0.2) {
            robotHardware.setMotorStopBrake(false);
        }
        robotHardware.mecanumDrive2(power, movAngle, turn);

        if(gamepad1.options && gamepad1.dpad_right){
            robotHardware.resetImu();
            imuAngleOffset = -Math.PI/2;
            fieldMode = true;
        }
    }

    // Use touch pad to control lift with 1 finger

    public void handleLift() {
        if (!liftPressed && gamepad1.dpad_up) {
            if (robotHardware.getLiftTargetPosition()>=robotProfile.hardwareSpec.liftOutMin-30) {
                int newLiftPos = Math.min(robotProfile.hardwareSpec.liftMax, robotHardware.getLiftTargetPosition()+
                        robotProfile.hardwareSpec.liftIncrement);
                Logger.logFile("New lift pos: " + newLiftPos);
                robotHardware.setLiftPosition(newLiftPos);
            }
        }
        if (!liftPressed && gamepad1.dpad_down) {
            if (robotHardware.getLiftTargetPosition()>=robotProfile.hardwareSpec.liftOutMin-30) {
                int newLiftPos = Math.max(robotProfile.hardwareSpec.liftOutMin, robotHardware.getLiftTargetPosition()-
                        robotProfile.hardwareSpec.liftIncrement);
                Logger.logFile("New lift pos: " + newLiftPos);
                robotHardware.setLiftPosition(newLiftPos);
            }
        }
        liftPressed = gamepad1.dpad_down || gamepad1.dpad_up;
    }

    public void handleGripper() {
        if (currentTask==null && gamepad1.x) {
            if (!robotHardware.isGrabberUp()) {
                currentTask = new PixelUpTask(robotHardware, robotProfile.hardwareSpec.liftOutMin, true);
                currentTask.prepare();
            }
            else {
                currentTask = new RetractGrabberTask(robotHardware);
                currentTask.prepare();
            }
        }
        else if (currentTask==null && gamepad1.y) {
            if (!robotHardware.isGrabberUp()) {
                currentTask = new PixelUpTask(robotHardware, robotProfile.hardwareSpec.liftOutMin, false);
                currentTask.prepare();
            }
            else {
                currentTask = new RetractGrabberTask(robotHardware);
                currentTask.prepare();
            }
        }
        else if (currentTask==null && gamepad1.b) {
            if (robotHardware.isGrabberUp()) {
                currentTask = new DropPixelTask(robotHardware);;
                currentTask.prepare();
            }
        }
        if (currentTask==null && gamepad1.left_bumper && robotHardware.getLiftPosition()>robotProfile.hardwareSpec.liftOutMin-100) {
            robotHardware.grabberLeft();
        }
        else if (currentTask==null && gamepad1.right_bumper && robotHardware.getLiftPosition()>robotProfile.hardwareSpec.liftOutMin-100) {
            robotHardware.grabberRight();
        }
    }

    public void handleIntake() {
        if (currentTask==null || currentTask instanceof SmartIntakeActionTask) {
            if (!intakePressed && (gamepad1.left_trigger>0.3 || gamepad1.right_trigger>0.3)) {
                if (robotHardware.isGrabberUp()) {
                    // If the grabber is raised, lower the grabber rather than starting the intake.
                    if (currentTask instanceof SmartIntakeActionTask) {
                        robotHardware.stopIntake();
                    }
                    currentTask = new RetractGrabberTask(robotHardware);
                    currentTask.prepare();
                } else {
                    // Regular intake functions
                    RobotHardware.IntakeMode currMode = robotHardware.getIntakeMode();
                    if (gamepad1.options) {
                        // If the option button is pressed...
                        if (currMode == RobotHardware.IntakeMode.REVERSE) {
                            // ...and the intake is already reversed...
                            // ...stop the intake.
                            robotHardware.stopIntake();
                        } else {
                            // ...and the intake isn't reversed...
                            // ...start the intake in reverse.
                            currentTask = null; // in case smart intake is going on
                            robotHardware.reverseIntake();
                        }
                    } else {
                        // If the option button isn't pressed...
                        if (!(currentTask instanceof SmartIntakeActionTask)) {
                            // ...and the intake is off...
                            // ...start the intake.
                            //robotHardware.startIntake();
                            currentTask = new SmartIntakeActionTask(robotHardware, 30000);
                            currentTask.prepare();
                        } else {
                            // ...and the intake isn't off...
                            // ...stop the intake.
                            currentTask = null;
                            robotHardware.stopIntake();
                        }
                    }
                }
            }
        }
        intakePressed = gamepad1.left_trigger>0.3 || gamepad1.right_trigger>0.3;
    }

    public void handleLauncher() {
        if (gamepad1.dpad_right && !launchPressed) {
            if (launchStage == 0) {
                robotHardware.droneShootPosition();
                launchStage++;
            } else if (launchStage == 1) {
                robotHardware.droneRelease();
                launchStage++;
            } else if (launchStage == 2) {
                robotHardware.droneHook();
                robotHardware.droneInitPosition();
                launchStage++;
            }
        }
        launchPressed = gamepad1.dpad_right || gamepad1.options;
    }

    public void handleHanger() {
        if (gamepad1.a) {
            robotHardware.hang(-1);
        } else if (gamepad1.dpad_left) {
            robotHardware.hang(1);
        } else {
            robotHardware.hang(0);
        }
    }

    /**
     * Define combo task for driver op mode
     */
    void setupCombos() {

    }
}
