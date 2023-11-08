package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;


@TeleOp(name="SOLO DriverOpMode", group="Main")
public class SoloDriverOpMode extends OpMode {
    class LastLiftExtTut {
        double extension = -1;
        int liftPos = -1;
        int tutPos = -1;
        double robHead = -1;

        public String toString() {
            return "lift:" + liftPos + " Turret:" + tutPos + " Ext:" + extension + " Heading:" + robHead;
        }
    }
    RobotHardware robotHardware;
    RobotProfile robotProfile;

    boolean fieldMode;
    boolean isRedTeam;   // always true for current autonomous set up
    boolean leftBumperPressed = false;
    double imuAngleOffset = Math.PI/2;
    boolean liftCanChange = true;
    boolean gripperCanChange = true;
    boolean justAutoPole = false;
    boolean touching = false;
    double touchX, touchY;
    int touchTurret;
    long loopCnt;
    double currHeading;
    double coneReflection;
    double touchExtension;

    // Auto pick up & drop REST IN PEACE
    LastLiftExtTut lastPick = new LastLiftExtTut();
    LastLiftExtTut lastDrop = new LastLiftExtTut();

    // DriveThru combos
    RobotControl grabAndLift, poleDeliverTask, groundDeliverTask, forPickUp, forGroundJunction;
    SequentialComboTask repeatPick;
    RobotControl currentTask = null;
    boolean safeDrive = false;

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
        robotHardware.turnUpSignalBlocker();
        robotHardware.enableManualCaching(true);
        robotHardware.clearBulkCache();
        robotHardware.getLocalizer().update();
        currHeading = robotHardware.getGyroHeading();
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(robotHardware.getHardwareMap());
        String startPosMode = prefs.getString(AutonomousOptions.START_POS_MODES_PREF, AutonomousOptions.START_POS_MODES[0]);
        isRedTeam = startPosMode.startsWith("RED");

        Logger.logFile("IMU Offset is " + Math.toDegrees(imuAngleOffset));
        Logger.logFile("Current IMU Angle " + Math.toDegrees(currHeading));
        Logger.logFile("StartPos: " + startPosMode);
        setupCombos();
        robotHardware.grabberOpen();
        coneReflection = 0;
        loopCnt = 0;
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


        if (currentTask == null && gamepad1.dpad_left) {
            if (lastPick.liftPos!=-1) {
                if (lastDrop.liftPos==-1) {
                    // we only want to record this once per dropping series for speed
                    recordLiftExtTut("drop", lastDrop);
                }
                currentTask = repeatPick;
                repeatPick.prepare();
            }
        }
        if (currentTask!=repeatPick) {
            handleMovement();
            handleGripper();
            handleLift();
            if (gamepad1.b) {
                currentTask.prepare();
            }
        }
        if (gamepad1.x) {
            currentTask = forPickUp;
            currentTask.prepare();
        }
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
        // robotHardware.isMagneticTouched() liftMax
        if (liftCanChange) {
            int[] pos = (robotHardware.isGripOpen())?robotProfile.hardwareSpec.liftPickPos:robotProfile.hardwareSpec.liftDropPos;
            int currPos = robotHardware.getTargetLiftPosition();
            if (gamepad1.right_bumper) {    // going up
                int n = 0;
                while (n<pos.length-1 && pos[n]<currPos+15) {   // need the 15 range because lift maybe shifting up/down a bit
                    n++;
                }
                robotHardware.setLiftPosition(pos[n]);
            }
            else if (gamepad1.left_bumper) { // going down
                if (robotHardware.isGripOpen()) {
                    // going for pick up
                    currentTask = forPickUp;
                    currentTask.prepare();
                    safeDrive = false;
                }
                else {
                    if (currPos<robotProfile.hardwareSpec.liftDropPos[3]) { // less than the low pole
                        // going to ground junction
                        currentTask = forGroundJunction;
                        currentTask.prepare();
                        safeDrive = false;
                    }
                    else {
                        int n = pos.length - 1;
                        while (n>0 && pos[n] >= currPos-10) {
                            n--;
                        }
                        robotHardware.setLiftPosition(pos[n]);
                    }
                }
            }
            else if(gamepad1.dpad_up) {
                robotHardware.setLiftPosition(currPos + 100);
            }
            else if (!gamepad1.options && gamepad1.dpad_down) {
                robotHardware.setLiftPosition(currPos - 100);
            }
        }
        liftCanChange = !gamepad1.right_bumper && !gamepad1.left_bumper && !gamepad1.dpad_up && !gamepad1.dpad_down;
    }

    public void handleGripper() {
        if((gamepad1.left_trigger > 0.15 || gamepad1.right_trigger > 0.15) && gripperCanChange){
            if(!robotHardware.isGripOpen()){
                if (robotHardware.getTargetLiftPosition()>robotProfile.hardwareSpec.liftDropPos[2] + 50) {
                    currentTask = poleDeliverTask;
                }
                else {
                    currentTask = groundDeliverTask;
                }
                lastDrop.liftPos = -1;  // clear the drop pos
                currentTask.prepare();
                safeDrive = true;
            }
            else if (safeDrive) {
                // If during safeDrive, grip open and click bumper, lower to pick up
                currentTask = forPickUp;
                currentTask.prepare();
                safeDrive = false;
            }
            else if (robotHardware.isGripOpen()) {
                recordLiftExtTut("pick", lastPick);
                robotHardware.setMotorStopBrake(true);
                currentTask = grabAndLift;
                currentTask.prepare();
                safeDrive = true;
            }
        }
        gripperCanChange = (gamepad1.left_trigger < 0.1) && (gamepad1.right_trigger < 0.1);
        if (currentTask==null && robotHardware.isGripOpen() && robotHardware.getLiftPosition() < robotProfile.hardwareSpec.liftPickPos[4]+20) {
            if (loopCnt % 10==5) {  // let's read I2C only 1 in 10 times
                if (robotHardware.pickUpCheck(isRedTeam)) {
                    recordLiftExtTut("pick", lastPick);
                    robotHardware.setMotorStopBrake(true);
                    currentTask = grabAndLift;
                    currentTask.prepare();
                    safeDrive = true;
                }
            }
        }
    }

    void recordLiftExtTut(String name, LastLiftExtTut last) {
        last.extension = robotHardware.getExtensionPosition();
        last.robHead = robotHardware.getGyroHeading();
        last.liftPos = robotHardware.getTargetLiftPosition();
        Logger.logFile("Recording " + name + " lift:" + last.liftPos + " tullett:" + last.tutPos + " gyro: " + last.robHead);
    }

    /**
     * Define combo task for driver op mode
     */
    void setupCombos() {

    }
}
