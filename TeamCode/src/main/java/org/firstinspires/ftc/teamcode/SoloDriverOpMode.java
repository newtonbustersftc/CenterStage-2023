package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;


@TeleOp(name="SOLO DriverOpMode", group="Main")
public class SoloDriverOpMode extends OpMode {
    RobotHardware robotHardware;
    RobotProfile robotProfile;

    boolean fieldMode;
    boolean isRedTeam = true;   // always true for current autonomous set up
    boolean leftBumperPressed = false;
    double imuAngleOffset = Math.PI/2;
    boolean liftCanChange = true;
    boolean gripperCanChange = true;
    PoleRecognition poleRecognition;
    boolean justAutoPole = false;
    boolean touching = false;
    double touchX, touchY;
    int touchTurret;
    double touchExtension;

    // DriveThru combos
    RobotControl grabAndLift, poleDeliverTask, groundDeliverTask, forPickUp, forGroundJunction;
    RobotControl currentTask = null;
    boolean safeDrive = false;

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
        Logger.logFile("OpMode - DriverOpMode");

        //Obtain the RobotHardware object from factory
        robotHardware = RobotFactory.getRobotHardware(hardwareMap, robotProfile);
        robotHardware.resetDriveAndEncoders();
        robotHardware.enableManualCaching(true);
        robotHardware.clearBulkCache();
        robotHardware.getLocalizer().update();
        poleRecognition = new PoleRecognition(robotHardware.getRobotVision(), robotProfile);
        poleRecognition.startRecognition();
        Logger.logFile("IMU Offset is " + Math.toDegrees(imuAngleOffset));
        Logger.logFile("Current IMU Angle " + Math.toDegrees(robotHardware.getGyroHeading()));
        setupCombos();
        robotHardware.grabberOpen();
    }

    /**
     * Main loop for the code
     */
    @Override
    public void loop() {
        robotHardware.clearBulkCache();
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
                    if (currentTask instanceof AutoConePlacementTask) {
                        justAutoPole = true;
                    }
                    currentTask = null;
                }
            }
        }
        else {
            if (gamepad1.share && gamepad1.dpad_down) {
                currentTask = new LiftResetTask(robotHardware, robotProfile);
                currentTask.prepare();
            }
        }

        if (currentTask instanceof AutoConePlacementTask) {
            if (gamepad1.a) {
                currentTask = null;
            }
            return;
        }

        handleMovement();
        handleGripper();
        handleLift();
        handleTurret();
        if (gamepad1.b) {
            currentTask = new AutoConePlacementTask(robotHardware, robotProfile, poleRecognition);
            currentTask.prepare();
        }
        if (gamepad1.x) {
            currentTask = forPickUp;
            currentTask.prepare();
        }
        telemetry.addData("Heading", Math.toDegrees(robotHardware.getGyroHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        robotHardware.stopAll();
        try {
            Logger.logFile("DriverOpMode stop() called");
            //robotVision.deactivateNavigationTarget();
            poleRecognition.stopRecognition();
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
            movAngle = padAngle + ((isRedTeam) ? Math.PI / 2 : -Math.PI / 2) - robotHardware.getGyroHeading()-imuAngleOffset;
        } else {
            movAngle = padAngle;
        }
        if (gamepad1.left_trigger > 0) {
            power = power / 3;
            turn = turn / 3;
        }
        telemetry.addData("Power:", power);
        telemetry.addData("Move Angle:", movAngle);
        telemetry.addData("Turn:", turn);
        telemetry.addData("RL Dir:", robotHardware.rlMotor.getDirection());
        telemetry.addData("RR Dir:", robotHardware.rrMotor.getDirection());
        robotHardware.mecanumDrive2(power, movAngle, turn);

        if(gamepad1.share && gamepad1.dpad_right){
            robotHardware.resetImu();
            imuAngleOffset = -Math.PI/2;
            fieldMode = true;
        }
    }

    // Use touch pad to control lift with 1 finger
    public void handleTurret() {
        if (gamepad1.touchpad_finger_1) {
            if (!touching) {
                touchX = gamepad1.touchpad_finger_1_x;
                touchY = gamepad1.touchpad_finger_1_y;
                touchTurret = robotHardware.getTurretPosition();
                touchExtension = robotHardware.getExtensionPosition();
            }
            else {
                robotHardware.setTurretPosition(touchTurret + (int)((gamepad1.touchpad_finger_1_x - touchX) * robotProfile.hardwareSpec.turret360/16));
                robotHardware.setExtensionPosition(touchExtension + (gamepad1.touchpad_finger_1_y - touchY)/8);
            }
        }
        touching = gamepad1.touchpad_finger_1;
    }

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
            else if (!gamepad1.share && gamepad1.dpad_down) {
                robotHardware.setLiftPosition(currPos - 100);
            }
        }
        liftCanChange = !gamepad1.right_bumper && !gamepad1.left_bumper && !gamepad1.dpad_up && !gamepad1.dpad_down;
    }

    public void handleGripper() {
        if((gamepad1.left_trigger > 0.15 || gamepad1.right_trigger > 0.15) && gripperCanChange){
            if(!robotHardware.isGripOpen()){
                if (robotHardware.getTargetLiftPosition()>robotProfile.hardwareSpec.liftSafeRotate) {
                    currentTask = poleDeliverTask;
                }
                else {
                    currentTask = groundDeliverTask;
                }
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
                currentTask = grabAndLift;
                currentTask.prepare();
                safeDrive = true;
            }
        }
        gripperCanChange = (gamepad1.left_trigger < 0.1) && (gamepad1.right_trigger < 0.1);
        if (robotHardware.isGripOpen() && robotHardware.getLiftPosition() < robotProfile.hardwareSpec.liftPickPos[4]+20
                && robotHardware.getConeReflection()>robotProfile.hardwareSpec.coneGrabColor) {
            currentTask = grabAndLift;
            currentTask.prepare();
            safeDrive = true;
        }
    }

    /**
     * Define combo task for driver op mode
     */
    void setupCombos() {
        poleDeliverTask = new ParallelComboTask(); // push down, open, retract
        ((ParallelComboTask)poleDeliverTask).add(new LiftArmTask(robotHardware, robotProfile.hardwareSpec.liftSafeRotate));
        SequentialComboTask dropSeq = new SequentialComboTask();
        dropSeq.add(new RobotSleep((100)));
        dropSeq.add(new GrabberTask(robotHardware, GrabberTask.GrabberState.INIT));
        dropSeq.add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionInitPos));
        ((ParallelComboTask)poleDeliverTask).add(dropSeq);

        groundDeliverTask = new SequentialComboTask();  // simple open, lift up, and retract
        ((SequentialComboTask)groundDeliverTask).add(new GrabberTask(robotHardware, GrabberTask.GrabberState.SAFE));
        ((SequentialComboTask)groundDeliverTask).add(new LiftArmTask(robotHardware, robotProfile.hardwareSpec.liftSafeRotate));
        ((SequentialComboTask)groundDeliverTask).add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionInitPos));

        grabAndLift = new SequentialComboTask();
        ((SequentialComboTask)grabAndLift).add(new GrabberTask(robotHardware, false));
        ((SequentialComboTask)grabAndLift).add(new RobotSleep(200));
        ((SequentialComboTask)grabAndLift).add(new LiftArmTask(robotHardware, robotProfile.hardwareSpec.liftSafeRotate, false, true));
        ((SequentialComboTask)grabAndLift).add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionInitPos));

        forPickUp = new SequentialComboTask();
        ((SequentialComboTask)forPickUp).add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionDriverMin));
        ((SequentialComboTask)forPickUp).add(new LiftArmTask(robotHardware, 0));
        ((SequentialComboTask)forPickUp).add(new GrabberTask(robotHardware, GrabberTask.GrabberState.OPEN));

        forGroundJunction = new SequentialComboTask();
        ((SequentialComboTask)forGroundJunction).add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionDriverMin));
        ((SequentialComboTask)forGroundJunction).add(new LiftArmTask(robotHardware, robotProfile.hardwareSpec.liftDropPos[0]));
    }
}
