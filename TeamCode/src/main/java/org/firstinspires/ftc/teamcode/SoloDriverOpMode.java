package org.firstinspires.ftc.teamcode;

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
    }
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
        currHeading = robotHardware.getGyroHeading();
        Logger.logFile("IMU Offset is " + Math.toDegrees(imuAngleOffset));
        Logger.logFile("Current IMU Angle " + Math.toDegrees(currHeading));
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
            if (gamepad1.options && gamepad1.dpad_down) {
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

        if (currentTask == null && gamepad1.dpad_left) {
            if (lastPick.liftPos!=-1) {
                robotHardware.setMotorStopBrake(true);
                recordLiftExtTut("drop", lastDrop);
                currentTask = repeatPick;
                repeatPick.prepare();
            }
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
        telemetry.addData("Heading", Math.toDegrees(currHeading));
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
            movAngle = padAngle + ((isRedTeam) ? Math.PI / 2 : -Math.PI / 2) - currHeading -imuAngleOffset;
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
        robotHardware.mecanumDrive2(power, movAngle, turn);

        if(gamepad1.options && gamepad1.dpad_right){
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
                if (Math.abs(gamepad1.touchpad_finger_1_x) > 0.95) {
                    touchX = gamepad1.touchpad_finger_1_x;
                    touchTurret = robotHardware.getTurretPosition() + robotProfile.hardwareSpec.turret360/24 * (int)Math.signum(touchX);
                    robotHardware.setTurretPosition(touchTurret);
                }
                else {
                    robotHardware.setTurretPosition(touchTurret + (int) ((gamepad1.touchpad_finger_1_x - touchX) * robotProfile.hardwareSpec.turret360 / 16));
                }
                if (Math.abs(gamepad1.touchpad_finger_1_y) > 0.95) {
                    touchY = gamepad1.touchpad_finger_1_y;
                    touchExtension = robotHardware.getExtensionPosition() + 0.01 * Math.signum(touchY);
                    robotHardware.setExtensionPosition(touchExtension);
                }
                else{
                    robotHardware.setExtensionPosition(touchExtension + (gamepad1.touchpad_finger_1_y - touchY) / 8);
                }
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
                recordLiftExtTut("drop", lastDrop);
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
                currentTask = grabAndLift;
                currentTask.prepare();
                safeDrive = true;
            }
        }
        gripperCanChange = (gamepad1.left_trigger < 0.1) && (gamepad1.right_trigger < 0.1);
        if (currentTask==null && robotHardware.isGripOpen() && robotHardware.getLiftPosition() < robotProfile.hardwareSpec.liftPickPos[4]+20) {
            if (loopCnt % 10==5) {  // let's read I2C only 1 in 10 times
                coneReflection = robotHardware.getConeReflection();
                if (coneReflection > robotProfile.hardwareSpec.coneGrabColor) {
                    recordLiftExtTut("pick", lastPick);
                    currentTask = grabAndLift;
                    currentTask.prepare();
                    safeDrive = true;
                }
            }
        }
    }

    void recordLiftExtTut(String name, LastLiftExtTut last) {
        last.extension = robotHardware.getExtensionPosition();
        last.tutPos = robotHardware.getTurretPosition();
        last.liftPos = robotHardware.getTargetLiftPosition();
        Logger.logFile("Recording " + name + " lift:" + last.liftPos + " tullett:" + last.tutPos);
    }

    /**
     * Define combo task for driver op mode
     */
    void setupCombos() {
        poleDeliverTask = new ParallelComboTask(); // push down, open, retract
        ((ParallelComboTask)poleDeliverTask).setTaskName("Pole Delivery Task");
        ((ParallelComboTask)poleDeliverTask).add(new LiftArmTask(robotHardware, robotProfile.hardwareSpec.liftSafeRotate));
        SequentialComboTask dropSeq = new SequentialComboTask();
        dropSeq.add(new RobotSleep((100)));
        dropSeq.add(new GrabberTask(robotHardware, GrabberTask.GrabberState.INIT));
        dropSeq.add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionInitPos));
        ((ParallelComboTask)poleDeliverTask).add(dropSeq);

        groundDeliverTask = new SequentialComboTask();  // simple open, lift up, and retract
        ((SequentialComboTask)groundDeliverTask).setTaskName("Ground Delivery Task");
        ((SequentialComboTask)groundDeliverTask).add(new GrabberTask(robotHardware, GrabberTask.GrabberState.SAFE));
        ((SequentialComboTask)groundDeliverTask).add(new LiftArmTask(robotHardware, robotProfile.hardwareSpec.liftSafeRotate));
        ((SequentialComboTask)groundDeliverTask).add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionInitPos));

        grabAndLift = new SequentialComboTask();
        ((SequentialComboTask)grabAndLift).setTaskName("Grab and Lift");
        ((SequentialComboTask)grabAndLift).add(new GrabberTask(robotHardware, false));
        ((SequentialComboTask)grabAndLift).add(new RobotSleep(200));
        ((SequentialComboTask)grabAndLift).add(new LiftArmTask(robotHardware, robotProfile.hardwareSpec.liftSafeRotate, false, true));
        ((SequentialComboTask)grabAndLift).add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionInitPos));

        forPickUp = new SequentialComboTask();
        ((SequentialComboTask)forPickUp).setTaskName("forPickUp");
        ((SequentialComboTask)forPickUp).add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionDriverMin));
        ((SequentialComboTask)forPickUp).add(new LiftArmTask(robotHardware, 0));
        ((SequentialComboTask)forPickUp).add(new GrabberTask(robotHardware, GrabberTask.GrabberState.OPEN));

        forGroundJunction = new SequentialComboTask();
        ((SequentialComboTask)forGroundJunction).add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionDriverMin));
        ((SequentialComboTask)forGroundJunction).add(new LiftArmTask(robotHardware, robotProfile.hardwareSpec.liftDropPos[0]));

        // AUTO PICK & DROP
        ParallelComboTask repeatToPick = new ParallelComboTask(); // push down then lift/ext/tut
        repeatToPick.setTaskName("Repeat to Pick");
        repeatToPick.add(new LiftArmTask(robotHardware, robotProfile.hardwareSpec.liftSafeRotate));
        SequentialComboTask dropSeq2 = new SequentialComboTask();
        dropSeq2.setTaskName("DropSeq2");
        dropSeq2.add(new RobotSleep((100)));
        dropSeq2.add(new GrabberTask(robotHardware, GrabberTask.GrabberState.INIT));
        dropSeq2.add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionInitPos));
        ((ParallelComboTask)repeatToPick).add(dropSeq2);

        repeatPick = new SequentialComboTask();
        repeatPick.setTaskName("Repeat Pick");
        repeatPick.add(repeatToPick);
        repeatPick.add(new LiftExtTutTask(robotHardware, lastPick));
        repeatPick.add(new RobotSleep(300));
        repeatPick.add(new GrabberTask(robotHardware, GrabberTask.GrabberState.CLOSE));
        repeatPick.add(new LiftExtTutTask(robotHardware, lastDrop));
    }
}
