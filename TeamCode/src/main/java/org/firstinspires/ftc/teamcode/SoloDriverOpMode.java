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

    // DriveThru combos
    RobotControl grabAndLift, deliverTask,  forPickUp, forGroundJunction;
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
        //Obtain the RobotHardware object from factory
        robotHardware = RobotFactory.getRobotHardware(hardwareMap, robotProfile);
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
        robotHardware.mecanumDrive2(power, movAngle, turn);

        if(gamepad1.share){
            robotHardware.resetImu();
            imuAngleOffset = 0;
            fieldMode = true;
        }
    }

    // Use touch pad to control lift with 1 finger
    public void handleTurret() {
        if (gamepad1.touchpad_finger_1) {
            robotHardware.turnTurret(gamepad1.touchpad_finger_1_x);
        }
    }

    public void handleLift() {
        // robotHardware.isMagneticTouched() liftMax
        if (liftCanChange) {
            int[] pos = (robotHardware.isGripOpen())?robotProfile.hardwareSpec.liftPickPos:robotProfile.hardwareSpec.liftDropPos;
            int currPos = robotHardware.getLiftPosition();
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
                    if ((robotProfile.hardwareSpec.liftDropPos[1] - currPos) > -10) {
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
        }
        liftCanChange = !gamepad1.right_bumper && !gamepad1.left_bumper;
    }

    public void handleGripper() {
        if((gamepad1.left_trigger > 0.15 || gamepad1.right_trigger > 0.15) && gripperCanChange){
            if(!robotHardware.isGripOpen()){
                currentTask = deliverTask;
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
    }

    /**
     * Define combo task for driver op mode
     */
    void setupCombos() {
        deliverTask = new ParallelComboTask();
        ((ParallelComboTask)deliverTask).add(new GrabberTask(robotHardware, GrabberTask.GrabberState.INIT));
        SequentialComboTask dropSeq = new SequentialComboTask();
        dropSeq.add(new RobotSleep((100)));
        dropSeq.add(new LiftArmTask(robotHardware, robotProfile.hardwareSpec.liftSafeRotate));
        dropSeq.add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionInitPos));
        ((ParallelComboTask)deliverTask).add(dropSeq);

        grabAndLift = new SequentialComboTask();
        ((SequentialComboTask)grabAndLift).add(new GrabberTask(robotHardware, false));
        ((SequentialComboTask)grabAndLift).add(new RobotSleep(200));
        ((SequentialComboTask)grabAndLift).add(new LiftArmTask(robotHardware, robotProfile.hardwareSpec.liftSafeRotate));
        ((SequentialComboTask)grabAndLift).add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionInitPos));

        forPickUp = new SequentialComboTask();
        ((SequentialComboTask)forPickUp).add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionDriverMin));
        ((SequentialComboTask)forPickUp).add(new LiftArmTask(robotHardware, 0));

        forGroundJunction = new SequentialComboTask();
        ((SequentialComboTask)forGroundJunction).add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionDriverMin));
        ((SequentialComboTask)forGroundJunction).add(new LiftArmTask(robotHardware, robotProfile.hardwareSpec.liftDropPos[0]));
    }
}
