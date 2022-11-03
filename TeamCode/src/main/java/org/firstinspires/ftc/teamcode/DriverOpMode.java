package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;

import static org.firstinspires.ftc.teamcode.AutonomousOptions.START_POS_MODES_PREF;


@TeleOp(name="Newton DriverOpMode", group="Main")
public class DriverOpMode extends OpMode {
    RobotHardware robotHardware;
    RobotProfile robotProfile;

    Pose2d currPose;
    double fieldHeadingOffset;

    boolean fieldMode;
    int fieldModeSign = -1;  // RED side = 1, BLUE side = -1
    boolean isRedTeam = true;   // always true for current autonomous set up
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
    double imuAngleOffset = Math.PI/2;
    double driveAngle = 0;
    boolean amCorrecting = true;
    boolean liftCanChange = true;
    boolean gripperCanChange = true;

    // DriveThru combos
    SequentialComboTask grabAndLift, deliverTask,  sharedHubTask;
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
        robotHardware = RobotFactory.getRobotHardware();
        //robotHardware = new RobotHardware();
        //robotHardware.init(hardwareMap, robotProfile);
        //robotHardware.resetLiftPos();
        //robotHardware.initRobotVision();
        //robotVision = robotHardware.getRobotVision();
        //robotVision.activateNavigationTarget();
        //robotHardware.initLeds();   // need to init everytime
       // robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));
        //ensure lift is reset at the beginning and the end
        // Based on the Autonomous mode starting position, define the heading offset for field mode
        Logger.logFile("IMU Offset is " + Math.toDegrees(imuAngleOffset));
        Logger.logFile("Current IMU Angle " + Math.toDegrees(robotHardware.getGyroHeading()));
        setupCombos();
    }

    /**
     * Main loop for the code
     */
    @Override
    public void loop() {
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

        handleMovement();
        handleExtension();
        handleGripper();
        handleLift();
        handleTurret();

        telemetry.addData("Heading", Math.toDegrees(robotHardware.getGyroHeading()));


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

    public void handleTurret() {
        robotHardware.turnTurret(gamepad2.left_stick_x);
        telemetry.addData("Turret Pos", robotHardware.getTurretPosition());

//        if (gamepad2.left_stick_x < -.3) {
//            robotHardware.turretMotor.setPower(-.5);
//        } else if (gamepad2.left_stick_x > .3) {
//            robotHardware.turretMotor.setPower(.5);
//        } else {
//            robotHardware.turretMotor.setPower(0);
//        }
    }

    public void handleExtension() {
        double extensionTempPos = Math.max(0,-gamepad2.right_stick_y) * (robotHardware.profile.hardwareSpec.extensionFullOutPos - robotHardware.profile.hardwareSpec.extensionDriverMin) + robotHardware.profile.hardwareSpec.extensionDriverMin;
        robotHardware.setExtensionPosition(extensionTempPos);
        telemetry.addData("Extension Pos", extensionTempPos);
    }

    public void handleLift() {
        // robotHardware.isMagneticTouched() liftMax
        if (liftCanChange) {
            int[] pos = (robotHardware.isGripOpen())?robotProfile.hardwareSpec.liftPickPos:robotProfile.hardwareSpec.liftDropPos;
            int currPos = robotHardware.getLiftPosition();
            if (gamepad2.right_bumper) {    // going up
                int n = 0;
                while (n<pos.length-1 && pos[n]<currPos+50) {   // need the 50 range because lift maybe shifting up/down a bit
                    n++;
                }
                robotHardware.setLiftPosition(pos[n]);
            }
            else if (gamepad2.right_trigger > 0.3) { // going down
                int n = pos.length - 1;
                while (n>0 && pos[n] >= currPos-50) {
                    n--;
                }
                robotHardware.setLiftPosition(pos[n]);
            }
        }
        liftCanChange = !gamepad2.right_bumper && gamepad2.right_trigger <0.3;
    }

    public void handleGripper() {
        if(gamepad2.left_bumper){
            if(!robotHardware.isGripOpen()){
                currentTask = deliverTask;
                currentTask.prepare();
            }
        } else if (gamepad2.left_trigger > 0.3){
            if (robotHardware.isGripOpen()) {
                currentTask = grabAndLift;
                currentTask.prepare();
            }
        }
    }

    /**
     * Define combo task for driver op mode
     */
    void setupCombos() {
        deliverTask = new SequentialComboTask();
        deliverTask.addTask(new GrabberTask(robotHardware, true));
        deliverTask.addTask(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionDriverMin));
        deliverTask.addTask(new LiftArmTask(robotHardware, 0));

        grabAndLift = new SequentialComboTask();
        grabAndLift.addTask(new GrabberTask(robotHardware, false));
        grabAndLift.addTask(new LiftArmTask(robotHardware, robotProfile.hardwareSpec.liftDropPos[2]));
    }
}
