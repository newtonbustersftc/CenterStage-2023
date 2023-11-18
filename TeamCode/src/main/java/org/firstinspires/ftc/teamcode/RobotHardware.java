package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.List;

public class RobotHardware {
    public double extensionPos;
    HardwareMap hardwareMap;
    DcMotorEx rrMotor, rlMotor, frMotor, flMotor;
    DcMotorEx intakeMotor, hangerMotor;
    private DcMotorEx[] liftMotors;        // make it private so we can prevent mistakes by lift down while arm is retracted in
    //private
    Servo dronePivotServo, droneReleaseServo, intakeServo1, intakeServo2,
            gripperServo, gripperInOutServo, gripperRotateServo, droppingServo;

    TouchSensor magneticSensor, liftTouch;
    NormalizedColorSensor coneSensor;

    LynxModule expansionHub1;
    LynxModule expansionHub2;
    NBMecanumDrive mecanumDrive;
    BNO055IMU imu;
    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;
    DistanceSensor distanceSensorLeft, distanceSensorRight;
    double gyroOffset;
    boolean gripOpen = false;

    DecimalFormat nf2 = new DecimalFormat("#.##");
    RobotProfile profile;
    enum IntakeMode { ON, REVERSE, OFF }
    IntakeMode intakeMode;
    AprilTagDetection aprilTag;


    public void init(HardwareMap hardwareMap, RobotProfile profile) {
        Logger.logFile("RobotHardware init()");
        this.hardwareMap = hardwareMap;
        this.profile = profile;
        expansionHub1 = hardwareMap.get(LynxModule.class, "Control Hub");
        rrMotor = hardwareMap.get(DcMotorEx.class, "RRMotor");
        rlMotor = hardwareMap.get(DcMotorEx.class, "RLMotor");
        frMotor = hardwareMap.get(DcMotorEx.class, "FRMotor");
        flMotor = hardwareMap.get(DcMotorEx.class, "FLMotor");
        dronePivotServo = hardwareMap.servo.get("dronePivotServo");
        droneReleaseServo = hardwareMap.servo.get("droneReleaseServo");
        droppingServo = hardwareMap.servo.get("droppingServo");

        expansionHub2 = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        intakeServo1 = hardwareMap.servo.get("intakeServo1");
        intakeServo2 = hardwareMap.servo.get("intakeServo2");
        gripperServo = hardwareMap.servo.get("gripperServo");
        gripperInOutServo = hardwareMap.servo.get("gripperInOutServo");
        gripperRotateServo = hardwareMap.servo.get("gripperRotateServo");

        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangerMotor = hardwareMap.get(DcMotorEx.class,"hangerMotor");

        liftMotors = new DcMotorEx[2];
        liftMotors[0] = hardwareMap.get(DcMotorEx.class,"liftMotorLeft");
        liftMotors[1] = hardwareMap.get(DcMotorEx.class,"liftMotorRight");
        liftMotors[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER); //??
        liftMotors[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);  //??
        liftMotors[1].setDirection(DcMotorSimple.Direction.REVERSE);

        //two distance sensors
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        // Use manual cache mode for most efficiency, but each program
        // needs to call clearBulkCache() in the while loop
        expansionHub1.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        //expansionHub1.clearBulkCache();
        expansionHub2.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        mecanumDrive = new NBMecanumDrive(this, profile);
        //mecanumDrive.setLocalizer(realSenseLocalizer);
        initGyro();
        resetImu();
        resetDriveAndEncoders();
        intakeMode = IntakeMode.OFF;
    }

    public List<DcMotorEx> getDriveMotors() {
        return Arrays.asList(flMotor, rlMotor, rrMotor, frMotor);
    }

    public void initGyro() {
        if (profile.hardwareSpec.useControlHubImu) {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
        }
        else {
            navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
            gyro = (IntegratingGyroscope)navxMicro;
        }
    }

    public void intakeDropSpike(int step) {
        if (step==0) {
            intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeMotor.setTargetPosition(0);
            intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeMotor.setPower(0.5); //0.15
            Logger.logFile("intakeDropSpike step=0");
        }
        else {
            intakeMotor.setTargetPosition((profile.hardwareSpec.intakeDropSpikeStep*step));
            Logger.logFile("intakeDropSpike step = "+ step + ", targetPosition = "+profile.hardwareSpec.intakeDropSpikeStep*step);
        }
    }

    public void resetIntakeMotor() {
        intakeMotor.setPower(0);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetDriveAndEncoders() {
        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rlMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotors[1].setDirection(DcMotorSimple.Direction.FORWARD);
        expansionHub1.clearBulkCache();
        expansionHub2.clearBulkCache();
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public MecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }

    public Localizer getLocalizer() {
        return mecanumDrive.getLocalizer();
    }

    public void startIntake() {
        intakeMode = IntakeMode.ON;
        setLiftPosition(profile.hardwareSpec.liftIntakePos);
        grabberOpen();
        grabberIn();
        intakeMotor.setPower(profile.hardwareSpec.intakePower);
        intakeServo1.setPosition(profile.hardwareSpec.intakeServo1In);
        intakeServo2.setPosition(profile.hardwareSpec.intakeServo2In);
    }
    public void reverseIntake() {
        intakeMode = IntakeMode.REVERSE;
        intakeMotor.setPower(-profile.hardwareSpec.intakePower);
        intakeServo1.setPosition(profile.hardwareSpec.intakeServo1Out);
        intakeServo2.setPosition(profile.hardwareSpec.intakeServo2Out);
    }

    public void stopIntake() {
        intakeMode = IntakeMode.OFF;
        setLiftPosition(0);
        intakeMotor.setPower(0);
        intakeServo1.setPosition(profile.hardwareSpec.intakeServo1Stop);
        intakeServo2.setPosition(profile.hardwareSpec.intakeServo2Stop);
    }

    public IntakeMode getIntakeMode() {
        return intakeMode;
    }
    public double getDistanceSensorLeft(){
        return distanceSensorLeft.getDistance(DistanceUnit.INCH);
    }

    public double getDistanceSensorRight(){
        return distanceSensorRight.getDistance(DistanceUnit.INCH);
    }

    public int getLiftPosition() {
        return liftMotors[1].getCurrentPosition();
    }

    public int getLiftTargetPosition() {
        return liftMotors[1].getTargetPosition();
    }

    public String getLiftMotorPos() {
        return "Lift12: " + liftMotors[0].getCurrentPosition() + "," + liftMotors[1].getCurrentPosition() ;
    }

    public double getLiftVelocity() {
        return liftMotors[1].getVelocity();
    }

    public void setLiftPosition(int newLiftPos) {
        int currPos = liftMotors[0].getCurrentPosition();
        setLiftPosition(newLiftPos, (currPos<newLiftPos)?profile.hardwareSpec.liftPowerUp:
                profile.hardwareSpec.liftPowerDown);
    }

    public void setLiftPosition(int newLiftPos, double power) {
        for(DcMotorEx liftMotor : liftMotors) {
            liftMotor.setTargetPosition(newLiftPos);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(power);
        }
    }

    public void setLiftPower(double power) {
        for(DcMotorEx liftMotor : liftMotors) {
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor.setPower(power);
        }
    }

    public int getTargetLiftPosition() {
        return liftMotors[1].getTargetPosition();
    }

    public boolean isLiftMoving() {
        return Math.abs(liftMotors[1].getVelocity())>5;
    }

    public void mecanumDrive2(double power, double angle, double rotation){
        //10/28/2019, Will, Ian Athena implemented and tested the drive method
        double robotAngle = Math.PI / 2 + angle - Math.PI / 4;
        double frontLeft = power * Math.cos(robotAngle) + rotation;
        double frontRight = power * Math.sin(robotAngle) - rotation;
        double rearLeft = power * Math.sin(robotAngle) + rotation;
        double rearRight = power * Math.cos(robotAngle) - rotation;


        double biggest = 0;
        if (Math.abs(frontRight) > biggest){
            biggest = Math.abs(frontRight);
        }
        if (Math.abs(rearLeft) > biggest){
            biggest = Math.abs(rearLeft);
        }
        if (Math.abs(rearRight) > biggest){
            biggest = Math.abs(rearRight);
        }
        if (Math.abs(frontLeft) > biggest){
            biggest = Math.abs(frontLeft);
        }

        power = Math.max(power, Math.abs(rotation));
        frontLeft = frontLeft/biggest*power;
        frontRight = frontRight/biggest*power;
        rearLeft = rearLeft/biggest*power;
        rearRight = rearRight/biggest*power;

        setMotorPower(frontLeft, frontRight, rearLeft, rearRight);        // TO FIX!!!
    }

    public void setMotorPower(double flPower, double frPower, double rlPower, double rrPower) {
        try {
            flMotor.setPower(flPower);
            frMotor.setPower(frPower);
            rlMotor.setPower(rlPower);
            rrMotor.setPower(rrPower);
        }
        catch (Exception ex) {
            Logger.logFile("setMotorPower exception: " + ex);
            ex.printStackTrace();
        }
    }

    public void setMotorStopBrake(boolean brake) {
        flMotor.setZeroPowerBehavior(brake?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
        frMotor.setZeroPowerBehavior(brake?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
        rlMotor.setZeroPowerBehavior(brake?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
        rrMotor.setZeroPowerBehavior(brake?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void enableManualCaching(boolean enable) {
        Logger.logFile("hub1:"+expansionHub1);
        Logger.logFile("hub2:"+expansionHub2);
        if (enable) {
            expansionHub1.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            expansionHub2.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        else {
            expansionHub1.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            expansionHub2.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void clearBulkCache() {
        expansionHub1.clearBulkCache();
        expansionHub2.clearBulkCache();
    }

    public void stopAll() {
        setMotorPower(0, 0, 0, 0);
        for(DcMotorEx liftMotor:liftMotors) {
            liftMotor.setPower(0);
        }

    }

    public enum EncoderType {LEFT, RIGHT, HORIZONTAL}

    public double getGyroHeading() {
        if (profile.hardwareSpec.useControlHubImu) {
            double firstAngle = imu.getAngularOrientation().firstAngle;
            return firstAngle;
        }
        else {
            Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            return angles.firstAngle - gyroOffset;
        }
    }

    public double getGyroVelocity() {
        if (profile.hardwareSpec.useControlHubImu) {
            AngularVelocity angles = imu.getAngularVelocity();
            return angles.zRotationRate;
        }
        else {
            AngularVelocity angles = gyro.getAngularVelocity(AngleUnit.RADIANS);
            return angles.zRotationRate;
        }
    }

    public void resetImu() {
        if (profile.hardwareSpec.useControlHubImu) {
            Logger.logFile("Resetting Built-in IMU");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = true;
            boolean result = imu.initialize(parameters);
            //If Robot Controller is vertically placed, uncomment the following line
            if (profile.hardwareSpec.revHubVertical) {
                BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
            }
            Logger.logFile("Reset Built-in IMU " + result);
        }
        else {
            calibrateNavxGyro(null);
            gyroOffset = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        }
    }


    void calibrateNavxGyro(Telemetry telemetry) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        gyroOffset = 0;
        while (navxMicro.isCalibrating()) {
            if (telemetry != null) {
                telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
                telemetry.update();
            }
            try {
                Thread.sleep(50);
            }
            catch (InterruptedException ex)
            {}
        }
        Logger.logFile("NavXGyro calibrated in "+ Math.round(timer.seconds()));
    }

    RobotProfile getRobotProfile() {
        return profile;
    }

    public int getFLMotorEncoderCnt(){
        return this.flMotor.getCurrentPosition();
    }

    public int getFRMotorEncoderCnt(){
        return this.frMotor.getCurrentPosition();
    }

    public int getRlMotorEncoderCnt(){
        return this.rlMotor.getCurrentPosition();
    }

    public int getRRMotorEncoderCnt(){
        return this.rrMotor.getCurrentPosition();
    }

    public void resetLiftPos() {
        for(DcMotorEx liftMotor : liftMotors) {
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void grabberOpen() {
        gripOpen = true;
        gripperServo.setPosition(profile.hardwareSpec.grabberOpenPos);
    }

    public void grabberClose() {
        grabberClose(true);
    }
    public void grabberClose(boolean isOne) {
        gripOpen = false;
        gripperServo.setPosition(isOne?profile.hardwareSpec.gripperServo1Pixel:profile.hardwareSpec.gripperServo2Pixel);
    }

    public boolean isGripOpen() {
        return gripOpen;
    }

    public void grabberUp() {
        gripperRotateServo.setPosition(profile.hardwareSpec.gripperRotateServoUp);
    }

    public void grabberLeft() {
        gripperRotateServo.setPosition(profile.hardwareSpec.gripperRotateServoLeft);
    }
    public void grabberRight() {
        gripperRotateServo.setPosition(profile.hardwareSpec.gripperRotateServoRight);
    }

    public void grabberIn() {
        gripperInOutServo.setPosition(profile.hardwareSpec.gripperInOutServoIn);
    }

    public void grabberOut() {
        gripperInOutServo.setPosition(profile.hardwareSpec.gripperInOutServoOut);
    }

    public void droneShootPosition() {
        dronePivotServo.setPosition(profile.hardwareSpec.droneServoShootPos);
    }

    public void droneInitPosition() {
        dronePivotServo.setPosition(profile.hardwareSpec.droneServoInitPos);
    }

    public void droneLoadPosition() {
        dronePivotServo.setPosition(profile.hardwareSpec.droneServoLoadPos);
    }

    public void droneRelease() {
        droneReleaseServo.setPosition(profile.hardwareSpec.droneHookOpenPos);
    }
    public void droneHook() {
        droneReleaseServo.setPosition(profile.hardwareSpec.droneHookClosePos);
    }

    public void initSetupNoAuto(OpMode opmod) {
        /*
        grabberOpen();
        resetLiftPos();
        setLiftPosition(profile.hardwareSpec.liftOutMin);
        long t = System.currentTimeMillis();
        while (!isLiftTouched() && (System.currentTimeMillis()-t)<3000) {
            try {
                Thread.sleep(100);
            }
            catch (Exception ex) {
            }
        }
        resetLiftPos();
        resetLiftPos();
         */
    }

    public void initSetup(LinearOpMode opmode) {
        resetDriveAndEncoders();
        opmode.telemetry.clearAll();
        boolean done = false;
        int frMin = 0, frMax = 0, rrMin = 0, rrMax = 0, flMin = 0, flMax = 0, rlMin = 0, rlMax = 0;
        int MIN_MAX = 1250;
        while (!opmode.isStopRequested() && !done) {
            opmode.telemetry.addData("MSG:", "PLEASE MOVE THE ROBOT AROUND");
            opmode.telemetry.addData("FR", frMotor.getCurrentPosition());
            opmode.telemetry.addData("FL", flMotor.getCurrentPosition());
            opmode.telemetry.addData("RR", rrMotor.getCurrentPosition());
            opmode.telemetry.addData("RL", rlMotor.getCurrentPosition());
            opmode.telemetry.addData("VOLT", expansionHub1.getInputVoltage(VoltageUnit.VOLTS));
            frMin = Math.min(frMin, frMotor.getCurrentPosition());
            frMax = Math.max(frMax, frMotor.getCurrentPosition());
            flMin = Math.min(flMin, flMotor.getCurrentPosition());
            flMax = Math.max(flMax, flMotor.getCurrentPosition());
            rrMin = Math.min(rrMin, rrMotor.getCurrentPosition());
            rrMax = Math.max(rrMax, rrMotor.getCurrentPosition());
            rlMin = Math.min(rlMin, rlMotor.getCurrentPosition());
            rlMax = Math.max(rlMax, rlMotor.getCurrentPosition());
            done = (frMax - frMin > MIN_MAX) && (flMax - flMin > MIN_MAX) &&
                    (rlMax - rlMin > MIN_MAX) && (rrMax - rrMin > MIN_MAX)  || opmode.gamepad1.dpad_right;
            opmode.telemetry.update();
        }
        String text;

        if (expansionHub1.getInputVoltage(VoltageUnit.VOLTS) < 12.8) {
            text = "******************\nCHANGE BATTERY NOW\n******************";
        }
        else{
            text = "Press UP to start auto init ...";
        }
        waitforUp(opmode, text);
        if (opmode.isStopRequested()) return;
        // Lift position reset
        long startTime = System.currentTimeMillis();
        resetLiftPos();
        // 1. move up
        setLiftPosition(profile.hardwareSpec.liftOutMin+100, 0.3);
        while (getLiftPosition() < profile.hardwareSpec.liftOutMin && (System.currentTimeMillis() - startTime)<3000) {
            opmode.sleep(10);
        }
        setLiftPower(0);
        grabberOpen();
        grabberUp();
        grabberIn();
        opmode.sleep(500);
        setLiftPower(-0.1);
        long downStart = System.currentTimeMillis();
        while (isLiftMoving() || (System.currentTimeMillis() - downStart) < 100) {
            opmode.sleep(10);
        }
        resetLiftPos();
        opmode.sleep(100);
        setLiftPower(0.1);
        opmode.sleep(300);
        setLiftPosition(0);
        droneRelease();
        droneLoadPosition();
        waitforUp(opmode, "Get Drone In Place Please, then UP");
        droneHook();
        try { Thread.sleep(500); } catch (Exception ex) {}
        waitforUp(opmode, "UP -> Ready");
        droneInitPosition();
    }

    void waitforUp(LinearOpMode opmode, String text) {
        opmode.telemetry.clearAll();
        // wait until dpad-up pressed
        while (!opmode.isStopRequested() && !opmode.gamepad1.dpad_up) {
//            if (Math.abs(frMotor.getCurrentPosition()-rlMotor.getCurrentPosition())>400 ||
//                    Math.abs(flMotor.getCurrentPosition()-rrMotor.getCurrentPosition())>400) {
//                opmode.telemetry.addLine("******************\nONE OF DEAD WHEEL MAY BE STUCK\n******************");
//            }
            opmode.telemetry.addData("MSG", text);
            opmode.telemetry.addData("FR", frMotor.getCurrentPosition());
            opmode.telemetry.addData("FL", flMotor.getCurrentPosition());
            opmode.telemetry.addData("RR", rrMotor.getCurrentPosition());
            opmode.telemetry.addData("RL", rlMotor.getCurrentPosition());
            opmode.sleep(50);
            opmode.telemetry.update();
        }
        // wait until dpad-up released
        while (!opmode.isStopRequested() && opmode.gamepad1.dpad_up) {
            opmode.sleep(50);
        }
    }

    DcMotorEx[] getLiftMotors() {
        return liftMotors;
    }

    public void initDroppingStick(){
        this.droppingServo.setPosition(0.2);
    }

    public void releaseDroppingStick(){
        this.droppingServo.setPosition(0.6);
    }

    public double getDroppingServoNumber(){
        return this.droppingServo.getPosition();
    }

    public void storeDesiredAprilTag(AprilTagDetection desiredTag){
        this.aprilTag = desiredTag;
        Logger.logFile("Tag id="+this.aprilTag.id);
        Logger.logFile("range="+this.aprilTag.ftcPose.range);
        Logger.logFile("bearing="+this.aprilTag.ftcPose.bearing);
        Logger.logFile("yaw="+this.aprilTag.ftcPose.yaw);
        Logger.logFile("x="+this.aprilTag.ftcPose.x);
        Logger.logFile("y="+this.aprilTag.ftcPose.y);
        Logger.logFile("z="+this.aprilTag.ftcPose.z);
        Logger.logFile("roll="+this.aprilTag.ftcPose.roll);
        Logger.logFile("pitch="+this.aprilTag.ftcPose.pitch);
        Logger.logFile("elevation="+this.aprilTag.ftcPose.elevation);
        Logger.logFile("field position="+this.aprilTag.metadata.fieldPosition.toString());
        Logger.flushToFile();
    }

    public AprilTagDetection getDesiredAprilTag(){
        return this.aprilTag;
    }

    public int getDesireAprilTagID(){
        if(this.aprilTag == null)
            return -1;
        else
            return this.aprilTag.id;
    }
}