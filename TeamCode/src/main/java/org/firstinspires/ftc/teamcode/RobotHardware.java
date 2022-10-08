package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

import java.text.DecimalFormat;

public class RobotHardware {
    public enum LiftPosition {
        NOT_INIT, ZERO, ONE, BOTTOM, MIDDLE, TOP;
        private static LiftPosition[] vals = values();

        public LiftPosition next() {
            return (this.ordinal() < vals.length - 1) ? vals[this.ordinal() + 1] : this;
        }

        public LiftPosition prev() { // can not goto init position
            return (this.ordinal() > 0) ? vals[this.ordinal() - 1] : vals[0];
        }
    }
    LiftPosition currLiftPos;
    boolean ignoreT265Confidence = false;
    int currLiftEncoder;
    int ogDuckEncoder;

    public enum Freight {
        NONE, CUBE, BALL
    }

    HardwareMap hardwareMap;
    DcMotor rrMotor, rlMotor, frMotor, flMotor;
    DcMotor liftMotor, duckMotor, intakeMotor;
    DcMotor targetLight;
    //DigitalChannel led1, led2;
    Servo boxFlapServo, boxLidServo;
    ColorSensor colorSensor;
    DigitalChannel liftBottom;
    LynxModule expansionHub1;
    //LynxModule expansionHub2;
    NBMecanumDrive mecanumDrive;
    BNO055IMU imu;

    RobotVision robotVision;
    DecimalFormat nf2 = new DecimalFormat("#.##");
    //Servo ;
    //DigitalChannel ;
    //Rev2mDistanceSensor ;
    RobotProfile profile;

    boolean isDelivered;

    public void init(HardwareMap hardwareMap, RobotProfile profile) {
        Logger.logFile("RobotHardware init()");
        this.hardwareMap = hardwareMap;
        this.profile = profile;
        expansionHub1 = hardwareMap.get(LynxModule.class, "Control Hub");
        rrMotor = hardwareMap.dcMotor.get("RRMotor");
        rlMotor = hardwareMap.dcMotor.get("RLMotor");
        frMotor = hardwareMap.dcMotor.get("FRMotor");
        flMotor = hardwareMap.dcMotor.get("FLMotor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        resetImu();

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

        //expansionHub2 = hardwareMap.get(LynxModule.class, "Expansion Hub");

//        led1 = hardwareMap.digitalChannel.get("LED1");
//        led2 = hardwareMap.digitalChannel.get("LED2");
//        initLeds();
        // Use manual cache mode for most efficiency, but each program
        // needs to call clearBulkCache() in the while loop
        expansionHub1.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        //expansionHub2.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        //TODO
//        DriveConstants.kA = profile.rrFeedForwardParam.kA;
//        DriveConstants.kV = profile.rrFeedForwardParam.kV;
//        DriveConstants.kStatic = profile.rrFeedForwardParam.kStatic;
//        SampleMecanumDrive.HEADING_PID = new PIDCoefficients(profile.rrHeadingPID.p,profile.rrHeadingPID.i,profile.rrHeadingPID.d);
//        SampleMecanumDrive.TRANSLATIONAL_PID = new PIDCoefficients(profile.rrTranslationPID.p,profile.rrTranslationPID.i,profile.rrTranslationPID.d);
        mecanumDrive = new NBMecanumDrive(hardwareMap, profile);
        //mecanumDrive.setLocalizer(realSenseLocalizer);
        robotVision = new RobotVision(this, profile);
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public RobotVision getRobotVision() {
        return robotVision;
    }

    public void initLeds() {
//        led1.setMode(DigitalChannel.Mode.OUTPUT);
//        led2.setMode(DigitalChannel.Mode.OUTPUT);
//        led1.setState(true);
//        led2.setState(true);
    }

    public MecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }

    public Localizer getLocalizer() {
        return mecanumDrive.getLocalizer();
    }

//    public Localizer getLocalizer(){
//        return realSenseLocalizer;
//    }
//
//    public void getBulkData1() {
//        bulkData1 = expansionHub1.getBulkInputData();
//    }
//
//    public void getBulkData2() {
//        bulkData2 = expansionHub2.getBulkInputData();
//    }

    public int getEncoderCounts(EncoderType encoder) {
        if (EncoderType.RIGHT == encoder) {
            return rrMotor.getCurrentPosition();
        }
        else if (EncoderType.LEFT == encoder) {
            return rlMotor.getCurrentPosition();
        }
        else if (EncoderType.HORIZONTAL == encoder) {
            return frMotor.getCurrentPosition();
        }
        return 0;
    }

    public int getEncoderVelocity(EncoderType encoder) {
//        if(encoder == EncoderType.LIFT) {
//            return bulkData2.getMotorVelocity(liftMotor);
//        }
//        else if (encoder == EncoderType.INTAKE){
//            return bulkData2.getMotorVelocity(intakeMotor);
//        }
//        else {
//            return 0;
//        }
        return 0;
    }

    public void mecanumDriveTest(double power, double angle, double rotation, int sign){
        double frontLeft = 0;
        double frontRight = 0;
        double rearLeft = 0;
        double rearRight = 0;
        //10/28/2019, Will, Ian Athena implemented and tested the drive method
        //double robotAngle = Math.PI / 2 - angle - Math.PI / 4;
        if(sign == 0) {
            frontLeft = power * Math.cos(angle) + rotation;
            frontRight = power * Math.sin(angle) - rotation;
            rearLeft = power * Math.sin(angle) + rotation;
            rearRight = power * Math.cos(angle) - rotation;
        }
        else if(sign ==1){  //left side less encoder counts
            frontLeft = power *0.95 * Math.cos(angle) + rotation;
            frontRight = power * Math.sin(angle) - rotation;
            rearLeft = power * 0.95 * Math.sin(angle) + rotation;
            rearRight = power * Math.cos(angle) - rotation;
        }
        else if(sign == 2){   //right side less encoder counts
            frontLeft = power * Math.cos(angle) + rotation;
            frontRight = power * 0.95 * Math.sin(angle) - rotation;
            rearLeft = power * Math.sin(angle) + rotation;
            rearRight = power * 0.95 * Math.cos(angle) - rotation;
        }

        double biggest = 0.1;
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

        power = (power == 0 && rotation !=0) ? 1 : power;
        frontLeft = frontLeft/biggest*power;
        frontRight = frontRight/biggest*power;
        rearLeft = rearLeft/biggest*power;
        rearRight = rearRight/biggest*power;
//        Logger.logFile("Power - FL" + nf2.format(frontLeft) + " FR:"+ nf2.format(frontRight) +
//                        " RL:" + nf2.format(rearLeft) + " RR:" + nf2.format(rearRight));
        setMotorPower(frontLeft, frontRight, rearLeft, rearRight);
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

        setMotorPower(frontLeft, frontRight, rearLeft, rearRight);
    }

    public void setMotorPower(double flPower, double frPower, double rlPower, double rrPower) {
        try {
            flMotor.setPower(flPower);
            frMotor.setPower(frPower);
            rlMotor.setPower(rlPower);
            rrMotor.setPower(rrPower);
        }
        catch (Exception ex) {
            // nothing to handle
        }
    }

    public void setMotorStopBrake(boolean brake) {
        flMotor.setZeroPowerBehavior(brake?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
        frMotor.setZeroPowerBehavior(brake?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
        rlMotor.setZeroPowerBehavior(brake?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
        rrMotor.setZeroPowerBehavior(brake?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void clearBulkCache() {
        expansionHub1.clearBulkCache();
        //expansionHub2.clearBulkCache();
    }


    public void stopAll() {
        setMotorPower(0, 0, 0, 0);
        liftMotor.setPower(0);
        intakeMotor.setPower(0);
        duckMotor.setPower(0);
    }

    public enum EncoderType {LEFT, RIGHT, HORIZONTAL}

//    public void setLed1(boolean on) {
//        led1.setState(!on);
//    }
//
//    public void setLed2(boolean on) {
//        led2.setState(!on);
//    }


    public double getImuHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public void resetImu() {
        Logger.logFile("Resetting IMU");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
        //If Robot Controller is vertically placed, uncomment the following line
        if (profile.hardwareSpec.revHubVertical) {
            BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
        }
    }

    RobotProfile getRobotProfile() {
        return profile;
    }
}
