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
import com.qualcomm.robotcore.hardware.IMU;
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

import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.List;

public class RobotHardware {
    public double extensionPos;
    HardwareMap hardwareMap;
    DcMotorEx rrMotor, rlMotor, frMotor, flMotor;
    private DcMotorEx[] liftMotors;        // make it private so we can prevent mistakes by lift down while arm is retracted in
    //private
    Servo grabberServo, extensionServo, lightServo, signalBlockerServo;
    TouchSensor magneticSensor, liftTouch;
    NormalizedColorSensor coneSensor;

    LynxModule expansionHub1;
    LynxModule expansionHub2;
    NBMecanumDrive mecanumDrive;
    BNO055IMU imu;
    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;
    double gyroOffset;
    boolean gripOpen = false;

    RobotVision robotVision;
    DecimalFormat nf2 = new DecimalFormat("#.##");
    RobotProfile profile;

    public void init(HardwareMap hardwareMap, RobotProfile profile) {
        Logger.logFile("RobotHardware init()");
        this.hardwareMap = hardwareMap;
        this.profile = profile;
        expansionHub1 = hardwareMap.get(LynxModule.class, "Control Hub");
        rrMotor = hardwareMap.get(DcMotorEx.class, "RRMotor");
        rlMotor = hardwareMap.get(DcMotorEx.class, "RLMotor");
        frMotor = hardwareMap.get(DcMotorEx.class, "FRMotor");
        flMotor = hardwareMap.get(DcMotorEx.class, "FLMotor");
        resetDriveAndEncoders();
        expansionHub2 = hardwareMap.get(LynxModule.class, "Expansion Hub");
        magneticSensor = hardwareMap.touchSensor.get("Magnetic Sensor");
        liftTouch = hardwareMap.touchSensor.get("Lift Touch");
        extensionServo = hardwareMap.servo.get("Arm Extension");
        grabberServo = hardwareMap.servo.get("Gripper Open/Close");
        lightServo = hardwareMap.servo.get("LightControl");
        signalBlockerServo = hardwareMap.servo.get("SignalBlocker");

        liftMotors = new DcMotorEx[3];
        liftMotors[0] = hardwareMap.get(DcMotorEx.class,"Lift Motor1");
        liftMotors[1] = hardwareMap.get(DcMotorEx.class,"Lift Motor2");
        liftMotors[2] = hardwareMap.get(DcMotorEx.class,"Lift Motor3");
        coneSensor = hardwareMap.get(NormalizedColorSensor.class, "ConeDistance");

        // Use manual cache mode for most efficiency, but each program
        // needs to call clearBulkCache() in the while loop
        expansionHub1.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        //expansionHub1.clearBulkCache();
        expansionHub2.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        mecanumDrive = new NBMecanumDrive(this, profile);
        //mecanumDrive.setLocalizer(realSenseLocalizer);
        robotVision = new RobotVision(this, profile);
        initGyro();
        resetImu();
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
        expansionHub1.clearBulkCache();
        //expansionHub2.clearBulkCache();
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public RobotVision getRobotVision() {
        return robotVision;
    }

    public MecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }

    public Localizer getLocalizer() {
        return mecanumDrive.getLocalizer();
    }

    public int getLiftPosition() {
        return liftMotors[0].getCurrentPosition();
    }

    public String getLiftMotorPos() {
        return "Lift123: " + liftMotors[0].getCurrentPosition() + "," + liftMotors[1].getCurrentPosition() + "," + liftMotors[2].getCurrentPosition();
    }

    public double getLiftVelocity() {
        return liftMotors[0].getVelocity();
    }

    public void setLiftPosition(int newLiftPos) {
        long currPos = liftMotors[0].getCurrentPosition();
        boolean goUp = newLiftPos>currPos;
        if (!goUp) {
            if (extensionPos < profile.hardwareSpec.extensionDriverMin && newLiftPos < profile.hardwareSpec.liftSafeRotate) {
                newLiftPos = profile.hardwareSpec.liftSafeRotate;
            }
            else {
                newLiftPos = Math.max(0, newLiftPos);
            }
        }
        else {
            newLiftPos = Math.min(newLiftPos, profile.hardwareSpec.liftMax);
        }
        for(DcMotorEx liftMotor : liftMotors) {
            liftMotor.setTargetPosition(newLiftPos);
            liftMotor.setPower(goUp ? profile.hardwareSpec.liftPowerUp : profile.hardwareSpec.liftPowerDown);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void setLiftPositionUnsafe(int newLiftPos, double power) {
        for(DcMotorEx liftMotor : liftMotors) {
            liftMotor.setTargetPosition(newLiftPos);
            liftMotor.setPower(power);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void setLiftPower(double power) {
        for(DcMotorEx liftMotor : liftMotors) {
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor.setPower(power);
        }
    }

    public int getTargetLiftPosition() {
        return liftMotors[0].getTargetPosition();
    }

    public boolean pickUpCheck(boolean isRed) {
        if (((DistanceSensor)coneSensor).getDistance(DistanceUnit.INCH)>profile.hardwareSpec.coneDistInch) {
            return false;
        }
        NormalizedRGBA rgba = coneSensor.getNormalizedColors();
        double total = rgba.red + rgba.blue + rgba.green;
        double rp = rgba.red/total;
        double bp = rgba.blue/total;
        double gp = rgba.green/total;
        if (isRed && rp>profile.hardwareSpec.mainColorMin && bp<profile.hardwareSpec.otherColorMax && gp<profile.hardwareSpec.otherColorMax) {
            return true;
        }
        if (!isRed && bp>profile.hardwareSpec.mainColorMin && rp<profile.hardwareSpec.otherColorMax && gp<profile.hardwareSpec.otherColorMax) {
            return true;
        }
        return false;
    }

    public void turnOnLight(boolean isOn) {
        double p = (isOn) ? profile.hardwareSpec.lightPower : 0.5;
        lightServo.setPosition(p);
    }

    public boolean isLiftMoving() {
        return Math.abs(liftMotors[0].getVelocity())>10;
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
            // nothing to handle
        }
    }

    public void setMotorStopBrake(boolean brake) {
        flMotor.setZeroPowerBehavior(brake?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
        frMotor.setZeroPowerBehavior(brake?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
        rlMotor.setZeroPowerBehavior(brake?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
        rrMotor.setZeroPowerBehavior(brake?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void enableManualCaching(boolean enable) {
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
        turnUpSignalBlocker();
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

    public boolean isLiftTouched() {return liftTouch.isPressed();}

    public boolean isMagneticTouched() {return magneticSensor.isPressed();}

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

    public void extensionExtend() {
        extensionPos = Math.min(profile.hardwareSpec.extensionFullOutPos, extensionPos + .1);
        extensionServo.setPosition(extensionPos);
    }

    public void extensionRetract() {
        extensionPos = Math.max(profile.hardwareSpec.extensionInitPos, extensionPos - .1);
        extensionServo.setPosition(extensionPos);
    }

    public void setExtensionPosition(double extensionPos) {
        this.extensionPos = Math.max(Math.min(extensionPos, profile.hardwareSpec.extensionFullOutPos),
                profile.hardwareSpec.extensionInitPos);
        extensionServo.setPosition(this.extensionPos);
    }

    public double getExtensionPosition() {
        return extensionPos;
    }

    public void grabberOpen() {
        gripOpen = true;
        grabberServo.setPosition(profile.hardwareSpec.grabberOpenPos);
        turnOffLight();
    }

    public void grabberClose() {
        gripOpen = false;
        grabberServo.setPosition(profile.hardwareSpec.grabberClosePos);
        turnOnLight();
    }

    public void grabberMoveSafe() {
        gripOpen = true;
        grabberServo.setPosition(profile.hardwareSpec.grabberSafePos);
        turnOffLight();
    }

    public boolean isGripOpen() {
        return gripOpen;
    }

    public void grabberInit() {
        gripOpen = true;
        grabberServo.setPosition(profile.hardwareSpec.grabberInitPos);
        turnOffLight();
    }

    public void turnOnLight() {
        lightServo.setPosition(profile.hardwareSpec.lightPower);
    }

    public void turnOffLight() {
        lightServo.setPosition(0.5);
    }

    public void turnDownSignalBlocker(){
        signalBlockerServo.setPosition(profile.hardwareSpec.signalBlockerDown);
    }

    public void turnUpSignalBlocker(){
        signalBlockerServo.setPosition(profile.hardwareSpec.signalBlockerUp);
    }

    public void initSetupNoAuto(OpMode opmod) {
        extensionServo.setPosition(profile.hardwareSpec.extensionDriverMin);
        grabberClose();
        // Goes down to touch first
        setLiftPositionUnsafe(-5000, 0.3);
        long t = System.currentTimeMillis();
        while (!isLiftTouched() && (System.currentTimeMillis()-t)<3000) {
            try {
                Thread.sleep(100);
            }
            catch (Exception ex) {
            }
        }
        resetLiftPos();
        setLiftPosition(0);
        // goes up again until touch no more
        int i = 0;
        while (isLiftTouched()) {
            i = i+1;
            setLiftPosition(i);
            try {
                Thread.sleep(10);
            }
            catch (Exception ex) {
            }
        }
        resetLiftPos();
        extensionServo.setPosition(profile.hardwareSpec.extensionInitPos);
        grabberInit();
    }

    public void initSetup(LinearOpMode opmode) {
        resetDriveAndEncoders();
        opmode.telemetry.addLine("PLEASE MOVE THE ROBOT AROUND");
        boolean done = false;
        int frMin = 0, frMax = 0, rrMin = 0, rrMax = 0, flMin = 0, flMax = 0, rlMin = 0, rlMax = 0;
        int MIN_MAX = 1250;
        while (!opmode.isStopRequested() && !done) {
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
        extensionServo.setPosition(profile.hardwareSpec.extensionDriverMin);
        grabberClose();

        if (opmode.isStopRequested()) return;

        // Lift position reset
        long startTime = System.currentTimeMillis();
        setLiftPower(-0.3);
        while (!isLiftTouched() && (System.currentTimeMillis() - startTime<3000) && !opmode.isStopRequested()) {
            try {
                Thread.sleep(10);
            }
            catch (Exception ex) {
            }
        }
        // continue to hold for half a second
        try {
            Thread.sleep(500);
        }
        catch (Exception ex) {
        }
        // now let's tension for go up
        Logger.logFile("Lift reset up tension");
        resetLiftPos();
        setLiftPositionUnsafe(0, 0.6); // holding
        // tension for lift motor 0, holding position 0 using motor 1,2
        liftMotors[0].setPower(0.2);
        liftMotors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        try {
            Thread.sleep(500);
        }
        catch (Exception ex) {
        }
        // using motor 0 to hold position
        liftMotors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotors[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotors[0].setTargetPosition(0);
        liftMotors[0].setPower(0.8);
        liftMotors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotors[1].setPower(0.1);
        liftMotors[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotors[2].setPower(0.1);
        try {
            Thread.sleep(500);
        }
        catch (Exception ex) {
        }
        resetLiftPos();
        setLiftPositionUnsafe(0, 0.5);

        if (opmode.isStopRequested()) return;
        extensionServo.setPosition(profile.hardwareSpec.extensionInitPos);
        grabberInit();
    }

    void waitforUp(LinearOpMode opmode, String text) {
        opmode.telemetry.clearAll();
        // wait until dpad-up pressed
        while (!opmode.isStopRequested() && !opmode.gamepad1.dpad_up) {
            opmode.telemetry.addLine(text);
            if (Math.abs(frMotor.getCurrentPosition()-rlMotor.getCurrentPosition())>400 ||
                    Math.abs(flMotor.getCurrentPosition()-rrMotor.getCurrentPosition())>400) {
                opmode.telemetry.addLine("******************\nONE OF DEAD WHEEL MAY BE STUCK\n******************");
            }

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
}