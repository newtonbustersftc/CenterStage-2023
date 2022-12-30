package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorKLNavxMicro;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;
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
    private DcMotorEx turretMotor;
    private DcMotorEx[] liftMotors;        // make it private so we can prevent mistakes by lift down while arm is retracted in
    private Servo grabberServo, extensionServo;
    TouchSensor magneticSensor, liftTouch;
    LynxModule expansionHub1;
    LynxModule expansionHub2;
    NBMecanumDrive mecanumDrive;
    //BNO055IMU imu;        // not available on our Expansion Hub
    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;
    double gyroOffset;
    boolean gripOpen = false;
    int turretTargetPos;

    RobotVision robotVision;
    DecimalFormat nf2 = new DecimalFormat("#.##");
    RobotProfile profile;

    public void init(HardwareMap hardwareMap, RobotProfile profile) {
        Logger.logFile("RobotHardware init()");
        this.hardwareMap = hardwareMap;
        this.profile = profile;
        expansionHub1 = hardwareMap.get(LynxModule.class, "Control Hub");
        rrMotor = hardwareMap.get(DcMotorEx.class, "Rear Right");
        rlMotor = hardwareMap.get(DcMotorEx.class, "Rear Left");
        frMotor = hardwareMap.get(DcMotorEx.class, "Front Right");
        flMotor = hardwareMap.get(DcMotorEx.class, "Front Left");
        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        //resetImu();
        resetDriveAndEncoders();
        expansionHub2 = hardwareMap.get(LynxModule.class, "Expansion Hub");
        magneticSensor = hardwareMap.touchSensor.get("Magnetic Sensor");
        liftTouch = hardwareMap.touchSensor.get("Lift Touch");
        extensionServo = hardwareMap.servo.get("Arm Extension");
        grabberServo = hardwareMap.servo.get("Gripper Open/Close");
        liftMotors = new DcMotorEx[3];
        liftMotors[0] = hardwareMap.get(DcMotorEx.class,"Lift Motor1");
        liftMotors[1] = hardwareMap.get(DcMotorEx.class,"Lift Motor2");
        liftMotors[2] = hardwareMap.get(DcMotorEx.class,"Lift Motor3");
        turretMotor = hardwareMap.get(DcMotorEx.class,"Turret Motor");
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope)navxMicro;
        calibrateGyro(null);

        // Use manual cache mode for most efficiency, but each program
        // needs to call clearBulkCache() in the while loop
        expansionHub1.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        //expansionHub1.clearBulkCache();
        expansionHub2.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        mecanumDrive = new NBMecanumDrive(this, profile);
        //mecanumDrive.setLocalizer(realSenseLocalizer);
        robotVision = new RobotVision(this, profile);
    }

    public List<DcMotorEx> getDriveMotors() {
        return Arrays.asList(flMotor, rlMotor, rrMotor, frMotor);
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

    public int getTargetLiftPosition() {
        return liftMotors[0].getTargetPosition();
    }

    public boolean isLiftMoving() {
        return Math.abs(liftMotors[0].getVelocity())>10;
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
        turretMotor.setPower(0);
    }

    public enum EncoderType {LEFT, RIGHT, HORIZONTAL}

    public double getGyroHeading() {
//      return imu.getAngularOrientation().firstAngle;
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return angles.firstAngle - gyroOffset;
    }

    public double getGyroVelocity() {
//        AngularVelocity angles = imu.getAngularVelocity();
//        return angles.zRotationRate;
        AngularVelocity angles = gyro.getAngularVelocity(AngleUnit.RADIANS);
        return angles.zRotationRate;
    }

    public void resetImu() {
//        Logger.logFile("Resetting Built-in IMU");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.mode = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled = true;
//        boolean result = imu.initialize(parameters);
//        //If Robot Controller is vertically placed, uncomment the following line
//        if (profile.hardwareSpec.revHubVertical) {
//            BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
//        }
//        Logger.logFile("Reset Built-in IMU " + result);
        gyroOffset = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    public void calibrateGyro(Telemetry telemetry) {
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
        Logger.logFile("Gyro calibrated in "+ Math.round(timer.seconds()));
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

    public void resetTurretPos() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setPower(profile.hardwareSpec.turretPower);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTurretPosition(int pos) {
        turretMotor.setTargetPosition(pos);
        turretMotor.setPower(profile.hardwareSpec.turretPower);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void turnTurret(double stickPos) {
        int currPos = getTurretPosition();
        if (Math.abs(stickPos) < 0.1) {
            turretTargetPos = currPos;
        }
        else {  // rotate to target updated, but not allow to target too far away
            turretTargetPos += (int) (stickPos * profile.hardwareSpec.turretMoveMax);
            int maxAhead = (int)Math.abs(stickPos * profile.hardwareSpec.turretMaxAhead);
            if (turretTargetPos > currPos) {
                turretTargetPos = Math.max(turretTargetPos, currPos + maxAhead);
            } else {
                turretTargetPos = Math.min(turretTargetPos, currPos - maxAhead);
            }
        }
        setTurretPosition(turretTargetPos);
    }

    public boolean isTurretTurning() {
        return Math.abs(turretMotor.getVelocity())>10;
    }

    public int getTurretPosition() {
        return turretMotor.getCurrentPosition();
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
    }

    public void grabberClose() {
        gripOpen = false;
        grabberServo.setPosition(profile.hardwareSpec.grabberClosePos);
    }

    public boolean isGripOpen() {
        return gripOpen;
    }

    public void grabberInit() {
        gripOpen = true;
        grabberServo.setPosition(profile.hardwareSpec.grabberInitPos);
    }

    public void initSetupNoAuto(OpMode opmod) {
        extensionServo.setPosition(profile.hardwareSpec.extensionDriverMin);
        grabberClose();
        int tu = getTurretPosition();
        if (isMagneticTouched()) {
            setTurretPosition(tu - profile.hardwareSpec.turretOffset);
            try {
                Thread.sleep(500);
            }
            catch (Exception ex) {
            }
        }
        else {
            // fast rotate first
            while (!isMagneticTouched()) {
                tu = getTurretPosition();
                setTurretPosition(tu + 50);
                try {
                    Thread.sleep(5);
                }
                catch (Exception ex) {
                }
            }
            //rotate back
            tu = getTurretPosition();
            setTurretPosition(tu - profile.hardwareSpec.turretOffset);
            try {
                Thread.sleep(500);
            }
            catch (Exception ex) {
            }
        }
        while (!isMagneticTouched()) {
            tu = getTurretPosition();
            setTurretPosition(tu + 5);
            try {
                Thread.sleep(5);
            }
            catch (Exception ex) {
            }
        }
        tu = getTurretPosition();
        setTurretPosition(tu - profile.hardwareSpec.turretOffset);
        try {
            Thread.sleep(500);
        }
        catch (Exception ex) {
        }
        resetTurretPos();
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
        waitforUp(opmode, "Press UP to start auto init...");
        if (opmode.isStopRequested()) return;
        extensionServo.setPosition(profile.hardwareSpec.extensionDriverMin);
        grabberClose();
        // make sure magnetic is not touched
        int tu = getTurretPosition();
        if (isMagneticTouched()) {
            setTurretPosition(tu - profile.hardwareSpec.turretOffset);
            try {
                Thread.sleep(500);
            }
            catch (Exception ex) {
            }
        }
        else {
            // fast rotate first
            while (!isMagneticTouched() && !opmode.isStopRequested()) {
                tu = getTurretPosition();
                opmode.telemetry.addData("Tullett position", tu);
                opmode.telemetry.update();
                setTurretPosition(tu + 50);
                try {
                    Thread.sleep(5);
                }
                catch (Exception ex) {
                }
            }
            //rotate back
            tu = getTurretPosition();
            setTurretPosition(tu - profile.hardwareSpec.turretOffset);
            try {
                Thread.sleep(500);
            }
            catch (Exception ex) {
            }
        }
        opmode.telemetry.clearAll();
        while (!isMagneticTouched() && !opmode.isStopRequested()) {
            tu = getTurretPosition();
            opmode.telemetry.addData("Tullett position", tu);
            opmode.telemetry.update();
            setTurretPosition(tu + 5);
            try {
                Thread.sleep(5);
            }
            catch (Exception ex) {
            }
        }
        if (opmode.isStopRequested()) return;
        tu = getTurretPosition();
        setTurretPosition(tu - profile.hardwareSpec.turretOffset);
        try {
            Thread.sleep(500);
        }
        catch (Exception ex) {
        }
        resetTurretPos();
        if (opmode.isStopRequested()) return;
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
        while (isLiftTouched() && !opmode.isStopRequested()) {
            i = i+1;
            setLiftPosition(i);
            try {
                Thread.sleep(10);
            }
            catch (Exception ex) {
            }
        }
        resetLiftPos();
        if (opmode.isStopRequested()) return;
        extensionServo.setPosition(profile.hardwareSpec.extensionInitPos);
        grabberInit();
    }

    void waitforUp(LinearOpMode opmode, String text) {
        opmode.telemetry.clearAll();
        opmode.telemetry.addLine(text);
        opmode.telemetry.update();
        // wait until dpad-up pressed
        while (!opmode.isStopRequested() && !opmode.gamepad1.dpad_up) {
            opmode.sleep(50);
        }
        // wait until dpad-up released
        while (!opmode.isStopRequested() && opmode.gamepad1.dpad_up) {
            opmode.sleep(50);
        }
    }
}