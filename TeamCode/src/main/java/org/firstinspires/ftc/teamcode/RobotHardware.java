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
    private DcMotorEx liftMotor;        // make it private so we can prevent mistakes by lift down while arm is retracted in
    private Servo grabberServo, extensionServo;
    DigitalChannel liftBottom;
    TouchSensor magneticSensor, liftTouch;
    LynxModule expansionHub1;
    LynxModule expansionHub2;
    NBMecanumDrive mecanumDrive;
    BNO055IMU imu;
    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;
    double gyroOffset;
    boolean gripOpen = false;
    int turretTargetPos;

    RobotVision robotVision;
    DecimalFormat nf2 = new DecimalFormat("#.##");
    //Servo ;
    //DigitalChannel ;
    //Rev2mDistanceSensor ;
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
        liftMotor = hardwareMap.get(DcMotorEx.class,"Lift Motor");
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
        return liftMotor.getCurrentPosition();
    }

    public void setLiftPosition(int newLiftPos) {
        long currPos = liftMotor.getCurrentPosition();
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
        liftMotor.setTargetPosition(newLiftPos);
        liftMotor.setPower(goUp?profile.hardwareSpec.liftPowerUp:profile.hardwareSpec.liftPowerDown);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean isLiftMoving() {
        return Math.abs(liftMotor.getVelocity())>10;
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

        setMotorPower(-frontLeft, frontRight, -rearLeft, rearRight);        // TO FIX!!!
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
        liftMotor.setPower(0);
        turretMotor.setPower(0);
    }

    public enum EncoderType {LEFT, RIGHT, HORIZONTAL}

    public double getGyroHeading() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return angles.firstAngle - gyroOffset;
        //return imu.getAngularOrientation().firstAngle;
    }

    public double getGyroVelocity() {
        AngularVelocity angles = gyro.getAngularVelocity(AngleUnit.RADIANS);
        //AngularVelocity angles = imu.getAngularVelocity();
        return angles.zRotationRate;
    }

    public void resetImu() {
        gyroOffset = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
//        if (imu!=null) {
//            Logger.logFile("Resetting IMU");
//            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//            parameters.mode = BNO055IMU.SensorMode.IMU;
//            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//            parameters.loggingEnabled = false;
//            imu.initialize(parameters);
//            //If Robot Controller is vertically placed, uncomment the following line
//            if (profile.hardwareSpec.revHubVertical) {
//                BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
//            }
//        }
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

    public void resetLiftPos() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

    public void initSetup(LinearOpMode opmode) {
        waitforUp(opmode, "Press UP to start...");
        if (opmode.isStopRequested()) return;
        extensionServo.setPosition(profile.hardwareSpec.extensionDriverMin);
        waitforUp(opmode, "Move tullet to front, press UP ...");
        if (opmode.isStopRequested()) return;
        extensionServo.setPosition(profile.hardwareSpec.extensionInitPos);
        grabberOpen();
        liftMotor.setPower(0);
        turretMotor.setPower(0);
        waitforUp(opmode, "Down and Center Lift, press UP ...");
        if (opmode.isStopRequested()) return;
        resetLiftPos();
        resetTurretPos();
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