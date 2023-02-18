package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class LiftResetTask implements RobotControl {
    enum Mode { DOWN, UP1, UP2, DONE };
    Mode mode;
    RobotHardware robotHardware;
    long startTime;
    RobotProfile profile;

    public LiftResetTask(RobotHardware hardware, RobotProfile profile) {
        this.robotHardware = hardware;
        this.profile = profile;
    }

    public String toString() {
        return "Lift reset task";
    }

    @Override
    public void prepare() {
        Logger.logFile("Resetting Lift Position");
        startTime = System.currentTimeMillis();
        robotHardware.setExtensionPosition(profile.hardwareSpec.extensionDriverMin);
        robotHardware.setLiftPower(-0.3);
        mode = Mode.DOWN;
    }

    @Override
    public void execute() {
        if (mode==Mode.DOWN) {
            if (robotHardware.isLiftTouched()) {
                robotHardware.resetLiftPos();
                robotHardware.setLiftPositionUnsafe(0, 0.8);
                startTime = System.currentTimeMillis();
                mode = Mode.UP1;
                robotHardware.turnOnLight(true);
                // use motor 1, 2 to hold position, lift motor 0 for tension
                robotHardware.getLiftMotors()[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robotHardware.getLiftMotors()[0].setPower(0.2);
            }
        }
        if (mode==Mode.UP1) {
            if (System.currentTimeMillis() - startTime > 500) {
                // use motor 0 to hold position, lift motor 1, 2 for tension
                robotHardware.getLiftMotors()[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robotHardware.getLiftMotors()[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robotHardware.getLiftMotors()[0].setPower(0.8);
                robotHardware.getLiftMotors()[0].setTargetPosition(0);
                robotHardware.getLiftMotors()[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robotHardware.getLiftMotors()[1].setPower(0.1);
                robotHardware.getLiftMotors()[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robotHardware.getLiftMotors()[2].setPower(0.1);
                mode = Mode.UP2;
            }
        }
        else if (mode==Mode.UP2) {
            if (System.currentTimeMillis() - startTime > 1000) {
                robotHardware.resetLiftPos();
                robotHardware.setLiftPositionUnsafe(0, 0.5);
                robotHardware.turnOnLight(false);
                mode = Mode.DONE;
            }
        }
    }

    @Override
    public void cleanUp() {
    }

    @Override
    public boolean isDone() {
        return mode==Mode.DONE || (System.currentTimeMillis()-startTime)>5000;
    }
}