package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotProfile;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

public class FourStandardTrackingWheelLocalizer extends FourTrackingWheelLocalizer {
    //public static double TICKS_PER_REV = 4000;
    //public static double WHEEL_RADIUS = 1.496/2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    //public static double LATERAL_DISTANCE = 16.14; // in; distance between the left and right wheels
    //public static double FORWARD_OFFSET = 0; // in; offset of the lateral wheel

    public Encoder leftEncoder, rightEncoder, frontEncoder, backEncoder;
    RobotProfile profile;

    public FourStandardTrackingWheelLocalizer(HardwareMap hardwareMap, RobotProfile profile) {
        super(Arrays.asList(
                new Pose2d(profile.hardwareSpec.encoderOffset, profile.hardwareSpec.trackWidth / 2, 0), // left
                new Pose2d(profile.hardwareSpec.encoderOffset, -profile.hardwareSpec.trackWidth / 2, 0), // right
                new Pose2d(profile.hardwareSpec.forwardOffset, profile.hardwareSpec.encoderOffset, Math.toRadians(90)), // front
                new Pose2d(-profile.hardwareSpec.forwardOffset, profile.hardwareSpec.encoderOffset, -Math.toRadians(90)) // back
        ));
        this.profile = profile;

       leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "Rear Left"));
       rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "Front Right"));
       frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "Front Left"));
       backEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "Rear Right"));
//        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "RLMotor"));
//        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "RRMotor"));
//        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FRMotor"));
        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        if (profile.hardwareSpec.leftEncoderReverse) {
            leftEncoder.setDirection(Encoder.Direction.REVERSE);
        }
        if (profile.hardwareSpec.rightEncoderReverse) {
            rightEncoder.setDirection(Encoder.Direction.REVERSE);
        }
        if (profile.hardwareSpec.horizontalEncoderReverse) {
            frontEncoder.setDirection(Encoder.Direction.REVERSE);
        }
        if (profile.hardwareSpec.horizontalEncoderReverse) { //this might be wrong, might have to create a new horizontal
            backEncoder.setDirection(Encoder.Direction.REVERSE);
        }
    }

    public double encoderTicksToInches(double ticks) {
        return  profile.hardwareSpec.wheelRadius * 2 * Math.PI * GEAR_RATIO * ticks / profile.hardwareSpec.ticksPerRev;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(frontEncoder.getCurrentPosition()),
                encoderTicksToInches(backEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getRawVelocity()),
                encoderTicksToInches(rightEncoder.getRawVelocity()),
                encoderTicksToInches(frontEncoder.getRawVelocity()),
                encoderTicksToInches(backEncoder.getRawVelocity())
        );
    }
}
