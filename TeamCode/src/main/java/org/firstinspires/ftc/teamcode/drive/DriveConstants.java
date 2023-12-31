package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


/*
 * Constants shared between multiple drive types.
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 * fields may also be edited through the dashboard (connect to the robot's WiFi network and
 * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */
@Config
public class DriveConstants {

    /*
     * These are motor constants that should be listed online for your motors.
     */
    public static final double TICKS_PER_REV = 2000; //385
    public static final double MAX_RPM = 312; //435

    /*
     * Set the first flag appropriately. If using the built-in motor velocity PID, update
     * MOTOR_VELO_PID with the tuned coefficients from DriveVelocityPIDTuner.
     */
    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID;

    public static double LATERAL_MULTIPLIER = 60.25/60;
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;
    static {
        MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
                getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));
    }

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public static double WHEEL_RADIUS = 0.945; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 14; // 1.79in
    public static double WHEEL_BASE = 12;

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public static double kV = 0.017; //1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0.00001;
    public static double kStatic = 0.085;

    //test by BL
    public static double MAX_ACCEL = 20;
    public static double MAX_VEL = 20;   //need to verify about 30 : MAX_VEL = rpmToVelocity(MAX_RPM); AutomaticFeedforwardTuner.java
    public static double kP = 10;
    public static double kI = 0.5;
    public static double kD = 0.1;
    public static double MAX_ANG_VEL =Math.toRadians(60); //; 3.41
    public static double MAX_ANG_ACCEL = Math.toRadians(60);
//    public static final PIDCoefficients MOTOR_VELO_PID = null;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. The velocity and
     * acceleration values are required, and the jerk values are optional (setting a jerk of 0.0
     * forces acceleration-limited profiling). All distance units are inches.
     */
//    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
//            MAX_VEL, MAX_ACCEL, 0.0,
//            Math.toRadians(180.0), Math.toRadians(180.0), 0.0
//    );


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        //return 32767 * 60.0 / (MAX_RPM * TICKS_PER_REV);
        return 32767 / ticksPerSecond;
    }
}
