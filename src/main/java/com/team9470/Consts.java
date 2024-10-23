package com.team9470;

import com.pathplanner.lib.util.PIDConstants;
import com.team9470.shooter.ShotParameters;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class Consts {
    public static class IndexerConstants{
        public static final int BOTTOM_ROLLER_ID = 14;
        public static final int TOP_ROLLER_ID = 15;
        public static final int BEAM_BREAK_ID = 1;
        public static final double FORWARD_VOLTAGE = 6;
        public static final double BELT_MAX_FORWARD_VOLTAGE = 12;
        public static final double BELT_BACKWARD_VOLTAGE = -6;

    }

    public static class ShooterConstants{
        public static final int ID_TOP = 17;
        public static final int ID_BOTTOM = 18;
        public static final double SHOOTER_RATIO = 24.0f/15;
        public static final double FF_S = 0.15681;
        public static final double FF_V = 0.0013954;
        public static final double FF_A = 0.00052151;
        public static final double FF2_S = 0.099739;
        public static final double FF2_V = 0.0013913;
        public static final double FF2_A = 0.0005497;
        public static final TunableNumber PID_P = new TunableNumber("Shooter/PID_P", 0.0009259259259, true);
        public static final double TOLERANCE = 100;
        public static final ShotParameters SUBWOOFER = new ShotParameters(1.1, 0, 45, new Rotation2d()); // ignore rotation
        public static final ShotParameters PODIUM = new ShotParameters(2.63, 5400, 40, new Rotation2d());
        public static final ShotParameters PODIUM_SIDE = new ShotParameters(2.97, 5400, 35, new Rotation2d());
        public static final ShotParameters AMP = null;
    }

    public static class IntakeConstants{
        public static final int INTAKE_ROLLER_ID = 22;
        public static final double INTAKE_TAKE_IN_VOLTAGE = 8.0;
    }

    /**
     * Constants for the Hood subsystem.
     */
    public static class HoodConstants {
        // Motor and encoder configuration
        public static final int MOTOR_ID = 16;
        public static final int ENCODER_PORT = 0;

        // Feedforward constants
        public static final double FF_G = 0.47;
        public static final double FF_V = 0;
        public static final double ABSOLUTE_OFFSET = 5.84;
        public static final double ENCODER_RATIO = 1.0;

        // PID Tuning
        public static final TunableNumber PID_P = new TunableNumber("Hood/PID_P", 0.3, true);
        public static final TunableNumber PID_D = new TunableNumber("Hood/PID_D", 0.01, true);

        // Motion constraints
        public static final TunableNumber MAX_VELOCITY = new TunableNumber("Hood/Max_Velocity", 75, true);
        public static final TunableNumber MAX_ACCEL = new TunableNumber("Hood/Max_Accel", 100, true);

        // Angle limits in degrees
        public static final double MIN_ANGLE_DEGREES = 14;
        public static final double MAX_ANGLE_DEGREES = 45;
        public static final double TOLERANCE_DEGREES = 0.5;
    }


    public static class SwerveConstants {
        public static final double MAX_SPEED = Units.feetToMeters(16.6); // TODO: is 80% of free speed correct?
        public static final double TOLERANCE = 1; // degrees
        public static final double DEADBAND = 0.1;
    }

    public static class AutonConstants {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(1.2, 0, 0);
        public static final PIDConstants ROTATION_PID = new PIDConstants(2.2, 0, 0);
    }

    public static class VisionConstants {
        public static final Transform3d FRONT_LEFT_CAMERA_OFFSET = new Transform3d(Units.inchesToMeters(-6.416074), Units.inchesToMeters(+6.234908), Units.inchesToMeters(24.876993),
                new Rotation3d(-0.12384803489944651, -0.3272010156831353, 0.3695356336033198));
        public static final Transform3d FRONT_RIGHT_CAMERA_OFFSET = new Transform3d(Units.inchesToMeters(-6.416074), Units.inchesToMeters(-6.234908), Units.inchesToMeters(24.876993),
                new Rotation3d(0.12384803489944651, -0.3272010156831353, -0.3695356336033198));


    }

    public static class AmpevatorConstants {
        public static final int ID_1 = 20;
        public static final int ID_2 = 21;
        public static final boolean INVERTED = true;
        public static final int ROLLER_ID = 19;

        public static final double KP = 5;
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        public static final double VELOCITY_LIMIT = .6; //(m/s)
        public static final double ACCELERATION_LIMIT = 2.0; //(m/s)

        public static final double KS = 0.0;
        public static final double KG = 0.22;
        public static final double KV = 7.01; // v/(m/s)
        public static final double KA = 0.03; // v/(m/s^2)


        public static final double RATIO_MPR = Units.inchesToMeters(9/72.0 * 22.0*.25);
        public static final double EXTENSION_HEIGHT = Units.inchesToMeters(18.7);
        public static final double HOMING_SPEED = -0.5;

        public static final double TOLERANCE = 0.01; // 1cm

        public static final int BEAM_BREAK_PORT = 3;

        // targets
        public static final double AMP = Units.inchesToMeters(12);
        public static final double TRAP = Units.inchesToMeters(18.5);
        public static final double ROLLER_SPEED = 12.0; // volts
    }

    public static class ClimberConstants {
        public static final int ID_1 = 23;
        public static final int ID_2 = 24;
        public static final boolean INVERTED = true;

        public static final double RELEASE_VOLTAGE = -12.0;
        public static final double CLIMB_VOLTAGE = 12.0;

    }

}
