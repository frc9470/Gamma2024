package com.team9470;

import com.pathplanner.lib.util.PIDConstants;
import com.team9470.shooter.ShotParameters;
import com.team9470.subsystems.arm.ArmConfiguration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class Consts {
    public static class IndexerConstants{
        public static final int BOTTOM_ROLLER_ID = 19;
        public static final int TOP_ROLLER_ID = 20;
        public static final int BEAM_BREAK_ID = 2;
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
        public static final double TOLERANCE = 50;
        public static final ShotParameters SUBWOOFER = new ShotParameters(1.1, 5400, 1.3, new Rotation2d()); // ignore rotation
        public static final ShotParameters PODIUM = new ShotParameters(2.63, 5400, 0.59, new Rotation2d());
        public static final ShotParameters PODIUM_SIDE = new ShotParameters(2.97, 5400, 1, new Rotation2d());
        public static final ShotParameters AMP = null;
    }

    public static class IntakeConstants{
        public static final int INTAKE_ROLLER_ID = 21;
        public static final boolean IS_TUNING = false;
        public static final TunableNumber INTAKE_TAKE_IN_VOLTAGE = new TunableNumber("Intake/Roller_Voltage", 6.0, IS_TUNING);
    }

    public static class HoodConstants{
        public static final double TOLERANCE = 0.01;

        public static final boolean IS_TUNING = false;
        public static final ArmConfiguration HOOD = new ArmConfiguration(
                "Hood",
                16,  // motorId
                1,   // encoderPort
                0.64, // ffG
                0.64+.42,   // absoluteOffset
                1.0,  // encoderRatio (assuming no ratio for hood)
                10);

        public static final double AMP_POS = 1.35;
        public static final double STEADY_POS = 1;
    }

    public static class SwerveConstants {
        public static final double MAX_SPEED = Units.feetToMeters(16.6); // TODO: is 80% of free speed correct?
        public static final double TOLERANCE = 2.5; // degrees
        public static final double DEADBAND = 0.1;
    }

    public static class AutonConstants {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(1.2, 0, 0);
        public static final PIDConstants ROTATION_PID = new PIDConstants(1.2, 0, 0);
    }

    public static class VisionConstants {
        public static final Transform3d FRONT_LEFT_CAMERA_OFFSET = new Transform3d(Units.inchesToMeters(9.611), Units.inchesToMeters(9.481),Units.inchesToMeters(7.495), new Rotation3d(3.14, 0.49, 0.52));
        public static final Transform3d FRONT_RIGHT_CAMERA_OFFSET = new Transform3d(9.611, -9.481,7.495, new Rotation3d(0, 0.49, -0.52));
        public static final Transform3d BACK_CAMERA_OFFSET = new Transform3d(9.611, 0,0, new Rotation3d(0, 0, 0));


    }

    public static class AmpevatorConstants {
        public static final int ID_1 = 14;
        public static final int ID_2 = 15;
        public static final boolean INVERTED = false;
        public static final int ROLLER_ID = 16;

        public static final double KP = .1;
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        public static final double VELOCITY_LIMIT = .1; //(m/s)
        public static final double ACCELERATION_LIMIT = .3; //(m/s)

        public static final double KS = 0.0;
        public static final double KG = 0.26;
        public static final double KV = 7.01; // v/(m/s)
        public static final double KA = 0.03; // v/(m/s^2)


        public static final double RATIO_MPR = Units.inchesToMeters(9/72.0 * 22.0*.25);
        public static final double EXTENSION_HEIGHT = Units.inchesToMeters(18.7);
        public static final double HOMING_SPEED = -0.5;

        public static final double TOLERANCE = 0.01; // 1cm

        public static final int BEAM_BREAK_PORT = 3;

        // targets
        public static final double AMP = Units.inchesToMeters(16);
        public static final double TRAP = Units.inchesToMeters(18.5);
        public static final double ROLLER_SPEED = 12.0; // volts
    }

    public static class ClimberConstants {
        public static final int ID_1 = 14;
        public static final int ID_2 = 15;


    }

}
