package com.team9470;

import com.pathplanner.lib.util.PIDConstants;
import com.team9470.shooter.ShotParameters;
import com.team9470.subsystems.arm.ArmConfiguration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static class IndexerConstants{
        public static final int INDEX_ID_1 = 19;
        public static final int INDEX_ID_2 = 20;
        public static final int BEAM_BREAK_ID = 2;
        public static final double BELT_FORWARD_VOLTAGE = 6;
        public static final double BELT_BACKWARD_VOLTAGE = -6;

    }

    public static class ShooterConstants{
        public static final int ID_TOP = 17;
        public static final int ID_BOTTOM = 18;
        public static final double SHOOTER_RATIO = 24.0f/15;
        public static final double FF_S = 0.29106;
        public static final double FF_V = 0.0014144;
        public static final double FF_A = 0.00065971;
        public static final TunableNumber PID_P = new TunableNumber("Shooter/PID_P", 0.000088945, true);
        public static final double TOLERANCE = 50;
        public static final ShotParameters SUBWOOFER = new ShotParameters(2, 5000, .8, new Rotation2d()); // ignore rotation
        public static final ShotParameters PODIUM = new ShotParameters(FieldLayout.kPodiumX, 6000, .75, new Rotation2d());
        public static final ShotParameters AMP = null;
    }

    public static class IntakeConstants{
        public static final int INTAKE_ROLLER_ID = 15;
        public static final boolean IS_TUNING = false;
        public static final TunableNumber INTAKE_TAKE_IN_VOLTAGE = new TunableNumber("Intake/Roller_Voltage", 6.0, IS_TUNING);

        public static final ArmConfiguration INTAKE_ARM = new ArmConfiguration(
                "IntakeArm",
                14,  // motorId
                0,   // encoderPort
                0.7, // ffG
                .42, // absoluteOffset
                18.0 / 30.0 // encoderRatio
        );

        public static final double UP_GOAL = 1.9;
        public static final double DOWN_GOAL = -.18;
        public static final double INDEXER_GOAL = 2.26;
    }

    public static class HoodConstants{
        public static final double TOLERANCE = 0.05;

        public static final boolean IS_TUNING = false;
        public static final ArmConfiguration HOOD = new ArmConfiguration(
                "Hood",
                16,  // motorId
                1,   // encoderPort
                0.64, // ffG
                0.64,   // absoluteOffset
                1.0  // encoderRatio (assuming no ratio for hood)
        );

        public static final double AMP_POS = 1.3;
        public static final double STEADY_POS = 1;
    }

    public static class SwerveConstants {
        public static final double MAX_SPEED = Units.feetToMeters(16.6); // TODO: is 80% of free speed correct?
        public static final double TOLERANCE = 2.5; // degrees
    }

    public static class AutonConstants {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(1.2, 0, 0);
        public static final PIDConstants ROTATION_PID = new PIDConstants(1.2, 0, 0);
    }

    public static class VisionConstants {
        public static final Transform3d FRONT_LEFT_CAMERA_OFFSET = new Transform3d(9.611, 9.481,7.495, new Rotation3d(0, 28.125, 30));
        public static final Transform3d FRONT_RIGHT_CAMERA_OFFSET = new Transform3d(9.611, -9.481,7.495, new Rotation3d(0, 28.125, -30));
        public static final Transform3d BACK_CAMERA_OFFSET = new Transform3d(9.611, 0,0, new Rotation3d(0, 0, 0));


    }

}
