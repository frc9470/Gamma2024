package com.team9470;

import com.pathplanner.lib.util.PIDConstants;
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
        public static final int SHOOTER_PIVOT_ID_1 = 16;
        public static final double SHOOTER_SHOOT_VOLTAGE = 6;
        public static final double SHOOTER_RATIO = 24.0f/15;
        public static final double FF_S = 0;
        public static final double FF_V = 0;
        public static final double PID_P = 0;
        public static final double TOLERANCE = 0;
    }

    public static class IntakeConstants{
        public static final int INTAKE_ROLLER_ID = 15;
        public static final int INTAKE_ARM_ID = 14;
        public static final int THROUGH_BORE = 0;
        public static final boolean IS_TUNING = false;
        public static final TunableNumber INTAKE_TAKE_IN_VOLTAGE = new TunableNumber("Intake/Roller_Voltage", 6.0, IS_TUNING);
        public static final double INTAKE_ENCODER_RATIO = 18.0/30; // driving/driven - converts through bore output to intake output

        public static final double FF_G = .7;
        public static final TunableNumber PID_P = new TunableNumber("Intake/PID_P", 5.0, IS_TUNING);
        public static final TunableNumber PID_I = new TunableNumber("Intake/PID_I", 0.0, IS_TUNING);
        public static final TunableNumber PID_D = new TunableNumber("Intake/PID_D", 0.5, IS_TUNING);
        public static final TunableNumber MAX_VELOCITY = new TunableNumber("Intake/Max_Velocity", 5.0, IS_TUNING);
        public static final TunableNumber MAX_ACCEL = new TunableNumber("Intake/Max_Accel", 15.0, IS_TUNING);
        public static final double ABSOLUTE_OFFSET = -0.02538034471-0.02752074068801852-0.02;
        public static final double UP_GOAL = 1.9;
        public static final TunableNumber DOWN_GOAL = new TunableNumber("Intake/DownGoal", -0.18, IS_TUNING);
    }

    public static class HoodConstants{
        public static final int HOOD_ID = 16;
        public static final int THROUGH_BORE = 1;

        public static final boolean IS_TUNING = false;
        public static final double FF_G = .7;
        public static final TunableNumber PID_P = new TunableNumber("Hood/PID_P", 5.0, IS_TUNING);
        public static final TunableNumber PID_I = new TunableNumber("Hood/PID_I", 0.0, IS_TUNING);
        public static final TunableNumber PID_D = new TunableNumber("Hood/PID_D", 0.5, IS_TUNING);
        public static final TunableNumber MAX_VELOCITY = new TunableNumber("Hood/Max_Velocity", 5.0, IS_TUNING);
        public static final TunableNumber MAX_ACCEL = new TunableNumber("Hood/Max_Accel", 15.0, IS_TUNING);
        public static final double ABSOLUTE_OFFSET = -0.02538034471-0.02752074068801852-0.02;
        public static final double UP_GOAL = 1.9;
        public static final TunableNumber DOWN_GOAL = new TunableNumber("Hood/DownGoal", -0.18, IS_TUNING);
    }

    public static class SwerveConstants {
        public static final double MAX_SPEED = Units.feetToMeters(16.6); // TODO: is 80% of free speed correct?
        public static final double TOLERANCE = 2.5;
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
