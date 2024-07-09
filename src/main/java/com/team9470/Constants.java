package com.team9470;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static class IndexerConstants{
        public static final int INDEX_ID_1 = 19;
        public static final int INDEX_ID_2 = 20;
        public static final int BEAM_BREAK_ID = 3;
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
        public static final int THROUGH_BORE = 1;
        public static final double INTAKE_TAKE_IN_VOLTAGE = 6.0f;
        public static final double INTAKE_RATIO = 32.0f/18;
        public static final double FF_S = 0;
        public static final double FF_G = 0;
        public static final double FF_V = 0;
        public static final double PID_P = 0;
        public static final double MAX_VELOCITY = 0;
        public static final double ABSOLUTE_OFFSET = 0;
        public static final double UP_GOAL = .8f;
        public static final double DOWN_GOAL = 0;
    }

    public static class HoodConstants{
        public static final int HOOD_ID = 16;
        public static final int THROUGH_BORE = 1;
        public static final double FF_S = 0;
        public static final double FF_G = 0;
        public static final double FF_V = 0;
        public static final double PID_P = 0;
        public static final double MAX_VELOCITY = 0;
        public static final double ABSOLUTE_OFFSET = 0;
        public static final double UP_GOAL = .6f;
        public static final double DOWN_GOAL = 0;
    }

    public static class SwerveConstants {
        public static final double MAX_SPEED = Units.feetToMeters(16.6); // TODO: is 80% of free speed correct?
    }

    public static class AutonConstants {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(1.2, 0, 0);
        public static final PIDConstants ROTATION_PID = new PIDConstants(1.2, 0, 0);
    }

}
