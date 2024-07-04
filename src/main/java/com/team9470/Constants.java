package com.team9470;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static class IndexerConstants{
        public static final int INDEX_ID_1 = 19;
        public static final int INDEX_ID_2 = 20;
        public static final int BEAM_BREAK_ID = 3;
        public static final float BELT_FORWARD_VOLTAGE = 6;
        public static final float BELT_BACKWARD_VOLTAGE = -6;

    }
    public static class ClimberConstants{
        public static final int WINCH_ID_1 = 21;
        public static final int WINCH_ID_2 = 22;
        public static final float CLIMBER_UP_VOLTAGE = 6;
        public static final float CLIMBER_DOWN_VOLTAGE = -12;

    }
    public static class ShooterConstants{
        public static final int SHOOT_ID_1 = 17;
        public static final int SHOOT_ID_2 = 18;
        public static final int SHOOTER_PIVOT_ID_1 = 16;
        public static final float SHOOTER_SHOOT_VOLTAGE = 6;
        public static final float SHOOTER_RATIO = 24.0f/15;
        public static final float FF_S = 0;
        public static final float FF_V = 0;
        public static final float PID_P = 0;
        public static final float TOLERANCE = 0;
    }
    public static class IntakeConstants{
        public static final int INTAKE_ROLLER_ID = 15;
        public static final int INTAKE_ARM_ID = 14;
        public static final int THROUGH_BORE = 1;
        public static final float INTAKE_TAKE_IN_VOLTAGE = 6.0f;
        public static final float INTAKE_RATIO = 32.0f/18;
        public static final float FF_S = 0;
        public static final float FF_V = 0;
        public static final float PID_P = 0;
        public static final float MAX_VELOCITY = 0;
        public static final float MAX_ACCELERATION = 0;
        public static final float ABSOLUTE_OFFSET = 0;
        public static final float UP_GOAL = 5;
        public static final float DOWN_GOAL = 0;
    }
    public static class HoodConstants{
        public static final int HOOD_ID = 16;
        public static final int THROUGH_BORE = 1;
        public static final float FF_S = 0;
        public static final float FF_V = 0;
        public static final float PID_P = 0;
        public static final float MAX_VELOCITY = 0;
        public static final float MAX_ACCELERATION = 0;
        public static final float ABSOLUTE_OFFSET = 0;
        public static final float UP_GOAL = 5;
        public static final float DOWN_GOAL = 0;
    }

    public static class SwerveConstants {
        public static final double MAX_SPEED = Units.feetToMeters(16.6 * .8); // TODO: is 80% of free speed correct?
    }
}
