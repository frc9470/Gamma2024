package com.team9470;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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

        public static final float K_G = 0;
        public static final float K_S = 0;
        public static final float K_A = 0;
        public static final float K_V = 0;
        public static final float FF_S = 0;
        public static final float FF_V = 0;
        public static final float PID_P = 0;
        public static final float TOLERANCE = 0;
    }
    public static class IntakeConstants{
        public static final int INTAKE_ROLLER_ID = 15;
        public static final int INTAKE_ARM_ID = 14;
        public static final int THROUGH_BORE = 1;
        public static final float INTAKE_UP_VOLTAGE = 6.0f;
        public static final float INTAKE_DOWN_VOLTAGE = 6.0f;
        public static final float INTAKE_TAKE_IN_VOLTAGE = 6.0f;
        public static final float INTAKE_RATIO = 32.0f/18;
        public static final float FF_S = 0;
        public static final float FF_V = 0;
        public static final float PID_P = 0;
        public static final float TOLERANCE = 0;
        public static final float ABSOLUTE_OFFSET = 0;
    }

    public static class SwerveConstants {
        public static final double MAX_SPEED = Units.feetToMeters(16.6 * .8); // TODO: is 80% of free speed correct?
        public static final double TOLERANCE = 2.5;
    }
    
    public static class VisionConstants {
        public static final Transform3d FRONT_LEFT_CAMERA_OFFSET = new Transform3d(9.611, 9.481,7.495,new Rotation3d(0, 0, 0));
        public static final Transform3d FRONT_RIGHT_CAMERA_OFFSET = new Transform3d(9.611, 0,0,new Rotation3d(0, 0, 0));
        public static final Transform3d BACK_CAMERA_OFFSET = new Transform3d(9.611, 0,0,new Rotation3d(0, 0, 0));


    }
}
