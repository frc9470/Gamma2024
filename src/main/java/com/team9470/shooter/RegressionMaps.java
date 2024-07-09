package com.team9470.shooter;

import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;

public class RegressionMaps {
    public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> ANGLE_MAP, SPEED_MAP, YAW_MAP;

    static {
        ANGLE_MAP = new InterpolatingTreeMap<>();

        for(int i = 0; i < SpeakerRegression.DIST_TO_ANGLE.length; i++) {
            ANGLE_MAP.put(
                    new InterpolatingDouble(SpeakerRegression.DIST_TO_ANGLE[i][0]),
                    new InterpolatingDouble(SpeakerRegression.DIST_TO_ANGLE[i][1])
            );
        }

        SPEED_MAP = new InterpolatingTreeMap<>();
        for(int i = 0; i < SpeakerRegression.DIST_TO_SPEED.length; i++) {
            SPEED_MAP.put(
                    new InterpolatingDouble(SpeakerRegression.DIST_TO_SPEED[i][0]),
                    new InterpolatingDouble(SpeakerRegression.DIST_TO_SPEED[i][1])
            );
        }

        YAW_MAP = new InterpolatingTreeMap<>();
        for(int i = 0; i < SpeakerRegression.DIST_TO_YAW.length; i++) {
            YAW_MAP.put(
                    new InterpolatingDouble(SpeakerRegression.DIST_TO_YAW[i][0]),
                    new InterpolatingDouble(SpeakerRegression.DIST_TO_YAW[i][1])
            );
        }
    }
}
