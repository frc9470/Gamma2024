package com.team9470.shooter;

import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;

public class RegressionMaps {
    public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> ANGLE_MAP, SPEED_MAP, YAW_MAP;

    static {
        ANGLE_MAP = InterpolatingTreeMap.ofDouble(SpeakerRegression.DIST_TO_ANGLE);
        SPEED_MAP = InterpolatingTreeMap.ofDouble(SpeakerRegression.DIST_TO_SPEED);
        YAW_MAP = InterpolatingTreeMap.ofDouble(SpeakerRegression.DIST_TO_YAW);
    }
}
