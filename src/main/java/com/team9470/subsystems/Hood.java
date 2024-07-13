package com.team9470.subsystems;

import com.team9470.subsystems.arm.AbstractArm;
import edu.wpi.first.wpilibj2.command.Command;

import static com.team9470.Constants.HoodConstants.HOOD;
import static com.team9470.Constants.HoodConstants.TOLERANCE;

public class Hood extends AbstractArm {

    private static Hood instance;

    private Hood() {
        super(HOOD);
    }

    public static Hood getInstance() {
        if (instance == null) {
            instance = new Hood();
        }
        return instance;
    }

    public Command waitReady() {
        return new Command() {
            @Override
            public boolean isFinished() {
                return Math.abs(getPosition()-goal) < TOLERANCE;
            }
        };
    }
}
