package com.team9470.subsystems;

import com.team9470.Util;
import com.team9470.subsystems.arm.AbstractArm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static com.team9470.Consts.HoodConstants.HOOD;
import static com.team9470.Consts.HoodConstants.TOLERANCE;

public class Hood extends AbstractArm {

    private static Hood instance;

    private Hood() {
        super(HOOD, true);
        motor.setInverted(true);
    }

    public static Hood getInstance() {
        if (instance == null) {
            instance = new Hood();
        }
        return instance;
    }

    @Override
    public void periodic() {
        super.periodic();
        goal = Util.clamp(goal, 0.1, 1.5);
    }

    public Command waitReady() {
        return new Command() {
            @Override
            public boolean isFinished() {
                return Math.abs(getPosition()-goal) < TOLERANCE;
            }
        };
    }

    public Command angleCommand(double angle){
        return new InstantCommand(() -> goal = angle);
    }

    @Override
    protected double modifyInput(double input) {
        return 1.0 - input;
    }
}
