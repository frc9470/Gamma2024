package com.team9470.subsystems.arm;

import com.team9470.TunableNumber;

public record ArmConfiguration(
        int motorId,
        int encoderPort,
        double ffG,
        double absoluteOffset,
        TunableNumber pidP,
        TunableNumber pidD,
        TunableNumber maxVelocity,
        TunableNumber maxAccel,
        double encoderRatio
) {
    public ArmConfiguration(String name, int motorId, int encoderPort, double ffG, double absoluteOffset, double encoderRatio, double PID_P) {
        this(
                motorId,
                encoderPort,
                ffG,
                absoluteOffset,
                new TunableNumber(name + "/PID_P", PID_P, true),
                new TunableNumber(name + "/PID_D", 0.5, true),
                new TunableNumber(name + "/Max_Velocity", 5.0, true),
                new TunableNumber(name + "/Max_Accel", 15.0, true),
                encoderRatio
        );
    }
}