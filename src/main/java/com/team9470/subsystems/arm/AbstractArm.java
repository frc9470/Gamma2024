package com.team9470.subsystems.arm;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractArm extends SubsystemBase {
    protected final CANSparkMax motor;
    protected final DutyCycleEncoder encoder;
    protected final ArmFeedforward ff;
    protected final ProfiledPIDController pid;
    protected final ArmConfiguration config;

    protected boolean init = true;
    protected double goal = 0;
    protected boolean enabled = true;

    protected AbstractArm(ArmConfiguration config) {
        this.config = config;
        this.motor = new CANSparkMax(config.motorId(), CANSparkMax.MotorType.kBrushless);
        this.encoder = new DutyCycleEncoder(config.encoderPort());
        this.ff = new ArmFeedforward(0, config.ffG(), 0);
        this.pid = new ProfiledPIDController(
                config.pidP().get(), 0, config.pidD().get(),
                new TrapezoidProfile.Constraints(config.maxVelocity().get(), config.maxAccel().get())
        );

        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    protected AbstractArm(ArmConfiguration config, boolean enabled){
        this(config);
        this.enabled = enabled;
    }

    @Override
    public void periodic() {
        if (init) {
            goal = getPosition();
            pid.reset(goal);
            init = false;
        }

        if (config.pidP().hasChanged() || config.pidD().hasChanged() || config.maxVelocity().hasChanged() || config.maxAccel().hasChanged()) {
            pid.setPID(config.pidP().get(), 0, config.pidD().get());
            pid.setConstraints(new TrapezoidProfile.Constraints(config.maxVelocity().get(), config.maxAccel().get()));
        }

        double output = pid.calculate(getPosition(), goal) + ff.calculate(getPosition(), 0);
        if (enabled) motor.setVoltage(output);

        updateSmartDashboard();
    }

    public double getPosition() {
        return (modifyInput(encoder.getAbsolutePosition()) * config.encoderRatio()) * 2 * Math.PI - config.absoluteOffset();
    }

    protected double modifyInput(double input){
        return input;
    }


    public void setGoal(double goal) {
        this.goal = goal;
    }

    protected void updateSmartDashboard() {
        String name = getClass().getSimpleName();
        SmartDashboard.putNumber(name + "/Position", getPosition());
        SmartDashboard.putNumber(name + "/RawEncoderOutput", encoder.getAbsolutePosition());
        SmartDashboard.putNumber(name + "/ArmOutput", motor.getAppliedOutput());
        SmartDashboard.putNumber(name + "/Goal", goal);
        SmartDashboard.putNumber(name + "/Setpoint", pid.getSetpoint().position);
        SmartDashboard.putNumber(name + "/Velocity", pid.getSetpoint().velocity);
    }
}

