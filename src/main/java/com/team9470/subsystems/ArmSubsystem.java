package com.team9470.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import static com.team9470.Constants.ShooterConstants.*;

public class ArmSubsystem extends ProfiledPIDSubsystem {
    private final CANSparkMax m_motor = new CANSparkMax(SHOOTER_PIVOT_ID_1, CANSparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final ArmFeedforward m_feedforward =
            new ArmFeedforward(
                    K_S, K_G,
                    K_V, K_A);

    /** Create a new ArmSubsystem. */
    public ArmSubsystem() {
        super(
                new ProfiledPIDController(
                        PID_P,
                        0,
                        0,
                        new TrapezoidProfile.Constraints(
                                ArmConstants.kMaxVelocityRadPerSecond,
                                ArmConstants.kMaxAccelerationRadPerSecSquared)),
                0);
        m_encoder.setDistancePerPulse(ArmConstants.kEncoderDistancePerPulse);
        // Start arm at rest in neutral position
        //setGoal(ArmConstants.kArmOffsetRads);
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the sepoint
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        // Add the feedforward to the PID output to get the motor output
        m_motor.setVoltage(output + feedforward);
    }

//    @Override
//    public double getMeasurement() {
//        return m_encoder.getPosition() + ArmConstants.kArmOffsetRads;
//    }
}
