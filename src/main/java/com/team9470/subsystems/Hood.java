package com.team9470.subsystems;

import com.revrobotics.CANSparkMax;
import com.team9470.Util;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team9470.Consts.HoodConstants;

/**
 * Controls vertical orientation of shooter, with input/output in degrees.
 */
public class Hood extends SubsystemBase {

    private static Hood instance;

    private final CANSparkMax motor;
    private final DutyCycleEncoder encoder;
    private final ArmFeedforward ff;
    private final ProfiledPIDController pid;

    private boolean init = true;
    private double goalDegrees = 0;
    private boolean enabled = true;

    private Hood() {
        // Initialize motor and encoder with hardcoded IDs and ports
        motor = new CANSparkMax(HoodConstants.MOTOR_ID, CANSparkMax.MotorType.kBrushless);
        encoder = new DutyCycleEncoder(HoodConstants.ENCODER_PORT);
        ff = new ArmFeedforward(0, HoodConstants.FF_G, HoodConstants.FF_V);

        // Set up the PID controller with hardcoded constraints and gains
        pid = new ProfiledPIDController(
                HoodConstants.PID_P.get(), 0, HoodConstants.PID_D.get(),
                new TrapezoidProfile.Constraints(HoodConstants.MAX_VELOCITY.get(), HoodConstants.MAX_ACCEL.get())
        );

        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        motor.setInverted(false);
    }

    public static Hood getInstance() {
        if (instance == null) {
            instance = new Hood();
        }
        return instance;
    }



    @Override
    public void periodic() {
//        if (init) {
//            goalDegrees = getPositionDegrees();
//            pid.reset(goalDegrees);
//            init = false;
//        }

        // Update PID constants if changed
        if (HoodConstants.PID_P.hasChanged() || HoodConstants.PID_D.hasChanged() ||
                HoodConstants.MAX_VELOCITY.hasChanged() || HoodConstants.MAX_ACCEL.hasChanged()) {
            pid.setPID(HoodConstants.PID_P.get(), 0, HoodConstants.PID_D.get());
            pid.setConstraints(new TrapezoidProfile.Constraints(HoodConstants.MAX_VELOCITY.get(), HoodConstants.MAX_ACCEL.get()));
        }

        // Calculate motor output
        if (DriverStation.isEnabled()) {
            double output = pid.calculate(getPositionDegrees(), goalDegrees) +
                    ff.calculate(getPositionRadians() + Math.toRadians(+7.5), pid.getSetpoint().velocity);
            if (enabled) motor.setVoltage(output);
        } else {
            goalDegrees = getPositionDegrees();
            pid.reset(goalDegrees);
        }

        // Clamp goal within a certain range of degrees
        goalDegrees = Util.clamp(goalDegrees, HoodConstants.MIN_ANGLE_DEGREES, HoodConstants.MAX_ANGLE_DEGREES);

        SmartDashboard.putBoolean("Hood/READY", Math.abs(getPositionDegrees() - goalDegrees) < HoodConstants.TOLERANCE_DEGREES);

        updateSmartDashboard();
    }



    /**
     * Get the current position of the encoder in radians.
     */
    public double getPositionRadians() {
        return (modifyInput(encoder.getAbsolutePosition()) * HoodConstants.ENCODER_RATIO) * 2 * Math.PI - HoodConstants.ABSOLUTE_OFFSET / 180 * Math.PI;
    }

    /**
     * Get the current position of the encoder in degrees.
     */
    public double getPositionDegrees() {
        return Math.toDegrees(getPositionRadians());
    }

    private double modifyInput(double input) {
        return input;  // Adjust encoder input as needed
    }

    /**
     * Set the goal in degrees.
     */
    public void setGoalDegrees(double goalDegrees) {
        this.goalDegrees = goalDegrees;
    }

    private void updateSmartDashboard() {
        SmartDashboard.putNumber("Hood/Position (Degrees)", getPositionDegrees());
        SmartDashboard.putNumber("Hood/RawEncoderOutput", encoder.getAbsolutePosition());
        SmartDashboard.putNumber("Hood/ArmOutput", motor.getAppliedOutput());
        SmartDashboard.putNumber("Hood/Goal (Degrees)", goalDegrees);
        SmartDashboard.putNumber("Hood/Setpoint (Degrees)", pid.getSetpoint().position);
        SmartDashboard.putNumber("Hood/Velocity (dps)", pid.getSetpoint().velocity);
    }

    /**
     * Wait until the hood is ready (goal achieved).
     */
    public Command waitReady() {
        return new Command() {
            @Override
            public boolean isFinished() {
                return Math.abs(getPositionDegrees() - goalDegrees) < HoodConstants.TOLERANCE_DEGREES;
            }
        };
    }

    /**
     * Command to set a new angle in degrees.
     */
    public Command angleCommand(double angleDegrees) {
        return new InstantCommand(() -> setGoalDegrees(angleDegrees));
    }
}
