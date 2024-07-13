package com.team9470.subsystems;


import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team9470.Constants.ShooterConstants.*;

/**
 * Shooter
 * <ul>
 * <li>command to rpm</li>
 * <li>conditional wait until rpm</li>
 * </ul>
 */
public class Shooter extends SubsystemBase {
    private static Shooter instance;
    public final CANSparkMax top = new CANSparkMax(ID_TOP, CANSparkMax.MotorType.kBrushless);
    public final CANSparkMax bottom = new CANSparkMax(ID_BOTTOM, CANSparkMax.MotorType.kBrushless);
    public final RelativeEncoder topEncoder = top.getEncoder();
    public final RelativeEncoder bottomEncoder = bottom.getEncoder();
    public final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(FF_S, FF_V);
    public final PIDController TOP_PID = new PIDController(PID_P, 0, 0);
    public final PIDController BOTTOM_PID = new PIDController(PID_P, 0, 0);
    private double topSetpoint = 0;
    private double bottomSetpoint = 0;

    private Shooter(){
        top.restoreFactoryDefaults();
        top.setIdleMode(CANSparkBase.IdleMode.kBrake);
        topEncoder.setVelocityConversionFactor(SHOOTER_RATIO);

        bottom.restoreFactoryDefaults();
        bottom.setIdleMode(CANSparkBase.IdleMode.kBrake);
        bottomEncoder.setVelocityConversionFactor(SHOOTER_RATIO);
        bottom.setInverted(true);
    }

    public static Shooter getInstance(){
        if(instance == null){
            instance = new Shooter();
        }
        return instance;
    }

    @Override
    public void periodic() {
        top.setVoltage(ff.calculate(topSetpoint) + TOP_PID.calculate(topEncoder.getVelocity(), topSetpoint));
        bottom.setVoltage(ff.calculate(bottomSetpoint) + BOTTOM_PID.calculate(bottomEncoder.getVelocity(), bottomSetpoint));
    }

    public void setVelocity(double top, double bottom){
        topSetpoint = top;
        bottomSetpoint = bottom;
    }

    public void setVelocity(double rpm){
        setVelocity(rpm, rpm);
    }

    public Command waitReady() {
        return new Command() {
            @Override
            public boolean isFinished() {
                return Math.abs(topEncoder.getVelocity() - topSetpoint) < TOLERANCE
                        && Math.abs(bottomEncoder.getVelocity() - bottomSetpoint) < TOLERANCE;
            }
        };
    }
}