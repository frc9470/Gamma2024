package com.team9470.subsystems;


import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team9470.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    public final CANSparkMax shoot1 = new CANSparkMax(SHOOT_ID_1, CANSparkMax.MotorType.kBrushless);
    public final CANSparkMax shoot2 = new CANSparkMax(SHOOT_ID_2, CANSparkMax.MotorType.kBrushless);
    public final RelativeEncoder shoot1Encoder = shoot1.getEncoder();
    public final RelativeEncoder shoot2Encoder = shoot2.getEncoder();
    public final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(FF_S, FF_V);
    public final PIDController PID1 = new PIDController(PID_P, 0, 0);
    public final PIDController PID2 = new PIDController(PID_P, 0, 0);
    private int setPoint = 0;

    public Shooter (){
        shoot1.restoreFactoryDefaults();
        shoot1.setIdleMode(CANSparkBase.IdleMode.kBrake);
        shoot1Encoder.setVelocityConversionFactor(SHOOTER_RATIO);
        shoot2.restoreFactoryDefaults();
        shoot2.setIdleMode(CANSparkBase.IdleMode.kBrake);
        shoot2Encoder.setVelocityConversionFactor(SHOOTER_RATIO);
        shoot2.setInverted(true);
    }

    @Override
    public void periodic() {
        shoot1.setVoltage(ff.calculate(setPoint) + PID1.calculate(shoot1Encoder.getVelocity(), setPoint));
        shoot2.setVoltage(ff.calculate(setPoint) + PID2.calculate(shoot2Encoder.getVelocity(), setPoint));

    }
}
