package com.team9470.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team9470.Constants.IntakeConstants.*;

public class IntakeDeploy extends SubsystemBase {
    public final CANSparkMax intakeArmMotor = new CANSparkMax(INTAKE_ARM_ID, CANSparkMax.MotorType.kBrushless);
    private int setpoint = 0;
    public final RelativeEncoder intakeArmEncoder = intakeArmMotor.getEncoder();
    public final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(FF_S, FF_V);
    public final PIDController PID = new PIDController(PID_P, 0, 0);
    public IntakeDeploy(){
        //settings
        intakeArmMotor.restoreFactoryDefaults();
        intakeArmMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }
    public void setVoltage (Float voltage){
        intakeArmMotor.setVoltage(voltage);
    }
    @Override
    public void periodic() {
        intakeArmMotor.setVoltage(ff.calculate(setpoint) + PID.calculate(intakeArmEncoder.getVelocity(), setpoint));
    }
    public Command intakeUp (){
        return new InstantCommand();
    }
    public Command intakeDown (){
        return new InstantCommand();
    }
}
