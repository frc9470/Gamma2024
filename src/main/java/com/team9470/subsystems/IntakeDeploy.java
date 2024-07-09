package com.team9470.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team9470.Constants.IntakeConstants.*;

public class IntakeDeploy extends SubsystemBase { // MAKE SURE ABSOLUTE ENCODER DOES NOT WRAP
    private final CANSparkMax intakeArmMotor = new CANSparkMax(INTAKE_ARM_ID, CANSparkMax.MotorType.kBrushless);
    private double setpoint = 0;
    private final DutyCycleEncoder intakeArmEncoder = new DutyCycleEncoder(THROUGH_BORE);
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(FF_S, FF_V);
    private final ProfiledPIDController PID = new ProfiledPIDController(
            PID_P,
            0,
            0,
            new TrapezoidProfile.Constraints(
                    MAX_VELOCITY,
                    MAX_ACCELERATION)
    );

    // we don't want the intake to move as soon as the robot is turned on or right after auto, it should only do so after driver input
    private boolean init = true;


    public IntakeDeploy(){
        //settings
        intakeArmMotor.restoreFactoryDefaults();
        intakeArmMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }
    private void setVoltage (float voltage){
        intakeArmMotor.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        if( init ){
            setpoint = getRawPosition();
            init = false;
        }
        // raw pid control
        intakeArmMotor.setVoltage(ff.calculate(setpoint) + PID.calculate(getRawPosition(), setpoint));
    }

    public double getRawPosition(){
        return (intakeArmEncoder.getAbsolutePosition() + ABSOLUTE_OFFSET)/INTAKE_RATIO;
    }


    public Command intakeUp(){
        return new InstantCommand(() -> PID.setGoal(UP_GOAL));
    }
    public Command intakeDown (){
        return new InstantCommand(() -> PID.setGoal(DOWN_GOAL));
    }
}
