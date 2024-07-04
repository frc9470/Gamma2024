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

import static com.team9470.Constants.HoodConstants.*;

public class Hood extends SubsystemBase { // MAKE SURE ABSOLUTE ENCODER DOES NOT WRAP
    private final CANSparkMax hoodMotor = new CANSparkMax(HOOD_ID, CANSparkMax.MotorType.kBrushless);
    private double setpoint = 0;
    private final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(THROUGH_BORE);
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


    public Hood(){
        //settings
        hoodMotor.restoreFactoryDefaults();
        hoodMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }
    private void setVoltage (float voltage){
        hoodMotor.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        if( init ){
            setpoint = getRawPosition();
            init = false;
        }
        // raw pid control
        hoodMotor.setVoltage(ff.calculate(setpoint) + PID.calculate(getRawPosition(), setpoint));
    }

    public double getRawPosition(){
        return (hoodEncoder.getAbsolutePosition() + ABSOLUTE_OFFSET);
    }


    public Command intakeUp(){
        return new InstantCommand(() -> PID.setGoal(UP_GOAL));
    }
    public Command intakeDown (){
        return new InstantCommand(() -> PID.setGoal(DOWN_GOAL));
    }
}
