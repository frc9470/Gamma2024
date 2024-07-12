package com.team9470.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team9470.Constants.HoodConstants.*;
import static com.team9470.Constants.HoodConstants.MAX_ACCEL;
import static com.team9470.Constants.HoodConstants.MAX_VELOCITY;
import static com.team9470.Constants.HoodConstants.PID_D;
import static com.team9470.Constants.HoodConstants.PID_I;
import static com.team9470.Constants.HoodConstants.PID_P;

public class Hood extends SubsystemBase { // MAKE SURE ABSOLUTE ENCODER DOES NOT WRAP
    private static Hood instance;

    private final CANSparkMax hoodMotor = new CANSparkMax(HOOD_ID, CANSparkMax.MotorType.kBrushless);
    private final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(THROUGH_BORE);
    private final ArmFeedforward ff = new ArmFeedforward(0, FF_G, 0);
    private final ProfiledPIDController PID = new ProfiledPIDController(
            PID_P.get(),
            PID_I.get(),
            PID_D.get(),
            new TrapezoidProfile.Constraints(
                    MAX_VELOCITY.get(),
                    MAX_ACCEL.get()
            )
    );

    // we don't want the Hood to move as soon as the robot is turned on or right after auto, it should only do so after driver input
    private boolean init = true;
    private double goal = 0;


    private Hood(){
        //settings
        hoodMotor.restoreFactoryDefaults();
        hoodMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }

    public static Hood getInstance(){
        if(instance == null){
            instance = new Hood();
        }
        return instance;
    }

    @Override
    public void periodic() {
        if( init ){
            goal = getPosition();
            PID.reset(goal);
            init = false;
        }
        
        if(PID_P.hasChanged() || PID_I.hasChanged() || PID_D.hasChanged() || MAX_VELOCITY.hasChanged() || MAX_ACCEL.hasChanged()){
            PID.setPID(PID_P.get(), PID_I.get(), PID_D.get());
            PID.setConstraints(new TrapezoidProfile.Constraints(
                    MAX_VELOCITY.get(),
                    MAX_ACCEL.get())
            );
        }

        double output = PID.calculate(getPosition(), goal) + ff.calculate(getPosition(), 0);
        hoodMotor.setVoltage(output);

        SmartDashboard.putNumber("Hood/Position", getPosition());
        SmartDashboard.putNumber("Hood/RawEncoderOutput", hoodEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Hood/ArmOutput", output);
        SmartDashboard.putNumber("Hood/Goal", goal);
        SmartDashboard.putNumber("Hood/Setpoint", PID.getSetpoint().position);
        SmartDashboard.putNumber("Hood/Velocity", PID.getSetpoint().velocity);
    }

    public double getPosition(){
        return (hoodEncoder.getAbsolutePosition() + ABSOLUTE_OFFSET) * 2 * Math.PI;
    }

    public void setGoal(double goal){
        this.goal = goal;
    }

}
