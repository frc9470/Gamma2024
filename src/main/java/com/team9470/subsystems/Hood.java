package com.team9470.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team9470.Constants.HoodConstants.*;

public class Hood extends SubsystemBase { // MAKE SURE ABSOLUTE ENCODER DOES NOT WRAP
    private final CANSparkMax hoodMotor = new CANSparkMax(HOOD_ID, CANSparkMax.MotorType.kBrushless);
    private final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(THROUGH_BORE);
    private final ArmFeedforward ff = new ArmFeedforward(FF_S, FF_G, FF_V);
    private final ProfiledPIDController PID = new ProfiledPIDController(
            PID_P,
            0,
            0,
            new TrapezoidProfile.Constraints(
                    MAX_VELOCITY,
                    ff.maxAchievableAcceleration(12.0, 0, MAX_VELOCITY))
    );

    // we don't want the intake to move as soon as the robot is turned on or right after auto, it should only do so after driver input
    private boolean init = true;
    private double goal = 0;


    public Hood(){
        //settings
        hoodMotor.restoreFactoryDefaults();
        hoodMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }

    private double lastSpeed = 0;
    double lastTime = Timer.getFPGATimestamp();

    @Override
    public void periodic() {
        if( init ){
            goal = getRawPosition();
            PID.reset(goal);
            init = false;
        }
        double acceleration = (PID.getSetpoint().velocity-lastSpeed)/(Timer.getFPGATimestamp()-lastTime);
        hoodMotor.setVoltage(ff.calculate(12.0, PID.getSetpoint().velocity, acceleration) + PID.calculate(getRawPosition(), goal));

        lastSpeed = PID.getSetpoint().velocity;
        lastTime = Timer.getFPGATimestamp();
    }

    public double getRawPosition(){
        return (hoodEncoder.getAbsolutePosition() + ABSOLUTE_OFFSET);
    }

    public void setGoal(double goal){
        this.goal = goal;
    }

}
