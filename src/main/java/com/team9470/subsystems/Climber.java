package com.team9470.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import static com.team9470.Consts.ClimberConstants;
import static com.team9470.Util.clamp;

public class Climber extends SubsystemBase {

    private static Climber instance;
    private final CANSparkMax climber1 = new CANSparkMax(ClimberConstants.ID_1, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax climber2 = new CANSparkMax(ClimberConstants.ID_2, CANSparkLowLevel.MotorType.kBrushless);
    
    private boolean needsHome = true;
    private double goal;

    private Climber(){
        climber1.restoreFactoryDefaults();
        climber2.restoreFactoryDefaults();

        climber2.follow(climber1, false);

    }

    public static Climber getInstance(){
        if(instance == null){
            instance = new Climber();
        }
        return instance;
    }

    @Override
    public void periodic() {
        if(needsHome){
            climber1.setVoltage(ClimberConstants.HOMING_SPEED);
            double current = climber1.getOutputCurrent();
            if(current > 5/*amp*/){
                climber1.set(0);
                needsHome = false;
                encoder.setPosition(0);
                goal = encoder.getPosition();
                controller.reset(encoder.getPosition());
            }
        } else {
            goal = clamp(goal, 0, ClimberConstants.EXTENSION_HEIGHT);
            double output = controller.calculate(encoder.getPosition(), goal) + feedforward.calculate(controller.getSetpoint().velocity);
            climber1.setVoltage(output); // climber2 should follow
        }
        updateSmartDashboard();
    }

    protected void updateSmartDashboard() {
        String name = getClass().getSimpleName();
        SmartDashboard.putNumber(name + "/Position", getPosition());
        SmartDashboard.putNumber(name + "/ArmOutput", climber1.getAppliedOutput());
        SmartDashboard.putNumber(name + "/Goal", goal);
        SmartDashboard.putNumber(name + "/Setpoint", controller.getSetpoint().position);
        SmartDashboard.putNumber(name + "/Velocity", controller.getSetpoint().velocity);
    }

    public double getPosition() {
        return encoder.getPosition();
    }

}
