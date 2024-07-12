package com.team9470.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static com.team9470.Constants.IntakeConstants.*;
import static edu.wpi.first.units.Units.*;

public class IntakeArm extends SubsystemBase { // MAKE SURE ABSOLUTE ENCODER DOES NOT WRAP
    private static IntakeArm instance;
    private final CANSparkMax intakeArmMotor = new CANSparkMax(INTAKE_ARM_ID, CANSparkMax.MotorType.kBrushless);
    private final DutyCycleEncoder intakeArmEncoder = new DutyCycleEncoder(THROUGH_BORE);
    private final ArmFeedforward ff = new ArmFeedforward(0, FF_G, 0);
    private final ProfiledPIDController PID = new ProfiledPIDController(
            PID_P.get(),
            PID_I.get(),
            PID_D.get(),
            new TrapezoidProfile.Constraints(
                    MAX_VELOCITY.get(),
                    MAX_ACCEL.get())
    );

    private double goal = 0;

    // we don't want the intake to move as soon as the robot is turned on or right after auto, it should only do so after driver input
    private boolean init = true;

    private IntakeArm(){
        //settings
        intakeArmMotor.restoreFactoryDefaults();
        intakeArmMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        intakeArmMotor.setInverted(true);
        intakeArmMotor.getEncoder().setPositionConversionFactor((18*24)/(56*3*30.0) * 2 * Math.PI);
        intakeArmMotor.getEncoder().setVelocityConversionFactor((18*24)/(56*3*30.0) * 2 * Math.PI / 60.); // all data in radians

        intakeArmMotor.getEncoder().setPosition(getPosition());


        PID.setIZone(0.05);
    }

    public static IntakeArm getInstance(){
        if(instance == null){
            instance = new IntakeArm();
        }
        return instance;
    }

    public void runVolts(double volts){
        SmartDashboard.putNumber("Intake/ArmOutput", volts);
        intakeArmMotor.setVoltage(volts);
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
        intakeArmMotor.setVoltage(output);

        SmartDashboard.putNumber("Intake/Position", getPosition());
        SmartDashboard.putNumber("Intake/RawEncoderOutput", intakeArmEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Intake/ArmOutput", output);
        SmartDashboard.putNumber("Intake/Goal", goal);
        SmartDashboard.putNumber("Intake/Setpoint", PID.getSetpoint().position);
        SmartDashboard.putNumber("Intake/Velocity", PID.getSetpoint().velocity);
    }

    public double getPosition(){
        return ((intakeArmEncoder.getAbsolutePosition())*INTAKE_ENCODER_RATIO+ABSOLUTE_OFFSET) * 2 * Math.PI;
    }


    public Command intakeUp(){
        return new InstantCommand(() -> goal = UP_GOAL);
    }
    public Command intakeDown (){
        return new InstantCommand(() -> goal = DOWN_GOAL.get());
    }

    public void setGoal(double goal){
        this.goal = goal;
    }

    // SysID that doesn't work

    private final SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(.2).per(Seconds.of(1)), Volts.of(1.5), Seconds.of(10)),
            new SysIdRoutine.Mechanism(
                    (voltage) -> this.runVolts(voltage.in(Volts)),
                    null, // No log consumer, since data is recorded by URCL
                    this
            )
    );

    double topLimit = 1.8;
    double bottomLimit = -.1;

    public Command getQuasistatic(SysIdRoutine.Direction direction){
        return routine.quasistatic(direction).until(() -> {
            if(direction == SysIdRoutine.Direction.kForward) return getPosition() > topLimit;
            else return getPosition() < bottomLimit;
        });
    }

    public Command getDynamic(SysIdRoutine.Direction direction){
        return routine.dynamic(direction).until(() -> {
            if(direction == SysIdRoutine.Direction.kForward) return getPosition() > topLimit;
            else return getPosition() < bottomLimit;
        });
    }
}
