package com.team9470.subsystems;


import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static com.team9470.Consts.ShooterConstants.*;
import static edu.wpi.first.units.Units.Volts;

/**
 * shoots the note [into the speaker]
 * <br><br>
 * contains:
 * <ul>
 * <li>command to approach rpm</li>
 * <li>conditional wait until rpm</li>
 * </ul>
 * <br>
 * <b>IMPORTANT NOTES:</b>
 * <ul>
 * <li> SHOOTER IS ALWAYS SPINNING;
 *      THIS WILL OCCUR THE MOMENT THE ROBOT IS ENABLED </li>
 * </ul>
 */
public class Shooter extends SubsystemBase {
    private static Shooter instance;
    public final CANSparkMax top = new CANSparkMax(ID_TOP, CANSparkMax.MotorType.kBrushless);
    public final CANSparkMax bottom = new CANSparkMax(ID_BOTTOM, CANSparkMax.MotorType.kBrushless);
    public final RelativeEncoder topEncoder = top.getEncoder();
    public final RelativeEncoder bottomEncoder = bottom.getEncoder();
    public SimpleMotorFeedforward ffTOP = new SimpleMotorFeedforward(FF_S, FF_V, FF_A);
    public SimpleMotorFeedforward ffBOT = new SimpleMotorFeedforward(FF2_S, FF2_V, FF2_A);
    public final PIDController TOP_PID = new PIDController(PID_P.get(), 0, 0);
    public final PIDController BOTTOM_PID = new PIDController(PID_P.get(), 0, 0);
    private double topSetpoint = 0;
    private double bottomSetpoint = 0;

    private SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    (voltage) -> this.setVoltage(voltage.in(Volts)),
                    null,
                    this
            )
    );

    private Shooter(){
        top.restoreFactoryDefaults();
        top.setIdleMode(CANSparkBase.IdleMode.kCoast);
        topEncoder.setVelocityConversionFactor(SHOOTER_RATIO);

        bottom.restoreFactoryDefaults();
        bottom.setIdleMode(CANSparkBase.IdleMode.kCoast);
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
        double output = ffTOP.calculate(topSetpoint) + TOP_PID.calculate(topEncoder.getVelocity(), topSetpoint);
        top.setVoltage(output);
        bottom.setVoltage(ffBOT.calculate(bottomSetpoint) + BOTTOM_PID.calculate(bottomEncoder.getVelocity(), bottomSetpoint));

        if(PID_P.hasChanged()){
            TOP_PID.setP(PID_P.get());
            BOTTOM_PID.setP(PID_P.get());
        }
        SmartDashboard.putNumber("Shooter/Top Output", output);
        SmartDashboard.putNumber("Shooter/Top Setpoint", topSetpoint);
        SmartDashboard.putNumber("Shooter/Bottom Setpoint", bottomSetpoint);
        SmartDashboard.putNumber("Shooter/Top Velocity", topEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter/Bottom Velocity", bottomEncoder.getVelocity());
        SmartDashboard.putBoolean("Shooter/TOP READY", Math.abs(topEncoder.getVelocity() - topSetpoint) < TOLERANCE);
        SmartDashboard.putBoolean("Shooter/BOTTOM READY", Math.abs(bottomEncoder.getVelocity() - bottomSetpoint) < TOLERANCE);
    }

    public void setVelocity(double top, double bottom){
        topSetpoint = top;
        bottomSetpoint = bottom;
    }

    public void setVoltage(double voltage){
        top.setVoltage(voltage);
        bottom.setVoltage(voltage);
    }

    public void setVelocity(double rpm){
        setVelocity(rpm, rpm);
    }

    public Command waitReady() {
        return new Command() {
            @Override
            public boolean isFinished() {
                return Math.abs(topSetpoint - topEncoder.getVelocity())  < TOLERANCE
                        && Math.abs(bottomSetpoint -bottomEncoder.getVelocity()) < TOLERANCE;
            }
        };
    }

    public Command getQuasistatic(SysIdRoutine.Direction direction){
        return routine.quasistatic(direction);
    }

    public Command getDynamic(SysIdRoutine.Direction direction){
        return routine.dynamic(direction);
    }
}