package com.team9470.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static com.team9470.Consts.AmpevatorConstants;
import static com.team9470.Util.clamp;

public class Ampevator extends SubsystemBase {

    private static Ampevator instance;
    private final CANSparkMax elevator1 = new CANSparkMax(AmpevatorConstants.ID_1, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax elevator2 = new CANSparkMax(AmpevatorConstants.ID_2, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax rollers = new CANSparkMax(AmpevatorConstants.ROLLER_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder encoder = elevator1.getEncoder();
    private final DigitalInput beamBreak = new DigitalInput(AmpevatorConstants.BEAM_BREAK_PORT);

    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(AmpevatorConstants.KS, AmpevatorConstants.KG, AmpevatorConstants.KV, AmpevatorConstants.KA);
    private final ProfiledPIDController controller =
            new ProfiledPIDController(AmpevatorConstants.KP, AmpevatorConstants.KI, AmpevatorConstants.KD, new TrapezoidProfile.Constraints(AmpevatorConstants.VELOCITY_LIMIT, AmpevatorConstants.ACCELERATION_LIMIT));

    private boolean needsHome = true;
    private double goal;

    private Ampevator(){
        elevator1.restoreFactoryDefaults();
        elevator2.restoreFactoryDefaults();
        elevator1.setInverted(AmpevatorConstants.INVERTED);
        elevator2.setInverted(AmpevatorConstants.INVERTED);

        elevator2.follow(elevator1, false);

        encoder.setPositionConversionFactor(AmpevatorConstants.RATIO_MPR);
    }

    public static Ampevator getInstance(){
        if(instance == null){
            instance = new Ampevator();
        }
        return instance;
    }

    @Override
    public void periodic() {
        if(needsHome){
            elevator1.setVoltage(AmpevatorConstants.HOMING_SPEED);
            double current = elevator1.getOutputCurrent();
            if(current > 5/*amp*/){
                elevator1.set(0);
                needsHome = false;
                encoder.setPosition(0);
                goal = encoder.getPosition();
                controller.reset(encoder.getPosition());
            }
        } else {
            goal = clamp(goal, 0, AmpevatorConstants.EXTENSION_HEIGHT);
            double output = controller.calculate(encoder.getPosition(), goal) + feedforward.calculate(controller.getSetpoint().velocity);
            elevator1.setVoltage(output); // elevator2 should follow
        }
        updateSmartDashboard();
    }

    protected void updateSmartDashboard() {
        String name = getClass().getSimpleName();
        SmartDashboard.putNumber(name + "/Position", getPosition());
        SmartDashboard.putNumber(name + "/ArmOutput", elevator1.getAppliedOutput());
        SmartDashboard.putNumber(name + "/Goal", goal);
        SmartDashboard.putNumber(name + "/Setpoint", controller.getSetpoint().position);
        SmartDashboard.putNumber(name + "/Velocity", controller.getSetpoint().velocity);
    }

    private double getPosition() {
        return encoder.getPosition();
    }

    private boolean getBeamBreak() {
        return beamBreak.get();
    }

    public Command toTarget(double target) {
        return new Command() {
            @Override
            public void initialize() {
                goal = clamp(target, 0, AmpevatorConstants.EXTENSION_HEIGHT);
            }

            @Override
            public boolean isFinished() {
                return Math.abs(encoder.getPosition()-goal) < AmpevatorConstants.TOLERANCE;
            }
        };
    }

    public Command ampNote() {
        return new SequentialCommandGroup(
                    toTarget(AmpevatorConstants.AMP),
                    rollerOut()
                            .deadlineWith(
                                    new WaitCommand(1)
                                            .andThen(toTarget(0))
                            )
                );
    }

    private Command rollerOut() {
        return this.runEnd(() -> rollers.setVoltage(AmpevatorConstants.ROLLER_SPEED), rollers::stopMotor);
    }
}
