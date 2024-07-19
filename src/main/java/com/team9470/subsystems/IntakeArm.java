package com.team9470.subsystems;

import com.team9470.subsystems.arm.AbstractArm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static com.team9470.Consts.IntakeConstants.*;

public class IntakeArm extends AbstractArm {
    private static IntakeArm instance;

    private IntakeArm() {
        super(INTAKE_ARM);

        motor.setInverted(true);

        pid.setIZone(0.05);
    }

    public static IntakeArm getInstance() {
        if (instance == null) {
            instance = new IntakeArm();
        }
        return instance;
    }

    public Command intakeUp(){
        return new InstantCommand(() -> goal = UP_GOAL);
    }
    public Command intakeDown (){
        return new InstantCommand(() -> goal = DOWN_GOAL);
    }

    public Command waitReady(){
        return new Command() {

            @Override
            public boolean isFinished() {
                return Math.abs(getPosition() - goal) < 0.06;
            }
        };
    }

    // SysID that doesn't work

//    private final SysIdRoutine routine = new SysIdRoutine(
//            new SysIdRoutine.Config(Volts.of(.2).per(Seconds.of(1)), Volts.of(1.5), Seconds.of(10)),
//            new SysIdRoutine.Mechanism(
//                    (voltage) -> this.runVolts(voltage.in(Volts)),
//                    null, // No log consumer, since data is recorded by URCL
//                    this
//            )
//    );
//
//    double topLimit = 1.8;
//    double bottomLimit = -.1;
//
//    public Command getQuasistatic(SysIdRoutine.Direction direction){
//        return routine.quasistatic(direction).until(() -> {
//            if(direction == SysIdRoutine.Direction.kForward) return getPosition() > topLimit;
//            else return getPosition() < bottomLimit;
//        });
//    }
//
//    public Command getDynamic(SysIdRoutine.Direction direction){
//        return routine.dynamic(direction).until(() -> {
//            if(direction == SysIdRoutine.Direction.kForward) return getPosition() > topLimit;
//            else return getPosition() < bottomLimit;
//        });
//    }
}
