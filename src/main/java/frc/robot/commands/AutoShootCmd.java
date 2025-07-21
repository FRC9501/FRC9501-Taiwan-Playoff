package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSub;
import frc.robot.subsystems.elevator.elevator;

public class AutoShootCmd extends Command{
    private IntakeSub intake;
    private elevator elevator;

    public AutoShootCmd(IntakeSub intakeSub,elevator elevator) {
        this.intake = intakeSub;
        this.elevator = elevator;
        addRequirements(intakeSub);
    }

    @Override
    public void initialize() {
        intake.shoot(0.6);
    }

    @Override
    public void execute() {
        if (intake.Distance() < 1000) {
            intake.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished(){
        if (intake.Distance() < 1000) {
            elevator.L0();
            return true;
        }else{
            return false;
        }
    }
    
}
