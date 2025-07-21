package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake.IntakeSub;
import frc.robot.subsystems.elevator.elevator;

public class AutoElevatorCmd extends Command{

    private elevator elevator;
    private IntakeSub intake;

    public AutoElevatorCmd(elevator Elevator, IntakeSub intakeSub) {
        this.elevator = Elevator;
        this.intake = intakeSub;
        addRequirements(elevator,intakeSub);
    }

    @Override
    public void initialize() {
        elevator.L3();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished(){
        if (elevator.get() > 40) {
            return true;
        }else{
            return false;
        }
    }
    
}
