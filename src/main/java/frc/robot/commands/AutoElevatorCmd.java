package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.subsystems.Intake.IntakeSub;
import frc.robot.subsystems.Elevator.Elevator;

public class AutoElevatorCmd extends Command{

    private Elevator Elevator;
    // private IntakeSub intake;

    public AutoElevatorCmd(Elevator Elevator) {
        this.Elevator = Elevator;
        // this.intake = intakeSub;
        addRequirements(Elevator);
    }

    @Override
    public void initialize() {
        Elevator.L3();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished(){
        if (Elevator.get() > 40) {
            return true;
        }else{
            return false;
        }
    }
    
}
