package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoElevatorCmd extends Command{

    private ElevatorSubsystem m_ElevatorSubsystem;
    // private IntakeSub intake;

    public AutoElevatorCmd(ElevatorSubsystem elevatorSubsystem) {
        this.m_ElevatorSubsystem = elevatorSubsystem;
        // this.intake = intakeSub;
        addRequirements(m_ElevatorSubsystem);
    }

    @Override
    public void initialize() {
        m_ElevatorSubsystem.L3();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished(){
        if (m_ElevatorSubsystem.nowposition() > 40) {
            return true;
        }else{
            return false;
        }
    }
    
}
