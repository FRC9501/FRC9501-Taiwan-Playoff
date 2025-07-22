package frc.robot.commands;

import java.util.function.DoubleSupplier;
// import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCmd extends Command{
    private final ClimberSubsystem m_ClimberSubsystem;
    private final DoubleSupplier Rotation;
   
    public ClimberCmd(ClimberSubsystem climberSubsystem, DoubleSupplier Rotation) {
        this.Rotation = Rotation;
        this.m_ClimberSubsystem = climberSubsystem;
        addRequirements(m_ClimberSubsystem);
    }   

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        m_ClimberSubsystem.setAngle(Rotation);
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }  
}
