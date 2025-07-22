package frc.robot.subsystems.Climber;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberCmd extends Command{
    private final Climber climber = new Climber();
    private final Supplier<Double> Rotation;
   
    public ClimberCmd(Climber climber,Supplier<Double> Rotation) {
        this.Rotation = Rotation;
        addRequirements(climber);
    }   

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        climber.setAngle(Rotation);
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }  
}
