package frc.robot.subsystems.Climber;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class climberCmd extends Command{
    private final climber climber = new climber();
    private final Supplier<Double> Rotation;
   
    public climberCmd(climber climber,Supplier<Double> Rotation) {
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
