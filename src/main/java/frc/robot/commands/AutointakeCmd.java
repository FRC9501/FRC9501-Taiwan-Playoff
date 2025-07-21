package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSub;

public class AutointakeCmd extends Command{

    private IntakeSub intake;

    public AutointakeCmd(IntakeSub Intake) {
        this.intake = Intake;
        addRequirements(Intake);
    }

    // @Override
    // public void initialize() {
    //     intake.motorSet(1, 0.4);
    // }

    @Override
    public void execute() {
        if (intake.Distance() > 900) {
            intake.stop();
        }
    }

    @Override
    public void end(boolean interrupted){
        
    }

    // @Override
    // public boolean isFinished() {
    //     if (intake.Distance() > 900){
    //         intake.motorSet(0, 0);
    //         return true;
    //     }else{
    //         return false;
    //     }
    // }
    
}
