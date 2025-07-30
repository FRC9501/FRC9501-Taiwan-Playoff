package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.LeftReefVision;

public class AutoLeftVisionCmd extends Command {

    public AutoLeftVisionCmd() {
    
    }

    @Override
    public void initialize() {
    }



    @Override
    public void execute() {
        // if (TargetStatus) {
        //     new SwerveJoystickCmd(
        //         swerveSubsystem,
        //         () -> -m_leftVision.ySpeedOutput(),
        //         () -> m_leftVision.xSpeedOutput(),
        //         () -> 0.0,
        //         () -> false);
        //     }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
