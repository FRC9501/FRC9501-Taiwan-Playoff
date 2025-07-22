package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.LeftReefVision;

public class AutoLeftVisionCmd extends Command {

    private final LeftReefVision m_leftVision;
    private final SwerveDriveSubsystem m_SwerveSubsystem;
    private Boolean TargetStatus = true;

    public AutoLeftVisionCmd(LeftReefVision left,SwerveDriveSubsystem swerve) {
        this.m_leftVision = left;
        this.m_SwerveSubsystem = swerve;
        addRequirements(m_leftVision,m_SwerveSubsystem);
    }

    @Override
    public void initialize() {
        if (m_leftVision.states()) {
            TargetStatus = true;
        }else{
            TargetStatus = false;
        }
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
        if (!TargetStatus){
            return true;
        }else{
            return false;
        }
    }
}
