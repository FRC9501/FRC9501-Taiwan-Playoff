package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.RightReefVision;

public class AutoRightVisionCmd extends Command {

    private final RightReefVision m_RightVision;
    private final SwerveDriveSubsystem m_SwerveSubsystem;
    private Boolean TargetStatus = true;

    public AutoRightVisionCmd(RightReefVision right,SwerveDriveSubsystem swerve) {
        this.m_RightVision = right;
        this.m_SwerveSubsystem = swerve;
        addRequirements(m_RightVision,m_SwerveSubsystem);
    }

    @Override
    public void initialize() {
        if (m_RightVision.states()) {
            TargetStatus = true;
        }else{
            TargetStatus = false;
        }
    }

    @Override
    public void execute() {
        // if (TargetStatus) {
        //      new SwerveJoystickCmd(
        //         swerveSubsystem,
        //         () -> -m_RightVision.ySpeedOutput(),
        //         () -> m_RightVision.xSpeedOutput(),
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
