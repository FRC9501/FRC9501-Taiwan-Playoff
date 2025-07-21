package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import frc.robot.subsystems.Vision.RightReefVision;

public class AutoRightVisionCmd extends Command {

    private final RightReefVision rightVision;
    private final DriveSubsystem swerveSubsystem;
    private Boolean TargetStatus = true;

    public AutoRightVisionCmd(RightReefVision right,DriveSubsystem swerve) {
        this.rightVision = right;
        this.swerveSubsystem = swerve;
        addRequirements(right,swerve);
    }

    @Override
    public void initialize() {
        if (rightVision.states()) {
            TargetStatus = true;
        }else{
            TargetStatus = false;
        }
    }

    @Override
    public void execute() {
        if (TargetStatus) {
             new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -rightVision.ySpeedOutput(),
                () -> rightVision.xSpeedOutput(),
                () -> 0.0,
                () -> false);
            }
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
