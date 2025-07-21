package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import frc.robot.subsystems.Vision.LeftReefVision;

public class AutoLeftVisionCmd extends Command {

    private final LeftReefVision leftVision;
    private final DriveSubsystem swerveSubsystem;
    private Boolean TargetStatus = true;

    public AutoLeftVisionCmd(LeftReefVision left,DriveSubsystem swerve) {
        this.leftVision = left;
        this.swerveSubsystem = swerve;
        addRequirements(left,swerve);
    }

    @Override
    public void initialize() {
        if (leftVision.states()) {
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
                () -> -leftVision.ySpeedOutput(),
                () -> leftVision.xSpeedOutput(),
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
