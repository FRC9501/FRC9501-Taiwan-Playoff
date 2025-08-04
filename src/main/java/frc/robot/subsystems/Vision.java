package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
    private double xSpeedOutput;
    private double ySpeedOutput;
    private double RotationOutput;
    private PIDController xSpeedController;
    private PIDController ySpeedController;
    private PIDController RotationController;

    public Vision() {
        xSpeedController = new PIDController(0.02, 0, 0);
        ySpeedController = new PIDController(0.3, 0, 0);
        RotationController = new PIDController(0.02, 0, 0);

        
    }

    public void vision(){
        
    }

    @Override
    public void periodic() {
    }


}

