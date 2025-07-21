package frc.robot.subsystems.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LeftReefVision extends SubsystemBase {
    private double xSpeedOutput;
    private double ySpeedOutput;
    private double RotationOutput;
    private PIDController xSpeedController;
    private PIDController ySpeedController;
    private PIDController RotationController;

    public LeftReefVision() {
        xSpeedController = new PIDController(0.02, 0, 0);
        ySpeedController = new PIDController(0.3, 0, 0);
        RotationController = new PIDController(0.02, 0, 0);
    }

    @Override
    public void periodic() {
        Target();
        displayXOutput();
        displayYOutput();
        displayturnOutput();
    }

    public void chagePiepeline() {
        LimelightHelpers.setPipelineIndex("limelight", 0);
    }

    public boolean states() {
       return LimelightHelpers.getTV("limelight");
    }

    public double TX() {
        return LimelightHelpers.getTX("limelight");
    }


    public double TA() {
        if (LimelightHelpers.getTA("limelight") > 12.5) {
            return 0;
        }else{
            return LimelightHelpers.getTA("limelight");
        }
    }

    public void Target() {
        SmartDashboard.putBoolean("State Left", states());
    }

    public double Tid() {
        return LimelightHelpers.getFiducialID("limelight");
    }

    public double xSpeedOutput() {
        // 計算 PID 控制器的輸出，目標設為 17
        xSpeedOutput = xSpeedController.calculate(TX(),0);
        
        // 當 TX 非常接近 17 時停止輸出
        if (Math.abs(TX() - 17) < 0.005) {
            return 0;  // TX 接近 17 時，停止輸出
        }
    
        double threshold = 0.005; 
        if (Math.abs(xSpeedOutput) < threshold) {
            return 0; // 當接近目標時停止
        }
        
        // 保持在 -1 到 1 之間，避免過度搖擺
        if (xSpeedOutput > 1) {
            return 0.3; // 設定一個限制的正數速度
        } else if (xSpeedOutput < -1) {
            return -0.3; // 設定一個限制的負數速度
        }
        
        // 其餘情況下返回原始計算結果
        return xSpeedOutput;
    }
    
    
       

    
    public double ySpeedOutput() {
        ySpeedOutput = ySpeedController.calculate(TA(), 0);
        double scaleFactor = Math.max(0.1, 1 - TA() / 20.0); 
        ySpeedOutput *= scaleFactor;
        if (ySpeedOutput > 1) {
            return 0.4; 
        } else if (ySpeedOutput < -1) {
            return -0.4; 
        } else {
            return ySpeedOutput;
        }
    }
    

    public double RotationOutput() {
        RotationOutput = RotationController.calculate(TX(), 0);
        
        double threshold = 0.005; 
        if (Math.abs(RotationOutput) < threshold) {
            return 0; // 當接近目標時停止
        }
        
        // 保持在 -1 到 1 之間，避免過度搖擺
        if (RotationOutput > 1) {
            return 0.3; // 設定一個限制的正數速度
        } else if (RotationOutput < -1) {
            return -0.3; // 設定一個限制的負數速度
        }
        
        // 其餘情況下返回原始計算結果
        return RotationOutput;
    }
    
    

    public void displayturnOutput() {
        SmartDashboard.putNumber("TurnOutput", RotationOutput());
    }

    public void displayXOutput() {
        SmartDashboard.putNumber("XOutput", xSpeedOutput());
    }

    public void displayYOutput() {
        SmartDashboard.putNumber("YOutput", ySpeedOutput());
    }
}

