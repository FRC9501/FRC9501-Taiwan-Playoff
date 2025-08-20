// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ToPrimitive extends Command {
  private ElevatorSubsystem m_ElevatorSubsystem;
  private ArmSubsystem m_ArmSubsystem;
  /** Creates a new Ready. */
  public ToPrimitive(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ElevatorSubsystem = elevatorSubsystem;
    this.m_ArmSubsystem = armSubsystem;
    addRequirements(m_ElevatorSubsystem, m_ArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (LEDConstants.isAlgaeMode) {
      m_ArmSubsystem.primitive_Algae_Pivot();
    }else{
      m_ElevatorSubsystem.toSafePosition();
      m_ArmSubsystem.stopWheel();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_ArmSubsystem.arriveSetpoint() && LEDConstants.isAlgaeMode) {
      m_ElevatorSubsystem.primitive_Algae();
    }
    if(m_ElevatorSubsystem.isSafe() && LEDConstants.isAlgaeMode == false){
      m_ArmSubsystem.intakeCoral_Pivot();
    }
    if ((Math.abs(m_ArmSubsystem.getAngle_Degrees_PID() - ArmConstants.coralIntakePosition) <= 5) && (LEDConstants.isAlgaeMode == false)) {
      m_ElevatorSubsystem.primitive_Coral();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
