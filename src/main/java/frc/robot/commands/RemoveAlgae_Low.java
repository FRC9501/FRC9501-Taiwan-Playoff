// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RemoveAlgae_Low extends Command {
  /** Creates a new RemoveAlgae_Low. */
  private final ArmSubsystem m_ArmSubsystem;
  private final ElevatorSubsystem m_ElevatorSubsystem;
  public RemoveAlgae_Low(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ArmSubsystem = armSubsystem;
    this.m_ElevatorSubsystem = elevatorSubsystem;
    addRequirements(m_ArmSubsystem, m_ElevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ElevatorSubsystem.toSafePosition();
    m_ArmSubsystem.removeAlgaeLow_Wheel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_ElevatorSubsystem.isSafe()) {
      m_ArmSubsystem.removeAlgaeLow_Pivot();
    }
    if (Math.abs(m_ArmSubsystem.getAngle_Degrees_PID() - ArmConstants.algaeLowPosition) <= 1) {
      m_ElevatorSubsystem.intakeAlgaeLow();
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
