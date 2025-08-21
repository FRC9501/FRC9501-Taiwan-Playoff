// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PutL1_Auto extends Command {
  /** Creates a new PutL1_Auto. */
  private final ArmSubsystem m_ArmSubsystem;
  private final ElevatorSubsystem m_ElevatorSubsystem;
  public PutL1_Auto(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ArmSubsystem = armSubsystem;
    this.m_ElevatorSubsystem = elevatorSubsystem;
    addRequirements(m_ArmSubsystem, m_ElevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_ArmSubsystem.arriveSetpoint() && m_ElevatorSubsystem.arriveSetpoint()) {
      m_ArmSubsystem.putL1_Wheel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ArmSubsystem.stopWheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
