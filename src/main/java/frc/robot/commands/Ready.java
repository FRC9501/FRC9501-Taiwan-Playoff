// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PawSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Ready extends Command {
  /** Creates a new * w ready. */
  private ElevatorSubsystem m_elevatorSubsystem;
  private PawSubsystem m_PawSubsystem;
  public Ready(ElevatorSubsystem elevatorSubsystem, PawSubsystem pawSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_elevatorSubsystem = elevatorSubsystem;
    this.m_PawSubsystem = pawSubsystem;
    addRequirements(m_PawSubsystem, m_elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSubsystem.readyPosition();
    m_PawSubsystem.readyPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
