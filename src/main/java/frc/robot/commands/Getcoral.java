// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Getcoral extends Command {
    private final PawSubsystem m_PawSubsystem;
    private final ElevatorSubsystem m_ElevatorSubsystem;
  /** Creates a new Hand. */
  public Getcoral(PawSubsystem pawSubsystem, ElevatorSubsystem elevatorSubsystem) {
    this.m_ElevatorSubsystem = elevatorSubsystem;
    this.m_PawSubsystem = pawSubsystem;
    addRequirements(m_ElevatorSubsystem, m_PawSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_PawSubsystem.coast();
    m_ElevatorSubsystem.readyPosition();
    m_PawSubsystem.readyPosition();
  
  }
  //test
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_PawSubsystem.setpoint()&&m_ElevatorSubsystem.setpoint() == true) {
      m_ElevatorSubsystem.takecoral();
      m_PawSubsystem.takeIn();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
