// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PutProcesser extends Command {
  /** Creates a new L0. */
  private final ElevatorSubsystem m_ElevatorSubsystem;
  private final PawSubsystem m_PawSubsystem;
  private final BooleanSupplier ifFeedFunc;
  private boolean ifFeed;
  
  
  public PutProcesser(ElevatorSubsystem elevatorSubsystem, PawSubsystem pawSubsystem, BooleanSupplier ifFeedFunc) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ElevatorSubsystem = elevatorSubsystem;
    this.m_PawSubsystem = pawSubsystem;
    this.ifFeedFunc = ifFeedFunc;    
    addRequirements(m_ElevatorSubsystem, m_PawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ElevatorSubsystem.processer();
    m_PawSubsystem.processer();
    // This is where you would put any initialization code for the command.
    // For example, you might set a motor to a specific speed or position.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ifFeed = ifFeedFunc.getAsBoolean();
    if (m_PawSubsystem.setpoint()&& m_ElevatorSubsystem.setpoint()&& ifFeed == true) {
      m_PawSubsystem.shoot();
    }
  }
//
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // This command will run indefinitely until interrupted.
  }
}
