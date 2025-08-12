// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PutL4_Auto extends Command {
  private ArmSubsystem m_ArmSubsystem;
  private ElevatorSubsystem m_ElevatorSubsystem;

  /** Creates a new L4. */
  public PutL4_Auto(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem) {
    this.m_ArmSubsystem = armSubsystem;
    this.m_ElevatorSubsystem = elevatorSubsystem;
    addRequirements(m_ArmSubsystem, m_ElevatorSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ArmSubsystem.coast();
    m_ArmSubsystem.readyL4_Pivot();
    m_ElevatorSubsystem.putL4();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((m_ArmSubsystem.arriveSetpoint() && m_ElevatorSubsystem.arriveSetpoint()) && LEDConstants.arriveSetpoint_Base) {
      m_ArmSubsystem.putL4_Pivot();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ArmSubsystem.brake();
    LEDConstants.arriveSetpoint_Base = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
