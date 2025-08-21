// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PutL2 extends Command {
  private ArmSubsystem m_ArmSubsystem;
  private ElevatorSubsystem m_ElevatorSubsystem;
  private BooleanSupplier ifFeedFunc;
  private boolean ifFeed;

  /** Creates a new L4. */
  public PutL2(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, BooleanSupplier ifFeedFunc) {
    this.m_ArmSubsystem = armSubsystem;
    this.m_ElevatorSubsystem = elevatorSubsystem;
    this.ifFeedFunc = ifFeedFunc;
    addRequirements(m_ArmSubsystem, m_ElevatorSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ElevatorSubsystem.toSafePosition();
    LEDConstants.isAlgaeMode = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ifFeed = ifFeedFunc.getAsBoolean();
    if (m_ElevatorSubsystem.isSafe()) {
      m_ArmSubsystem.readyL2_Pivot();
    }
    if (Math.abs(m_ArmSubsystem.getAngle_Degrees_PID() - ArmConstants.coralL2ReadyPosition) <= 1) {
      m_ElevatorSubsystem.putL2();
    }
    if((m_ArmSubsystem.arriveSetpoint() && m_ElevatorSubsystem.arriveSetpoint()) && (ifFeed || LEDConstants.arriveSetpoint_Base) && m_ElevatorSubsystem.getMode() == "putL2") {
      m_ArmSubsystem.putL2_Pivot();
      m_ArmSubsystem.putL2_Wheel();

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
