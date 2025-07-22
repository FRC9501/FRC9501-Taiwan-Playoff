// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSub;
import frc.robot.subsystems.Elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PutAlgae extends Command {
  /** Creates a new L0. */
  private final Elevator m_elevatorSub;
  private final IntakeSub m_intakeSub;
  private final BooleanSupplier ifFeedFunc;
  private boolean coral;
  
  
  public PutAlgae(Elevator elevatorSub, IntakeSub intakeSub, BooleanSupplier ifFeedFunc) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_elevatorSub = elevatorSub;
    this.m_intakeSub = intakeSub;
    this.ifFeedFunc = ifFeedFunc;    
    addRequirements(m_intakeSub, m_elevatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSub.L4();
    m_intakeSub.netPosition();
    // This is where you would put any initialization code for the command.
    // For example, you might set a motor to a specific speed or position.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coral = ifFeedFunc.getAsBoolean();
    if (m_intakeSub.setpoint()&& m_elevatorSub.setpoint()&& coral == true) {
      m_intakeSub.readyPosition();
      m_intakeSub.shoot();
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
