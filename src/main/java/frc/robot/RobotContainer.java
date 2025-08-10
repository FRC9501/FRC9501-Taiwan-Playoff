package frc.robot;

import frc.robot.commands.TakeAlgae_Low;
import frc.robot.commands.Ready;
import frc.robot.commands.TakeAlgae_High;
import frc.robot.commands.Getcoral;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.IntakeStopCmd;
import frc.robot.commands.PutProcesser;
import frc.robot.commands.PutL1;
import frc.robot.commands.PutL2;
import frc.robot.commands.PutL3;
import frc.robot.commands.PutL4;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.LeftVisionCmd;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.RightVisionCmd;
import frc.robot.subsystems.LeftVisionSubsystem;
import frc.robot.subsystems.RightVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  // private final Vision leftVision = new Vision();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();  
  private final ArmSubsystem m_PawSubsystem = new ArmSubsystem();
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  private final RightVisionSubsystem m_RightVisionSubsystem = new RightVisionSubsystem();
  private final LeftVisionSubsystem m_LeftVisionSubsystem = new LeftVisionSubsystem();

  private final CommandJoystick button = new CommandJoystick(1);
  private final CommandJoystick driverController = new CommandJoystick(OperatorConstants.kDriverControllerPort);

  private final SendableChooser<Command> m_chooser = AutoBuilder.buildAutoChooser();

  public RobotContainer() {
    SmartDashboard.putData("AutoMode", m_chooser);
    configureBindings();
  }

  private void configureBindings() {
  //搖桿
  button.button(10).whileTrue(new IntakeCmd(m_IntakeSubsystem));
  button.button(11).whileTrue(new IntakeStopCmd(m_PawSubsystem, m_IntakeSubsystem));
  button.button(2).whileTrue(new Getcoral(m_PawSubsystem, m_ElevatorSubsystem));

  DoubleSupplier xSpeedFunc = ()-> driverController.getRawAxis(1);
  DoubleSupplier ySpeedFunc = ()-> driverController.getRawAxis(0);
  DoubleSupplier zSpeedFunc = ()-> driverController.getRawAxis(4);
  BooleanSupplier isSlowFunc = ()-> driverController.getHID().getRawButton(6);
  driverController.button(6).whileTrue(new RightVisionCmd(m_RightVisionSubsystem, m_SwerveSubsystem, xSpeedFunc, ySpeedFunc, zSpeedFunc));
  driverController.button(5).whileTrue(new LeftVisionCmd(m_LeftVisionSubsystem, m_SwerveSubsystem, xSpeedFunc, ySpeedFunc, zSpeedFunc));
  m_SwerveSubsystem.setDefaultCommand(new ManualDrive(m_SwerveSubsystem, xSpeedFunc, ySpeedFunc, zSpeedFunc, isSlowFunc));
  driverController.button(3).whileTrue(Commands.runOnce(() -> m_SwerveSubsystem.resetGyro()));
  //按鈕盤
  BooleanSupplier ifFeedFunc = () -> button.button(1).getAsBoolean();  
  button.button(3).whileTrue(new PutL1(m_ElevatorSubsystem, m_PawSubsystem, ifFeedFunc));
  button.button(4).whileTrue(new PutL2(m_ElevatorSubsystem, m_PawSubsystem, ifFeedFunc));
  button.button(5).whileTrue(new PutL3(m_ElevatorSubsystem, m_PawSubsystem, ifFeedFunc));
  button.button(6).whileTrue(new PutL4(m_ElevatorSubsystem, m_PawSubsystem, ifFeedFunc));
  button.button(7).whileTrue(new TakeAlgae_Low(m_ElevatorSubsystem, m_PawSubsystem, ifFeedFunc));
  button.button(8).whileTrue(new TakeAlgae_High(m_ElevatorSubsystem, m_PawSubsystem, ifFeedFunc));
  button.button(9).whileTrue(new PutProcesser(m_ElevatorSubsystem, m_PawSubsystem, ifFeedFunc));
  button.button(0).whileTrue(new Ready(m_ElevatorSubsystem, m_PawSubsystem));
}


  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

}