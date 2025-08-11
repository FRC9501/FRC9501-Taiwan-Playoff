package frc.robot;

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
  BooleanSupplier ifFeed  = ()-> driverController.getRawAxis(3) <= 0.4;
  DoubleSupplier xSpeedFunc = ()-> driverController.getRawAxis(1);
  DoubleSupplier ySpeedFunc = ()-> driverController.getRawAxis(0);
  DoubleSupplier zSpeedFunc = ()-> driverController.getRawAxis(4);
  
  driverController.button(6).whileTrue(new RightVisionCmd(m_RightVisionSubsystem, m_SwerveSubsystem, xSpeedFunc, ySpeedFunc, zSpeedFunc));
  driverController.button(5).whileTrue(new LeftVisionCmd(m_LeftVisionSubsystem, m_SwerveSubsystem, xSpeedFunc, ySpeedFunc, zSpeedFunc));
  m_SwerveSubsystem.setDefaultCommand(new ManualDrive(m_SwerveSubsystem, xSpeedFunc, ySpeedFunc, zSpeedFunc));
  driverController.button(3).whileTrue(Commands.runOnce(() -> m_SwerveSubsystem.resetGyro()));
  //按鈕盤 
}


  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

}