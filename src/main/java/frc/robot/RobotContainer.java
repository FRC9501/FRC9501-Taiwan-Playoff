package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.FloorAlgae;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.TraclLeftReef;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.PutL1;
import frc.robot.commands.PutL2;
import frc.robot.commands.PutL3;
import frc.robot.commands.PutL4;
import frc.robot.commands.Auto.PutL4_Auto;
import frc.robot.commands.Auto.TrackLeftReef_Auto;
import frc.robot.commands.Auto.TrackMiddleReef_Auto;
import frc.robot.commands.Auto.TrackRightReef_Auto;
import frc.robot.commands.PutNet;
import frc.robot.commands.PutProcessor;
import frc.robot.commands.ToPrimitive;
import frc.robot.commands.TrackRightReef;
import frc.robot.commands.TakeHighAlgae;
import frc.robot.commands.TakeLowAlgae;
import frc.robot.subsystems.LeftVisionSubsystem;
import frc.robot.subsystems.RightVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ArmSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  private final RightVisionSubsystem m_RightVisionSubsystem = new RightVisionSubsystem();
  private final LeftVisionSubsystem m_LeftVisionSubsystem = new LeftVisionSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandJoystick panel = new CommandJoystick(OperatorConstants.kpanelPort);

  private final SendableChooser<Command> m_chooser = AutoBuilder.buildAutoChooser();

  public RobotContainer() {
    NamedCommands.registerCommand("TrackRightReef", new TrackRightReef_Auto(m_RightVisionSubsystem, m_SwerveSubsystem));
    NamedCommands.registerCommand("TrackLeftReef", new TrackLeftReef_Auto(m_LeftVisionSubsystem, m_SwerveSubsystem));
    NamedCommands.registerCommand("TrackMiddleReef", new TrackMiddleReef_Auto(m_RightVisionSubsystem, null, m_SwerveSubsystem));
    NamedCommands.registerCommand("TakeHighAlgae", new TakeHighAlgae(m_ArmSubsystem, m_ElevatorSubsystem));
    NamedCommands.registerCommand("PutL4", new PutL4_Auto(m_ArmSubsystem, m_ElevatorSubsystem));
    NamedCommands.registerCommand("TakeLowAlgae", new TakeLowAlgae(m_ArmSubsystem, m_ElevatorSubsystem));

    SmartDashboard.putData("AutoMode", m_chooser);
    configureBindings();
  }

  private void configureBindings() {
  //搖桿
  BooleanSupplier ifFeed  = ()-> driverController.getRightTriggerAxis() > 0.5;
  DoubleSupplier xSpeedFunc = ()-> driverController.getRawAxis(1);
  DoubleSupplier ySpeedFunc = ()-> driverController.getRawAxis(0);
  DoubleSupplier zSpeedFunc = ()-> driverController.getRawAxis(4);
  driverController.leftTrigger().whileTrue(new IntakeCoral(m_ArmSubsystem, m_ElevatorSubsystem));
  driverController.rightBumper().whileTrue(new TraclLeftReef(m_LeftVisionSubsystem, m_SwerveSubsystem, xSpeedFunc, ySpeedFunc, zSpeedFunc));
  driverController.leftBumper().whileTrue(new TrackRightReef(m_RightVisionSubsystem, m_SwerveSubsystem, xSpeedFunc, ySpeedFunc, zSpeedFunc));
  m_SwerveSubsystem.setDefaultCommand(new ManualDrive(m_SwerveSubsystem, xSpeedFunc, ySpeedFunc, zSpeedFunc));
  driverController.x().whileTrue(Commands.runOnce(() -> m_SwerveSubsystem.resetGyro()));
  //按鈕盤 
  panel.button(0).onTrue(new PutNet(m_ArmSubsystem, m_ElevatorSubsystem, ifFeed));
  panel.button(1).onTrue(new TakeHighAlgae(m_ArmSubsystem, m_ElevatorSubsystem));
  panel.button(2).onTrue(new TakeLowAlgae(m_ArmSubsystem, m_ElevatorSubsystem));
  panel.button(3).onTrue(new PutProcessor(m_ArmSubsystem, m_ElevatorSubsystem, ifFeed));
  panel.button(4).onTrue(new FloorAlgae(m_ArmSubsystem, m_ElevatorSubsystem, ifFeed));
  panel.button(5).onTrue(new PutL4(m_ArmSubsystem, m_ElevatorSubsystem, ifFeed));
  panel.button(6).onTrue(new PutL3(m_ArmSubsystem, m_ElevatorSubsystem, ifFeed));
  panel.button(7).onTrue(new PutL2(m_ArmSubsystem, m_ElevatorSubsystem, ifFeed));
  panel.button(8).onTrue(new PutL1(m_ArmSubsystem, m_ElevatorSubsystem, ifFeed));
  panel.button(9).onTrue(new ToPrimitive(m_ElevatorSubsystem, m_ArmSubsystem));

}


  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

}