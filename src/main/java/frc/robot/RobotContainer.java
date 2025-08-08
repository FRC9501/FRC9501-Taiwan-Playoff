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
// import frc.robot.commands.LeftVisionCmd;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.RightVisionCmd;
import frc.robot.subsystems.LeftVisionSubsystem;
import frc.robot.subsystems.RightVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.RightVisionSubsystem;
import frc.robot.subsystems.LeftVisionSubsystem;

import java.time.Instant;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.subsystems.Vision;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PawSubsystem;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class RobotContainer {
  // private final Vision leftVision = new Vision();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();  
  private final PawSubsystem m_PawSubsystem = new PawSubsystem();
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  private final RightVisionSubsystem m_RightVisionSubsystem = new RightVisionSubsystem();
  private final LeftVisionSubsystem m_LeftVisionSubsystem = new LeftVisionSubsystem();

  // private final IntakeSub intakeSub = new IntakeSub();
  private final CommandJoystick button = new CommandJoystick(1);
  private final CommandJoystick driverController = new CommandJoystick(OperatorConstants.kDriverControllerPort);


  private String m_autoSelected;
  private static final String Test = "test";
  private static final String BlueMR = "Blue MR";
  private static final String BlueML = "Blue ML";
  private static final String BlueR = "Blue R";
  private static final String BlueM = "Blue M";

  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  

  public RobotContainer() {
    // NamedCommands.registerCommand("intake", new AutointakeCmd(intakeSub).until(() -> intakeSub.Distance() > 1000));
    // NamedCommands.registerCommand("shoot", new AutoShootCmd(intakeSub,elevator));
    // // NamedCommands.registerCommand("elevator",new AutoElevatorCmd(elevator,intakeSub).until(()-> elevator.get() >38));
    // NamedCommands.registerCommand("Vision", new SwerveJoystickCmd(swerveSubsystem, 
    //    () -> -leftVision.ySpeedOutput() ,
    //    () -> leftVision.xSpeedOutput(), 
    //    () -> 0.0,
    //    () -> false).withTimeout(1.2));
    // NamedCommands.registerCommand("Right", new SwerveJoystickCmd(swerveSubsystem, 
    //    () -> -rightVision.ySpeedOutput(),
    //    () -> rightVision.xSpeedOutput(), 
    //    () -> 0.0,
    //    () -> false).withTimeout(1.2));

    m_chooser.setDefaultOption("test", Test);
    m_chooser.addOption("Blue MR", BlueMR);
    m_chooser.addOption("Blue ML", BlueML);
    m_chooser.addOption("Blue M", BlueM);
    m_chooser.addOption("Blue R", BlueR);
    m_autoSelected =  m_chooser.getSelected();
    SmartDashboard.putData("Auto choices", m_chooser);  
    System.out.println("Auto Selected" + m_autoSelected);

    switch (m_autoSelected) {
      case Test:
        break;
      case BlueMR:
        break;
      case BlueML:
        break;
      case BlueM:
        break;
      case BlueR:
        break;
    }




  }

  private void configureBindings() {

  //搖桿
  BooleanSupplier ifFeedFunc = () -> button.button(1).getAsBoolean();  
  button.button(10).whileTrue(new IntakeCmd(m_IntakeSubsystem));
  button.button(11).whileTrue(new IntakeStopCmd(m_PawSubsystem, m_IntakeSubsystem));
  button.button(2).whileTrue(new Getcoral(m_PawSubsystem, m_ElevatorSubsystem));
  DoubleSupplier xSpeedFunc = ()-> driverController.getRawAxis(1);
  DoubleSupplier ySpeedFunc = ()-> driverController.getRawAxis(0);
  DoubleSupplier zSpeedFunc = ()-> driverController.getRawAxis(4);
  driverController.button(6).whileTrue(new RightVisionCmd(m_RightVisionSubsystem, m_SwerveSubsystem, xSpeedFunc, ySpeedFunc, zSpeedFunc));
  driverController.button(5).whileTrue(new LeftVisionCmd(m_LeftVisionSubsystem, m_SwerveSubsystem, xSpeedFunc, ySpeedFunc, zSpeedFunc));

  BooleanSupplier isSlowFunc = ()-> driverController.getHID().getRawButton(6);

  m_SwerveSubsystem.setDefaultCommand(new ManualDrive(m_SwerveSubsystem, xSpeedFunc, ySpeedFunc, zSpeedFunc, isSlowFunc));
  driverController.button(3).whileTrue(Commands.runOnce(() -> m_SwerveSubsystem.resetGyro()));
  //按鈕盤
  button.button(3).whileTrue(new PutL1(m_ElevatorSubsystem, m_PawSubsystem, ifFeedFunc));
  button.button(4).whileTrue(new PutL2(m_ElevatorSubsystem, m_PawSubsystem, ifFeedFunc));
  button.button(5).whileTrue(new PutL3(m_ElevatorSubsystem, m_PawSubsystem, ifFeedFunc));
  button.button(6).whileTrue(new PutL4(m_ElevatorSubsystem, m_PawSubsystem, ifFeedFunc));
  button.button(7).whileTrue(new TakeAlgae_Low(m_ElevatorSubsystem, m_PawSubsystem, ifFeedFunc));
  button.button(8).whileTrue(new TakeAlgae_High(m_ElevatorSubsystem, m_PawSubsystem, ifFeedFunc));
  button.button(9).whileTrue(new PutProcesser(m_ElevatorSubsystem, m_PawSubsystem, ifFeedFunc));
  button.button(0).whileTrue(new Ready(m_ElevatorSubsystem, m_PawSubsystem));
  
}


  public PathPlannerAuto getAutonomousCommand() {
    return new PathPlannerAuto(m_chooser.getSelected());
  }

}