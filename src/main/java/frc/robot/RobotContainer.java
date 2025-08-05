package frc.robot;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TakeAlgae_Low;
import frc.robot.commands.TakeAlgae_High;
import frc.robot.commands.Getcoral;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.IntakeStopCmd;
import frc.robot.commands.PutProcesser;
import frc.robot.commands.PutL1;
import frc.robot.commands.PutL2;
import frc.robot.commands.PutL3;
import frc.robot.commands.PutL4;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveDriveSubsystem;
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

  private final SwerveDriveSubsystem swerveSubsystem = new SwerveDriveSubsystem();
  private final Joystick joystick = new Joystick(OIConstants.kDriverControllerPort);
  // private final IntakeSub intakeSub = new IntakeSub();
  private final CommandJoystick button = new CommandJoystick(1);
  private double climberPosition = 0;

  private String m_autoSelected;
  private static final String Test = "test";
  private static final String BlueMR = "Blue MR";
  private static final String BlueML = "Blue ML";
  private static final String BlueR = "Blue R";
  private static final String BlueM = "Blue M";

  private final POVButton up = new POVButton(joystick, 0);
  private final POVButton down = new POVButton(joystick, 180);
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



    swerveSubsystem.setDefaultCommand(
      new SwerveJoystickCmd(swerveSubsystem, 
      () -> -joystick.getRawAxis(OIConstants.kDriverXAxis),
      () -> -joystick.getRawAxis(OIConstants.kDriverYAxis), 
      () -> -joystick.getRawAxis(OIConstants.kDriverRotAxis), 
      () -> true
    ));
    configureBindings();
    
    m_ElevatorSubsystem.resetEncoder();
  }



  private void configureBindings() {
    new JoystickButton(joystick, Button.kX.value).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

    up.whileTrue(new InstantCommand(() -> {
      climberPosition+=1;}));
    down.whileTrue(new InstantCommand(() -> {
      climberPosition-=1;}));
    // up.whileTrue(new ClimberCmd(climber,()-> climberPosition));
    // down.whileTrue(new ClimberCmd(climber,()-> climberPosition));


  //控制按鈕設置
  BooleanSupplier ifFeedFunc = () -> button.button(1).getAsBoolean();  
  button.button(2).onTrue(new Getcoral(m_PawSubsystem, m_ElevatorSubsystem));
  button.button(3).onTrue(new PutL1(m_ElevatorSubsystem, m_PawSubsystem, ifFeedFunc));
  button.button(4).onTrue(new PutL2(m_ElevatorSubsystem, m_PawSubsystem, ifFeedFunc));
  button.button(5).onTrue(new PutL3(m_ElevatorSubsystem, m_PawSubsystem, ifFeedFunc));
  button.button(6).onTrue(new PutL4(m_ElevatorSubsystem, m_PawSubsystem, ifFeedFunc));
  button.button(7).onTrue(new TakeAlgae_Low(m_ElevatorSubsystem, m_PawSubsystem, ifFeedFunc));
  button.button(8).onTrue(new TakeAlgae_High(m_ElevatorSubsystem, m_PawSubsystem, ifFeedFunc));
  button.button(9).onTrue(new PutProcesser(m_ElevatorSubsystem, m_PawSubsystem, ifFeedFunc));
  button.button(10).onTrue(new IntakeCmd(m_IntakeSubsystem));
  button.button(11).onTrue(new IntakeStopCmd(m_PawSubsystem, m_IntakeSubsystem));
  }



  public PathPlannerAuto getAutonomousCommand() {
    return new PathPlannerAuto(m_chooser.getSelected());
  }
}
