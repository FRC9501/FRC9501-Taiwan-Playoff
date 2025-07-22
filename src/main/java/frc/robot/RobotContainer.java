package frc.robot;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoElevatorCmd;
import frc.robot.commands.AutoLeftVisionCmd;
import frc.robot.commands.AutoRightVisionCmd;
// import frc.robot.commands.AutoShootCmd;
// import frc.robot.commands.AutointakeCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Climber.climber;
import frc.robot.subsystems.Climber.climberCmd;
// import frc.robot.subsystems.Intake.IntakeSub;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import frc.robot.subsystems.Vision.LeftReefVision;
import frc.robot.subsystems.Vision.RightReefVision;
import frc.robot.subsystems.elevator.elevator;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class RobotContainer {
  private final LeftReefVision leftVision = new LeftReefVision();
  private final RightReefVision rightVision = new RightReefVision();
  private final elevator elevator = new elevator();
  private boolean isL1Active = false;
  private boolean isL2Active = false;
  private boolean isL3Active = false;
  
  // 新增變數來追蹤algaeGetter的狀態
  private boolean isDefaultPositionActive = false; 
  private boolean isReefPositionActive = false;
  private boolean isProPositionActive = false;
  private boolean isBargePositionActive = false;

  private final DriveSubsystem swerveSubsystem = new DriveSubsystem();
  private final Joystick joystick = new Joystick(OIConstants.kDriverControllerPort);
  // private final IntakeSub intakeSub = new IntakeSub();
  private final GenericHID button = new GenericHID(1);
  private double climberPosition = 0;
  private final climber climber = new climber();

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
    // NamedCommands.registerCommand("elevator",new AutoElevatorCmd(elevator,intakeSub).until(()-> elevator.get() >38));
    NamedCommands.registerCommand("Vision", new SwerveJoystickCmd(swerveSubsystem, 
       () -> -leftVision.ySpeedOutput() ,
       () -> leftVision.xSpeedOutput(), 
       () -> 0.0,
       () -> false).withTimeout(1.2));
    NamedCommands.registerCommand("Right", new SwerveJoystickCmd(swerveSubsystem, 
       () -> -rightVision.ySpeedOutput(),
       () -> rightVision.xSpeedOutput(), 
       () -> 0.0,
       () -> false).withTimeout(1.2));

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
    
    elevator.resetEncoder();
  }

  private void configureBindings() {
  //控制elevator的按鈕設置
    new JoystickButton(joystick, Button.kX.value).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

    up.whileTrue(new InstantCommand(() -> {
      climberPosition+=1;}));
    down.whileTrue(new InstantCommand(() -> {
      climberPosition-=1;}));
    up.whileTrue(new climberCmd(climber,()-> climberPosition));
    down.whileTrue(new climberCmd(climber,()-> climberPosition));

    new JoystickButton(button, 2).onTrue(new RunCommand(() ->elevator.L0(), elevator));
    new JoystickButton(button, 5).onTrue(new RunCommand(() ->elevator.L1(), elevator));
    new JoystickButton(button, 8).onTrue(new RunCommand(() ->elevator.L2(), elevator));
    new JoystickButton(button, 11).onTrue(new RunCommand(() ->elevator.L3(), elevator));

    new JoystickButton(joystick, Button.kA.value).onTrue(new InstantCommand(()-> leftVision.chagePiepeline()));
    new JoystickButton(joystick, Button.kY.value).onTrue(new InstantCommand(()-> rightVision.chagePiepeline()));

    new JoystickButton(joystick, Button.kA.value).whileTrue(new SwerveJoystickCmd(swerveSubsystem, 
       () -> -leftVision.ySpeedOutput(),
       () -> leftVision.xSpeedOutput(), 
       () -> 0.0,
       () -> false
    ));

    new JoystickButton(joystick, Button.kY.value).whileTrue(new SwerveJoystickCmd(swerveSubsystem, 
    () -> -rightVision.ySpeedOutput(),
    () -> rightVision.xSpeedOutput(), 
    () -> 0.0,
    () -> false
 ));
  }

  public PathPlannerAuto getAutonomousCommand() {
    return new PathPlannerAuto(m_chooser.getSelected());
  }
}
