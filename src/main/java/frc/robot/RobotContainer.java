// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveZero;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.LED_VIBE;
import frc.robot.commands.MoveClimber;
import frc.robot.commands.PivotManual;
import frc.robot.commands.RunBTS;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ShootNote;
import frc.robot.commands.SwerveCommandField;
import frc.robot.commands.ZeroGyro;
import frc.robot.commands.runBTSfortime;
import frc.robot.commands.runIntakeforTime;
import frc.robot.commands.setHeadingCorrection;
import frc.robot.subsystems.BTS;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.PivotState;
import frc.robot.subsystems.Shooter.ShooterState;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
  // Control Devices
  
  //public  final LED ledManager = new LED();
  private final Shooter shooter = new Shooter(drivebase);
  Command oneNote;
  private final BTS bts = new BTS();
  private final Intake intakemaxxxer = new Intake();
  private final Climber climberLeft = new Climber(true);
  private final Climber climberRight = new Climber(false);

  private final CommandJoystick m_controller1 =
      new CommandJoystick(OperatorConstants.JOYSTICK_1_PORT);
  private final CommandJoystick m_controller2 =
      new CommandJoystick(OperatorConstants.JOYSTICK_2_PORT);
  private final CommandJoystick m_guitar =
      new CommandJoystick(OperatorConstants.GUITAR_PORT);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
        configureBindings();
        Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
          () -> MathUtil.applyDeadband(m_controller1.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
          () -> MathUtil.applyDeadband(m_controller1.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
          () -> m_controller2.getX());
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      oneNote = new ParallelCommandGroup(new AutoShoot(shooter, 1), new runBTSfortime(bts,1,1));
      NamedCommands.registerCommand("intake", intakemaxxxer.setIntakeState(IntakeState.INTAKING));
      NamedCommands.registerCommand("ZeroGyro",new ZeroGyro(drivebase));
      NamedCommands.registerCommand("shoot", oneNote);
      NamedCommands.registerCommand("SetHeadingCorrectionTrue", new setHeadingCorrection(drivebase, true));
      NamedCommands.registerCommand("SetHeadingCorrectionFalse", new setHeadingCorrection(drivebase, false));
      NamedCommands.registerCommand("stop", new DriveZero(drivebase));
      Command intake = NamedCommands.getCommand("intake");
      Command resetGyro = NamedCommands.getCommand("ZeroGyro");
      Command shoot = oneNote;
      Command headingTrue = NamedCommands.getCommand("SetHeadingCorrectionTrue");
      Command headingFalse = NamedCommands.getCommand("SetHeadingCorrectionFalse");
      Command stop = NamedCommands.getCommand("stop");
      
      //set up autos
      // SequentialCommandGroup sourceAutothreeNote = new SequentialCommandGroup(
      //   headingTrue,
      //   resetGyro,
      //   drivebase.postPathplannerPath("sourceside1"),
      //   stop,
      //   shoot,
      //   drivebase.postPathplannerPath("sourceside2"),
      //   stop,
      //   intake,
      //   drivebase.postPathplannerPath("sourceside3"),
      //   stop,
      //   shoot,
      //   drivebase.postPathplannerPath("sourceside4"),
      //   stop,
      //   intake,
      //   drivebase.postPathplannerPath("sourceside5"),
      //   stop,
      //   shoot
      // );
      // SequentialCommandGroup fourNote = new SequentialCommandGroup(
      //   headingTrue,
      //   resetGyro,
      //   shoot,
      //   drivebase.postPathplannerPath("4note1"),
      //   stop,
      //   intake,
      //   drivebase.postPathplannerPath("4note2"),
      //   stop,
      //   shoot,
      //   drivebase.postPathplannerPath("4note3"),
      //   stop,
      //   intake,
      //   drivebase.postPathplannerPath("4note4"),
      //   stop,
      //   shoot,
      //   drivebase.postPathplannerPath("4note5"),
      //   stop,
      //   intake,
      //   drivebase.postPathplannerPath("4note6"),
      //   stop,
      //   shoot,
      //   headingFalse
      // );
      // SequentialCommandGroup twonotemid = new SequentialCommandGroup(
      //     headingTrue,
      //     resetGyro,
      //     shoot,
      //     drivebase.postPathplannerPath("2note1"),
      //     stop,
      //     intake,
      //     drivebase.postPathplannerPath("2note2"),
      //     stop,
      //     shoot,
      //     headingFalse
      // );
      // SequentialCommandGroup threenoteMid = new SequentialCommandGroup(
      //   headingTrue,
      //   resetGyro,
      //   shoot,
      //   drivebase.postPathplannerPath("3notemid1"),
      //   stop,
      //   intake,
      //   drivebase.postPathplannerPath("3notemid2"),
      //   stop,
      //   shoot,
      //   drivebase.postPathplannerPath("3notemid3"),
      //   stop,
      //   intake,
      //   drivebase.postPathplannerPath("3notemid4"),
      //   stop,
      //   shoot,
      //   headingFalse
      //);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //configure some button bindings
    m_controller1.button(5).onTrue(new AutoAim(shooter, 45));
    m_controller2.button(2).whileTrue(new ParallelCommandGroup(new RunIntake(intakemaxxxer, -0.69), new RunBTS(bts, 0.15)));
    m_controller1.button(2).whileTrue(new RunIntake(intakemaxxxer,1));
    //m_controller2.button(2).toggleOnTrue(new ZeroGyro(drivebase));
    m_controller1.button(1).whileTrue(new RunIntake(intakemaxxxer, -0.69));
    m_controller1.button(11).onTrue(new ZeroGyro(drivebase));
    m_controller1.button(3).onTrue(shooter.setPivotMode(PivotState.SPEAKERAIM));
    m_controller1.button(4).onTrue(shooter.setPivotMode(PivotState.AMPAIM));
    m_controller1.button(5).onTrue(shooter.setPivotMode(PivotState.STOW));
    m_controller2.button(1).whileTrue(new ParallelCommandGroup(new RunBTS(bts,1), new ShootNote(shooter, 1)));
    m_controller2.button(3).whileTrue(new ShootNote(shooter, -0.5));
   // m_controller1.button(9).toggleOnTrue(new LED_VIBE(ledManager));
    m_controller2.button(5).whileTrue(new PivotManual(shooter, 0.75));
    m_controller2.button(6).whileTrue(new PivotManual(shooter, -0.75));
    m_guitar.button(1).whileTrue(new MoveClimber(climberLeft,  .5));
    m_guitar.button(2).whileTrue(new MoveClimber(climberRight,  .5));
    m_guitar.button(3).whileTrue(new MoveClimber(climberLeft, (.5*-1)));
    m_guitar.button(4).whileTrue(new MoveClimber(climberRight, (.5*-1)));

    
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("test");
    double x = drivebase.getPose().getX();
    double y = drivebase.getPose().getY();
    Rotation2d rotate = drivebase.getHeading();
    Pose2d zero = new Pose2d(drivebase.getPose().getX(),drivebase.getPose().getY(),Rotation2d.fromDegrees(0));
    //return new SequentialCommandGroup(drivebase.driveToPose(new Pose2d(x,y,rotate)), new DriveZero(drivebase));
    PathPlannerPath path = PathPlannerPath.fromPathFile("4note1");
    
    drivebase.resetOdometry(path.getPreviewStartingHolonomicPose());
    //drivebase.driveToPose(new Pose2d(drivebase.getPose().getX(),drivebase.getPose().getY(),Rotation2d.fromDegrees(0)));
    // return new SequentialCommandGroup(
    //  new setHeadingCorrection(drivebase, true),
    //  new ZeroGyro(drivebase),
    //  drivebase.postPathplannerPath("test"),
    //  new DriveZero(drivebase),
    //  drivebase.postPathplannerPath("test2"),
    //  new DriveZero(drivebase),
    //  drivebase.postPathplannerPath("test3"),
    //  new DriveZero(drivebase),
    //  drivebase.postPathplannerPath("test4"),
    //  new DriveZero(drivebase),
    //  new setHeadingCorrection(drivebase, false)
    // );
    return new SequentialCommandGroup(
      //new AutoShoot(shooter, 1),
      new setHeadingCorrection(drivebase, true),
      new ZeroGyro(drivebase),
      drivebase.postPathplannerPath("2note1"),
      new DriveZero(drivebase),
      new runIntakeforTime(intakemaxxxer,0.5, 0.5),
      drivebase.postPathplannerPath("2note2"),
     new DriveZero(drivebase),
     //new AutoShoot(shooter,1)
     new setHeadingCorrection(drivebase, false)
    );
    
}
}

