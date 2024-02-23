// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAim;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveZero;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SwerveCommandField;
import frc.robot.commands.ZeroGyro;
import frc.robot.commands.runIntakeforTime;
import frc.robot.commands.setHeadingCorrection;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Shooter.ShooterState;

import java.io.File;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  
  private final Shooter shooter = new Shooter(drivebase);
  private final Intake intakemaxxxer = new Intake();
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
          () -> MathUtil.applyDeadband(m_controller1.getRawAxis(0), OperatorConstants.LEFT_Y_DEADBAND),
          () -> MathUtil.applyDeadband(m_controller1.getRawAxis(1), OperatorConstants.LEFT_X_DEADBAND),
          () -> m_controller2.getX());
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

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
    m_controller1.button(1).toggleOnTrue(new RunIntake(intakemaxxxer,-1));
    m_controller1.button(2).toggleOnTrue(new RunIntake(intakemaxxxer, 1));
    m_controller2.button(2).toggleOnTrue(shooter.shootNote());
    m_controller1.button(11).onTrue(new ZeroGyro(drivebase));
    m_controller1.button(3).onTrue(shooter.setShooterMode(ShooterState.SPEAKERAIM));
    m_controller1.button(4).onTrue(shooter.setShooterMode(ShooterState.AMPAIM));
    m_controller1.button(5).onTrue(shooter.setShooterMode(ShooterState.STOW));
    
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
    PathPlannerPath path = PathPlannerPath.fromPathFile("test");
    
    drivebase.resetOdometry(path.getPreviewStartingHolonomicPose());
    //drivebase.driveToPose(new Pose2d(drivebase.getPose().getX(),drivebase.getPose().getY(),Rotation2d.fromDegrees(0)));
    return new SequentialCommandGroup(
     new setHeadingCorrection(drivebase, true),
     new ZeroGyro(drivebase),
     drivebase.postPathplannerPath("test"),
     new DriveZero(drivebase),
     drivebase.postPathplannerPath("test2"),
     new DriveZero(drivebase),
     drivebase.postPathplannerPath("test3"),
     new DriveZero(drivebase),
     drivebase.postPathplannerPath("test4"),
     new DriveZero(drivebase),
     new setHeadingCorrection(drivebase, false)
    );
}
}

