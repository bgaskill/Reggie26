// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.auto.;;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    
  // The robot's subsystems
  private final ArmSubsystem m_arm = new ArmSubsystem();
  final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final FeederSubsystem m_feeder = new FeederSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ClimbSubsystem m_climber = new ClimbSubsystem();
  private final SendableChooser<Command> autoChooser;
  // The driver's controller
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  
  public RobotContainer() {
    //NamedCommands.registerCommand("autoTest", new RunCommand(() -> System.out.println("AutoPrint")));
     // Build an auto chooser. This will use Commands.none() as the default option.
   NamedCommands.registerCommand("Shoot Close", new RunCommand(()
   -> m_shooter.shoot(.46 , 1), m_shooter).withTimeout(3).withName("Shoot Close"));
      NamedCommands.registerCommand("Start intake", new RunCommand(()
   -> m_intake.intake(.4,false, 1), m_intake).withTimeout(3).withName("Start intake"));
   NamedCommands.registerCommand("Stop intake", new RunCommand(()
   -> m_intake.intake(.4,false, 2), m_intake).withTimeout(3).withName("Stop intake"));
    NamedCommands.registerCommand("Lower arm", new RunCommand(()
   -> m_arm.move(-1), m_arm).withTimeout(3).withName("Lower arm"));
   NamedCommands.registerCommand("Raise arm", new RunCommand(()
   -> m_arm.move(1), m_arm).withTimeout(3).withName("Raise arm"));
    NamedCommands.registerCommand("Shoot mid", new RunCommand(()
   -> m_shooter.shoot(1, 1), m_shooter).withTimeout(3).withName("Shoot mid"));
   NamedCommands.registerCommand("Shoot Trench", new RunCommand(()
   -> m_shooter.shoot(.53, 1), m_shooter).withTimeout(3).withName("Shoot Trench"));
    NamedCommands.registerCommand("Feed", new RunCommand(()
   -> m_feeder.feed(1), m_feeder).withTimeout(.02).withName("Stop Feed"));
   NamedCommands.registerCommand("Stop Shoot", new RunCommand(()
   -> m_shooter.shoot(0, 2), m_shooter).withTimeout(.02).withName("Stop Shoot"));
   NamedCommands.registerCommand("Arm Dxown", new RunCommand(()
   -> m_shooter.shoot(0, 2), m_shooter).withTimeout(.02).withName("Stop Shoot"));

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    
    // Configure the button bindings
    configureButtonBindings();
    autoChooser = AutoBuilder.buildAutoChooser("Arm down intake");
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getTwist(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
    
    m_arm.setDefaultCommand(
        new RunCommand(
        () -> m_arm.move(m_operatorController.getRawAxis(5)*-.3), m_arm)
        );
    m_climber.setDefaultCommand(
        new RunCommand(
        () -> m_climber.climb(m_operatorController.getRawAxis(1)*1), m_climber)
        );
    
    
    //m_shooter.setDefaultCommand(
      //  new RunCommand(
       // () -> m_shooter.shooterStop(0),
       // m_feeder)
        // );      
    
         }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, 7)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    new JoystickButton(m_driverController, 1)
        .whileTrue(new RunCommand(
            () -> m_feeder.feed(1),
            m_feeder));

    new JoystickButton(m_operatorController, 1)
        .whileTrue(new RunCommand(
            () -> m_shooter.shoot(.45, 0),
            m_shooter));

    new JoystickButton(m_operatorController, 2)
        .whileTrue(new RunCommand(
            () -> m_shooter.shoot(.53, 0),
            m_shooter));

    new JoystickButton(m_operatorController, 4)
        .whileTrue(new RunCommand(
            () -> m_shooter.shoot(.55, 0),
            m_shooter));

    new JoystickButton(m_operatorController, 3)
        .whileTrue(new RunCommand(
            () -> m_shooter.shoot(.8, 0),
            m_shooter));

    new JoystickButton(m_operatorController, 5)
        .whileTrue(new RunCommand(
            () -> m_intake.intake(.4,false, 0),
            m_intake));

    new JoystickButton(m_operatorController, 6)
        .whileTrue(new RunCommand(
            () -> m_intake.intake(0.4, true, 0),
            m_intake));

    new JoystickButton(m_operatorController, 8)
        .whileTrue(new RunCommand(
            () -> m_shooter.shoot(0, 0),
            m_shooter));

    new JoystickButton(m_operatorController, 9)
        .whileTrue(new RunCommand(
            () -> m_arm.manual(),
            m_arm));
    new JoystickButton(m_operatorController, 10)
        .whileTrue(new RunCommand(
            () -> m_arm.zero(),
            m_arm));
    }
    

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //System.out.println("Auto");
    return autoChooser.getSelected();
  }
}
