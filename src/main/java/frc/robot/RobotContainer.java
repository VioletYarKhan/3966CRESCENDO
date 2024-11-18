// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Combinations.aimShoot;
import frc.robot.commands.Shooter.shootCommands;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveController;

import java.io.File;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer{

  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  public final Arm m_robotArm = new Arm();
  public final Shooter m_shooter = new Shooter();
  public final Intake m_intake = new Intake();
 // A chooser for autonomous commands
 SendableChooser<Command> m_chooser = new SendableChooser<>();

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(){
    // Configure the trigger bindings
    configureBindings();

    TeleopDrive closedFieldRel = new TeleopDrive(
        drivebase,
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX(), () -> true);

    // Add commands to the autonomous command chooser

    // Put the chooser on the dashboard
    SmartDashboard.putData(m_chooser);
    
    drivebase.setDefaultCommand(closedFieldRel);
    
    m_robotArm.setDefaultCommand(
        new RunCommand(() -> {
            if (driverXbox.getPOV() == 0) {
                m_robotArm.armUp();
            } else if (driverXbox.getPOV() == 180) {
                m_robotArm.armDown();
            } else {
              m_robotArm.moveToPosition(); 
              if (driverXbox.getYButton()){
                  new aimShoot(m_shooter, m_intake, m_robotArm, ArmConstants.speakerFrontAngle).aimShootMacro.schedule();
              } else if (driverXbox.getXButton()){
                  new aimShoot(m_shooter, m_intake, m_robotArm, ArmConstants.ampAngle).aimShootMacro.schedule();
              } else if (driverXbox.getAButton()){
                  new aimShoot(m_shooter, m_intake, m_robotArm, ArmConstants.groundAngle).aimShootMacro.schedule();
              } else if (driverXbox.getBButton()){
                  new aimShoot(m_shooter, m_intake, m_robotArm, ArmConstants.speakerSideAngle).aimShootMacro.schedule();
              }
            }
            if (driverXbox.getPOV() == 270){
                m_robotArm.zeroOffset();
            }
        }, m_robotArm)
    );

    m_shooter.setDefaultCommand(new RunCommand(() -> {
      if (m_shooter.getCurrentCommand() == m_shooter.getDefaultCommand()){
        if (driverXbox.getRightTriggerAxis() > 0.5){
            new shootCommands(m_shooter, m_intake).shootMacro.schedule();
        } else {
            if (driverXbox.getPOV() == 90){
                m_shooter.shoot();
            } else if (driverXbox.getLeftBumper()){
                m_shooter.shooterBack();
            } else {
                m_shooter.stopShoot();
            }
        }
      }
    }, m_shooter));

    m_intake.setDefaultCommand(new RunCommand(()-> {
        if (m_intake.getCurrentCommand() == m_intake.getDefaultCommand()){
            if (driverXbox.getLeftTriggerAxis() > 0.5) {
                    m_intake.intake();
            } else if (driverXbox.getLeftBumper()){
                m_intake.intakeBack();
            } else {
                m_intake.stopIntake();
            }
        }
    }, m_intake));
  }
  


  private void configureBindings(){
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */


  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public void setDriveMode(){
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake){
    // drivebase.setMotorBrake(brake);
  }
}
