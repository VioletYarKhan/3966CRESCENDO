// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final Arm m_robotArm = new Arm();
    private final Shooter m_shooter = new Shooter();
    private final Intake m_intake = new Intake();
    // The driver's controller
    XboxController m_driverController = new XboxController(0);
    private double armMacroValue = m_robotArm.startAngle;
    private double autoArmMacro = 0;
    private boolean wantedValueMet = false;
    private boolean macroUp = false;
    private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    // Macros
    ParallelCommandGroup shooterMacro = new ParallelCommandGroup(
        new SequentialCommandGroup(
            new RunCommand(()->{
                m_shooter.shooterBack();
            }, m_shooter).withTimeout(0.1),

            new RunCommand(()->{
                m_shooter.shoot();
            }, m_shooter).withTimeout(2.3),

            new RunCommand(()->{
                m_shooter.stopShoot();
            }, m_shooter).withTimeout(0.7)
        ),
        new SequentialCommandGroup(
            new RunCommand(()->{
                m_intake.intakeBack();
            }, m_intake).withTimeout(0.07),

            new RunCommand(()->{
                m_intake.stopIntake();
            }
            , m_intake).withTimeout(2.13),

            new RunCommand(()->{
                m_intake.intake();
            }, m_intake).withTimeout(0.9)
        )
        
    );

    SequentialCommandGroup armMacro = new SequentialCommandGroup(
        new RunCommand(()->{
            if (m_driverController.getYButton() || autoArmMacro == 1) {
                armMacroValue = (m_robotArm.startAngle + 8.4); 
                if (m_robotArm.getEncoderValue() > armMacroValue){
                    macroUp = false;
                } else {
                    macroUp = true;
                }
            } else if (m_driverController.getXButton() || autoArmMacro == 2) {
                armMacroValue = (m_robotArm.startAngle + 34.5);
                if (m_robotArm.getEncoderValue() > (armMacroValue)){
                    macroUp = false;
                } else {
                    macroUp = true;
                }
            } else if (m_driverController.getAButton() || autoArmMacro == 3) {
                armMacroValue = (m_robotArm.startAngle);
                if (m_robotArm.getEncoderValue() > (armMacroValue - 0.2)){
                    macroUp = false;
                } else {
                    macroUp = true;
                }
            } else if (m_driverController.getBButton() || autoArmMacro == 4) {
                armMacroValue = (m_robotArm.startAngle + 6.4);
                if (m_robotArm.getEncoderValue() > (armMacroValue)){
                    macroUp = false;
                } else {
                    macroUp = true;
                }
            }
            if (macroUp){
                m_robotArm.armUpMacro();
            } else {
                m_robotArm.armDownMacro();
            }
            wantedValueMet = (((m_robotArm.getEncoderValue() >= armMacroValue - 0.05) && (macroUp)) || ((m_robotArm.getEncoderValue() <= armMacroValue + 0.1) && !(macroUp)));
            autoArmMacro = 0;
        }, m_robotArm).until(()-> (wantedValueMet || (m_robotArm.getEncoderValue() < m_robotArm.startAngle - 5) || (m_robotArm.getEncoderValue() > m_robotArm.startAngle + 37))));
        

    // Auto Commands
    NamedCommands.registerCommand("toFloor", new RunCommand(()->{autoArmMacro = 3;armMacro.schedule();}).asProxy().until(()-> (wantedValueMet || (m_robotArm.getEncoderValue() < m_robotArm.startAngle - 5) || (m_robotArm.getEncoderValue() > m_robotArm.startAngle + 37))));
    NamedCommands.registerCommand("toAmp", new RunCommand(()->{autoArmMacro = 2;armMacro.schedule();}).asProxy().until(()-> (wantedValueMet || (m_robotArm.getEncoderValue() < m_robotArm.startAngle - 5) || (m_robotArm.getEncoderValue() > m_robotArm.startAngle + 37))));
    NamedCommands.registerCommand("toSpeakerFlat", new RunCommand(()->{autoArmMacro = 1;armMacro.schedule();}).asProxy().until(()-> (wantedValueMet || (m_robotArm.getEncoderValue() < m_robotArm.startAngle - 5) || (m_robotArm.getEncoderValue() > m_robotArm.startAngle + 37))));
    NamedCommands.registerCommand("toSpeakerAngled", new RunCommand(()->{autoArmMacro = 4;armMacro.schedule();}).asProxy().until(()-> (wantedValueMet || (m_robotArm.getEncoderValue() < m_robotArm.startAngle - 5) || (m_robotArm.getEncoderValue() > m_robotArm.startAngle + 37))));
    NamedCommands.registerCommand("shoot", new RunCommand(()->{shooterMacro.schedule();}).asProxy().withTimeout(3.1));
    NamedCommands.registerCommand("intake", new RunCommand(()->m_intake.intake(), m_intake).asProxy().withTimeout(0.4));
    NamedCommands.registerCommand("stopShoot", new RunCommand(()-> {m_shooter.stopShoot();}, m_shooter).asProxy().withTimeout(0.2));
    NamedCommands.registerCommand("stopIntake", new RunCommand(()-> {m_intake.stopIntake();}, m_intake).asProxy().withTimeout(0.2));

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> {
                if (m_driverController.getRightBumper()){
                    m_robotDrive.setX();
                } else{
                m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband), true, true);}},
            m_robotDrive));
    m_robotArm.setDefaultCommand(
            new RunCommand(() -> {
                SmartDashboard.putNumber("Arm Encoder", m_robotArm.getEncoderValue());
                SmartDashboard.putNumber("Arm Encoder Start", m_robotArm.startAngle);
                if (armMacro.isScheduled() != true){
                    if (m_driverController.getYButtonPressed() || m_driverController.getXButtonPressed() || m_driverController.getAButtonPressed() || m_driverController.getBButtonPressed()){
                        armMacro.schedule();
                    } else {
                        if (m_driverController.getPOV() == 0) {
                            m_robotArm.armUp();
                        } else if (m_driverController.getPOV() == 180) {
                            m_robotArm.armDown();
                        } else{
                            m_robotArm.armHold();
                        }
                    }
                    if (m_driverController.getPOV() == 270){
                        m_robotArm.resetStartValue();
                    }
                }
            }, m_robotArm)
        );


    m_shooter.setDefaultCommand(new RunCommand(() -> {
        if (shooterMacro.isScheduled() != true){
            if (m_driverController.getRightTriggerAxis() > 0.5){
                shooterMacro.schedule();
            } else {
                if (m_driverController.getPOV() == 90){
                    m_shooter.shoot();
                } else if (m_driverController.getLeftBumper()){
                    m_shooter.shooterBack();
                } else {
                    m_shooter.stopShoot();
                }
            }
        }
    }, m_shooter));

    m_intake.setDefaultCommand(new RunCommand(()-> {
        if (shooterMacro.isScheduled() != true){
            if (m_driverController.getLeftTriggerAxis() > 0.5) {
                    m_intake.intake();
            } else if (m_driverController.getLeftBumper()){
                m_intake.intakeBack();
            } else {
                m_intake.stopIntake();
            }
        }
    }, m_intake));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}