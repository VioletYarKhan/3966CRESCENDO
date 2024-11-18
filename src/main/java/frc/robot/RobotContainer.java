// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    
    // The robot's subsystems
    public final DriveSubsystem m_robotDrive = new DriveSubsystem();
    public final Arm m_robotArm = new Arm();
    public final Shooter m_shooter = new Shooter();
    public final Intake m_intake = new Intake();
    // public final Limelight m_limelight = new Limelight();
    // The driver's controller

    XboxController m_driverController = new XboxController(0);
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    public Field2d m_field = new Field2d();
    
    // Macros

    public ParallelCommandGroup shooterMacro = new ParallelCommandGroup(
        new SequentialCommandGroup(
            new RunCommand(()->{
                m_shooter.shooterBack();
            }, m_shooter).withTimeout(0.1),

            new RunCommand(()->{
                m_shooter.shoot();
            }, m_shooter).until(()->m_shooter.getCurrentSpeed() > SmartDashboard.getNumber("Shoot Speed", 0.9) * 4.5),

            new RunCommand(()->{
                m_shooter.shoot();
            }, m_shooter).withTimeout(0.3),

            new RunCommand(()->{
                m_shooter.stopShoot();
            }, m_shooter).withTimeout(0.03)
        ),
        new SequentialCommandGroup(
            new RunCommand(()->{
                m_intake.intakeBack();
            }, m_intake).withTimeout(0.07),

            new RunCommand(()->{
                m_intake.stopIntake();
            }
            , m_intake).until(()->m_shooter.getCurrentSpeed() > SmartDashboard.getNumber("Shoot Speed", 0.9) * 4.5),

            new RunCommand(()->{
                m_intake.intake();
            }, m_intake).withTimeout(0.33)
        )
        
    );
        

    public RunCommand autoIntake = new RunCommand(()-> {
        m_intake.intake();
    }, m_intake);


  public RobotContainer() {
    // Data
    SmartDashboard.putNumber("Arm Encoder", m_robotArm.getEncoderValue());
    SmartDashboard.putNumber("Arm Encoder Start", m_robotArm.getEncoderValue());
    SmartDashboard.putData("Field", m_field);
    

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> {
                m_field.setRobotPose(m_robotDrive.getPose());
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
                if (m_driverController.getPOV() == 0) {
                    m_robotArm.armUp(); 
                } else if (m_driverController.getPOV() == 180) {
                    m_robotArm.armDown();
                } /*else if (m_driverController.getYButton()){
                    m_robotArm.setArmPosition(ArmConstants.speakerFrontAngle);
                } else if (m_driverController.getXButton()){
                    m_robotArm.setArmPosition(ArmConstants.ampAngle);
                } else if (m_driverController.getAButton()){
                    m_robotArm.setArmPosition(ArmConstants.groundAngle);
                } else if (m_driverController.getBButton()){
                    m_robotArm.setArmPosition(ArmConstants.speakerSideAngle);
                }*/
                if (m_driverController.getPOV() == 270){
                    m_robotArm.zeroOffset();
                }
                m_robotArm.moveToPosition();
            }, m_robotArm)
        );


    m_shooter.setDefaultCommand(new RunCommand(() -> {
        SmartDashboard.putNumber("Shooter Speed", m_shooter.getCurrentSpeed());
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

    autoChooser.setDefaultOption("None", new RunCommand((()-> m_robotDrive.setX()), m_robotDrive));
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}