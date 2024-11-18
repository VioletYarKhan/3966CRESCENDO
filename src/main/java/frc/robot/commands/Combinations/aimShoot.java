// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Combinations;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.Shooter.shootCommands;



public class aimShoot{

  private Shooter shooter;
  private Intake intake;
  private Arm arm;
  private shootCommands shoot;
  private double angle = 0;

  public SequentialCommandGroup aimShootMacro;

  public aimShoot(Shooter shootSystem, Intake intakeSystem, Arm armSystem, double inputAngle){
    this.shooter = shootSystem;
    this.intake = intakeSystem;
    this.arm = armSystem;
    this.angle = inputAngle;
    this.shoot = new shootCommands(shooter, intake, arm);
    
    aimShootMacro = new SequentialCommandGroup(
      new InstantCommand(()->{
        arm.setArmPosition(angle);
      }, arm),
      
      new RunCommand(()->{
        shoot.shootMacro.schedule();
      }, arm, shooter, intake).until(()->shoot.shootMacro.isFinished())

    ).andThen(
      new RunCommand(()->{
        arm.setArmPosition(0);
      }, arm).until(()->arm.isWithinError()));
  }
}
