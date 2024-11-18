// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm;


public class shootCommands{

  private Shooter shooter;
  private Intake intake;
  private Arm arm;
  public ParallelCommandGroup shootMacro;

    public shootCommands(Shooter shoot, Intake intakeSystem){
        this.shooter = shoot;
        this.intake = intakeSystem;
        
        shootMacro = new ParallelCommandGroup(
            new SequentialCommandGroup(
                new RunCommand(()->{
                    shooter.shooterBack();
                }, shooter).withTimeout(0.1),

                new RunCommand(()->{
                    shooter.shoot();
                }, shooter).until(()->shooter.getCurrentSpeed() > SmartDashboard.getNumber("Shoot Speed", 0.9) * 4.5),

                new RunCommand(()->{
                    shooter.shoot();
                }, shooter).withTimeout(0.3),

                new RunCommand(()->{
                    shooter.stopShoot();
                }, shooter).withTimeout(0.03)
            ),
            new SequentialCommandGroup(
                new RunCommand(()->{
                    intake.intakeBack();
                }, intake).withTimeout(0.07),

                new RunCommand(()->{
                    intake.stopIntake();
                }
                , intake).until(()->shooter.getCurrentSpeed() > SmartDashboard.getNumber("Shoot Speed", 0.9) * 4.5),

                new RunCommand(()->{
                    intake.intake();
                }, intake).withTimeout(0.33)
            )
        );
    }

    public shootCommands(Shooter shoot, Intake intakeSystem, Arm armSystem)
  {
    this.shooter = shoot;
    this.intake = intakeSystem;
    this.arm = armSystem;
    
    shootMacro = new ParallelCommandGroup(
        new SequentialCommandGroup(
            new RunCommand(()->{
                shooter.shooterBack();
            }, shooter).withTimeout(0.1),

            new RunCommand(()->{
                shooter.shoot();
            }, shooter).until(()->shooter.getCurrentSpeed() > SmartDashboard.getNumber("Shoot Speed", 0.9) * 4.5 && arm.isWithinError()),

            new RunCommand(()->{
                shooter.shoot();
            }, shooter).withTimeout(0.3),

            new RunCommand(()->{
                shooter.stopShoot();
            }, shooter).withTimeout(0.03)
        ),
        new SequentialCommandGroup(
            new RunCommand(()->{
                intake.intakeBack();
            }, intake).withTimeout(0.07),

            new RunCommand(()->{
                intake.stopIntake();
            }
            , intake).until(()->shooter.getCurrentSpeed() > SmartDashboard.getNumber("Shoot Speed", 0.9) * 4.5),

            new RunCommand(()->{
                intake.intake();
            }, intake).withTimeout(0.33)
        )
    );
  }
}
