package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.*;
import frc.robot.RobotContainer;

public class Autos extends RobotContainer {
    
    PathPlannerPath speakerMidSpikes = PathPlannerPath.fromPathFile("Spiky Auto Path");
    
    public SequentialCommandGroup spikyAuto = new SequentialCommandGroup(
        new RunCommand(()-> {
            computeMacroValues(ArmConstants.speakerFrontAngle);
            armMacro.schedule();
        }),

        new RunCommand(()->{
            computeMacroValues(ArmConstants.groundAngle);
            armMacro.schedule();
        }),

        new ParallelCommandGroup(
            new RunCommand(()-> {AutoBuilder.followPath(speakerMidSpikes);}),
    
            new SequentialCommandGroup(
            // Pickup from mid spike
                new RunCommand(()->{
                    autoIntake.withTimeout(1.3);
                }),

                new RunCommand(()->{
                    computeMacroValues(6.5);
                    armMacro.schedule();
                }),
            // Pickup from top spike
                new RunCommand(()->{
                    computeMacroValues(ArmConstants.groundAngle);
                    armMacro.schedule();
                }),

                new RunCommand(()->{
                    autoIntake.withTimeout(1.3);
                }),

                new RunCommand(()->{
                    computeMacroValues(7);
                    armMacro.schedule();
                }),
            // Pickup from bottom spike
                new RunCommand(()->{
                    computeMacroValues(ArmConstants.groundAngle);
                    armMacro.schedule();
                }),

                new RunCommand(()->{
                    autoIntake.withTimeout(1.3);
                }),

                new RunCommand(()->{
                    computeMacroValues(10);
                    armMacro.schedule();
                })
            )
        )
    );
}