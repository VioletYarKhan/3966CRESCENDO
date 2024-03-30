package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    Spark intake = new Spark(0);

    public Intake(){}

    public void intake() {
        intake.set(-0.6);
    }

    public void stopIntake() {
        intake.set(0);
    }

    public void intakeBack(){
        intake.set(0.8);
    }

}
