package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    CANSparkMax shooterL = new CANSparkMax(10, MotorType.kBrushless);
    CANSparkMax shooterR = new CANSparkMax(11, MotorType.kBrushless);

    public Shooter(){
        shooterL.setInverted(true);
        shooterR.setInverted(true);
        shooterL.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shooterR.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }
    
    public void shoot() {
        shooterL.set(0.9);
        shooterR.set(0.9);
    }
    
    public void stopShoot() {
        shooterL.set(0.0);
        shooterR.set(-0.0);
    }

    public void shooterBack(){
        shooterL.set(0.8);
        shooterR.set(-0.8);
    }

}
