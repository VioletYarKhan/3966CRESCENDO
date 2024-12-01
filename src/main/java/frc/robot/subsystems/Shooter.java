package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    CANSparkMax shooterL = new CANSparkMax(10, MotorType.kBrushless);
    CANSparkMax shooterR = new CANSparkMax(11, MotorType.kBrushless);
    RelativeEncoder shooterEncoderL = shooterL.getEncoder();
    RelativeEncoder shooterEncoderR = shooterR.getEncoder();



    public Shooter(){
        shooterL.setInverted(true);
        shooterR.setInverted(true);
        shooterL.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shooterR.setIdleMode(CANSparkMax.IdleMode.kBrake);
    } 
    
    public void shoot() {
        shooterL.set(SmartDashboard.getNumber("Shoot Speed", 0.9)); // between 0.9 and 0.97 is comp speeds
        shooterR.set(SmartDashboard.getNumber("Shoot Speed", 0.9)); // between 0.9 and 0.97 is comp speeds
    }
    
    public void stopShoot() {
        shooterL.set(0.0);
        shooterR.set(0.0);
    }

    public void shooterBack(){
        shooterL.set(-0.8);
        shooterR.set(-0.8);
    }

    public double getCurrentSpeed(){
        return (shooterEncoderL.getVelocity() + shooterEncoderR.getVelocity())/2;
    }
}
