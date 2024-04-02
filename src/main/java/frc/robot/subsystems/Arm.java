package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    CANSparkMax armL = new CANSparkMax(9, MotorType.kBrushless);
    CANSparkMax armR = new CANSparkMax(12, MotorType.kBrushless);
    RelativeEncoder armEncoder = armL.getEncoder();
    public double startAngle = 0;

    public Arm(){
        armL.setInverted(true);
        armR.setInverted(true);
        armL.setIdleMode(CANSparkMax.IdleMode.kBrake);
        armR.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }
    
    public void armUp() {
        armL.set(0.2);
        armR.set(-0.2);
    }

    public void armDown() {
        armL.set(-0.2);
        armR.set(0.2);
    }

    public void armUpMacro() {
        armL.set(0.15);
        armR.set(-0.15);
    }

    public void armDownMacro() {
        armL.set(-0.125);
        armR.set(0.125);
    }

    public void armHold() {
        if(armEncoder.getPosition() < (startAngle + 30)){
            armL.set(0.03);
            armR.set(-0.03);
        } else {
            armL.set(0);
            armR.set(0);
        }
    }

    public double getEncoderValue() {
        SmartDashboard.putNumber("Arm Encoder", armEncoder.getPosition());
        SmartDashboard.putNumber("Arm Encoder Start", startAngle);
        return armEncoder.getPosition();
    }

    public void resetStartValue(){
        startAngle = armEncoder.getPosition();
    }
}
