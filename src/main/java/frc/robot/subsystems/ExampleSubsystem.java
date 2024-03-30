package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
    CANSparkMax motor1 = new CANSparkMax(8, MotorType.kBrushless);
    CANSparkMax motor2 = new CANSparkMax(9, MotorType.kBrushless);

    public ExampleSubsystem(){
        motor1.setInverted(true);
        motor2.setInverted(true);
        motor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }
    
}
