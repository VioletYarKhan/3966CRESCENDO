package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;

public class Arm extends SubsystemBase {
    CANSparkMax armL = new CANSparkMax(9, MotorType.kBrushless);
    CANSparkMax armR = new CANSparkMax(12, MotorType.kBrushless);
    RelativeEncoder armEncoderL = armL.getEncoder();
    RelativeEncoder armEncoderR = armR.getEncoder();
    SparkPIDController armPidR = armR.getPIDController();
    SparkPIDController armPidL = armL.getPIDController();

    private static final double kP = 0.5;
    private static final double kI = 0;
    private static final double kD = 0.001;
    private static final double kFF = 0.1;
    public double currentPosition = (armEncoderL.getPosition() + armEncoderR.getPosition()) / 2;
    private double setpoint = currentPosition;
    public double offset = 0.0;
    private Boolean RunPid = false;

    /*
     * Make it so positive is up for both sides
     */
    public Arm() {
        armPidR.setP(kP);
        armPidR.setI(kI);
        armPidR.setD(kD);
        armPidR.setFF(kFF);
        armPidL.setP(kP);
        armPidL.setI(kI);
        armPidL.setD(kD);
        armPidL.setFF(kFF);

        armPidR.setOutputRange(-0.3, 0.3);
        armPidL.setOutputRange(-0.3, 0.3);

        // L clockwise = down
        armL.setInverted(true);
        // R clockwise = up
        armR.setInverted(false);
        armL.setIdleMode(CANSparkMax.IdleMode.kBrake);
        armR.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    /**
     * @param targetAngle is the encoder position requested by the driver
     */
    public void setArmPosition(double targetAngle) {
        setpoint = offset + targetAngle;
        RunPid = true;
    }


    /*
     * Uses PID controls to move the arm to the setpoint
     */
    public void moveToPosition() {
        currentPosition = (armEncoderL.getPosition() + armEncoderR.getPosition()) / 2;
        if(RunPid){
            armPidL.setReference(setpoint, ControlType.kPosition);
            armPidR.setReference(setpoint, ControlType.kPosition);

            SmartDashboard.putNumber("Query Angle", setpoint);
            SmartDashboard.putNumber("Arm Speed", ((armEncoderL.getVelocity() + armEncoderR.getVelocity())/2)*armEncoderL.getVelocityConversionFactor());
        }
    }

    public void armUp() {
        RunPid = false;
        armL.set(0.2);
        armR.set(0.2);
        setpoint = (armEncoderL.getPosition() + armEncoderR.getPosition()) / 2;
    }

    public void armDown() {
        RunPid = false;
        armL.set(-0.2);
        armR.set(-0.2);
        setpoint = (armEncoderL.getPosition() + armEncoderR.getPosition()) / 2;
    }

    public double getEncoderValue() {
        return (armEncoderL.getPosition() + armEncoderR.getPosition()) / 2;
    }

    public void zeroOffset() {
        offset = currentPosition;
        SmartDashboard.putNumber("Arm Offset", offset);
    }

    public double getArmPosition() {
        return Units.rotationsToRadians((currentPosition / ArmConstants.gearRatio) + offset);
    }
}
