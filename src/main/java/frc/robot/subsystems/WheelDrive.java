package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import 

public class WheelDrive {

    private WPI_TalonSRX angleMotor;
    private WPI_TalonSRX speedMotor;
    private PIDController pidController;

    private final double MAX_VOLTS = 4.95;

    public WheelDrive (int angleMotor, int speedMotor, int encoder) {
        this.angleMotor = new WPI_TalonSRX(angleMotor);
        this.speedMotor = new WPI_TalonSRX(speedMotor);
        this.pidController = new PIDController(1, 0, 0);
    
        pidController.setOutputRange(-1, 1);
        pidController.enableContinuousInput();
        pidController.enable();

        pidController.
    }
}
