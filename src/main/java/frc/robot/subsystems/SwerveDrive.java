package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// LOCAL IMPORTS
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase  {

    public WPI_TalonSRX frontRightSpeedMotor;
    public WPI_TalonSRX frontLeftSpeedMotor;
    public WPI_TalonSRX backRightSpeedMotor;
    public WPI_TalonSRX backLeftSpeedMotor;
    
    public WPI_TalonSRX frontRightAngleMotor;
    public WPI_TalonSRX frontLeftSAngleMotor;
    public WPI_TalonSRX backRightAngleMotor;
    public WPI_TalonSRX backLeftAngleMotor;

    public ADXRS450_Gyro gyro;
    
    public SwerveDrive() {
       
        frontRightSpeedMotor = new WPI_TalonSRX(Constants.FRONT_RIGHT_SPEED_MOTOR);
        frontLeftSpeedMotor  = new WPI_TalonSRX(Constants.FRONT_LEFT_SPEED_MOTOR);
        backRightSpeedMotor  = new WPI_TalonSRX(Constants.BACK_RIGHT_SPEED_MOTOR);
        backLeftSpeedMotor   = new WPI_TalonSRX(Constants.BACK_LEFT_SPEED_MOTOR);
        
        frontRightAngleMotor = new WPI_TalonSRX(Constants.FRONT_RIGHT_ANGLE_MOTOR);
        frontLeftSAngleMotor = new WPI_TalonSRX(Constants.FRONT_LEFT_ANGLE_MOTOR);
        backRightAngleMotor  = new WPI_TalonSRX(Constants.BACK_RIGHT_ANGLE_MOTOR);
        backLeftAngleMotor   = new WPI_TalonSRX(Constants.BACK_LEFT_ANGLE_MOTOR);

        addChild("frontRightSpeedMotor Motor", frontRightSpeedMotor);
        addChild("frontLeftSpeedMotor Motor",  frontLeftSpeedMotor);
        addChild("backRightSpeedMotor Motor",  backRightSpeedMotor);
        addChild("backLeftSpeedMotor Motor",   backLeftSpeedMotor);

        addChild("frontRightAngleMotor Motor", frontRightAngleMotor);
        addChild("frontLeftSAngleMotor Motor", frontLeftSAngleMotor);
        addChild("backRightAngleMotor Motor",  backRightAngleMotor);
        addChild("backLeftAngleMotor Motor",   backLeftAngleMotor);

        frontRightSpeedMotor.setInverted(false);
        frontLeftSpeedMotor.setInverted(false);
        backRightSpeedMotor.setInverted(false);
        backLeftSpeedMotor.setInverted(false);

        frontRightAngleMotor.setInverted(false);
        frontLeftSAngleMotor.setInverted(false);
        backRightAngleMotor.setInverted(false);
        backLeftAngleMotor.setInverted(false);
        
        frontRightAngleMotor.set(ControlMode.Position, 0);
        frontLeftSAngleMotor.set(ControlMode.Position, 0);
        backRightAngleMotor.set(ControlMode.Position,  0);
        backLeftAngleMotor.set(ControlMode.Position,   0);

        frontRightAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        frontLeftSAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        backRightAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        backLeftAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        
        frontRightSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        frontLeftSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        backRightSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        backLeftSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

		gyro = new ADXRS450_Gyro();
		gyro.reset();
    }

}
