package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// LOCAL IMPORTS
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase  {

    public WPI_TalonSRX frontRightSpeedMotor;
    public WPI_TalonSRX frontLeftSpeedMotor;
    public WPI_TalonSRX backRightSpeedMotor;
    public WPI_TalonSRX backLeftSpeedMotor;
    
    public WPI_TalonSRX frontRightAngleMotor;
    public WPI_TalonSRX frontLeftAngleMotor;
    public WPI_TalonSRX backRightAngleMotor;
    public WPI_TalonSRX backLeftAngleMotor;

    public Encoder frontLeftEncoder;
    public Encoder frontRightEncoder;
    public Encoder backLeftEncoder;
    public Encoder backRightEncoder;

    public ADXRS450_Gyro gyro;
    
    public SwerveDrive() {
       
        frontRightSpeedMotor = new WPI_TalonSRX(Constants.FRONT_RIGHT_SPEED_MOTOR);
        frontLeftSpeedMotor  = new WPI_TalonSRX(Constants.FRONT_LEFT_SPEED_MOTOR);
        backRightSpeedMotor  = new WPI_TalonSRX(Constants.BACK_RIGHT_SPEED_MOTOR);
        backLeftSpeedMotor   = new WPI_TalonSRX(Constants.BACK_LEFT_SPEED_MOTOR);
        
        frontRightAngleMotor = new WPI_TalonSRX(Constants.FRONT_RIGHT_ANGLE_MOTOR);
        frontLeftAngleMotor = new WPI_TalonSRX(Constants.FRONT_LEFT_ANGLE_MOTOR);
        backRightAngleMotor  = new WPI_TalonSRX(Constants.BACK_RIGHT_ANGLE_MOTOR);
        backLeftAngleMotor   = new WPI_TalonSRX(Constants.BACK_LEFT_ANGLE_MOTOR);

        addChild("frontRightSpeedMotor Motor", frontRightSpeedMotor);
        addChild("frontLeftSpeedMotor Motor",  frontLeftSpeedMotor);
        addChild("backRightSpeedMotor Motor",  backRightSpeedMotor);
        addChild("backLeftSpeedMotor Motor",   backLeftSpeedMotor);

        addChild("frontRightAngleMotor Motor", frontRightAngleMotor);
        addChild("frontLeftSAngleMotor Motor", frontLeftAngleMotor);
        addChild("backRightAngleMotor Motor",  backRightAngleMotor);
        addChild("backLeftAngleMotor Motor",   backLeftAngleMotor);

        frontRightSpeedMotor.setInverted(true);
        frontLeftSpeedMotor.setInverted(false);
        backRightSpeedMotor.setInverted(false);
        backLeftSpeedMotor.setInverted(false);

        frontRightAngleMotor.setInverted(false);
        frontLeftAngleMotor.setInverted(false);
        backRightAngleMotor.setInverted(false);
        backLeftAngleMotor.setInverted(false);
    
        frontRightAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        frontLeftAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        backRightAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        backLeftAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        
        frontRightSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        frontLeftSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        backRightSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        backLeftSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        //turn off ramping for angleMotors
        backLeftAngleMotor.configOpenloopRamp(0);
        backLeftAngleMotor.configClosedloopRamp(0);

        backRightAngleMotor.configOpenloopRamp(0);
        backRightAngleMotor.configClosedloopRamp(0);

        frontLeftAngleMotor.configOpenloopRamp(0);
        frontLeftAngleMotor.configClosedloopRamp(0);

        frontRightAngleMotor.configOpenloopRamp(0);
        frontRightAngleMotor.configClosedloopRamp(0);

        //brake mode for speed motors
        backLeftSpeedMotor.setNeutralMode(NeutralMode.Brake);
        backRightSpeedMotor.setNeutralMode(NeutralMode.Brake);
        frontLeftSpeedMotor.setNeutralMode(NeutralMode.Brake);
        frontRightSpeedMotor.setNeutralMode(NeutralMode.Brake);

        //brake mode for angle motors
        backLeftAngleMotor.setNeutralMode(NeutralMode.Brake);
        backRightAngleMotor.setNeutralMode(NeutralMode.Brake);
        frontLeftAngleMotor.setNeutralMode(NeutralMode.Brake);
        frontRightAngleMotor.setNeutralMode(NeutralMode.Brake);

		gyro = new ADXRS450_Gyro();
		gyro.reset();

        configureEncoders();
    }

    
    private void configureEncoders() {

        frontRightEncoder = new Encoder(Constants.FRONT_RIGHT_ENCODER_A, Constants.FRONT_RIGHT_ENCODER_B);
        frontLeftEncoder  = new Encoder(Constants.FRONT_LEFT_ENCODER_A, Constants.FRONT_LEFT_ENCODER_B);
        backRightEncoder  = new Encoder(Constants.BACK_RIGHT_ENCODER_A, Constants.BACK_RIGHT_ENCODER_B);
        backLeftEncoder   = new Encoder(Constants.BACK_LEFT_ENCODER_A, Constants.BACK_LEFT_ENCODER_B);
        

        frontRightEncoder.setDistancePerPulse(1./ Constants.TICKS_TO_DEGREES);
        frontLeftEncoder.setDistancePerPulse(1./ Constants.TICKS_TO_DEGREES);
        backRightEncoder.setDistancePerPulse(1./ Constants.TICKS_TO_DEGREES);
        backLeftEncoder.setDistancePerPulse(1./ Constants.TICKS_TO_DEGREES);
    }


}
