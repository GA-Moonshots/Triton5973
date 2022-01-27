// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command that watches controllers and handles default drive functions */
public class SwerveCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final SwerveDrive m_drive;
    private final Joystick m_xboxController;

    private final double SPEED = 0.4;

    private double comboStartTime = 0;
	private boolean alreadyToggled = false;

    private static final int lastTime = 0;

    private double forwardtemp;
    private double strafetemp;
    private double rotatetemp;

    // private boolean drivePOV = false;
    private boolean safeMode = false;

    private boolean fieldOrientedMode = true;

    private final double L = 23; //vehicle tracklength
    private final double W = 23; //vehicle trackwidth
    private final double R = Math.sqrt(Math.pow(L, 2) + Math.pow(W, 2));
    private final double PI = Math.PI; 

    private double positionAlongField = 0;
    private double positionAcrossField = 0;

    private double KpDrive = 0.04;
    private double KiDrive = 0;
    private double KdDrive = 0.002;

    private double KpAngle = 0.04;
    private double KiAngle = 0;
    private double KdAngle = 0.002;

    private double correction = 0;
    private double prevAngle = 0;
    private double angleCompensation = 0;

    private PIDController pidDrivefl;
    private PIDController pidDrivefr;
    private PIDController pidDrivebl;
    private PIDController pidDrivebr;

    private PIDController pidAnglefl;
    private PIDController pidAnglefr;
    private PIDController pidAnglebl;
    private PIDController pidAnglebr;



  /**
   * Creates a new DriveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  
    public SwerveCommand(SwerveDrive p_driveTrain, Joystick p_xbox) {
        // set the MEMBER "m_" variable equal to the PARAMETER var "p_"
        m_drive = p_driveTrain;
        m_xboxController = p_xbox;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_drive);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
        configurePID();
        
        // forwardtemp = () -> {return m_xboxController.getRawAxis(Constants.LEFT_XBOX_AXIS)};
        // private double strafetemp  = axis("strafe");
        // private double rotatetemp  = axis("rotate");

    }

    @Override
    public void execute() {
        if (safeMode) {

            if (comboStartTime == 0)
                comboStartTime = Timer.getFPGATimestamp();
            else if (Timer.getFPGATimestamp() - comboStartTime >= 3.0 && !alreadyToggled) {
            
                safeMode = !safeMode;
                alreadyToggled = true;
                System.out.println("Safemode is " + (safeMode ? "Enabled" : "Disabled") + ".");
            
            }

        } else {

            comboStartTime = 0;
            alreadyToggled = false;
        
        }
        // // //set controller deadzones
        // if(Math.abs(axis("forward")) < 0.1) {
           
        //     forwardtemp = 0;

        // } else {
            
        //     forwardtemp = axis("forward");
        // }

        // if (Math.abs(axis("strafe")) < 0.2) {
            
        //     strafetemp = 0;

        // } else {
            
        //     strafetemp = axis("strafe");

        // }
       
        // if (Math.abs(axis("rotate")) < 0.2) {
            
        //     rotatetemp = 0;

        // } else {
            
        //     rotatetemp = axis("rotate");

        // }
        

        calculateDrive(forwardtemp, -strafetemp, rotatetemp, m_drive.gyro.getAngle());
        calculateRobotPosition();
        

    }

    private void configurePID() {

        // FRONT LEFT   --  (P, I, D)
        pidDrivefl = new PIDController(0.04, 0, 0);
        // FRONT RIGHT      --  (P, I, D)
        pidDrivefr = new PIDController(0, 0, 0);
        // BACK LEFT    --  (P, I, D)
        pidDrivebl = new PIDController(0, 0, 0);
        // BACK RIGHT    --   (P, I, D)
        pidDrivebr = new PIDController(0, 0, 0);

        // FRONT LEFT   --   (P, I, D)
        pidAnglefl = new PIDController(0.04, 0, 0);
        // FRONT RIGHT   --   (P, I, D)
        pidAnglefr = new PIDController(0.04, 0, 0);
        // BACK LEFT   --   (P, I, D)
        pidAnglebl = new PIDController(0.04, 0, 0);
        // BACK RIGHT   --   (P, I, D)
        pidAnglebr = new PIDController(0.04, 0, 0);

	}

    private double getDistanceWrapped(Encoder encoder) {
        return encoder.getDistance() % 360;
    }
 
    private void calculateDrive(double FWD, double STR, double RCW, double gryroAngle) {
       
        if(fieldOrientedMode) {
            
            double temp = FWD*Math.cos(gryroAngle) + STR*Math.sin(gryroAngle);
        
            STR = -FWD*Math.sin(gryroAngle) + STR*Math.cos(gryroAngle);
            FWD = temp;

        } else {
            
            double temp = FWD*Math.cos(0) + STR*Math.sin(0);
        
            STR = -FWD*Math.sin(0) + STR*Math.cos(0);
            FWD = temp;

        }

        double A = STR - RCW*(L/R);
        double B = STR + RCW*(L/R);
        double C = FWD - RCW*(W/R);
        double D = FWD + RCW*(W/R); 

        double frontRightWheelSpeed = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2));
        double frontLeftWheelSpeed  = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));
        double backLeftWheelSpeed   = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));
        double backRightWheelSpeed  = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));

        double frontRightWheelAngle = (Math.atan2(B, C) * (180 / PI));
        double frontLeftWheelAngle  = (Math.atan2(B, D) * (180 / PI));//
        double backLeftWheelAngle   = (Math.atan2(A, D) * (180 / PI));
        double backRightWheelAngle  = (Math.atan2(A, C) * (180 / PI));//

        double max = frontRightWheelSpeed;

        //normalize wheel speeds
        if(frontLeftWheelSpeed > max) {
            
            max = frontLeftWheelSpeed;
        
        } else if(backLeftWheelSpeed > max) {
            
            max = backLeftWheelSpeed;

        } else if (backRightWheelSpeed > max) {
            
            max = backRightWheelSpeed;

        } else {

        }

        frontRightWheelSpeed = (max * SPEED); 
        frontLeftWheelSpeed  = (max * SPEED); 
        backLeftWheelSpeed   = (max * SPEED); 
        backRightWheelSpeed  = (max * SPEED);

        setAngleAndSpeed(m_drive.frontRightEncoder.getDistance(), frontRightWheelAngle, frontRightWheelSpeed, m_drive.frontRightSpeedMotor, m_drive.frontRightAngleMotor, pidAnglefr);
        setAngleAndSpeed(m_drive.frontLeftEncoder.getDistance(),  frontLeftWheelAngle,  frontLeftWheelSpeed,  m_drive.frontRightSpeedMotor, m_drive.frontRightAngleMotor, pidAnglefl);
        setAngleAndSpeed(m_drive.backLeftEncoder.getDistance(),   backLeftWheelAngle,   backLeftWheelSpeed,   m_drive.backLeftSpeedMotor,   m_drive.backLeftAngleMotor,   pidAnglebl);
        setAngleAndSpeed(m_drive.backRightEncoder.getDistance(),  backRightWheelAngle,  backRightWheelSpeed,  m_drive.backRightSpeedMotor,  m_drive.backLeftAngleMotor,   pidAnglebr);
        
    }

    private void setAngleAndSpeed(double currentAngle, double targetAngle, double wheelSpeed, WPI_TalonSRX speedMotor, WPI_TalonSRX angleMotor, PIDController pidController) {
        
        if (Math.abs(targetAngle - (currentAngle % 360)) > 90 && Math.abs(targetAngle - (currentAngle % 360)) < 270) {
            targetAngle = ((int)targetAngle + 180) % 360;
            wheelSpeed = -wheelSpeed;
        }

        speedMotor.set(wheelSpeed);
        angleMotor.set(pidController.calculate(currentAngle, targetAngle));
    }
    
    private void calculateRobotPosition() {
        
        double Bfl = Math.sin(m_drive.frontLeftEncoder.getDistance())  * m_drive.frontLeftSpeedMotor.getSelectedSensorVelocity();
        double Bfr = Math.sin(m_drive.frontRightEncoder.getDistance()) * m_drive.frontRightSpeedMotor.getSelectedSensorVelocity();
        double Abl = Math.sin(m_drive.backLeftEncoder.getDistance()) * m_drive.backLeftSpeedMotor.getSelectedSensorVelocity();
        double Abr = Math.sin(m_drive.backRightEncoder.getDistance()) * m_drive.backRightSpeedMotor.getSelectedSensorVelocity();

        double Dfl = Math.cos(m_drive.frontLeftEncoder.getDistance()) * m_drive.frontLeftSpeedMotor.getSelectedSensorVelocity();
        double Cfr = Math.cos(m_drive.frontRightEncoder.getDistance()) * m_drive.frontRightSpeedMotor.getSelectedSensorVelocity();
        double Dbl = Math.cos(m_drive.backLeftEncoder.getDistance()) * m_drive.backLeftSpeedMotor.getSelectedSensorVelocity();
        double Cbr = Math.cos(m_drive.backRightEncoder.getDistance()) * m_drive.backRightSpeedMotor.getSelectedSensorVelocity();

        double A = (Abr + Abl) / 2;
        double B = (Bfl + Bfr) / 2;
        double C = (Cfr + Cbr) / 2;
        double D = (Dfl + Dbl) / 2;

        double rotation1 = (B - A) / L;
        double rotation2 = (C - D) / W;
        double rotation = (rotation1  + rotation2) / 2;

        double forward1 = rotation * (L / 2) + A;
        double forward2 = -rotation * (L / 2) + B;
        double forward = (forward1 + forward2) / 2;

        double strafe1 = rotation * (W / 2) + C;
        double strafe2 = -rotation * ( W / 2) + D;
        double strafe = (strafe1 + strafe2) / 2;


        double forwardNew = (forward * Math.cos(m_drive.gyro.getAngle())) + (strafe *  Math.sin(m_drive.gyro.getAngle())); 
        double strafeNew  = (strafe *  Math.cos(m_drive.gyro.getAngle())) - (forward * Math.sin(m_drive.gyro.getAngle()));

        double timeStep = 0.020; //milliseconds //Timer.getFPGATimestamp() - lastTime //don't know how to do this so just constant from whitepaper
        
        positionAlongField = positionAlongField + (forwardNew * timeStep);
        positionAcrossField = positionAcrossField + (strafeNew * timeStep);

    }

    private double[] getRobotPosition() {

        double[] coordinates = {positionAcrossField, positionAlongField};
        return coordinates;
    }

    private void resetRobotPosition() {
        positionAlongField = 0;
        positionAcrossField = 0;
    }

    private void rotateToAngle(double rotationSpeed, double gyroAngle, double angleToRotate) {
        
        while(!(Math.abs(gyroAngle) < Math.abs(angleToRotate - 1) && Math.abs(gyroAngle) > Math.abs(angleToRotate + 1))) {
            
            if(gyroAngle < angleToRotate) {
                
                calculateDrive(0, 0, -rotationSpeed, gyroAngle);

            } else if (gyroAngle > angleToRotate) {
                
                calculateDrive(0, 0, rotationSpeed, gyroAngle);

            }

        }
    }


	// @Override
	// public void initSendable(SendableBuilder builder) {

	//   super.initSendable(builder);
	// 	builder.addDoubleProperty("PDrive", () -> KpDrive, (value) -> KpDrive = value);
	// 	builder.addDoubleProperty("IDrive", () -> KiDrive, (value) -> KiDrive = value);
	// 	builder.addDoubleProperty("DDrive", () -> KdDrive, (value) -> KdDrive = value);

    //     builder.addDoubleProperty("PAngle", () -> KpAngle, (value) -> KpAngle = value);
	// 	builder.addDoubleProperty("IAngle", () -> KiAngle, (value) -> KiAngle = value);
	// 	builder.addDoubleProperty("DAngle", () -> KdAngle, (value) -> KdAngle = value);

	// 	builder.addDoubleProperty("Forward", () -> Constants.AXIS_FWD, null);
	// 	builder.addDoubleProperty("Strafe", () -> axis("strafe"), null);
	// 	builder.addDoubleProperty("Rotate", () -> axis("Rotate"), null);
	// 	builder.addDoubleProperty("prevAngle", () -> prevAngle, null);
	// 	builder.addBooleanProperty("isPOV", () -> drivePOV, null);
	// 	builder.addDoubleProperty("angleCompensation", () -> angleCompensation, null);
	// 	builder.addBooleanProperty("isSafeMode", () -> safeMode, null);
	// 	builder.addDoubleProperty("frontRightSpeedMotor", () -> frontRightSpeedMotor.get(), null);		
	// 	builder.addDoubleProperty("frontLeftSpeedMotor",  () -> frontLeftSpeedMotor.get(),  null);
	// 	builder.addDoubleProperty("backRightSpeedMotor",  () -> backRightSpeedMotor.get(),  null);
	// 	builder.addDoubleProperty("backLeftSpeedMotor",   () -> backLeftSpeedMotor.get(),   null);
    // builder.addDoubleProperty("frontRightAngleMotor", () -> frontRightAngleMotor.get(), null);		
	// 	builder.addDoubleProperty("frontLeftSAngleMotor", () -> frontLeftSAngleMotor.get(), null);
	// 	builder.addDoubleProperty("backRightAngleMotor",  () -> backRightAngleMotor.get(),  null);
	// 	builder.addDoubleProperty("backLeftAngleMotor",   () -> backLeftAngleMotor.get(),   null);
		
	// }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}