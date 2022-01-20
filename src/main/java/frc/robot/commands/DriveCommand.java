// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Command that watches controllers and handles default drive functions */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final SwerveDrive m_driveTrain;
  private final Joystick m_xboxController;

  private final double SPEED = 0.7;
    
  private double KpDrive = 0.04;
  private double KiDrive = 0;
  private double KdDrive = 0.002;
  
  private double KpAngle = 0.04;
  private double KiAngle = 0;
  private double KdAngle = 0.002;

  private double correction = 0;
  private double prevAngle = 0;
  private double angleCompensation = 0;
    
  private double KpDrivefl = 0.04; //TODO find pid inputs for all pid controllers
  private double KiDrivefl = 0;
  private double KdDrivefl = 0;//0.002;

  private double KpDrivefr = 0;
  private double KiDrivefr = 0;
  private double KdDrivefr = 0;
  
  private double KpDrivebl = 0;
  private double KiDrivebl = 0;
  private double KdDrivebl = 0;

  private double KpDrivebr = 0;
  private double KiDrivebr = 0;
  private double KdDrivebr = 0;
  
  private double KpAnglefl = 0.04;
  private double KiAnglefl = 0;
  private double KdAnglefl = 0;

  private double KpAnglefr = 0.04;
  private double KiAnglefr = 0;
  private double KdAnglefr = 0;

  private double KpAnglebl = 0.04;
  private double KiAnglebl = 0;
  private double KdAnglebl = 0;

  private double KpAnglebr = 0.04;
  private double KiAnglebr = 0;
  private double KdAnglebr = 0;

  private PIDController pidDrivefl;
  private PIDController pidDrivefr;
  private PIDController pidDrivebl;
  private PIDController pidDrivebr;
  
  private PIDController pidAnglefl;
  private PIDController pidAnglefr;
  private PIDController pidAnglebl;
  private PIDController pidAnglebr;

  // private boolean drivePOV = false;
  private boolean safeMode = false;

  /**
   * Creates a new DriveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(SwerveDrive p_driveTrain, Joystick p_xbox) {
    // set the MEMBER "m_" variable equal to the PARAMETER var "p_"
    m_driveTrain = p_driveTrain;
    m_xboxController = p_xbox;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    configurePID();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (button("safeModeToggle")) {

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


    calculateDrive(axis("forward"), axis("strafe"), axis("rotate"), gyro.getAngle());
    
  }


  private void configurePID() {
		
      pidDrivefl = new PIDController(KpDrivefl, KiDrivefl, KdDrivefl);
      pidDrivefr = new PIDController(KpDrivefr, KiDrivefr, KdDrivefr);
      pidDrivebl = new PIDController(KpDrivebl, KiDrivebl, KdDrivebl);
      pidDrivebr = new PIDController(KpDrivebr, KiDrivebr, KdDrivebr);

      pidAnglefl = new PIDController(KpAnglefl, KiAnglefl, KdAnglefl);
      pidAnglefr = new PIDController(KpAnglefr, KiAnglefr, KdAnglefr);
      pidAnglebl = new PIDController(KpAnglebl, KiAnglebl, KdAnglebl);
      pidAnglebr = new PIDController(KpAnglebr, KiAnglebr, KdAnglebr);

	}

        
  private void calculateDrive(double FWD, double STR, double RCW, double gryroAngle) {
       
    double temp = FWD*Math.cos(gryroAngle) + STR*Math.sin(gryroAngle);
    
    STR = -FWD*Math.sin(gryroAngle) + STR*Math.cos(gryroAngle);
    FWD = temp;

    final double PI = Math.PI;
    final double L = 0; //TODO find vehicle wheelbase //units don't matter as it's a ratio
    final double W = 0; //TODO find vehicle trackwidth
    final double R = Math.sqrt(Math.pow(L, 2) + Math.pow(W, 2));

    double A = STR - RCW*(L/R);
    double B = STR + RCW*(L/R);
    double C = FWD - RCW*(W/R);
    double D = FWD + RCW*(W/R); 

    double frontRightWheelSpeed = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2));
    double frontLeftWheelSpeed  = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));
    double backLeftWheelSpeed   = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));
    double backRightWheelSpeed  = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));

    double frontRightWheelAngle = Math.atan2(B, C) * (180 / PI);
    double frontLeftWheelAngle  = Math.atan2(B, D) * (180 / PI);
    double backLeftWheelAngle   = Math.atan2(A, D) * (180 / PI);
    double backRightWheelAngle  = Math.atan2(A, C) * (180 / PI);

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

    frontRightWheelSpeed = max * SPEED;
    frontLeftWheelSpeed  = max * SPEED;
    backLeftWheelSpeed   = max * SPEED;
    backRightWheelSpeed  = max * SPEED;
    //set wheel speed
    m_driveTrain.frontRightSpeedMotor.set(frontRightWheelSpeed);
    m_driveTrain.frontLeftSpeedMotor.set(frontLeftWheelSpeed);
    m_driveTrain.backRightSpeedMotor.set(backLeftWheelSpeed);
    m_driveTrain.backLeftSpeedMotor.set(backRightWheelSpeed);

    //convert angle to talon ticks to set final angle/ 4096 ticks
    frontRightWheelAngle = ((frontRightWheelAngle + 180) * (4095 / 360));
    frontLeftWheelAngle  = ((frontLeftWheelAngle  + 180) * (4095 / 360));
    backLeftWheelAngle   = ((backLeftWheelAngle   + 180) * (4095 / 360));
    backRightWheelAngle  = ((backRightWheelAngle  + 180) * (4095 / 360));
    //set wheel angle
    m_driveTrain.frontRightAngleMotor.set(ControlMode.Position, frontRightWheelAngle);
    m_driveTrain.frontLeftSAngleMotor.set(ControlMode.Position, frontLeftWheelAngle);
    m_driveTrain.backRightAngleMotor.set(ControlMode.Position,  backLeftWheelAngle);
    m_driveTrain.backLeftAngleMotor.set(ControlMode.Position,   backRightWheelAngle);
}




	@Override
	public void initSendable(SendableBuilder builder) {

	  super.initSendable(builder);
		builder.addDoubleProperty("PDrive", () -> KpDrive, (value) -> KpDrive = value);
		builder.addDoubleProperty("IDrive", () -> KiDrive, (value) -> KiDrive = value);
		builder.addDoubleProperty("DDrive", () -> KdDrive, (value) -> KdDrive = value);

    builder.addDoubleProperty("PAngle", () -> KpAngle, (value) -> KpAngle = value);
		builder.addDoubleProperty("IAngle", () -> KiAngle, (value) -> KiAngle = value);
		builder.addDoubleProperty("DAngle", () -> KdAngle, (value) -> KdAngle = value);

		builder.addDoubleProperty("Forward", () -> axis("forward"), null);
		builder.addDoubleProperty("Strafe", () -> axis("strafe"), null);
		builder.addDoubleProperty("Rotate", () -> axis("Rotate"), null);
		builder.addDoubleProperty("prevAngle", () -> prevAngle, null);
		builder.addBooleanProperty("isPOV", () -> drivePOV, null);
		builder.addDoubleProperty("angleCompensation", () -> angleCompensation, null);
		builder.addBooleanProperty("isSafeMode", () -> safeMode, null);
		builder.addDoubleProperty("frontRightSpeedMotor", () -> frontRightSpeedMotor.get(), null);		
		builder.addDoubleProperty("frontLeftSpeedMotor",  () -> frontLeftSpeedMotor.get(),  null);
		builder.addDoubleProperty("backRightSpeedMotor",  () -> backRightSpeedMotor.get(),  null);
		builder.addDoubleProperty("backLeftSpeedMotor",   () -> backLeftSpeedMotor.get(),   null);
    builder.addDoubleProperty("frontRightAngleMotor", () -> frontRightAngleMotor.get(), null);		
		builder.addDoubleProperty("frontLeftSAngleMotor", () -> frontLeftSAngleMotor.get(), null);
		builder.addDoubleProperty("backRightAngleMotor",  () -> backRightAngleMotor.get(),  null);
		builder.addDoubleProperty("backLeftAngleMotor",   () -> backLeftAngleMotor.get(),   null);
		
	}
    

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