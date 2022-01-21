// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.MechanumDriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;

/** An example command that uses an example subsystem. */
public class MechanumDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final WPI_TalonSRX m_frontLeftMotor;
  private final WPI_TalonSRX m_frontRightMotor;
  private final WPI_TalonSRX m_backLeftMotor;
  private final WPI_TalonSRX m_backRightMotor;
  private final Joystick m_xboxController;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MechanumDriveCommand(WPI_TalonSRX p_frontLeftMotor, WPI_TalonSRX p_frontRightMotor, WPI_TalonSRX p_backLeftMotor, WPI_TalonSRX p_backRightMotor, Joystick p_xbox) {

    m_frontLeftMotor = p_frontLeftMotor;
    m_frontRightMotor = p_frontRightMotor;
    m_backRightMotor = p_backRightMotor;
    m_backLeftMotor = p_backLeftMotor;

    m_xboxController = p_xbox
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double r = Math.hypot(m_xboxController.getX(), m_xboxController.getY());
    double robotAngle = Math.atan2(m_xboxController.getY(), m_xboxController.getX()) - Math.PI / 4;
    double rightX = m_xboxController.getX(right);
    final double v1 = r * Math.cos(robotAngle) + rightX;
    final double v2 = r * Math.sin(robotAngle) - rightX;
    final double v3 = r * Math.sin(robotAngle) + rightX;
    final double v4 = r * Math.cos(robotAngle) - rightX;
    
    m_frontLeftMotor.setPower(v1);
    m_frontRightMotor.setPower(v2);
    m_backLeftMotor.setPower(v3)
    m_backRightMotor.setPower(v4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}