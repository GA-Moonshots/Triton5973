// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Jaguar;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class BallThrower extends SubsystemBase {

    public final Jaguar m_largeMotor, m_smallMotor;
  /** Creates a new ExampleSubsystem. */
    public BallThrower() {
        m_largeMotor = new Jaguar(Constants.BALL_THROWER_LARGE_WHEEL_PORT);
        m_smallMotor = new Jaguar(Constants.BALL_THROWER_SMALL_WHEEL_PORT);
    }
}