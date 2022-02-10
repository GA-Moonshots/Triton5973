package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Jaguar;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake {
    public final Jaguar m_intakeMotor;

    public Intake() {
        m_intakeMotor = new Jaguar(Constants.BALL_THROWER_LARGE_WHEEL_PORT);
    }
}
