package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.BallThrower;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ThrowerBackCommand extends CommandBase{
    
    private final BallThrower m_intake;
    private boolean turnOff;

    public ThrowerBackCommand(BallThrower subsystem) {
        m_intake = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(subsystem);
    }

     // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        turnOff = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_intake.m_smallMotor.set(Constants.BALL_THROWER_BACK_WHEEL_SPEED);
        turnOff = true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return turnOff;
    }
    
}
