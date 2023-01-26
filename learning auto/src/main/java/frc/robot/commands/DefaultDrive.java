package frc.robot.commands; 

import edu.wpi.first.wpilibj2.command.CommandBase; 
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class DefaultDrive extends CommandBase
{

    private Drivetrain drivetrain;
    private CommandXboxController controller;

    double forwardSpeed;
    double rearSpeed;
    double turnVal;

    public DefaultDrive(Drivetrain paramDrivetrain, CommandXboxController paramController)
    {

        drivetrain = paramDrivetrain;
        controller = paramController;

        addRequirements(drivetrain);
        
    }

    @Override
    public void initialize() {}

    @Override
    public void execute()
    {

        // LT forward, RT rear, LSB turnval
        forwardSpeed = controller.getLeftTriggerAxis();
        rearSpeed = controller.getRightTriggerAxis();
        turnVal = controller.getLeftX();

        drivetrain.arcadeDrive(forwardSpeed - rearSpeed, turnVal);

    }

    @Override
    public void end(boolean interrupted) {}


    @Override
    public boolean isFinished()
    {
        return false;
    }
}