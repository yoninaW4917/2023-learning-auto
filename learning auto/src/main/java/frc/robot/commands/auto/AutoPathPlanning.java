package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kDrivetrain;
import frc.robot.Constants.kDrivetrain.kAuto;
import frc.robot.subsystems.Drivetrain;

public class AutoPathPlanning extends SequentialCommandGroup {

    private final DifferentialDriveVoltageConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                kDrivetrain.ksVolts,
                kDrivetrain.kvVolts,
                kDrivetrain.kaVolts),
            kDrivetrain.kDriveKinematics,
            kDrivetrain.kAuto.kMaxVolts);

    public AutoPathPlanning(Drivetrain sys_drivetrain, Trajectory trajectory) {
        super(
            new RamseteCommand(
                trajectory,
                sys_drivetrain::getPose2d,
                new RamseteController(kAuto.kRamseteB, kAuto.kRamseteZeta),
                new SimpleMotorFeedforward(kDrivetrain.ksVolts, kDrivetrain.kvVolts, kDrivetrain.kaVolts),
                kDrivetrain.kDriveKinematics,
                sys_drivetrain::getWheelSpeeds,
                new PIDController(kDrivetrain.kPDriveVel, 0, 0),
                new PIDController(kDrivetrain.kPDriveVel, 0, 0),
                sys_drivetrain::tankDriveVoltages,
                sys_drivetrain)
        );
    }

}