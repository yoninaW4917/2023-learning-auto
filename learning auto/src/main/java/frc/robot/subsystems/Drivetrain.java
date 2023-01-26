package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kDrivetrain;

public class Drivetrain extends SubsystemBase
{

    // Instantiate motors
    private final WPI_TalonFX motorLeft1;
    private final WPI_TalonFX motorLeft2;
    private final WPI_TalonFX motorLeft3;
    private final WPI_TalonFX motorRight1;
    private final WPI_TalonFX motorRight2;
    private final WPI_TalonFX motorRight3;

    // Encoders
    private final WPI_CANCoder encLeft;
    private final WPI_CANCoder encRight;

    private final CANCoderConfiguration encCfg;

    // Current limiter object
    private final SupplyCurrentLimitConfiguration currentLimitCfg;

    // DifferentialDrive
    private final DifferentialDrive motorDiffDrive;

    public Drivetrain()
    {

        // Setup current limiting config
        currentLimitCfg = new SupplyCurrentLimitConfiguration();
        currentLimitCfg.enable = true;
        currentLimitCfg.currentLimit = kDrivetrain.kCurrentLimit;

        // Initialize motors
        motorLeft1 = new WPI_TalonFX(kDrivetrain.kMotor.kIdLeft1);
        motorLeft2 = new WPI_TalonFX(kDrivetrain.kMotor.kIdLeft2);
        motorLeft3 = new WPI_TalonFX(kDrivetrain.kMotor.kIdLeft3);

        motorRight1 = new WPI_TalonFX(kDrivetrain.kMotor.kIdRight1);
        motorRight2 = new WPI_TalonFX(kDrivetrain.kMotor.kIdRight2);
        motorRight3 = new WPI_TalonFX(kDrivetrain.kMotor.kIdRight3);

        // Motor config
        motorLeft1.configFactoryDefault();
        motorLeft2.configFactoryDefault();
        motorLeft3.configFactoryDefault();
        motorRight1.configFactoryDefault();
        motorRight2.configFactoryDefault();
        motorRight3.configFactoryDefault();

        motorLeft1.setInverted(true);
        motorLeft2.setInverted(true);
        motorLeft3.setInverted(true);

        motorLeft1.setNeutralMode(NeutralMode.Brake);
        motorLeft2.setNeutralMode(NeutralMode.Brake);
        motorLeft3.setNeutralMode(NeutralMode.Brake);
        motorRight1.setNeutralMode(NeutralMode.Brake);
        motorRight2.setNeutralMode(NeutralMode.Brake);
        motorRight3.setNeutralMode(NeutralMode.Brake);

        motorLeft1.configSupplyCurrentLimit(currentLimitCfg);
        motorLeft2.configSupplyCurrentLimit(currentLimitCfg);
        motorLeft3.configSupplyCurrentLimit(currentLimitCfg);
        motorRight1.configSupplyCurrentLimit(currentLimitCfg);
        motorRight2.configSupplyCurrentLimit(currentLimitCfg);
        motorRight3.configSupplyCurrentLimit(currentLimitCfg);

        motorLeft2.follow(motorLeft1);
        motorLeft3.follow(motorLeft1);

        motorRight2.follow(motorRight1);
        motorRight3.follow(motorRight1);

        // DifferentialDrive
        motorDiffDrive = new DifferentialDrive(motorLeft1, motorRight1);

        // Setup encoders
        encLeft = new WPI_CANCoder(kDrivetrain.kCANCoder.kIdEncLeft);
        encRight = new WPI_CANCoder(kDrivetrain.kCANCoder.kIdEncRight);

        encCfg = new CANCoderConfiguration();
        encCfg.sensorCoefficient = kDrivetrain.kCANCoder.kSensorCoeff;
        encCfg.unitString = kDrivetrain.kCANCoder.kEncUnit;

        encLeft.configAllSettings(encCfg);
        encRight.configAllSettings(encCfg);

    }

    // arcadeDrive
    public void arcadeDrive(double speed, double rotation)
    {
        motorDiffDrive.arcadeDrive(speed, rotation);
    }

    // Encoder methods
    public void resetEncoder()
    {
        encLeft.setPosition(0);
        encRight.setPosition(0);
    }

    public double getDistanceLeft()
    {
        return encLeft.getPosition();
    }

    public double getDistanceRight()
    {
        return encRight.getPosition();
    }

    public double getVelocityLeft()
    {
        return encLeft.getVelocity();
    }

    public double getVelocityRight()
    {
        return encRight.getVelocity();
    }
}