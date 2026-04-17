package frc.robot.subsystems.swerve.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.SwerveConstants;
import frc.robot.utils.SparkUtil;

public class ModuleIOSparkMax implements ModuleIO {
    private final SparkMax driveMotor;
    private final SparkMax angleMotor;
    private final CANcoder absoluteEncoder;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder angleEncoder;
    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController angleController;

    private final StatusSignal<Angle> absolutePosition;

    public ModuleIOSparkMax(
            int driveId,
            int angleId,
            int encoderId,
            double absoluteEncoderOffsetDegrees,
            boolean driveInverted,
            boolean angleInverted,
            boolean encoderInverted) {
        driveMotor = new SparkMax(driveId, MotorType.kBrushless);
        angleMotor = new SparkMax(angleId, MotorType.kBrushless);

        absoluteEncoder = new CANcoder(encoderId);

        driveEncoder = driveMotor.getEncoder();
        angleEncoder = angleMotor.getEncoder();

        driveController = driveMotor.getClosedLoopController();
        angleController = angleMotor.getClosedLoopController();

        absolutePosition = absoluteEncoder.getAbsolutePosition();

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = encoderInverted
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = -absoluteEncoderOffsetDegrees / 360.0;
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        absoluteEncoder.getConfigurator().apply(encoderConfig);

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig
                .inverted(driveInverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(SwerveConstants.PhysicalProperties.DRIVE_CURRENT_LIMIT)
                .voltageCompensation(SwerveConstants.PhysicalProperties.OPTIMAL_VOLTAGE_VOLTS)
                .openLoopRampRate(SwerveConstants.PhysicalProperties.DRIVE_RAMP_RATE_SECONDS)
                .closedLoopRampRate(SwerveConstants.PhysicalProperties.DRIVE_RAMP_RATE_SECONDS);
        driveConfig.encoder
                .positionConversionFactor(SwerveConstants.DRIVE_POSITION_FACTOR_METERS_PER_ROTATION)
                .velocityConversionFactor(SwerveConstants.DRIVE_VELOCITY_FACTOR_METERS_PER_SEC_PER_RPM);
        driveConfig.closedLoop.pid(
                SwerveConstants.PIDFProperties.DRIVE_kP,
                SwerveConstants.PIDFProperties.DRIVE_kI,
                SwerveConstants.PIDFProperties.DRIVE_kD);

        SparkMaxConfig angleConfig = new SparkMaxConfig();
        angleConfig
                .inverted(angleInverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(SwerveConstants.PhysicalProperties.ANGLE_CURRENT_LIMIT)
                .voltageCompensation(SwerveConstants.PhysicalProperties.OPTIMAL_VOLTAGE_VOLTS)
                .openLoopRampRate(SwerveConstants.PhysicalProperties.ANGLE_RAMP_RATE_SECONDS)
                .closedLoopRampRate(SwerveConstants.PhysicalProperties.ANGLE_RAMP_RATE_SECONDS);
        angleConfig.encoder
                .positionConversionFactor(SwerveConstants.ANGLE_POSITION_FACTOR_RAD_PER_ROTATION)
                .velocityConversionFactor(SwerveConstants.ANGLE_VELOCITY_FACTOR_RAD_PER_SEC_PER_RPM);
        angleConfig.closedLoop
                .pid(
                        SwerveConstants.PIDFProperties.ANGLE_kP,
                        SwerveConstants.PIDFProperties.ANGLE_kI,
                        SwerveConstants.PIDFProperties.ANGLE_kD)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(-Math.PI, Math.PI);

        SparkUtil.tryUntilOk(
                driveMotor,
                5,
                () -> driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters));
        SparkUtil.tryUntilOk(
                angleMotor,
                5,
                () -> angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters));

        syncAngleEncoder();
    }

    private void syncAngleEncoder() {
        BaseStatusSignal.refreshAll(absolutePosition);

        SparkUtil.tryUntilOk(
                angleMotor,
                5,
                () -> angleEncoder.setPosition(
                        Units.rotationsToRadians(absolutePosition.getValueAsDouble())));
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.encoderConnected = BaseStatusSignal.refreshAll(absolutePosition).isOK();
        inputs.angleAbsolutePositionRad = Units.rotationsToRadians(absolutePosition.getValueAsDouble());

        SparkUtil.sparkStickyFault = false;
        SparkUtil.ifOk(
                driveMotor,
                new java.util.function.DoubleSupplier[] {
                        driveEncoder::getPosition,
                        driveEncoder::getVelocity,
                        driveMotor::getAppliedOutput,
                        driveMotor::getBusVoltage,
                        driveMotor::getOutputCurrent,
                        driveMotor::getMotorTemperature
                },
                values -> {
                    inputs.drivePositionMeters = values[0];
                    inputs.driveVelocityMetersPerSec = values[1];
                    inputs.driveAppliedVolts = values[2] * values[3];
                    inputs.driveCurrentAmps = values[4];
                    inputs.driveTempCelsius = values[5];
                });
        inputs.driveConnected = !SparkUtil.sparkStickyFault;

        SparkUtil.sparkStickyFault = false;
        SparkUtil.ifOk(
                angleMotor,
                new java.util.function.DoubleSupplier[] {
                        angleEncoder::getPosition,
                        angleEncoder::getVelocity,
                        angleMotor::getAppliedOutput,
                        angleMotor::getBusVoltage,
                        angleMotor::getOutputCurrent,
                        angleMotor::getMotorTemperature
                },
                values -> {
                    inputs.anglePositionRad = values[0];
                    inputs.angleVelocityRadPerSec = values[1];
                    inputs.angleAppliedVolts = values[2] * values[3];
                    inputs.angleCurrentAmps = values[4];
                    inputs.angleTempCelsius = values[5];
                });
        inputs.angleConnected = !SparkUtil.sparkStickyFault;
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.01) {
            driveMotor.stopMotor();
            angleController.setSetpoint(
                    state.angle.getRadians(),
                    ControlType.kPosition,
                    ClosedLoopSlot.kSlot0);

            return;
        }

        driveController.setSetpoint(
                state.speedMetersPerSecond,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0);
        angleController.setSetpoint(
                state.angle.getRadians(),
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    @Override
    public void setAngleVoltage(double volts) {
        angleMotor.setVoltage(volts);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        SparkMaxConfig angleConfig = new SparkMaxConfig();

        driveConfig.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        angleConfig.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);

        SparkUtil.tryUntilOk(
                driveMotor,
                5,
                () -> driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters,
                        PersistMode.kPersistParameters));
        SparkUtil.tryUntilOk(
                angleMotor,
                5,
                () -> angleMotor.configure(angleConfig, ResetMode.kNoResetSafeParameters,
                        PersistMode.kPersistParameters));
    }

    @Override
    public void stop() {
        driveMotor.stopMotor();
        angleMotor.stopMotor();
    }

    public Rotation2d getAbsoluteAngle() {
        BaseStatusSignal.refreshAll(absolutePosition);
        return Rotation2d.fromRadians(Units.rotationsToRadians(absolutePosition.getValueAsDouble()));
    }
}
