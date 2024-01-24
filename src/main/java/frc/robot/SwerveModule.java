package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.constants.SwerveConstants;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.DemandType;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenix.sensors.CANCoder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;
    final VoltageOut mVoltage = new VoltageOut(0);
    private DutyCycleOut mDutyCycleOut;
    private VelocityDutyCycle mVelocityDutyCycle;
    private PositionDutyCycle mPositionDutyCycle;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.Swerve.driveKS, SwerveConstants.Swerve.driveKV, SwerveConstants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.Swerve.maxSpeed;
//            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
            mDutyCycleOut = new DutyCycleOut(percentOutput);
            mDriveMotor.setControl(mDutyCycleOut);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, SwerveConstants.Swerve.wheelCircumference, SwerveConstants.Swerve.driveGearRatio);
            mVelocityDutyCycle = new VelocityDutyCycle(velocity);
            mDriveMotor.setControl(mVelocityDutyCycle);
//           mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        mPositionDutyCycle = new PositionDutyCycle(Conversions.degreesToFalcon(angle.getDegrees(), SwerveConstants.Swerve.angleGearRatio));        
        mAngleMotor.setControl(mPositionDutyCycle);
//        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), SwerveConstants.Swerve.angleGearRatio));
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getPosition().getValueAsDouble(), SwerveConstants.Swerve.angleGearRatio));
//        return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), SwerveConstants.Swerve.angleGearRatio));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), SwerveConstants.Swerve.angleGearRatio);
//        mAngleMotor.setSelectedSensorPosition(absolutePosition);
        mAngleMotor.setPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
//        angleEncoder.configFactoryDefault();
//        angleEncoder.configAllSettings(CTREConfigs.swerveCanCoderConfig);

        CANcoderConfiguration swerveCanCoderConfig = new CANcoderConfiguration();
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();

        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magnetSensorConfigs.SensorDirection = SwerveConstants.Swerve.canCoderInvert;
        magnetSensorConfigs.MagnetOffset = 0.0;
        swerveCanCoderConfig.withMagnetSensor(magnetSensorConfigs);

        CANcoderConfigurator canCoderConfigurator = angleEncoder.getConfigurator();
//        canCoderConfigurator.apply(CTREConfigs.swerveCanCoderConfig);
        canCoderConfigurator.apply(swerveCanCoderConfig);

    }

    private void configAngleMotor(){
        // mAngleMotor.configFactoryDefault();
        // mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        // mAngleMotor.setInverted(SwerveConstants.Swerve.angleMotorInvert);
        // mAngleMotor.setNeutralMode(SwerveConstants.Swerve.angleNeutralMode);
        // resetToAbsolute();

        TalonFXConfigurator talonFXConfigurator = mAngleMotor.getConfigurator();
        TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        Slot0Configs slot0Configs = new Slot0Configs();

        currentLimitsConfigs.SupplyCurrentLimitEnable = SwerveConstants.Swerve.angleEnableCurrentLimit;
        currentLimitsConfigs.SupplyCurrentLimit = SwerveConstants.Swerve.angleContinuousCurrentLimit;
        currentLimitsConfigs.SupplyCurrentThreshold = SwerveConstants.Swerve.anglePeakCurrentLimit;
        currentLimitsConfigs.SupplyTimeThreshold = SwerveConstants.Swerve.anglePeakCurrentDuration;
        slot0Configs.withKP(SwerveConstants.Swerve.angleKP);
        slot0Configs.withKI(SwerveConstants.Swerve.angleKI);
        slot0Configs.withKD(SwerveConstants.Swerve.angleKD);
        slot0Configs.withKV(SwerveConstants.Swerve.angleKF);
        swerveAngleFXConfig.withCurrentLimits(currentLimitsConfigs);
        swerveAngleFXConfig.withSlot0(slot0Configs);
        talonFXConfigurator.apply(swerveAngleFXConfig);
//        talonFXConfigurator.apply(CTREConfigs.swerveAngleFXConfig);

        // set invert to CW+ and apply config change
        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        motorConfigs.Inverted = SwerveConstants.Swerve.angleMotorInvert;
        motorConfigs.NeutralMode = SwerveConstants.Swerve.angleNeutralMode;
        talonFXConfigurator.apply(motorConfigs);
        resetToAbsolute();
        
    }

    private void configDriveMotor(){        
        // mDriveMotor.configFactoryDefault();
        // mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        // mDriveMotor.setInverted(SwerveConstants.Swerve.driveMotorInvert);
        // mDriveMotor.setNeutralMode(SwerveConstants.Swerve.driveNeutralMode);
        // mDriveMotor.setSelectedSensorPosition(0);
        mDriveMotor.setPosition(0);

        TalonFXConfigurator talonFXConfigurator = mDriveMotor.getConfigurator();
        TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        Slot0Configs slot0Configs = new Slot0Configs();
        OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
        ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    
        currentLimitsConfigs.SupplyCurrentLimitEnable = SwerveConstants.Swerve.driveEnableCurrentLimit;
        currentLimitsConfigs.SupplyCurrentLimit = SwerveConstants.Swerve.driveContinuousCurrentLimit;
        currentLimitsConfigs.SupplyCurrentThreshold = SwerveConstants.Swerve.drivePeakCurrentLimit;
        currentLimitsConfigs.SupplyTimeThreshold = SwerveConstants.Swerve.drivePeakCurrentDuration;
        slot0Configs.withKP(SwerveConstants.Swerve.driveKP);
        slot0Configs.withKI(SwerveConstants.Swerve.driveKI);
        slot0Configs.withKD(SwerveConstants.Swerve.driveKD);
        slot0Configs.withKV(SwerveConstants.Swerve.driveKF);
        openLoopRampsConfigs.withDutyCycleOpenLoopRampPeriod(SwerveConstants.Swerve.openLoopRamp);
        closedLoopRampsConfigs.withDutyCycleClosedLoopRampPeriod(SwerveConstants.Swerve.closedLoopRamp);
        swerveDriveFXConfig.withCurrentLimits(currentLimitsConfigs);
        swerveDriveFXConfig.withSlot0(slot0Configs);
        swerveDriveFXConfig.withOpenLoopRamps(openLoopRampsConfigs);
        swerveDriveFXConfig.withClosedLoopRamps(closedLoopRampsConfigs);
        talonFXConfigurator.apply(swerveDriveFXConfig);
//        talonFXConfigurator.apply(CTREConfigs.swerveDriveFXConfig);

        // set invert to CW+ and apply config change
        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        motorConfigs.Inverted = SwerveConstants.Swerve.driveMotorInvert;
        motorConfigs.NeutralMode = SwerveConstants.Swerve.driveNeutralMode;
        talonFXConfigurator.apply(motorConfigs);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(mDriveMotor.getVelocity().getValueAsDouble(), SwerveConstants.Swerve.wheelCircumference, SwerveConstants.Swerve.driveGearRatio), 
            getAngle()
        );
         
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(mDriveMotor.getVelocity().getValueAsDouble(), SwerveConstants.Swerve.wheelCircumference, SwerveConstants.Swerve.driveGearRatio), 
            getAngle()
        );
    }
}