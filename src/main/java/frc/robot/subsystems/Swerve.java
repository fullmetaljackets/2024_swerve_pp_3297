package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.constants.SwerveConstants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

//import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public static SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public Swerve() {
        gyro = new Pigeon2(SwerveConstants.Swerve.gyroID);
         zeroGyro();

        //this creates the swerve modules used to drive. this should not need to be changed. only the constants will ever need to be updated.
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.Swerve.FrontLeft.constants),
            new SwerveModule(1, SwerveConstants.Swerve.FrontRight.constants),
            new SwerveModule(2, SwerveConstants.Swerve.BackLeft.constants),
            new SwerveModule(3, SwerveConstants.Swerve.BackRight.constants)
        };

        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(SwerveConstants.Swerve.swerveKinematics, getYaw(), getModulePositions());


        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

    }

    //this is what is referenced in teleop by teleopswerve. this allows the robot to actually drive. 
    // public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    public void drive(double translationVal, double strafeVal, double rotation, boolean fieldRelative, boolean isOpenLoop) {
         //     new Translation2d(translationVal, strafeVal).times(SwerveConstants.Swerve.maxSpeed), 
        //     rotationVal * SwerveConstants.Swerve.maxAngularVelocity, 

        SwerveModuleState[] swerveModuleStates =
            SwerveConstants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translationVal, 
                                    strafeVal, 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translationVal, 
                                    strafeVal, 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public void driveRobotRelative(ChassisSpeeds speeds){
        this.drive(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,speeds.omegaRadiansPerSecond,false,false);
    }
    
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return SwerveConstants.Swerve.swerveKinematics.toChassisSpeeds(mSwerveMods[0].getState(),
                                                                    mSwerveMods[1].getState(),
                                                                    mSwerveMods[2].getState(),
                                                                    mSwerveMods[3].getState());
    }
    
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void UpdateGyroDashboard()
    {
        SmartDashboard.putNumber("Gyro Heading:", gyro.getYaw().getValueAsDouble());
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public void OneEightyGyro()
    {
        gyro.setYaw(180);
    }

    public Rotation2d getYaw() {
        return (SwerveConstants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw().getValueAsDouble()) : Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public double getGyroYawReading()
    {
        return gyro.getYaw().getValueAsDouble();
    }

    public double getGyroPitchReading()
    {
        return gyro.getPitch().getValueAsDouble();
    }


    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  

        //this just posts everything to the smart dashboard for easy visibility.
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        SmartDashboard.putNumber("Gyro Yaw:", gyro.getYaw().getValueAsDouble());
        SmartDashboard.putNumber("Gyro Pitch:", gyro.getPitch().getValueAsDouble());
        SmartDashboard.putNumber("Gyro Roll:", gyro.getRoll().getValueAsDouble());
        SmartDashboard.putNumber("Gyro Degrees:", getGyroPitchReading());
        }
}