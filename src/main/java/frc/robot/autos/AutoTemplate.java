package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.constants.Constants;


import frc.robot.subsystems.Swerve;

public class AutoTemplate extends SequentialCommandGroup {

    
    public AutoTemplate(Swerve s_Swerve){

    //     //this configures the robot's speed as well as its size. this allows it to run everything correctly. all values should be tuned in the constants.
    //     TrajectoryConfig config =
    //         new TrajectoryConfig(
    //                 (Constants.AutoConstants.kMaxSpeedMetersPerSecond),
    //                 (Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared))
    //             .setKinematics(Constants.Swerve.swerveKinematics);
        
    //     var thetaController =
    //     new ProfiledPIDController(
    //         Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //     Trajectory exampleTrajectory =
    //         TrajectoryGenerator.generateTrajectory(
    //             // Start at the origin facing forward.
    //             new Pose2d(0, 0, new Rotation2d(0)),
    //             //create a list of points that you want the robot to pass through in meters.
    //             List.of(new Translation2d(1, 0),
    //             new Translation2d(2,0)),
    //             // put the desired end coordinates in meters.
    //             new Pose2d(3, 2, new Rotation2d(0)),
    //             config);

    //         SwerveControllerCommand ExampleTrajectoryCommand =
    //             new SwerveControllerCommand(
    //                 exampleTrajectory,
    //                 s_Swerve::getPose,
    //                 Constants.Swerve.swerveKinematics,
    //                 new PIDController(Constants.AutoConstants.kPXController, 0, 0),
    //                 new PIDController(Constants.AutoConstants.kPYController, 0, 0),
    //                 thetaController,
    //                 s_Swerve::setModuleStates,
    //                 s_Swerve);

    //     addCommands(

    //         new InstantCommand((() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose()))),
    //         new InstantCommand((() -> s_Swerve.zeroGyro())),
    //         ExampleTrajectoryCommand
            
    //     );

    }
}