package frc.robot.commands.autonomous;

import java.util.List;
import java.util.Map;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;  
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;

public class AutoCommand {
    //LiftSubsystem liftSubsystem = new LiftSubsystem();
    SmartDashboard smartDashboard;

    //static HashMap<String, Command> eventMap = new HashMap<>();
    
    
    static Map<String, Command> eventMap = Map.of(
        "event1", new InstantCommand(() -> System.out.println("event triggered"))); 
        
        public static Command makeAutoCommand(Drivetrain drivetrain, PoseEstimation poseEstimation, String name) {
        name = name.concat(AllianceUtils.isBlue()? ".blue" : ".red");
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(name, new PathConstraints(Constants.AutoConstants.MAX_SPEED_METERS_PER_SECOND, Constants.AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));

        RobotContainer.field.getObject("Auto Path").setTrajectory(pathGroup.get(0));

        // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            poseEstimation::getEstimatedPose, // Pose2d supplier
            (pose) -> poseEstimation.resetPose(pose), // Pose 2d consumer, used to reset odometry at the beginning of auto
            Constants.DriveConstants.DRIVE_KINEMATICS, // SwerveDriveKinematics
            new PIDConstants(Constants.AutoConstants.P_TRANSLATION_PATH_CONTROLLER, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(Constants.AutoConstants.P_THETA_PATH_CONTROLLER, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            drivetrain::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
        );
        
        return autoBuilder.fullAuto(pathGroup);
    }   
}
