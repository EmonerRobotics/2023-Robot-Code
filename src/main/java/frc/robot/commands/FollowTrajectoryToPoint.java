package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.RobotContainer;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.Set;
import java.util.function.Supplier;

//Uses PathPlanner to move the robot to the specified Pose2d
public class FollowTrajectoryToPoint implements Command {
    protected final Drivetrain drivetrain;
    protected final PoseEstimation poseEstimation;
    protected final Supplier<Pose2d> target;
    public PathPlannerTrajectory trajectory;

    private PPSwerveControllerCommand swerveControllerCommand;

    public FollowTrajectoryToPoint(Drivetrain drivetrain, PoseEstimation poseEstimation, Supplier<Pose2d> target) {
        this.drivetrain = drivetrain;
        this.poseEstimation = poseEstimation;
        this.target = target;
    }

    @Override
    public void initialize() {
        trajectory = buildTrajectory(target.get());


        RobotContainer.field.getObject("Alignment Target").setPose(trajectory.getEndState().poseMeters);
        RobotContainer.field.getObject("Alignment Target").setTrajectory(trajectory);
        RobotContainer.field.getObject("Target").setPose(target.get());

        swerveControllerCommand = new PPSwerveControllerCommand(
                trajectory,
                poseEstimation::getEstimatedPose,
                new PIDController(AutoConstants.P_TRANSLATION_PATH_CONTROLLER, 0.0, 0.0),
                new PIDController(AutoConstants.P_TRANSLATION_PATH_CONTROLLER, 0.0, 0.0),
                new PIDController(AutoConstants.P_THETA_PATH_CONTROLLER, 0.0, 0.0),
                drivetrain::drive
        );

        swerveControllerCommand.initialize();
    }

    protected PathPlannerTrajectory buildTrajectory(Pose2d target) {
        Pose2d initial = poseEstimation.getEstimatedPose();
        Translation2d initialV = poseEstimation.getEstimatedVelocity();

        return PathPlanner.generatePath(
                new PathConstraints(
                        AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                        AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
                ),
                new PathPoint(
                        initial.getTranslation(),
                        initialV.getNorm() == 0 ?
                                target.getTranslation().minus(initial.getTranslation()).getAngle() :
                                initialV.getAngle(),
                        initial.getRotation(),
                        initialV.getNorm()),
                new PathPoint(
                        target.getTranslation(),
                        target.getTranslation().minus(initial.getTranslation()).getAngle(),
                        target.getRotation())
        );
    }

    @Override
    public void execute() {
        swerveControllerCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        swerveControllerCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return swerveControllerCommand.isFinished();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }

}
