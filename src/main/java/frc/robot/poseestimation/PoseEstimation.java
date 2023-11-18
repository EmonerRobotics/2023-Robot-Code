package frc.robot.poseestimation;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

public class PoseEstimation {
    private PhotonVisionBackend photonVision;
    private SwerveDrivePoseEstimator poseEstimator;

    private GenericEntry usePhotonVisionEntry = RobotContainer.autoTab.add("Use PhotonVision", true).withWidget(BuiltInWidgets.kToggleButton).getEntry();

    private TimeInterpolatableBuffer<Pose2d> poseHistory = TimeInterpolatableBuffer.createBuffer(1.5);

    private static final double DIFFERENTIATION_TIME = Robot.kDefaultPeriod;

    public PoseEstimation() {
        poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            RobotContainer.drivetrain.getRotation(),
            RobotContainer.drivetrain.getModulePositions(),
            new Pose2d(),
            Constants.DriveConstants.ODOMETRY_STD_DEV,
            VecBuilder.fill(0, 0, 0) // will be overwritten for each measurement
        );

        try {
            photonVision = new PhotonVisionBackend();
        } catch (Exception e) {
            System.out.println("Failed to initialize PhotonVision");
            e.printStackTrace();
        }

    }

    public void periodic() {
        poseHistory.addSample(Timer.getFPGATimestamp(), poseEstimator.getEstimatedPosition());

        if (usePhotonVisionEntry.getBoolean(false)) {
            try {
                photonVision.getMeasurement().filter(measurement -> measurement.ambiguity < Constants.VisionConstants.AMBIGUITY_FILTER).ifPresent(this::addVisionMeasurement);
            }
            catch(Exception e) {
                e.printStackTrace();
            }
        }

        RobotContainer.field.setRobotPose(getEstimatedPose());
    }

    public void updateOdometry(Rotation2d gyro, SwerveModulePosition[] modulePositions) {
        poseEstimator.update(gyro, modulePositions);
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Translation2d getEstimatedVelocity() {
        double now = Timer.getFPGATimestamp();

        Translation2d current = poseHistory.getSample(now).get().getTranslation();
        Translation2d previous = poseHistory.getSample(now - DIFFERENTIATION_TIME).get().getTranslation();

        return current.minus(previous).div(DIFFERENTIATION_TIME);
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(RobotContainer.drivetrain.getRotation(), RobotContainer.drivetrain.getModulePositions(), pose);
    }

    private void addVisionMeasurement(VisionBackend.Measurement measurement) {
        poseEstimator.addVisionMeasurement(measurement.pose.toPose2d(), measurement.timestamp, measurement.stdDeviation);
    }
}