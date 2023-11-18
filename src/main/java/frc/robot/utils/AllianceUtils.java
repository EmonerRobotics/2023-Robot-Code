package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class AllianceUtils {
    public static Pose2d allianceToField(Pose2d alliancePose) {
        switch (DriverStation.getAlliance()) {
            case Blue:
                return alliancePose;
            case Red:
                return new Pose2d(
                    new Translation2d(
                        FieldConstants.fieldLength - alliancePose.getX(),
                        alliancePose.getY()
                    ),
                    alliancePose.getRotation().minus(Rotation2d.fromRadians(Math.PI))
                );
            default:
                return null;
        }
    }

    public static Pose2d fieldToAlliance(Pose2d fieldPose) {
        switch (DriverStation.getAlliance()) {
            case Blue:
                return fieldPose;
            case Red:
                return new Pose2d(
                        new Translation2d(
                                FieldConstants.fieldLength - fieldPose.getX(),
                                fieldPose.getY()
                        ),
                        fieldPose.getRotation().minus(new Rotation2d(Math.PI))
                );
            default:
                return null;
        }
    }

    public static boolean isBlue() {
        //Red seçince blue planı çalışıyo
        //Blue seçince red planı çalışıyo
        return DriverStation.getAlliance().equals(Alliance.Red);
    }

    public static double getDistanceFromAlliance(Pose2d pose) {
        return isBlue()? pose.getX() : FieldConstants.fieldLength - pose.getX();
    }

    public static Rotation2d getFieldOrientationZero() {
        return Rotation2d.fromRadians(isBlue() ? 0 : Math.PI);
    }
}
