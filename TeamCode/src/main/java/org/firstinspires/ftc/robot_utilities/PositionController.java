package org.firstinspires.ftc.robot_utilities;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.arcrobotics.ftclib.trajectory.Trajectory;

public class PositionController {

    public DifferentialDriveOdometry odometry;
    private PIDController pidDrive;
    private RotationController rotationController;

    private Trajectory m_trajectory;

    private Pose2d startPose;
    private Pose2d m_poseError = new Pose2d();
    private Pose2d m_poseTolerance = new Pose2d();

    public PositionController(Pose2d currentPose,
                              RotationController rotationController,
                              double b,
                              double zeta) {
        pidDrive = new PIDController(Vals.drive_kp, Vals.drive_ki, Vals.drive_kd);
        pidDrive.setTolerance(Vals.drive_tolerance);

        this.rotationController = rotationController;
        this.startPose = currentPose;
        odometry = new DifferentialDriveOdometry(new Rotation2d(rotationController.getAngleRadians()), currentPose);

    }

    public PositionController(Pose2d currentPose, RotationController rotationController) {
        this(currentPose, rotationController, 2.0, 0.7);
    }

    /**
     * Update current position
     * @param driveTrainDistance
     */
    public void update(double[] driveTrainDistance) {
        double leftDistanceInch = driveTrainDistance[0] / Vals.TICKS_PER_INCH_MOVEMENT;
        double rightDistanceInch = driveTrainDistance[1] / Vals.TICKS_PER_INCH_MOVEMENT;
        odometry.update(new Rotation2d(rotationController.getAngleRadians()), leftDistanceInch, rightDistanceInch);

    }

    /**
     * Functional code with no use outside of tuning
     */
    public void updatePID() {
        pidDrive.setPID(Vals.drive_kp, Vals.drive_ki, Vals.drive_kd);
        pidDrive.setTolerance(Vals.drive_tolerance);
    }

    /**
     * functional code - self explanatory
     * @param a
     * @param b
     * @return
     */
    private double getDistance(Pose2d a, Pose2d b) {
        return Math.hypot(a.getX() - b.getX(), a.getY() - b.getY());
    }

    private double getRotationToPoint(Pose2d target, Pose2d o) {
        double dy = target.getY() - o.getY();
        double dx = -target.getX() + o.getX();

//        if(dx == 0) return 0;

        return Math.toDegrees(Math.atan2(dx, dy));
    }

    /**
     * Returns sin(x) / x.
     *
     * @param x Value of which to take sinc(x).
     */
    @SuppressWarnings("ParameterName")
    private static double sinc(double x) {
        if (Math.abs(x) < 1e-9) {
            return 1.0 - 1.0 / 6.0 * x * x;
        } else {
            return Math.sin(x) / x;
        }
    }

    public void reset() {
        pidDrive.reset();
        startPose = odometry.getPoseMeters();
        rotationController.resetAngle();
    }




}
