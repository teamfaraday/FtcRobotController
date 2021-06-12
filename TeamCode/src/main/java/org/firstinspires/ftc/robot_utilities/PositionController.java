package org.firstinspires.ftc.robot_utilities;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.arcrobotics.ftclib.trajectory.Trajectory;

import org.firstinspires.ftc.robot.DriveTrain;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PositionController {

    private PController pDrive;
    private SimpleMotorFeedforward ffLeft;
    private SimpleMotorFeedforward ffRight;
    private RotationController rotationController;

    public DriveTrain driveTrain;

    private double lastLeftPower = 0;
    private double lastRightPower = 0;
    private int ticks;

    public PositionController(RotationController rotationController,
                              Motor driveLeft, Motor driveRight) {

        this.driveTrain = new DriveTrain(driveLeft, driveRight, Motor.RunMode.RawPower);

        pDrive = new PController(Vals.drive_position_kp);
        pDrive.setTolerance(Vals.drive_tolerance);

        ffLeft = new SimpleMotorFeedforward(Vals.drive_left_ks, Vals.drive_left_kv);
        ffRight = new SimpleMotorFeedforward(Vals.drive_right_ks, Vals.drive_right_kv);

        this.rotationController = rotationController;

        ticks = 0;

    }

    /**
     * Functional code with no use outside of tuning
     */
    public void updatePID() {
        pDrive.setP(Vals.drive_position_kp);
        pDrive.setTolerance(Vals.drive_tolerance);
        ffLeft = new SimpleMotorFeedforward(Vals.drive_left_ks, Vals.drive_left_kv);
        ffRight = new SimpleMotorFeedforward(Vals.drive_right_ks, Vals.drive_right_kv);
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

    public void stop() {
        driveTrain.stop();
    }

    public boolean rotateInPlace(double degrees) {
        double rotatePower = rotationController.rotate(degrees);
        double leftPower = -rotatePower;
        double rightPower = rotatePower;

        driveTrain.setSpeedPositiveForward(leftPower, rightPower);

        Vals.test_pDrive_val = rotatePower;
        Vals.test_rotation_sp = rotationController.atRotation();

        if (Math.abs(rotatePower) < Vals.POWER_THRESHOLD) {
            ticks++;
        }
        if(ticks > 100) {
            ticks = 0;
            return true;
        }

        return false;
    }

    public boolean goStraight(double dist, double maxSpeed) {
        double dx = driveTrain.getAverageDistance();
        double rotatePower = rotationController.rotate(0);
        double driveSpeed = Math.min(Math.max(pDrive.calculate(dx, dist), -maxSpeed), maxSpeed);
        Vals.test_pDrive_val = driveSpeed;

        double leftPower = -rotatePower + driveSpeed;// + ffLeft.calculate(lastLeftPower);
        double rightPower = rotatePower + driveSpeed;// + ffRight.calculate(lastRightPower);

        lastLeftPower = leftPower;
        lastRightPower = rightPower;

        driveTrain.setSpeedPositiveForward(leftPower, rightPower);

        Vals.test_pDrive_sp = pDrive.atSetPoint();
        Vals.test_rotation_sp = rotationController.atRotation();

        if((leftPower + rightPower) / 2 < Vals.POWER_THRESHOLD) {
            ticks++;
        }

        if(ticks > 100) {
            ticks = 0;
            return true;
        }

        return false;

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
        driveTrain.resetEncoders();
        pDrive.reset();
        rotationController.resetAngle();
    }




}
