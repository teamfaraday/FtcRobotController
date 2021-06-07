package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robot.DriveTrain;
import org.firstinspires.ftc.robot.FlyWheel;
import org.firstinspires.ftc.robot.Hitter;
import org.firstinspires.ftc.robot.Intake;
import org.firstinspires.ftc.robot.WobbleSystem;
import org.firstinspires.ftc.robot_utilities.RotationController;
import org.firstinspires.ftc.robot_utilities.VisionController;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "EchoBlueLeftAuto", group = "Echo Autos")
public class EchoBlueLeftAuto extends OpMode {

    ElapsedTime elapsedTime;
    VisionController visionController;
    RotationController rotationController;
    boolean notStarted = true;
    int ringStackSize = -1;
    int shotsFired = 0;
    private DriveTrain driveTrain;
    private FlyWheel flywheel;
    private WobbleSystem wobbleSystem;
    private Hitter hitter;
    private Intake intake;

    @Override
    public void init() {
        elapsedTime = new ElapsedTime();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvInternalCamera phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        visionController = new VisionController(phoneCam);

        rotationController = new RotationController(hardwareMap.get(BNO055IMU.class, "imu"));
        rotationController.resetAngle();

        driveTrain = new DriveTrain(new Motor(hardwareMap, "dl"), new Motor(hardwareMap, "dr"));
        driveTrain.resetEncoders();
        flywheel = new FlyWheel(new Motor(hardwareMap, "fw", Motor.GoBILDA.BARE), telemetry);
        wobbleSystem = new WobbleSystem(new Motor(hardwareMap, "wobbleArmMotor"), hardwareMap.servo.get("wobbleArmServo"));
        wobbleSystem.hand_close();
        hitter = new Hitter(hardwareMap.servo.get("sv"));
        intake = new Intake(new Motor(hardwareMap, "in1"), new Motor(hardwareMap, "in2"));

    }

    @Override
    public void loop() {
        if (notStarted) {
            ringStackSize = visionController.getRingPosition();
            elapsedTime.reset();

            notStarted = false;
            return;
        }

        double driveSpeed = 0.4;

        switch (ringStackSize) {

            case 0:

                //Rotate to appropriate angle (insert correct value for rotatePower)
                if (elapsedTime.seconds() < 1.7) {

                    double rotatePower = rotationController.rotate(-45);

                    double leftPower = -rotatePower;
                    double rightPower = rotatePower;

                    leftPower += driveSpeed;
                    rightPower += driveSpeed;


                    driveTrain.setSpeedPositiveForward(leftPower, rightPower);

                }

                elapsedTime.reset();

                //Shoot 3 high goals.
                if (shotsFired == 0) {
                    telemetry.addData("In Shots Fired", 0);
                    driveTrain.stop();
                    flywheel.on_slow();
                    if (flywheel.isReadySlow()) {
                        hitter.hitFullMotion(0.7);
                        shotsFired++;
                        rotationController.resetAngle();
                    }
                } else if (shotsFired == 1) {
                    telemetry.addData("In Shots Fired", 1);
                    flywheel.on_slow();

                    double shootingRotatePower = rotationController.rotate(5);
                    double shootingLeftPower = -shootingRotatePower;
                    double shootingRightPower = shootingRotatePower;
                    driveTrain.setSpeedPositiveForward(shootingLeftPower, shootingRightPower);

                    if (flywheel.isReadySlow() && rotationController.atRotation()) {
                        shotsFired++;
                        hitter.hitFullMotion(0.7);
                        rotationController.resetAngle();
                        driveTrain.stop();
                    }

                } else if (shotsFired == 2) {
                    telemetry.addData("In Shots Fired", 2);
                    flywheel.on_slow();

                    double shootingRotatePower = rotationController.rotate(5);
                    double shootingLeftPower = -shootingRotatePower;
                    double shootingRightPower = shootingRotatePower;
                    driveTrain.setSpeedPositiveForward(shootingLeftPower, shootingRightPower);

                    if (flywheel.isReadySlow() && rotationController.atRotation()) {
                        shotsFired++;
                        hitter.hitFullMotion(0.7);
                        rotationController.resetAngle();
                        driveTrain.stop();
                    }

                }

                elapsedTime.reset();

                //Deliver wobble goal to A (filled with mock data)
                if (elapsedTime.seconds() < 1.7) {
                    double wobbleRotatePower = rotationController.rotate(22);

                    double wobbleLeftPower = -wobbleRotatePower;
                    double wobbleRightPower = wobbleRotatePower;

                    wobbleLeftPower += driveSpeed;
                    wobbleRightPower += driveSpeed;


                    driveTrain.setSpeedPositiveForward(wobbleLeftPower, wobbleRightPower);

                } else if (elapsedTime.seconds() < 3) {

                    driveTrain.stop();

                } else if (elapsedTime.seconds() < 4.5) {

                    telemetry.addData("Wobble Arm Location", wobbleSystem.wobbleArm.getCurrentPosition());
                    telemetry.addData("Wobble Arm Speed", wobbleSystem.wobbleArm.get());
                    telemetry.addData("Wobble Hand Pos", wobbleSystem.wobbleHand.getPosition());
                    telemetry.update();
                    wobbleSystem.arm_down();

                } else if (elapsedTime.seconds() < 5.5) {

                    wobbleSystem.hand_open();

                } else if (elapsedTime.seconds() < 7) {

                    wobbleSystem.hand_close();

                } else if (elapsedTime.seconds() < 8.5) {

                    wobbleSystem.arm_up();

                } else if (elapsedTime.seconds() < 10) {

                    double wobbleRotatePower = rotationController.rotate(22);

                    double wobbleLeftPower = -wobbleRotatePower;
                    double wobbleRightPower = wobbleRotatePower;

                    wobbleLeftPower += driveSpeed;
                    wobbleRightPower += driveSpeed;


                    driveTrain.setSpeedPositiveForward(wobbleLeftPower, wobbleRightPower);

                } else if (elapsedTime.seconds() < 11.5) {

                    wobbleSystem.arm_down();

                } else if (elapsedTime.seconds() < 13) {

                    wobbleSystem.hand_close();

                }

                elapsedTime.reset();

                //Park
                if (elapsedTime.seconds() < 1.7) {
                    double parkRotatePower = rotationController.rotate(0);

                    double parkLeftPower = -parkRotatePower;
                    double parkRightPower = parkRotatePower;

                    parkLeftPower += driveSpeed;
                    parkRightPower += driveSpeed;


                    driveTrain.setSpeedPositiveForward(parkLeftPower, parkRightPower);

                }

                break;

            case 1:

                //Rotate to appropriate angle (insert correct value for rotatePower)
                if (elapsedTime.seconds() < 1.7) {

                    double rotatePower1 = rotationController.rotate(-45);

                    double leftPower1 = -rotatePower1;
                    double rightPower1 = rotatePower1;

                    leftPower1 += driveSpeed;
                    rightPower1 += driveSpeed;


                    driveTrain.setSpeedPositiveForward(leftPower1, rightPower1);

                }

                elapsedTime.reset();

                //Avoid rings.
                if (elapsedTime.seconds() < 1.7) {

                    double avoidRotatePower = rotationController.rotate(-45);

                    double avoidLeftPower = -avoidRotatePower;
                    double avoidRightPower = avoidRotatePower;

                    avoidLeftPower += driveSpeed;
                    avoidRightPower += driveSpeed;


                    driveTrain.setSpeedPositiveForward(avoidLeftPower, avoidRightPower);

                }

                elapsedTime.reset();

                //Shoot 3 high goals.
                if (shotsFired == 0) {
                    telemetry.addData("In Shots Fired", 0);
                    driveTrain.stop();
                    flywheel.on_slow();
                    if (flywheel.isReadySlow()) {
                        hitter.hitFullMotion(0.7);
                        shotsFired++;
                        rotationController.resetAngle();
                    }
                } else if (shotsFired == 1) {
                    telemetry.addData("In Shots Fired", 1);
                    flywheel.on_slow();

                    double shootingRotatePower = rotationController.rotate(5);
                    double shootingLeftPower = -shootingRotatePower;
                    double shootingRightPower = shootingRotatePower;
                    driveTrain.setSpeedPositiveForward(shootingLeftPower, shootingRightPower);

                    if (flywheel.isReadySlow() && rotationController.atRotation()) {
                        shotsFired++;
                        hitter.hitFullMotion(0.7);
                        rotationController.resetAngle();
                        driveTrain.stop();
                    }

                } else if (shotsFired == 2) {
                    telemetry.addData("In Shots Fired", 2);
                    flywheel.on_slow();

                    double shootingRotatePower = rotationController.rotate(5);
                    double shootingLeftPower = -shootingRotatePower;
                    double shootingRightPower = shootingRotatePower;
                    driveTrain.setSpeedPositiveForward(shootingLeftPower, shootingRightPower);

                    if (flywheel.isReadySlow() && rotationController.atRotation()) {
                        shotsFired++;
                        hitter.hitFullMotion(0.7);
                        rotationController.resetAngle();
                        driveTrain.stop();
                    }

                }

                elapsedTime.reset();

                //Deliver wobble goal to B.
                if (elapsedTime.seconds() < 1.7) {
                    double wobbleRotatePower = rotationController.rotate(22);

                    double wobbleLeftPower = -wobbleRotatePower;
                    double wobbleRightPower = wobbleRotatePower;

                    wobbleLeftPower += driveSpeed;
                    wobbleRightPower += driveSpeed;


                    driveTrain.setSpeedPositiveForward(wobbleLeftPower, wobbleRightPower);

                } else if (elapsedTime.seconds() < 3) {

                    driveTrain.stop();

                } else if (elapsedTime.seconds() < 4.5) {

                    telemetry.addData("Wobble Arm Location", wobbleSystem.wobbleArm.getCurrentPosition());
                    telemetry.addData("Wobble Arm Speed", wobbleSystem.wobbleArm.get());
                    telemetry.addData("Wobble Hand Pos", wobbleSystem.wobbleHand.getPosition());
                    telemetry.update();
                    wobbleSystem.arm_down();

                } else if (elapsedTime.seconds() < 5.5) {

                    wobbleSystem.hand_open();

                } else if (elapsedTime.seconds() < 7) {

                    wobbleSystem.hand_close();

                } else if (elapsedTime.seconds() < 8.5) {

                    wobbleSystem.arm_up();

                } else if (elapsedTime.seconds() < 10) {

                    double wobbleRotatePower = rotationController.rotate(22);

                    double wobbleLeftPower = -wobbleRotatePower;
                    double wobbleRightPower = wobbleRotatePower;

                    wobbleLeftPower += driveSpeed;
                    wobbleRightPower += driveSpeed;


                    driveTrain.setSpeedPositiveForward(wobbleLeftPower, wobbleRightPower);

                } else if (elapsedTime.seconds() < 11.5) {

                    wobbleSystem.arm_down();

                } else if (elapsedTime.seconds() < 13) {

                    wobbleSystem.hand_close();

                }

                elapsedTime.reset();

                //Park
                if (elapsedTime.seconds() < 1.7) {
                    double parkRotatePower = rotationController.rotate(0);

                    double parkLeftPower = -parkRotatePower;
                    double parkRightPower = parkRotatePower;

                    parkLeftPower += driveSpeed;
                    parkRightPower += driveSpeed;


                    driveTrain.setSpeedPositiveForward(parkLeftPower, parkRightPower);

                }

                break;

            case 4:

                //Rotate to appropriate angle (insert correct value for rotatePower)
                if (elapsedTime.seconds() < 1.7) {

                    double rotatePower = rotationController.rotate(-45);

                    double leftPower = -rotatePower;
                    double rightPower = rotatePower;

                    leftPower += driveSpeed;
                    rightPower += driveSpeed;


                    driveTrain.setSpeedPositiveForward(leftPower, rightPower);

                }

                elapsedTime.reset();

                //Shoot 3 high goals.
                if (shotsFired == 0) {
                    telemetry.addData("In Shots Fired", 0);
                    driveTrain.stop();
                    flywheel.on_slow();
                    if (flywheel.isReadySlow()) {
                        hitter.hitFullMotion(0.7);
                        shotsFired++;
                        rotationController.resetAngle();
                    }
                } else if (shotsFired == 1) {
                    telemetry.addData("In Shots Fired", 1);
                    flywheel.on_slow();

                    double shootingRotatePower = rotationController.rotate(5);
                    double shootingLeftPower = -shootingRotatePower;
                    double shootingRightPower = shootingRotatePower;
                    driveTrain.setSpeedPositiveForward(shootingLeftPower, shootingRightPower);

                    if (flywheel.isReadySlow() && rotationController.atRotation()) {
                        shotsFired++;
                        hitter.hitFullMotion(0.7);
                        rotationController.resetAngle();
                        driveTrain.stop();
                    }

                } else if (shotsFired == 2) {
                    telemetry.addData("In Shots Fired", 2);
                    flywheel.on_slow();

                    double shootingRotatePower = rotationController.rotate(5);
                    double shootingLeftPower = -shootingRotatePower;
                    double shootingRightPower = shootingRotatePower;
                    driveTrain.setSpeedPositiveForward(shootingLeftPower, shootingRightPower);

                    if (flywheel.isReadySlow() && rotationController.atRotation()) {
                        shotsFired++;
                        hitter.hitFullMotion(0.7);
                        rotationController.resetAngle();
                        driveTrain.stop();
                    }

                }

                elapsedTime.reset();

                //Deliver wobble goal to C (filled with mock data)
                if (elapsedTime.seconds() < 1.7) {
                    double wobbleRotatePower = rotationController.rotate(20);

                    double wobbleLeftPower = -wobbleRotatePower;
                    double wobbleRightPower = wobbleRotatePower;

                    wobbleLeftPower += driveSpeed;
                    wobbleRightPower += driveSpeed;


                    driveTrain.setSpeedPositiveForward(wobbleLeftPower, wobbleRightPower);

                } else if (elapsedTime.seconds() < 8) {

                    driveTrain.stop();

                } else if (elapsedTime.seconds() < 9) {

                    telemetry.addData("Wobble Arm Location", wobbleSystem.wobbleArm.getCurrentPosition());
                    telemetry.addData("Wobble Arm Speed", wobbleSystem.wobbleArm.get());
                    telemetry.addData("Wobble Hand Pos", wobbleSystem.wobbleHand.getPosition());
                    telemetry.update();
                    wobbleSystem.arm_down();

                } else if (elapsedTime.seconds() < 10.5) {

                    wobbleSystem.hand_open();

                } else if (elapsedTime.seconds() < 12) {

                    wobbleSystem.hand_close();

                } else if (elapsedTime.seconds() < 13.5) {

                    wobbleSystem.arm_up();

                } else if (elapsedTime.seconds() < 15) {

                    double wobbleRotatePower = rotationController.rotate(22);

                    double wobbleLeftPower = -wobbleRotatePower;
                    double wobbleRightPower = wobbleRotatePower;

                    wobbleLeftPower += driveSpeed;
                    wobbleRightPower += driveSpeed;


                    driveTrain.setSpeedPositiveForward(wobbleLeftPower, wobbleRightPower);

                } else if (elapsedTime.seconds() < 16.5) {

                    wobbleSystem.arm_down();

                } else if (elapsedTime.seconds() < 18) {

                    wobbleSystem.hand_close();

                }

                elapsedTime.reset();

                //Park
                if (elapsedTime.seconds() < 1.7) {
                    double parkRotatePower = rotationController.rotate(0);

                    double parkLeftPower = -parkRotatePower;
                    double parkRightPower = parkRotatePower;

                    parkLeftPower += driveSpeed;
                    parkRightPower += driveSpeed;


                    driveTrain.setSpeedPositiveForward(parkLeftPower, parkRightPower);

                }

                break;
        }
    }
}