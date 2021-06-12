package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robot.FlyWheel;
import org.firstinspires.ftc.robot.Hitter;
import org.firstinspires.ftc.robot.Intake;
import org.firstinspires.ftc.robot.WobbleSystem;
import org.firstinspires.ftc.robot_utilities.PositionController;
import org.firstinspires.ftc.robot_utilities.RotationController;
import org.firstinspires.ftc.robot_utilities.Vals;
import org.firstinspires.ftc.robot_utilities.VisionController;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "EchoBLWobbleAuto", group = "Echo Autos")
public class EchoBLWobbleAuto extends OpMode {

    ElapsedTime elapsedTime;
    VisionController visionController;
    RotationController rotationController;
    PositionController positionController;

    private FlyWheel flywheel;
    private WobbleSystem wobbleSystem;
    private Hitter hitter;
    private Intake intake;

    boolean notStarted = true;
    State state;
    int ringStackSize = -1;
    int tick = 0;


    public void init() {
        elapsedTime = new ElapsedTime();
        state = State.TURN_TO_DELIVER;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvInternalCamera phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        visionController = new VisionController(phoneCam);

        rotationController = new RotationController(hardwareMap.get(BNO055IMU.class, "imu"));
        positionController = new PositionController(rotationController,
                new Motor(hardwareMap, "dl"),
                new Motor(hardwareMap, "dr"));
        positionController.reset();

        flywheel = new FlyWheel(new Motor(hardwareMap, "fw", Motor.GoBILDA.BARE), telemetry);
        wobbleSystem = new WobbleSystem(new Motor(hardwareMap, "wobbleArmMotor"), hardwareMap.servo.get("wobbleArmServo"));
        wobbleSystem.hand_close();
        wobbleSystem.arm_up();
        hitter = new Hitter(hardwareMap.servo.get("sv"));
        intake = new Intake(new Motor(hardwareMap, "in1"),  new Motor(hardwareMap, "in2"));


    }

    @Override
    public void loop() {
        if(notStarted) {
            ringStackSize = visionController.getRingPosition();
            elapsedTime.reset();

            notStarted = false;
            return;
        }

        double maxDriveSpeed = 0.8;

        switch(ringStackSize) {
            case 0:
                switch(state) {
                    case TURN_TO_DELIVER:
                        if(positionController.rotateInPlace(20)) {
                            state = state.GO_TO_DELIVER;
                            positionController.stop();
                            positionController.reset();
                        }
                        break;
                    case GO_TO_DELIVER:
                        if(positionController.goStraight(1500, maxDriveSpeed)) {
                            state = state.DELIVER_ARM_DOWN;
                            positionController.stop();
                            positionController.reset();
                            elapsedTime.reset();
                        }
                        break;
                    case DELIVER_ARM_DOWN:
                        if(elapsedTime.seconds() < 2) {
                            wobbleSystem.arm_down();
                        } else {
                            state = state.DELIVER_RELEASE;
                            elapsedTime.reset();
                        }
                        break;
                    case DELIVER_RELEASE:
                        if(elapsedTime.seconds() < 1) {
                            wobbleSystem.hand_open();
                        } else {
                            state = state.MOVE_BACK;
                            elapsedTime.reset();
                        }
                        break;
                    case MOVE_BACK:
                        if(positionController.goStraight(-1000, maxDriveSpeed)) {
                            state = state.HAND_CLOSE;
                            positionController.stop();
                            positionController.reset();
                        }
                        break;
                    case HAND_CLOSE:
                        if(elapsedTime.seconds() < 1) {
                            wobbleSystem.hand_close();
                        } else {
                            state = state.ARM_UP;
                            elapsedTime.reset();
                        }
                        break;
                    case ARM_UP:
                        if(elapsedTime.seconds() < 2) {
                            wobbleSystem.arm_up();
                        } else {
                            state = state.PARK;
                            elapsedTime.reset();
                        }
                        break;
                    case PARK:
//                        if(positionController.goStraight(-1000, maxDriveSpeed)) {
//                            state = state.DONE;
//                            positionController.stop();
//                            positionController.reset();
//                        }
                        state = state.DONE;
                        break;
                    default:
                        requestOpModeStop();
                }
                break;
            case 1:
                break;
            case 4:
                break;
            default:
                stop();
        }

        telemetry.addData("WobbleArm Position", wobbleSystem.wobbleArm.getCurrentPosition());
        telemetry.addData("Flywheel Speed", flywheel.getFlywheelFilteredSpeed());
        telemetry.addData("pDrive Val", Vals.test_pDrive_val);
        telemetry.addData("pDrive At Set Point", Vals.test_pDrive_sp);
        telemetry.addData("Rotate At Set Point", Vals.test_rotation_sp);
    }

    enum State {
        DRIVE_TO_SHOOT,
        TURN_TO_SHOOT,
        SHOOT,
        TURN_TO_DELIVER,
        GO_TO_DELIVER,
        DELIVER_ARM_DOWN,
        DELIVER_RELEASE,
        MOVE_BACK,
        HAND_CLOSE,
        ARM_UP,
        PARK,
        DONE
    }
}

