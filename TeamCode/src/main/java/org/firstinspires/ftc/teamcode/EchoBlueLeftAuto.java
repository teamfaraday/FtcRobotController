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
import org.firstinspires.ftc.robot_utilities.VisionController;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "EchoBlueLeftAuto", group = "Echo Autos")
public class EchoBlueLeftAuto extends OpMode {

    ElapsedTime elapsedTime;
    VisionController visionController;
    RotationController rotationController;
    PositionController positionController;

    private FlyWheel flywheel;
    private WobbleSystem wobbleSystem;
    private Hitter hitter;
    private Intake intake;

    boolean notStarted = true;
    State state = State.DRIVE_TO_SHOOT;
    int ringStackSize = -1;
    int tick = 0;


    public void init() {
        elapsedTime = new ElapsedTime();

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
                    case DRIVE_TO_SHOOT:
                        if(positionController.goStraight(2000, maxDriveSpeed)) {
                            state = State.SHOOT;
                            positionController.reset();
                        }
                        break;
                    case TURN_TO_SHOOT:
                        if(positionController.rotateInPlace(10)) {
                            state = state.SHOOT;
                            positionController.reset();
                        }
                        break;
                    case SHOOT:
                        if(tick < 3) {
                            flywheel.on();
                            if(flywheel.isReady()) {
                                hitter.hitFullMotion(0.7);
                                tick++;
                            }
                        } else {
                            flywheel.off();
                            state = state.TURN_TO_DELIVER;
                        }
                        break;
                    case TURN_TO_DELIVER:
                        if(positionController.rotateInPlace(60)) {
                            state = state.GO_TO_DELIVER;
                            positionController.reset();
                        }
                        break;
                    case GO_TO_DELIVER:
                        if(positionController.goStraight(900, maxDriveSpeed)) {
                            state = state.DELIVER_HAND_DOWN;
                            positionController.reset();
                            elapsedTime.reset();
                        }
                        break;
                    case DELIVER_HAND_DOWN:
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
                            state = state.PARK;
                        }
                        break;
                    case PARK:
                        if(positionController.goStraight(-1000, maxDriveSpeed)) {
                            state = state.DONE;
                            positionController.reset();
                        }
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
    }

    enum State {
        DRIVE_TO_SHOOT,
        TURN_TO_SHOOT,
        SHOOT,
        TURN_TO_DELIVER,
        GO_TO_DELIVER,
        DELIVER_HAND_DOWN,
        DELIVER_RELEASE,
        PARK,
        DONE
    }
}

