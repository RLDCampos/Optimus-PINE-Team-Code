package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@Autonomous(name="Red Alliance B Place and Park", group="Autonomus")
//@Disabled

public class RedAllianceBPlaceAndPark extends LinearOpMode {

    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;
    DcMotor ySliderMotor;
    Servo clawServo;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    enum StateMachine {
        WAITING_FOR_START,
        CLOSE_CLAW,
        DRIVE_TO_TARGET_1,
        SLIDER_UP,
        DRIVE_TO_TARGET_2,
        SLIDER_DOWN,
        OPEN_CLAW,
        DRIVE_TO_TARGET_3,
        DRIVE_TO_TARGET_4,
        DRIVE_TO_TARGET_5,
        AT_TARGET
    }

    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM,-800,0,AngleUnit.DEGREES,0);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, -50, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM,150,0, AngleUnit.DEGREES,0);
    static final Pose2D TARGET_4 = new Pose2D(DistanceUnit.MM, 100, -2600, AngleUnit.DEGREES, -180);
    static final Pose2D TARGET_5 = new Pose2D(DistanceUnit.MM, 100, 0, AngleUnit.DEGREES, 0);



    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back");
        ySliderMotor = hardwareMap.get(DcMotor.class, "y_slider_motor");
        clawServo = hardwareMap.get(Servo.class, "Claw");

        // Motor configurations
        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-90.0, -300.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
        //nav.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
        //nav.setYawCoefficients(1,0,0.0, AngleUnit.DEGREES,2);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update(); // update odometry position

            switch (stateMachine){
                case WAITING_FOR_START:
                    //Drive to subversive
                    stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                    break;

                case CLOSE_CLAW:
                    clawServo.setPosition(0.1); // Close the claw slightly
                    telemetry.addLine("Claw closed slightly to hold specimen.");
                    telemetry.update();
                    sleep(500);
                    stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                    break;

                case DRIVE_TO_TARGET_1:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */
                    if (nav.driveTo(odo.getPosition(), TARGET_1, 0.7, 0)){
                        telemetry.addLine("at position #1!");
                        telemetry.update();
                        stateMachine = StateMachine.SLIDER_UP;
                    }
                    break;
                case SLIDER_UP:
                    moveSlider(150, 0.5, 2000); // Move the slider up by 150 mm
                    telemetry.addLine("Slider moved up 150mm.");
                    telemetry.update();
                    stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                    break;

                case DRIVE_TO_TARGET_2:
                    //drive to the second target
                    if (nav.driveTo(odo.getPosition(), TARGET_2, 0.7, 1)){
                        telemetry.addLine("at position #2!");
                        stateMachine = StateMachine.SLIDER_DOWN;
                    }
                    break;

                case SLIDER_DOWN:
                    moveSlider(-50, 0.5, 2000); // Move the slider down by 50 mm
                    telemetry.addLine("Slider moved down 50mm.");
                    telemetry.update();
                    stateMachine = StateMachine.OPEN_CLAW;
                    break;

                case OPEN_CLAW:
                    clawServo.setPosition(0.8); // Open the claw to release the specimen
                    telemetry.addLine("Claw opened to release specimen.");
                    telemetry.update();
                    sleep(500);
                    stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                    break;

                case DRIVE_TO_TARGET_3:
                    if(nav.driveTo(odo.getPosition(), TARGET_3, 0.7, 3)){
                        telemetry.addLine("at position #3");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;
                    }
                    break;
                case DRIVE_TO_TARGET_4:
                    if(nav.driveTo(odo.getPosition(),TARGET_4,0.7,1)){
                        telemetry.addLine("at position #4");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_5;
                    }
                    break;
                case DRIVE_TO_TARGET_5:
                    if(nav.driveTo(odo.getPosition(),TARGET_5,0.7,1)){
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.AT_TARGET;
                    }
                    break;

                case AT_TARGET:
                    telemetry.addLine("All tasks completed!");
                    telemetry.update();
                    break;
            }

            //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
            leftFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            rightFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            leftBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            rightBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            Pose2D pos = odo.getPosition();
            telemetry.addData("Position", String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                    pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES)));
            telemetry.addData("State", stateMachine);
            telemetry.update();
        }
    }

    private void moveSlider(int distance, double power, long timeoutMs) {
        ySliderMotor.setTargetPosition(distance);
        ySliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ySliderMotor.setPower(power);

        long startTime = System.currentTimeMillis();
        while (ySliderMotor.isBusy() && opModeIsActive() && (System.currentTimeMillis() - startTime < timeoutMs)) {
            telemetry.addData("Slider", "Moving to position: %d", distance);
            telemetry.update();
        }
        ySliderMotor.setPower(0);
        ySliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}