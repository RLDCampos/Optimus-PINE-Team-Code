package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Red AllianceB Place and Park", group = "Autonomous")
public class RedAllianceBPlaceAndPark extends LinearOpMode {

    // Motors and servo
    DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, ySliderMotor;
    Servo clawServo;

    // Feedforward constants (example values, tune as needed)
    public static final double kV = 0.017; // Velocity constant
    public static final double kA = 0.002; // Acceleration constant
    public static final double kStatic = 0.1; // Static constant

    // PID coefficients for error correction (example values, tune as needed)
    public static final double kP = 0.02, kI = 0.0, kD = 0.01;

    // Maximum power and distances
    public static final double MAX_POWER = 0.7;

    // Odometry and navigation
    GoBildaPinpointDriver odo;

    // Target poses
    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM, -560, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, -670, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM, -100, 650, AngleUnit.DEGREES, 90);

    // State machine for managing autonomous sequence
    enum StateMachine {
        WAITING_FOR_START,
        CLOSE_CLAW,
        DRIVE_TO_TARGET_1,
        SLIDER_UP,
        DRIVE_TO_TARGET_2,
        SLIDER_DOWN_AND_OPEN_CLAW,
        DRIVE_TO_TARGET_3,
        AT_TARGET
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware initialization
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
        ySliderMotor = hardwareMap.get(DcMotor.class, "y_slider_motor");
        clawServo = hardwareMap.get(Servo.class, "Claw");

        // Configure motor directions and braking
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ySliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Odometry initialization
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-90.0, 300.0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for start
        waitForStart();

        // State machine logic
        StateMachine stateMachine = StateMachine.WAITING_FOR_START;

        while (opModeIsActive()) {
            odo.update(); // Update odometry data

            switch (stateMachine) {
                case WAITING_FOR_START:
                    odo.resetPosAndIMU(); // Reset odometry and IMU
                    stateMachine = StateMachine.CLOSE_CLAW;
                    break;

                case CLOSE_CLAW:
                    clawServo.setPosition(0.5); // Close claw
                    sleep(500);
                    stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                    break;

                case DRIVE_TO_TARGET_1:
                    if (moveToTarget(TARGET_1)) {
                        stateMachine = StateMachine.SLIDER_UP;
                    }
                    break;

                case SLIDER_UP:
                    moveSlider(130); // Move slider up
                    stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                    break;

                case DRIVE_TO_TARGET_2:
                    if (moveToTarget(TARGET_2)) {
                        stateMachine = StateMachine.SLIDER_DOWN_AND_OPEN_CLAW;
                    }
                    break;

                case SLIDER_DOWN_AND_OPEN_CLAW:
                    moveSliderAndOpenClaw(-30);
                    stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                    break;

                case DRIVE_TO_TARGET_3:
                    if (moveToTarget(TARGET_3)) {
                        stateMachine = StateMachine.AT_TARGET;
                    }
                    break;

                case AT_TARGET:
                    telemetry.addLine("Autonomous Complete!");
                    telemetry.update();
                    break;
            }

            // Update telemetry with state and position
            Pose2D currentPose = odo.getPosition();
            telemetry.addData("State", stateMachine);
            telemetry.addData("Position", "{X: %.2f, Y: %.2f, H: %.2f}",
                    currentPose.getX(DistanceUnit.MM),
                    currentPose.getY(DistanceUnit.MM),
                    currentPose.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }

    private boolean moveToTarget(Pose2D targetPose) {
        Pose2D currentPose = odo.getPosition();

        // Calculate positional error
        double xError = targetPose.getX(DistanceUnit.MM) - currentPose.getX(DistanceUnit.MM);
        double yError = targetPose.getY(DistanceUnit.MM) - currentPose.getY(DistanceUnit.MM);
        double headingError = targetPose.getHeading(AngleUnit.DEGREES) - currentPose.getHeading(AngleUnit.DEGREES);

        // Calculate distance to target
        double distanceRemaining = Math.hypot(xError, yError);

        // PID control for error correction
        double xCorrection = kP * xError;
        double yCorrection = kP * yError;
        double headingCorrection = kP * headingError;

        // Feedforward control
        double velocity = distanceRemaining * kV;
        double acceleration = kA * velocity;

        double power = velocity + acceleration + kStatic;
        power = Math.min(power, MAX_POWER); // Limit power to max

        // Combine corrections and power
        double leftFrontPower = power + xCorrection - headingCorrection;
        double rightFrontPower = power - xCorrection - headingCorrection;
        double leftBackPower = power + yCorrection + headingCorrection;
        double rightBackPower = power - yCorrection + headingCorrection;

        // Set motor powers
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        // Stop condition
        return distanceRemaining < 10; // Stop when close to target
    }

    private void moveSlider(int mm) {
        int ticks = mm * 10; // Conversion
        ySliderMotor.setTargetPosition(ySliderMotor.getCurrentPosition() + ticks);
        ySliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ySliderMotor.setPower(0.5);
        while (ySliderMotor.isBusy() && opModeIsActive());
        ySliderMotor.setPower(0);
    }

    private void moveSliderAndOpenClaw(int mm) {
        int ticks = mm * 10;
        ySliderMotor.setTargetPosition(ySliderMotor.getCurrentPosition() + ticks);
        ySliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ySliderMotor.setPower(0.5);

        while (ySliderMotor.isBusy() && opModeIsActive()) {
            double progress = (ySliderMotor.getCurrentPosition() / (double) ticks);
            clawServo.setPosition(0.5 - (0.5 * progress));
        }
        ySliderMotor.setPower(0);
        clawServo.setPosition(0);
    }
}






