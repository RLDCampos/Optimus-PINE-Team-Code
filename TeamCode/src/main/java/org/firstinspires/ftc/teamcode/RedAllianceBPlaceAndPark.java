package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@Autonomous(name = "Red AllianceB Place and Park", group = "Autonomous")
public class RedAllianceBPlaceAndPark extends LinearOpMode {

    DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, ySliderMotor;
    Servo clawServo;

    GoBildaPinpointDriver odo; // Odometry object
    DriveToPoint nav = new DriveToPoint(this); // Navigation object

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

    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM, -560, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, -670, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM, -100, 650, AngleUnit.DEGREES, 90);

    @Override
    public void runOpMode() {
        // Initialize motors and servo
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
        ySliderMotor = hardwareMap.get(DcMotor.class, "y_slider_motor");
        clawServo = hardwareMap.get(Servo.class, "Claw");

        // Set motor behaviors
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ySliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-90.0, 300.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine = StateMachine.WAITING_FOR_START;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for start signal
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update();

            switch (stateMachine) {
                case WAITING_FOR_START:
                    odo.resetPosAndIMU(); // Initial reset and recalibration
                    sleep(500); // Ensure stability
                    stateMachine = StateMachine.CLOSE_CLAW;
                    break;

                case CLOSE_CLAW:
                    clawServo.setPosition(0.5); // Close claw
                    sleep(500); // Allow claw to close
                    stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                    break;

                case DRIVE_TO_TARGET_1:
                    if (smoothDriveTo(TARGET_1, 0.5)) {
                        telemetry.addLine("At position #1");
                        stateMachine = StateMachine.SLIDER_UP;
                    }
                    break;

                case SLIDER_UP:
                    moveSlider(130); // Move slider up 130mm
                    stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                    break;

                case DRIVE_TO_TARGET_2:
                    if (smoothDriveTo(TARGET_2, 0.4)) {
                        telemetry.addLine("At position #2");
                        stateMachine = StateMachine.SLIDER_DOWN_AND_OPEN_CLAW;
                    }
                    break;

                case SLIDER_DOWN_AND_OPEN_CLAW:
                    moveSliderAndOpenClaw(-30); // Move slider down and open claw simultaneously
                    sleep(2000); // Wait for placement to finish
                    stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                    break;

                case DRIVE_TO_TARGET_3:
                    if (smoothDriveTo(TARGET_3, 0.5)) {
                        telemetry.addLine("At position #3");
                        stateMachine = StateMachine.AT_TARGET;
                    }
                    break;

                case AT_TARGET:
                    telemetry.addLine("Autonomous Complete!");
                    break;

                default:
                    telemetry.addLine("Unknown State");
            }

            // Telemetry
            Pose2D pos = odo.getPosition();
            telemetry.addData("State", stateMachine);
            telemetry.addData("Position", "{X: %.2f, Y: %.2f, H: %.2f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }

    private boolean smoothDriveTo(Pose2D target, double maxSpeed) {
        Pose2D currentPos = odo.getPosition();
        double distanceRemaining = Math.hypot(
                target.getX(DistanceUnit.MM) - currentPos.getX(DistanceUnit.MM),
                target.getY(DistanceUnit.MM) - currentPos.getY(DistanceUnit.MM)
        );

        double totalDistance = 1000; // Example total distance (update with actual logic)
        double motorPower = calculateMotorPower(maxSpeed, distanceRemaining, totalDistance);

        leftFrontDrive.setPower(motorPower);
        rightFrontDrive.setPower(motorPower);
        leftBackDrive.setPower(motorPower);
        rightBackDrive.setPower(motorPower);

        return distanceRemaining < 10; // Stop condition when near the target
    }

    private double calculateMotorPower(double maxPower, double distanceRemaining, double totalDistance) {
        if (distanceRemaining > totalDistance * 0.7) {
            // Acceleration phase
            return maxPower * ((totalDistance - distanceRemaining) / (totalDistance * 0.3));
        } else if (distanceRemaining < totalDistance * 0.3) {
            // Deceleration phase
            return maxPower * (distanceRemaining / (totalDistance * 0.3));
        } else {
            // Constant speed phase
            return maxPower;
        }
    }

    private void moveSlider(int mm) {
        int ticks = (int) (mm * 10); // Example conversion factor
        ySliderMotor.setTargetPosition(ySliderMotor.getCurrentPosition() + ticks);
        ySliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ySliderMotor.setPower(0.5); // Adjust power for smooth motion
        while (ySliderMotor.isBusy() && opModeIsActive()) {
            // Wait for the motor to reach the position
        }
        ySliderMotor.setPower(0);
    }

    private void moveSliderAndOpenClaw(int mm) {
        int ticks = (int) (mm * 10); // Example conversion factor
        ySliderMotor.setTargetPosition(ySliderMotor.getCurrentPosition() + ticks);
        ySliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ySliderMotor.setPower(0.5); // Adjust power for smooth motion
        clawServo.setPosition(0); // Open claw while moving slider down
        while (ySliderMotor.isBusy() && opModeIsActive()) {
            // Wait for slider to move down
        }
        ySliderMotor.setPower(0);
    }
}
