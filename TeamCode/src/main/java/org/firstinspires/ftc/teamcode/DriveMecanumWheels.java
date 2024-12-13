package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Drive Mechanum Wheels", group="Linear OpMode")
//@Disabled
public class DriveMecanumWheels extends LinearOpMode {


    // Declare OpMode members for each of the 5 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor ysliderMotor = null;
    private DcMotor xsliderMotor = null;
    private Servo Claw = null;
    private Servo Bucket = null;
    private Servo Intake = null;
    private Servo Intake_Rotate = null;
    private Servo Roof = null;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive= hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrive= hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
        ysliderMotor = hardwareMap.get(DcMotor.class, "y_slider_motor");
        xsliderMotor = hardwareMap.get(DcMotor.class, "x_slider_motor");
        Claw = hardwareMap.get(Servo.class, "Claw");
        Bucket = hardwareMap.get(Servo.class, "Bucket");
        Intake = hardwareMap.get(Servo.class, "Intake");
        Intake_Rotate = hardwareMap.get(Servo.class, "Intake_Rotate");
        Roof = hardwareMap.get(Servo.class, "Roof");

        // ########################################################################################
        // !!!IMPORTANT Drive Information. Test your motor directions.!!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);

        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        xsliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ysliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Claw.scaleRange(0.2,0.8);

        //Intake.scaleRange(0.6,0.9);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // creates a number for the variable speed
            double varspeed = 0.8;

            // POV Mode uses left joystick to go forward & rotate,   and right joystick to strafe.
            // Ritchie Note: yaw and lateral input methods have been switched as this is how Wincent prefers to drive
            double axial = -gamepad1.left_stick_y*varspeed;// Note: pushing stick forward gives negative value
            double yaw = gamepad1.left_stick_x*varspeed;
            double lateral= gamepad1.right_stick_x*varspeed;




            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower= axial - lateral + yaw;
            double rightBackPower= axial + lateral - yaw;

            //if you press the right bumper the arm goes up
            double xsliderMotorPower = gamepad2.left_stick_y;

            //if you press the left n right triggers the claws close
            double ClawServoPower = -(gamepad1.left_trigger*0.5) + 0.5;

            double IntakeServoPower;
            if (gamepad2.a) IntakeServoPower = 0.3;
            else IntakeServoPower = 0.6;
            if (gamepad2.b) IntakeServoPower = 0.85;

            double IRotatePower = (gamepad2.right_trigger*0.5) + -(gamepad2.left_trigger*0.5) + 0.5;

            double BucketServoPower = -gamepad1.right_trigger + 0.7;

            double RoofServoPower;
            if (gamepad1.right_bumper || gamepad1.left_bumper || gamepad1.y) RoofServoPower = 0.4;
            else RoofServoPower = 0;

            //extends slider
            double ysliderMotorPower;
            if (gamepad1.right_bumper) ysliderMotorPower = -1;
            else ysliderMotorPower = 0;
            if (gamepad1.left_bumper) ysliderMotorPower = 1;

            //Hanging mode
            if (gamepad2.y) {
                xsliderMotorPower = 0.1;
                IntakeServoPower = 0.8;
                RoofServoPower = 0.8;
            }


            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));



            if (max > 1.0) {
                leftFrontPower/= max;
                rightFrontPower /= max;
                leftBackPower/= max;
                rightBackPower/= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //1) First get all the motors to take to correct positions on the robot
            //by adjusting your Robot Configuration if necessary.
            //2) Then make sure they run in the correct direction by modifying the
            //the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

    /*
    leftFrontPower= gamepad1.x ? 1.0 : 0.0;// X gamepad
    leftBackPower= gamepad1.a ? 1.0 : 0.0;// A gamepad
    rightFrontPower = gamepad1.y ? 1.0 : 0.0;// Y gamepad
    rightBackPower= gamepad1.b ? 1.0 : 0.0;// B gamepad
    */

            // Send calculated power to wheels and arm
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            xsliderMotor.setPower(xsliderMotorPower);
            ysliderMotor.setPower(ysliderMotorPower);
            Claw.setPosition(ClawServoPower);
            Bucket.setPosition(BucketServoPower);
            Intake_Rotate.setPosition(IRotatePower);
            Intake.setPosition(IntakeServoPower);
            Roof.setPosition(RoofServoPower);

        }
    }}