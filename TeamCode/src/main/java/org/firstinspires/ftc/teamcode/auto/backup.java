//
//package org.firstinspires.ftc.teamcode.auto;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.hardware.bosch.BHI260IMU;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//@Config
//@Autonomous
//public class jonathanAutoBase extends LinearOpMode {
//
//
//
//
//
//
//    // PID constants for turning
//    public  static double turnKp = 0.1;
//    public  static double turnKi = 0.01;
//    public  static double turnKd = 0.05;
////PID constants for forward/back
//    public  static double Kp_dist = 0.1;
//    public  static double Ki_dist = 0.01;
//    public  static double Kd_dist = 0.05;
////PID constants for heading
//    public  static double Kp_heading = 0.1;
//    public  static double Ki_heading = 0.01;
//    public  static double Kd_heading = 0.05;
////PID constants for strafing
//    public  static double Kp_strafe = 0.1;
//    public  static double Ki_strafe = 0.01;
//    public  static double Kd_strafe = 0.05;
//// PID Constants for Forward/back error.
//    public  static double Kp_fb = 0.1;
//    public  static double Ki_fb = 0.01;
//    public  static double Kd_fb = 0.05;
////Hardware Encoder Scale.
//    public  static double encoder_tick_scale = 0 ; // tune this!
//
//
//
//
//    //Motor Vars
//     public DcMotorEx leftFront = null;
//     public DcMotorEx leftBack = null;
//     public DcMotorEx rightFront = null;
//     public DcMotorEx rightBack = null;
//
//     public BHI260IMU imu = null;
//
//
//    //Vars
//    private double turn_previousError = 0; 		//dont touch (ㆆ_ㆆ)
//    private double turn_integral = 0; 				//dont touch (ㆆ_ㆆ)
//
//    private double previousErrorStrafe = 0;		//dont touch (ㆆ_ㆆ)
//    private double integralStrafe = 0;				//dont touch (ㆆ_ㆆ)
//
//    private double previousErrorFB = 0;				//dont touch (ㆆ_ㆆ)
//    private double integralFB = 0;						//dont touch (ㆆ_ㆆ)
//
//    private double previousErrorDist = 0; 		//dont touch (ㆆ_ㆆ)
//    private double integralDist = 0; 					//dont touch (ㆆ_ㆆ)
//
//    private double previousErrorHeading = 0; 	//dont touch (ㆆ_ㆆ)
//    private double integralHeading = 0; 			//dont touch (ㆆ_ㆆ)
//
//    public void AutonInit(HardwareMap hardwareMap){
//        //Add Motors and Encoders here.
//        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
//        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
//        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
//        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
//
//
//        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//
//        imu = hardwareMap.get(BHI260IMU.class,"imu");
//        imu.resetYaw();
//
//        //Assumed that your encoders are plugged into specific motor ports
//        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }
//
//
//    //motor control functions ----- ( ๑‾̀◡‾́)σ"
//    public void setLeftMotorPower(double power) {
//        leftFront.setPower(power);
//        leftBack.setPower(power);
//    }
//
//    public void setRightMotorPower(double power) {
//        rightFront.setPower(power);
//        rightBack.setPower(power);
//    }
//
//    public void setMotorPower(double rightFrontPower, double rightBackPower, double leftFrontPower, double leftBackPower) {
//
//        leftFront.setPower(leftFrontPower);
//        leftBack.setPower(leftBackPower);
//        rightFront.setPower(rightFrontPower);
//        rightBack.setPower(rightBackPower);
//
//
//    }
//
//
//    //functions start here ----- (っ＾▿＾)۶🍸🌟🍺٩(˘◡˘ )
//    public double getHeading(){
//       return  imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//    }
//
//    private double previousError, integral;
//
//    public void robot_turn(double degrees, double maxPower) {
//
//        double targetHeading = getHeading() + degrees;
//        double error = targetHeading - getHeading();
//
//
//        while (Math.abs(error) > 0.5) { // tolerance in degrees, adjust as needed
//            error = targetHeading - getHeading();
//            double derivative = error - turn_previousError;
//            turn_integral += error;
//
//            double output = (turnKp * error) + (turnKi * turn_integral) + (turnKd * derivative);
//
//            // clamp output to the maxPower
//            output = Math.min(Math.max(output, -maxPower), maxPower);
//
//            // cet motor powers
//            setLeftMotorPower(-output);
//            setRightMotorPower(output);
//
//            previousError = error;
//        }
//        // Stop motors after turning
//        setLeftMotorPower(0);
//        setRightMotorPower(0);
//
//        //reset variables
//
//        previousError = 0;
//        integral = 0;
//    }
//
//
//    public void robot_move(double distance, double maxPower) {
//        double targetDistanceTicks = distance * encoder_tick_scale;
//        double averageEncoder = (leftFront.getCurrentPosition() + rightFront.getCurrentPosition()) / 2.0;
//
//        double startHeading = getHeading();
//        double errorDist, errorHeading;
//
//        while (Math.abs(targetDistanceTicks - averageEncoder) > 10) { // Tolerance in ticks, adjust as needed
//            averageEncoder = (leftFront.getCurrentPosition() + rightFront.getCurrentPosition()) / 2.0;
//            errorDist = targetDistanceTicks - averageEncoder;
//
//            // Distance PID
//            double derivativeDist = errorDist - previousErrorDist;
//            integralDist += errorDist;
//            double outputDist = (Kp_dist * errorDist) + (Ki_dist * integralDist) + (Kd_dist * derivativeDist);
//
//            // Heading PID
//            errorHeading = startHeading - getHeading();
//            double derivativeHeading = errorHeading - previousErrorHeading;
//            integralHeading += errorHeading;
//            double outputHeading = (Kp_heading * errorHeading) + (Ki_heading * integralHeading) + (Kd_heading * derivativeHeading);
//
//            // Combine distance and heading outputs and clamp to maxPower
//            double leftPower = Math.min(Math.max(outputDist - outputHeading, -maxPower), maxPower);
//            double rightPower = Math.min(Math.max(outputDist + outputHeading, -maxPower), maxPower);
//
//            setLeftMotorPower(leftPower);
//            setRightMotorPower(rightPower);
//
//            previousErrorDist = errorDist;
//            previousErrorHeading = errorHeading;
//        }
//        // Stop motors
//        setLeftMotorPower(0);
//        setRightMotorPower(0);
//
//        //reset variables
//        previousErrorDist = 0;
//        integralDist = 0;
//        previousErrorHeading = 0;
//        integralHeading = 0;
//    }
//
//    public void robot_strafe(double distance, double maxPower) {
//        double targetDistanceTicks = distance * encoder_tick_scale;
//        double middleEncoderPosition = leftBack.getCurrentPosition();
//
//        double startHeading = getHeading();
//        double errorStrafe, errorHeading, errorFB;
//
//        while (Math.abs(targetDistanceTicks - middleEncoderPosition) > 10) { // Tolerance in ticks, adjust as needed
//            middleEncoderPosition = leftBack.getCurrentPosition();
//            errorStrafe = targetDistanceTicks - middleEncoderPosition;
//
//            // Strafe PID
//            double derivativeStrafe = errorStrafe - previousErrorStrafe;
//            integralStrafe += errorStrafe;
//            double outputStrafe = (Kp_strafe * errorStrafe) + (Ki_strafe * integralStrafe) + (Kd_strafe * derivativeStrafe);
//
//            // Heading PID
//            errorHeading = startHeading - getHeading();
//            double derivativeHeading = errorHeading - previousErrorHeading;
//            integralHeading += errorHeading;
//            double outputHeading = (Kp_heading * errorHeading) + (Ki_heading * integralHeading) + (Kd_heading * derivativeHeading);
//
//            // Forward/Backward Error PID
//            double averageEncoderFB = (leftFront.getCurrentPosition() + rightFront.getCurrentPosition()) / 2.0;
//            errorFB = averageEncoderFB - startHeading; // Assuming startHeading represents the initial average position
//            double derivativeFB = errorFB - previousErrorFB;
//            integralFB += errorFB;
//            double outputFB = (Kp_fb * errorFB) + (Ki_fb * integralFB) + (Kd_fb * derivativeFB);
//
//            // Combining outputs
//            double rightPower = Math.min(Math.max(outputStrafe - outputHeading - outputFB, -maxPower), maxPower);
//            double leftPower = Math.min(Math.max(outputStrafe + outputHeading - outputFB, -maxPower), maxPower);
//
//            // Set motor powers for strafing
//            setMotorPower(rightPower, rightPower, -leftPower, -leftPower);
//
//            previousErrorStrafe = errorStrafe;
//            previousErrorHeading = errorHeading;
//            previousErrorFB = errorFB;
//        }
//        // Stop motors
//        setMotorPower(0, 0, 0, 0);
//
//        //reset variables
//        previousErrorStrafe = 0;
//        integralStrafe = 0;
//        previousErrorHeading = 0;
//        integralHeading = 0;
//        previousErrorFB = 0;
//        integralFB = 0;
//    }
//
//    public void robot_drive_point(double forwardDistance, double strafeDistance, double turnAngle, double maxPower) {
//        double targetForwardTicks = forwardDistance * encoder_tick_scale;
//        double targetStrafeTicks = strafeDistance * encoder_tick_scale;
//        double targetHeading = getHeading() + turnAngle;
//
//        double forwardEncoderStart = (leftFront.getCurrentPosition() + rightFront.getCurrentPosition()) / 2.0;
//        double strafeEncoderStart = leftBack.getCurrentPosition();
//
//        double errorDist, errorStrafe;
//                double errorTurn= 0;
//        double currentStrafeEncoder=0;
//        double currentForwardEncoder=0;
//
//        while ((Math.abs(targetStrafeTicks - currentStrafeEncoder) > 10) || (Math.abs(targetForwardTicks - currentForwardEncoder) > 10) || (Math.abs(errorTurn) > 0.5)) { // Tolerance in ticks and degrees, adjust as needed
//            // Forward/Backward Movement
//            currentForwardEncoder = (leftFront.getCurrentPosition() + rightFront.getCurrentPosition()) / 2.0;
//            errorDist = targetForwardTicks - (currentForwardEncoder - forwardEncoderStart);
//            double derivativeDist = errorDist - previousErrorDist;
//            integralDist += errorDist;
//            double outputForward = (Kp_dist * errorDist) + (Ki_dist * integralDist) + (Kd_dist * derivativeDist);
//
//            // Strafing Movement
//            currentStrafeEncoder = leftBack.getCurrentPosition();
//            errorStrafe = targetStrafeTicks - (currentStrafeEncoder - strafeEncoderStart);
//            double derivativeStrafe = errorStrafe - previousErrorStrafe;
//            integralStrafe += errorStrafe;
//            double outputStrafe = (Kp_strafe * errorStrafe) + (Ki_strafe * integralStrafe) + (Kd_strafe * derivativeStrafe);
//
//            // Turning Movement
//            errorTurn = targetHeading - getHeading();
//            double derivativeTurn = errorTurn - turn_previousError;
//            turn_integral += errorTurn;
//            double outputTurn = (turnKp * errorTurn) + (turnKi * turn_integral) + (turnKd * derivativeTurn);
//
//            // Combine outputs and apply to motors
//            double leftFrontPower = Math.min(Math.max(outputForward + outputStrafe + outputTurn, -maxPower), maxPower);
//            double rightFrontPower = Math.min(Math.max(outputForward - outputStrafe - outputTurn, -maxPower), maxPower);
//            double leftRearPower = Math.min(Math.max(outputForward - outputStrafe + outputTurn, -maxPower), maxPower);
//            double rightRearPower = Math.min(Math.max(outputForward + outputStrafe - outputTurn, -maxPower), maxPower);
//
//            setMotorPower(leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
//
//            // Update previous errors
//            previousErrorDist = errorDist;
//            previousErrorStrafe = errorStrafe;
//            turn_previousError = errorTurn;
//        }
//
//        // Stop motors
//        setMotorPower(0, 0, 0, 0);
//
//        //reset variables
//        turn_previousError = 0;
//        turn_integral = 0;
//        previousErrorStrafe = 0;
//        integralStrafe = 0;
//        previousErrorDist = 0;
//        integralDist = 0;
//    }
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//    }
//}
