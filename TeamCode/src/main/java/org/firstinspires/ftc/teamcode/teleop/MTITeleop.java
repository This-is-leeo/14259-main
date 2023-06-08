package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.C;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.drivetrainimpl.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.input.Async;
import org.firstinspires.ftc.teamcode.input.AsyncThreaded;
import org.firstinspires.ftc.teamcode.input.Controller;
import org.firstinspires.ftc.teamcode.input.controllerimpl.GamepadController;
import org.firstinspires.ftc.teamcode.output.goBildaTouchDriver;
import org.firstinspires.ftc.teamcode.output.motorimpl.DcMotorExMotor;
import org.firstinspires.ftc.teamcode.output.motorimpl.ServoMotor;
import org.firstinspires.ftc.teamcode.pid.Pid;
import org.firstinspires.ftc.teamcode.utils.M;

@TeleOp
    public class MTITeleop extends LinearOpMode {
        //
    private Pid linSlidePid;
    private Drivetrain drivetrain;
    private DcMotorExMotor turret;
    private DcMotorExMotor pitch;
    private DcMotorExMotor leftLinSlide;
    private DcMotorExMotor rightLinSlide;
    private ServoMotor claw;
    private ServoMotor frontArm;
    private ServoMotor deposit;
    private ServoMotor latch;
    private ServoMotor leftArm;
    private ServoMotor rightArm;
    private Controller controller1;
    private Controller controller2;

    //Variable
    private int linSlidePosition = 0;
    private int depositPosition = 0;
    private int clawPosition = 0;
    private int latchPosition = 0;
    private int frontArmPosition = 1;
    private int armPosition = 2;
    private int pitchPosition;
    private double[] linSlidePositions = {0,0.45,1};
    private double pitchReset = 0;
    private double turretReset = 0;
    private double pitchLastPosition = 0;
    private double turretLastPosition = 0;

    private goBildaTouchDriver pitchTouchSensor;
    private final double[] clawPositions = { 0.0, 1.0};
    private final double[] frontArmPositions = {0,1,0.8};
    //Calculation variable!
    private double targetPitchPosition;
    private double targetTurretPosition;
    private double targetLinSlidePosition = this.linSlidePositions[linSlidePosition];
    private double targetFrontArmPosition = C.frontArmPositions[frontArmPosition];
    private double targetDepositPosition = C.depositPositions[depositPosition];
    private double targetArmPosition = 0.8;
    private double lastTurretPosition = 0.5;

    private boolean clawOpen = true;
    private boolean latchEngaged = false;
    private double targetPitchPower = 0;
    private double targetLinSlidePower = 0;
    private double targetTurretPower = 0;


    //Tests
    private boolean linSlideHigh = true;
    private boolean turretRTP = false;
    private boolean pitchRTP = false;
    private boolean linSlideRTP = true;
    private boolean armOut = false;
    private boolean pitchTS;
    private boolean turretSensorTouched;
    private boolean turretMode = false;
    private int scorePos = 1;

    private int intakeStep = 1;
    private int depositStep = 1;
    private int cycleStep = 1;

    private void initDrivetrain() {
        this.drivetrain = new MecanumDrivetrain(hardwareMap);
    }
    private void initMotor() {
        this.pitch = new DcMotorExMotor(hardwareMap.get(DcMotorEx.class, "pitch"))
                .setLowerBound(C.pitchLB)
                .setUpperBound(C.pitchUB)
                .setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        this.turret = new DcMotorExMotor(hardwareMap.get(DcMotorEx.class, "turret"))
                .setLowerBound(C.turretLB)
                .setUpperBound(C.turretUB)
                .setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }
    private void initPID() {
    }
    private void initAll() {
        this.initMotor();
        this.initDrivetrain();
        this.initPID();
    }
    private void updateServo() {
        this.deposit.update();
        this.claw.update();
        this.latch.update();
        this.rightArm.update();
        this.leftArm.update();
        this.frontArm.update();
    }
    private void updateMotor() {
        //this.leftLinSlide.update();
        //this.rightLinSlide.update();
        //this.pitch.update();
        this.turret.update();
    }
    private void updateDrivetrain() {
        this.drivetrain.addPowerX(gamepad1.left_stick_x);
        this.drivetrain.addPowerY(gamepad1.left_stick_y);
        this.drivetrain.addPowerR(-gamepad1.right_stick_x * 0.8);
        this.drivetrain.update();
    }
    private void updateTelemetry() {
//        telemetry.addData("this.leftLinSlide.getPower()", this.leftLinSlide.getPower());
//        telemetry.addData("linSlidePos", this.linSlidePositions[this.linSlidePosition]);
//        telemetry.addData("Veer is an absolute monkey V1 XD", this.leftLinSlide.getCurrentPosition());
//        telemetry.addData("Veer is an absolute monkey V1 XD", linSlideRTP);
//        telemetry.addData("Pitch Position", this.pitch.getCurrentPosition());
//        telemetry.addData("LinSlide target Position", this.targetLinSlidePosition);
//        telemetry.addData("ArmOut", this.armOut);
        telemetry.addData("pitch TS", this.pitchTS);
        telemetry.addData("Turret Mode", this.turretMode);
        telemetry.addData("latchEngaged" , this.latchEngaged);
//        telemetry.addData("arm Position", this.armPosition);
//        telemetry.addData("pitchRTP", this.pitchRTP);
//        telemetry.addData("pitchTargetPosition", this.targetPitchPosition);
//        telemetry.addData("limit", this.turretSensor.isPressed());
        telemetry.update();
    }
    private void updateVariable() {
        targetLinSlidePosition = M.clamp(this.linSlidePositions[this.linSlidePosition] + this.linSlidePositions[this.linSlidePosition]*(0.4 -this.targetPitchPosition) ,0,1);
        targetPitchPower = gamepad1.right_stick_y;
        targetTurretPower = (-gamepad1.left_trigger + gamepad1.right_trigger - 0.35*gamepad2.left_trigger + 0.35*gamepad2. right_trigger);
        targetLinSlidePower = 0.7*gamepad2.right_stick_y;
    }
    private void updateSensor() {
        pitchTS = this.pitchTouchSensor.check();
    }


    private void updateAll() { //order: DT, Var, sensor, Control, position, motor. servo, telemetry.
        //this.updateVariable();
        //this.updateSensor();
        this.updateMotor();
        //this.updateServo();
        this.updateTelemetry();
    }
    private void interact(){
        if(Math.abs(targetTurretPower) > 0.05){
            this.turretRTP = false;
            this.turret.setPower(targetTurretPower);
        }
        if(Math.abs(targetPitchPower) > 0.05){
            this.pitchRTP = false;
            if(targetPitchPower > 0 && this.pitch.getCurrentPosition() < 0.3) targetPitchPower = 0;
            if(targetPitchPower < 0 && this.pitch.getCurrentPosition() > 1.1) targetPitchPower = 0;
            this.pitch.setPower(targetPitchPower);
        }
        if(Math.abs(targetLinSlidePower) > 0.1){
            this.linSlideRTP = false;
            this.leftLinSlide.setPower(targetLinSlidePower);
            this.rightLinSlide.setPower(targetLinSlidePower);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        while(opModeInInit()){
            this.initAll();
            waitForStart();
        }
        waitForStart();
        while (opModeIsActive()) {
            this.interact();
            //this.updateAll();
        }
        AsyncThreaded.stopped = false;
    }

}