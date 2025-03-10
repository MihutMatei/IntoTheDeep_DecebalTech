package org.firstinspires.ftc.teamcode.MainThings.DriveDT;

import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MainThings.Systems.Extendo;
import org.firstinspires.ftc.teamcode.MainThings.Systems.Intake;
import org.firstinspires.ftc.teamcode.MainThings.Systems.Outtake;
import org.firstinspires.ftc.teamcode.MainThings.Systems.Sliders;
import org.firstinspires.ftc.teamcode.MainThings.utils.GlobalVars;

import org.firstinspires.ftc.teamcode.MainThings.utils.SampleMecanumDrive;

@TeleOp(name="DriveDT", group = "Main")
@Config

public class Drive extends LinearOpMode {
    GlobalVars.Modedrive currentMode = GlobalVars.Modedrive.DRIVER_CONTROL;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    boolean buttonWasPressed = false; int cnt = 0;
    boolean buttonWasPressed_sliders = false; int cnt_sliders = 1;
    double incrementPivot;

    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Extendo extendo = new Extendo(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Sliders sliders = new Sliders(hardwareMap);

        initializePositions(intake, extendo, outtake, sliders);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            handleSubsystemSequence(intake, extendo, outtake, sliders);
            handleManualControls(intake, extendo, outtake);

            if (gamepad1.right_trigger >= 0.5) currentMode = GlobalVars.Modedrive.TURBO;
            else if (gamepad1.left_trigger >= 0.5) currentMode = GlobalVars.Modedrive.PRECISION;
            else currentMode = GlobalVars.Modedrive.DRIVER_CONTROL;

            double driveScale = (currentMode == GlobalVars.Modedrive.TURBO) ? GlobalVars.TURBO_SCALE
                    : (currentMode == GlobalVars.Modedrive.PRECISION) ? GlobalVars.PRECISION_SCALE : GlobalVars.DRIVE_SCALE;

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y / driveScale,
                            -gamepad1.left_stick_x / driveScale,
                            -gamepad1.right_stick_x / driveScale
                    ));
            telemetry.addData("SL", sliders.sliderLeft.getCurrentPosition());
            telemetry.addData("SR", sliders.sliderRight.getCurrentPosition());
            telemetry.addData("Sensor", outtake.sensor.getDistance(DistanceUnit.MM));
            telemetry.update();

        }
    }
    private void handleManualControls(Intake intake, Extendo extendo, Outtake outtake) {
        if(gamepad2.right_stick_button) cnt = 16;
        if(gamepad2.right_bumper) cnt = 11;
        if(gamepad2.left_bumper) {timer.reset();cnt = 13;}
        if(gamepad2.cross) cnt = 1;
        if(gamepad2.dpad_up) cnt_sliders=5;
        if(gamepad2.dpad_down) cnt_sliders=1;
        if(gamepad2.square) outtake.clawOuttake(GlobalVars.ClawState.OPEN);
        else if(gamepad2.circle) outtake.clawOuttake(GlobalVars.ClawState.CLOSE);
//        if(gamepad2.left_stick_x>0.3)extendo.extend(gamepad2.left_stick_x);
//        else if(gamepad2.left_stick_x<-0.3)extendo.retract(gamepad2.left_stick_x);
    }
    public void retractAndResetPositions(Sliders sliders,double ampsValue) {
        sliders.sliderRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        sliders.sliderLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        sliders.retract(1);
        if (sliders.sliderLeft.getCurrent(CurrentUnit.AMPS) > ampsValue) {
            sliders.retract(0);
            sliders.sliderLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            sliders.sliderRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            sliders.sliderLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            sliders.sliderRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            sliders.moveSlidersToPos(3, 1);
            cnt_sliders=6;//TODO: INLOCUIRE CU VARIABILE IN GlobalVars SI DETERMINARE DIN TELEMETRY A ampsValue(amperajul real tras de motor cand e fortat)
        }
    }
    private void handleSubsystemSequence(Intake intake, Extendo extendo, Outtake outtake, Sliders sliders) {
        if (gamepad2.triangle) {
            if (!buttonWasPressed && timer.milliseconds() > 200) {
                buttonWasPressed = true;
                timer.reset();
                cnt++;
            } else {
                buttonWasPressed = false;
            }
        }

        incrementPivot=intake.angular.getPosition();
        switch (cnt) {
            case 0:
                outtake.moveArm(GlobalVars.armTransferPos, GlobalVars.extendoOuttakeRetractPos);
                cnt_sliders=1;
                break;
            case 1: intake.goIntakeToPos(GlobalVars.intakeInterPos);
                    intake.clawIntake(GlobalVars.ClawState.OPEN);
                    //TODO: LA ORICE RETRAGERE DE EXTENDO FUNCTIE FOLOSIND VOLTAGE SENSOR PT RESETARE
                    extendo.moveExtendo(GlobalVars.extendoState.EXTEND);
                    intake.goCoaxialToPos(GlobalVars.coaxialIntakePos);
                    incrementPivot=intake.angular.getPosition();
                    if(gamepad2.dpad_right) incrementPivot+=0.01;
                    else if(gamepad2.dpad_left) incrementPivot-=0.01;
                    intake.angular.setPosition(incrementPivot);
                    break;
            case 2: intake.goIntakeToPos(GlobalVars.intakeDownPos);
                    incrementPivot=intake.angular.getPosition();
                    if(gamepad2.dpad_right) incrementPivot+=0.01;
                    if(gamepad2.dpad_left) incrementPivot-=0.01 ;
                    intake.angular.setPosition(incrementPivot);
                    break;
            case 3: intake.clawIntake(GlobalVars.ClawState.MORE_CLOSED);
                    if(gamepad2.touchpad) {timer.reset(); cnt=17;}
                    break;
            case 4: intake.goIntakeToPos(GlobalVars.intakeUpPos);
                    intake.goCoaxialToPos(GlobalVars.coaxialSlideForTransferPos);
                    intake.moveAngular(GlobalVars.angularState.VERTICAL);
                    outtake.moveArm(GlobalVars.armTransferPos, GlobalVars.extendoOuttakeRetractPos);
                    if(timer.milliseconds() >400)intake.clawIntake(GlobalVars.ClawState.CLOSE);
                    if(timer.milliseconds()>550) {cnt++; timer.reset();}
                    break;
            case 5:intake.clawIntake(GlobalVars.ClawState.MORE_CLOSED);
                    outtake.clawOuttake(GlobalVars.ClawState.OPEN);
                    extendo.moveExtendo(GlobalVars.extendoState.RETRACT);
                    intake.goCoaxialToPos(GlobalVars.coaxialTransferPos);
                    if(timer.milliseconds()>350) outtake.moveArm(GlobalVars.armTransferPos,GlobalVars.extendoOuttakeRetractPos);
                    if(outtake.claw_has_detected()) {
                        timer.reset();
                        cnt++;
                    }
                    break;
            case 6: outtake.clawOuttake(GlobalVars.ClawState.CLOSE);
                    if(timer.milliseconds()>100) cnt++;
                    break;
            case 7: intake.clawIntake(GlobalVars.ClawState.OPEN);
                    if(timer.milliseconds()>250) cnt++;
                    break;
            case 8: outtake.moveArm(GlobalVars.armOuttakePos,GlobalVars.extendoOuttakeExtendedPos);
                    break;
            case 9: outtake.clawOuttake(GlobalVars.ClawState.OPEN);
                    if(timer.milliseconds()>800) cnt_sliders=1;
                    break;
            case 10: cnt = 1;
                     break;
            case 11:outtake.moveArm(GlobalVars.armIntakePos,GlobalVars.extendoOuttakeRetractPos);
                    outtake.clawOuttake(GlobalVars.ClawState.OPEN);
                    cnt_sliders=1;
            case 12:
                if(outtake.claw_has_detected()) {
                    timer.reset();
                    cnt = 13;
                }
                     break;
            case 13:    outtake.clawOuttake(GlobalVars.ClawState.CLOSE);
                        if(timer.milliseconds()>300){outtake.moveArm(GlobalVars.armBarPos,GlobalVars.extendoOuttakeExtendedPos);cnt_sliders=2;}
                        if(timer.milliseconds()>2000){outtake.clawOuttake(GlobalVars.ClawState.MORE_CLOSED);}
                        break;
            case 14: cnt_sliders=5;
                     outtake.moveArm(GlobalVars.armBarUpperPos,GlobalVars.extendoOuttakeExtendedPos);
                     if(timer.milliseconds()>400) outtake.clawOuttake(GlobalVars.ClawState.OPEN);
                     break;
            case 15: cnt=0;
                     break;
            case 16: intake.goIntakeToPos(GlobalVars.intakeUpPos);
                     intake.clawIntake(GlobalVars.ClawState.OPEN);
                     outtake.clawOuttake(GlobalVars.ClawState.OPEN);
                     outtake.moveArm(GlobalVars.armTransferPos,GlobalVars.extendoOuttakeRetractPos);
                     intake.goCoaxialToPos(GlobalVars.coaxialTransferPos);
                     intake.moveAngular(GlobalVars.angularState.VERTICAL);
                     extendo.moveExtendo(GlobalVars.extendoState.RETRACT);
                     cnt = 0;
                     break;
            case 17: intake.coaxial.setPosition(GlobalVars.coaxialSlideForTransferPos);
                     extendo.moveExtendo(GlobalVars.extendoState.RETRACT);
                     intake.moveAngular(GlobalVars.angularState.VERTICAL);
                     if(timer.milliseconds()>500){intake.clawIntake(GlobalVars.ClawState.MORE_CLOSED); cnt++;}
                     break;
            case 18: intake.coaxial.setPosition(GlobalVars.coaxialTransferPos);
            case 19: cnt=4;
                     break;
        }

        if (gamepad2.right_trigger >= 0.5) {
            if (!buttonWasPressed_sliders && timer2.milliseconds() > 300 && (cnt_sliders != 3&&cnt_sliders != 5)) {
                buttonWasPressed_sliders = true;
                timer2.reset();
                cnt_sliders++;
            } else {
                buttonWasPressed_sliders = false;
            }
        } else if(gamepad2.left_trigger >= 0.5){
            if (!buttonWasPressed_sliders && timer2.milliseconds() > 300 && (cnt_sliders != 1 && cnt_sliders != 6 && cnt_sliders != 5)) {
                buttonWasPressed_sliders = true;
                timer2.reset();
                cnt_sliders--;
            } else {
                buttonWasPressed_sliders = false;
            }
        }

        if(gamepad1.square) cnt_sliders=4;

        switch(cnt_sliders){
            case 1: this.retractAndResetPositions(sliders,GlobalVars.motorAmps);
                    break;
            case 2: sliders.moveSliders(GlobalVars.sliderState.BASKET_LOW);
                    break;
            case 3: sliders.moveSliders(GlobalVars.sliderState.HIGH);
                    break;
            case 4: sliders.sliderRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    sliders.sliderLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    if(gamepad1.square) {
                        sliders.sliderLeft.setPower(-1);
                        sliders.sliderRight.setPower(-1);
                    }
                    else {
                        sliders.sliderLeft.setPower(1);
                        sliders.sliderRight.setPower(1);
                        sliders.sliderRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        sliders.sliderLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        sliders.sliderRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        sliders.sliderLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        cnt_sliders=1;
                    }
                    break;
            case 5: sliders.moveSliders(GlobalVars.sliderState.UP);
                    break;
            case 6: sliders.moveSliders(GlobalVars.sliderState.DOWN);
                    break;
            case 7: cnt_sliders=2;
                    break;
        }

    }


    private void initializePositions(Intake intake, Extendo extendo, Outtake outtake, Sliders sliders) {
        intake.goIntakeToPos(GlobalVars.intakeUpPos);
        intake.clawIntake(GlobalVars.ClawState.OPEN);
        outtake.clawOuttake(GlobalVars.ClawState.OPEN);
        outtake.moveArm(GlobalVars.armTransferPos,GlobalVars.extendoOuttakeRetractPos);
        intake.goCoaxialToPos(GlobalVars.coaxialTransferPos);
        intake.moveAngular(GlobalVars.angularState.VERTICAL);
        extendo.moveExtendo(GlobalVars.extendoState.RETRACT);
        sliders.moveSliders(GlobalVars.sliderState.DOWN);
    }
}



