package org.firstinspires.ftc.teamcode.MainThings.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MainThings.Systems.Extendo;
import org.firstinspires.ftc.teamcode.MainThings.Systems.Intake;
import org.firstinspires.ftc.teamcode.MainThings.Systems.LL3A;
import org.firstinspires.ftc.teamcode.MainThings.Systems.Outtake;
import org.firstinspires.ftc.teamcode.MainThings.Systems.Sliders;
import org.firstinspires.ftc.teamcode.MainThings.utils.DriveConstants;
import org.firstinspires.ftc.teamcode.MainThings.utils.GlobalVars;
import org.firstinspires.ftc.teamcode.MainThings.utils.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.random.trajectorysequence.TrajectorySequence;

@Autonomous(name="CosRosu", group="AUTONOMOUSGOOD")
@Config



public class CosRosu extends LinearOpMode {
    public static double ok = 1;
    public static double coef = 0.125;
    Pose2d lastPosSub = new Pose2d(0,0,0);
    //de aici copiezi in alte clase

    public boolean trajecotoryHasEnded(TrajectorySequence trajectory){
        return  (drive.getPoseEstimate().getX() <= trajectory.end().getX() + 5 && drive.getPoseEstimate().getX() >= trajectory.end().getX() - 5) ||
                (drive.getPoseEstimate().getY() <= trajectory.end().getY() + 5 && drive.getPoseEstimate().getY() >= trajectory.end().getY() - 5);
    }
    //pana aici

    private SampleMecanumDrive drive;
    private double loopTime=0,loop;
    boolean robotEndedMove=false;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        Extendo extendo = new Extendo(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Sliders sliders = new Sliders(hardwareMap);
        LL3A limelight = new LL3A(hardwareMap,0);

        telemetry.setMsTransmissionInterval(100);
        sleep(1000);

        Pose2d startPose = new Pose2d(35.5, 60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        //-----------trajectory-----------
        TrajectorySequence transferasuwu = drive.trajectorySequenceBuilder(startPose)

                .addTemporalMarker(0.3, ()->{
                    sliders.moveSliders(GlobalVars.sliderState.HIGH);

                    outtake.moveArm(GlobalVars.armOuttakePos,GlobalVars.extendoOuttakeExtendedPos);

                })
                .lineToLinearHeading(new Pose2d(53, 44, Math.toRadians(72)))
                .build();

        TrajectorySequence drumulet = drive.trajectorySequenceBuilder(transferasuwu.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(58, 44.5, Math.toRadians(77)), Math.toRadians(-93))
                .addTemporalMarker(0.01, ()->{
                    outtake.moveArm(GlobalVars.armTransferPos,GlobalVars.extendoOuttakeRetractPos);
                    intake.goIntakeToPos(GlobalVars.intakeUpPos);
                })
                .addTemporalMarker(0.08, ()->{
                    sliders.moveSliders(GlobalVars.sliderState.DOWN);
                    extendo.moveExtendo(GlobalVars.extendoState.RETRACT);
                    intake.goCoaxialToPos(GlobalVars.coaxialTransferPos);

                })
                .addTemporalMarker(0.2, ()->{
                    intake.clawIntake(GlobalVars.ClawState.CLOSE);
                })
                .build();

        TrajectorySequence transferasuwu1 = drive.trajectorySequenceBuilder(drumulet.end())
                .lineToLinearHeading(new Pose2d(56.5, 42.5, Math.toRadians(90)))
                .addTemporalMarker(10, ()->{
                    extendo.moveExtendo(GlobalVars.extendoState.EXTEND_AUTO);
                    intake.goIntakeToPos(GlobalVars.intakeInterPos);
                    intake.goCoaxialToPos(GlobalVars.coaxialIntakePos);
                })
                .build();


        TrajectorySequence bipbip = drive.trajectorySequenceBuilder(transferasuwu1.end())
                .lineToLinearHeading(new Pose2d(46, 36.5, Math.toRadians(142)))
                .addTemporalMarker(0.01, ()->{
                    outtake.moveArm(GlobalVars.armOuttakePos,GlobalVars.extendoOuttakeExtendedPos);
                    sliders.moveSliders(GlobalVars.sliderState.DOWN);
                    outtake.clawOuttake(GlobalVars.ClawState.OPEN);
                    sliders.moveSliders(GlobalVars.sliderState.DOWN);
                    extendo.moveExtendo(GlobalVars.extendoState.EXTEND);
                    intake.goCoaxialToPos(GlobalVars.coaxialIntakePos);
                    intake.goIntakeToPos(GlobalVars.intakeInterPos);
                    intake.pivot();
                })
                .build();

        TrajectorySequence transfer3 = drive.trajectorySequenceBuilder(bipbip.end())
                .addTemporalMarker(0.1, ()->{sliders.moveSliders(GlobalVars.sliderState.HIGH);
                    outtake.moveArm(GlobalVars.armOuttakePos,GlobalVars.extendoOuttakeExtendedPos)
                    ;})
                .lineToLinearHeading(new Pose2d(46, 49.5, Math.toRadians(36)))
                .waitSeconds(0.3)
                .addTemporalMarker(10, ()->{
                    outtake.clawOuttake(GlobalVars.ClawState.OPEN);
                })
                .build();

        TrajectorySequence aliniere_submersible = drive.trajectorySequenceBuilder(transfer3.end())
                .setReversed(true)
                .addTemporalMarker(0.3, ()->{
                    sliders.moveSliders(GlobalVars.sliderState.DOWN);
                    outtake.moveArm(GlobalVars.armOuttakePos,GlobalVars.extendoOuttakeExtendedPos);
                    outtake.clawOuttake(GlobalVars.ClawState.CLOSE);
                })
                .splineToLinearHeading(new Pose2d(20,4,Math.toRadians(0)),Math.toRadians(180))
                .build();

        TrajectorySequence after_sub = drive.trajectorySequenceBuilder(aliniere_submersible.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(40,2,Math.toRadians(0)),Math.toRadians(180))
                .build();

        TrajectorySequence score = drive.trajectorySequenceBuilder(aliniere_submersible.end())
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(34, 4, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(44,50,Math.toRadians(40)),Math.toRadians(90))
                .addTemporalMarker(1.6, ()->{sliders.moveSliders(GlobalVars.sliderState.HIGH);
                                       outtake.moveArm(GlobalVars.armOuttakePos,GlobalVars.extendoExtendPos);

                })
                .build();

        TrajectorySequence parcare = drive.trajectorySequenceBuilder(score.end())
                .setReversed(true)
                .addTemporalMarker(0.3, ()->{
                    sliders.moveSliders(GlobalVars.sliderState.PARCARE);
                                       outtake.moveArm(GlobalVars.armOuttakePos,GlobalVars.extendoOuttakeExtendedPos);

                    outtake.clawOuttake(GlobalVars.ClawState.CLOSE);
                })
                .splineToSplineHeading(new Pose2d(37,1,Math.toRadians(180)),Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(20, 1 , Math.toRadians(180))
                        ,SampleMecanumDrive.getVelocityConstraint(100,100, DriveConstants.TRACK_WIDTH)
                        ,SampleMecanumDrive.getAccelerationConstraint(100))
                .build();

        TrajectorySequence parcare2 = drive.trajectorySequenceBuilder(aliniere_submersible.end())
                .setReversed(true)
                .addTemporalMarker(0.3, ()->{
                    sliders.moveSliders(GlobalVars.sliderState.PARCARE);
                     outtake.moveArm(GlobalVars.armOuttakePos,GlobalVars.extendoOuttakeExtendedPos);

                    outtake.clawOuttake(GlobalVars.ClawState.CLOSE);
                })
//                .splineToSplineHeading(new Pose2d(37,1,Math.toRadians(180)),Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(45, 1 , Math.toRadians(180)))

                .lineToLinearHeading(new Pose2d(18, 1 , Math.toRadians(180)))

                .build();



        while (!isStarted() && !isStopRequested()) {
            outtake.clawOuttake((GlobalVars.ClawState.CLOSE));
            intake.clawIntake(GlobalVars.ClawState.OPEN_AUTO);
            intake.goIntakeToPos(GlobalVars.intakeUpPos);
            intake.goCoaxialToPos(GlobalVars.coaxialTransferPos);
            intake.moveAngular(GlobalVars.angularState.VERTICAL);
            extendo.moveExtendo(GlobalVars.extendoState.RETRACT);
            outtake.moveArm(GlobalVars.armTransferPos+0.115,GlobalVars.extendoOuttakeRetractPos);


            loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.addData("sldier: ", sliders.sliderRight.getCurrentPosition());
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }

        drive.followTrajectorySequence(transferasuwu);
        sleep(200);
        outtake.clawOuttake(GlobalVars.ClawState.OPEN);
        extendo.moveExtendo(GlobalVars.extendoState.EXTEND);
        intake.goIntakeToPos(GlobalVars.intakeInterPos);
        intake.goCoaxialToPos(GlobalVars.coaxialIntakePos);
        sleep(400);
        intake.goIntakeToPos(GlobalVars.intakeDownPos);
        sleep(600);
        intake.clawIntake(GlobalVars.ClawState.CLOSE);
        sleep(100);
        drive.followTrajectorySequence(drumulet);
        sleep(100);

        outtake.clawOuttake(GlobalVars.ClawState.CLOSE);
        sleep(100);
        intake.clawIntake(GlobalVars.ClawState.OPEN_AUTO);
        sleep(100);
        sliders.moveSliders(GlobalVars.sliderState.HIGH);
        outtake.moveArm(GlobalVars.armOuttakePos,GlobalVars.extendoOuttakeExtendedPos);

        sleep(400);
        drive.followTrajectorySequence(transferasuwu1);
        sleep(400);
        outtake.clawOuttake(GlobalVars.ClawState.OPEN);
        intake.goIntakeToPos(GlobalVars.intakeDownPos);
        sleep(500);
        intake.clawIntake(GlobalVars.ClawState.CLOSE);
        sleep(100);
        outtake.moveArm(GlobalVars.armTransferPos,GlobalVars.extendoOuttakeRetractPos);
        sleep(100);

        intake.goIntakeToPos(GlobalVars.intakeUpPos);
        intake.goCoaxialToPos(GlobalVars.coaxialTransferPos);
        extendo.moveExtendo(GlobalVars.extendoState.RETRACT);
        sliders.moveSliders(GlobalVars.sliderState.DOWN);
        outtake.clawOuttake(GlobalVars.ClawState.OPEN);

        sleep(900);
        outtake.clawOuttake(GlobalVars.ClawState.CLOSE);
        sleep(200);
        intake.clawIntake(GlobalVars.ClawState.OPEN_AUTO);
        sleep(200);
        sliders.moveSliders(GlobalVars.sliderState.HIGH);
                           outtake.moveArm(GlobalVars.armOuttakePos,GlobalVars.extendoOuttakeExtendedPos);

        sleep(1100);
        outtake.clawOuttake(GlobalVars.ClawState.OPEN);
        sleep(100);
        drive.followTrajectorySequence(bipbip);
        intake.goIntakeToPos(GlobalVars.intakeDownPos);
        sleep(200);
        intake.clawIntake(GlobalVars.ClawState.MORE_CLOSED);
        sleep(100);
        intake.goCoaxialToPos(GlobalVars.coaxialTransferPos);
        intake.goIntakeToPos(GlobalVars.intakeUpPos);
        intake.moveAngular(GlobalVars.angularState.VERTICAL);
        sleep(300);
        intake.clawIntake(GlobalVars.ClawState.CLOSE);
        extendo.moveExtendo(GlobalVars.extendoState.RETRACT);
        sleep(200);
        outtake.moveArm(GlobalVars.armTransferPos,GlobalVars.extendoOuttakeRetractPos);
        sleep(500);
        outtake.clawOuttake(GlobalVars.ClawState.CLOSE);
        sleep(300);
        intake.clawIntake(GlobalVars.ClawState.OPEN);
        drive.followTrajectorySequence(transfer3);
        sleep(200);
        drive.followTrajectorySequence(aliniere_submersible);
        sleep(150);

//        if(trajecotoryHasEnded(aliniere_submersible)) {
//            while (!robotEndedMove){
//                limelight.alignLimeLight(drive, trajecotoryHasEnded(aliniere_submersible));
//
//                if(limelight.chasisPow == 0) {
//                    robotEndedMove = true;
//                    break;
//                }
//                telemetry.update();
//            }
//        }
//        limelight.determineCornersLimelight(robotEndedMove);
//        sleep(200);
//        boolean detected=false;
//
//        if(limelight.computedVal != 0) {
//            int cnt=0;
//            while(cnt<3 && !detected) {
//                robotEndedMove=false;
//                while (!robotEndedMove) {
//                    limelight.alignLimeLight(drive, true);
//                    if (limelight.chasisPow == 0) {
//                        robotEndedMove = true;
//                        break;
//                    }
//                }
//                sleep(300);
//                limelight.determineCornersLimelight(true);
//                sleep(800);
//
//                if(cnt == 1) coef += 0.025;
//                if(cnt == 2) coef += 0.028;
//                extendo.moveExtendoToPos(Math.max(0.33, 0.48 - limelight.computedVal) + coef);
//
//                if (limelight.maxXDelta > limelight.maxYDelta) {
//                    intake.moveAngular(GlobalVars.angularState.HORIZONTAL);
//                } else {
//                    intake.moveAngular(GlobalVars.angularState.VERTICAL);
//                }
//                // calculu magic explicat sus
//                intake.goCoaxialToPos(GlobalVars.coaxialIntakePos);
//                intake.clawIntakeSetPos(GlobalVars.intakeClawOpenPosAuto);
//                intake.goIntakeToPos(GlobalVars.intakeInterPos);
//                sleep(700);
//                intake.goIntakeToPos(GlobalVars.intakeDownPos);
//                sleep(200);
//                intake.clawIntake(GlobalVars.ClawState.MORE_CLOSED);
//                sleep(200);
//                intake.goIntakeToPos(GlobalVars.intakeParallelPos);
//                intake.goCoaxialToPos(GlobalVars.coaxialDetectiePos);
//                intake.moveAngular(GlobalVars.angularState.VERTICAL);
//                sleep(200);
//                intake.clawIntake(GlobalVars.ClawState.CLOSE);
//                sleep(400);
//                if (intake.detectedSample()) {
//                    outtake.moveArm(GlobalVars.armOuttakePos,GlobalVars.extendoOuttakeRetractPos);
//                    outtake.clawOuttake(GlobalVars.ClawState.OPEN);
//                    sleep(300);
//                    intake.goCoaxialToPos(GlobalVars.coaxialSlideForTransferPos);
//                    sleep(100);
//                    intake.moveAngular(GlobalVars.angularState.VERTICAL);
//                    intake.goIntakeToPos(GlobalVars.intakeUpPos);
//                    outtake.clawOuttake(GlobalVars.ClawState.OPEN);
//                    outtake.moveArm(GlobalVars.armOuttakePos,GlobalVars.extendoOuttakeRetractPos);
//                    sleep(400);
//                    intake.clawIntake(GlobalVars.ClawState.MORE_CLOSED);
//                    intake.goCoaxialToPos(GlobalVars.coaxialTransferPos);
//                    sleep(200);
//                    extendo.moveExtendo(GlobalVars.extendoState.RETRACT);
//                    sleep(400);
//                    outtake.clawOuttake(GlobalVars.ClawState.CLOSE);
//                    sleep(300);
//                    intake.clawIntake(GlobalVars.ClawState.OPEN);
//                    sleep(300);
//                                       outtake.moveArm(GlobalVars.armOuttakePos,GlobalVars.extendoOuttakeRetractPos)
//
//                    detected=true;
//                }
//                else {
//                    extendo.moveExtendo(GlobalVars.extendoState.RETRACT);
//                    intake.goCoaxialToPos(GlobalVars.coaxialTransferPos);
//                    intake.goIntakeToPos(GlobalVars.intakeUpPos);
//                    cnt++;
//                    sleep(400);
//                }
//            }
//        }  if (detected){//TODO: if detected, du te scoreaza si opreste; else parcheaza te si opreste
//            lastPosSub=drive.getPoseEstimate();
//            sleep(200);
//            drive.followTrajectorySequence(score);
//            sleep(100);
//            outtake.clawOuttake(GlobalVars.ClawState.OPEN);
//            sleep(100);
//            drive.followTrajectorySequence(parcare);
//        }
//
//        else {
            drive.followTrajectorySequence(parcare2);
//        }
        sleep(1000000);

        if (!opModeIsActive()) return;

        telemetry.update();
    }
}
