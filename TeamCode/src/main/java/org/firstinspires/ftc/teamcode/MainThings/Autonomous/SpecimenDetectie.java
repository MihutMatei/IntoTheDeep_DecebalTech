//package org.firstinspires.ftc.teamcode.MainThings.Autonomous;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.MainThings.Systems.Extendo;
//import org.firstinspires.ftc.teamcode.MainThings.Systems.Intake;
//import org.firstinspires.ftc.teamcode.MainThings.Systems.Outtake;
//import org.firstinspires.ftc.teamcode.MainThings.Systems.Sliders;
//import org.firstinspires.ftc.teamcode.MainThings.utils.GlobalVars;
//import org.firstinspires.ftc.teamcode.MainThings.utils.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.random.trajectorysequence.TrajectorySequence;
//
//
//@Disabled
//@Autonomous(name="SpecimenDetectie'", group="AUTONOMOUSGOOD")
//@Config
//
//public class SpecimenDetectie extends LinearOpMode {
//
//
//    private SampleMecanumDrive drive;
//    private double loopTime=0,loop;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        drive = new SampleMecanumDrive(hardwareMap);
//        Extendo extendo = new Extendo(hardwareMap);
//        Intake intake = new Intake(hardwareMap);
//        Outtake outtake = new Outtake(hardwareMap);
//        Sliders sliders = new Sliders(hardwareMap);
//
//
//        sleep(1000);
//
//        Pose2d startPose = new Pose2d(9, -60, Math.toRadians(90));
//        drive.setPoseEstimate(startPose);
//
//        //-----------trajectory-----------
//        TrajectorySequence preload = drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(6 ,-25,Math.toRadians(90)))
//
//
//                .addTemporalMarker(10, ()->{
//                    outtake.clawOuttake(GlobalVars.ClawState.OPEN);
//                })
//                .build();
//
//
//        TrajectorySequence drum = drive.trajectorySequenceBuilder(preload.end())
//                .setReversed(true)
//
//                .splineToLinearHeading(new Pose2d(34.2,-45.2, Math.toRadians(-135)),Math.toRadians(90))
//
//                .addTemporalMarker(0.01, ()->{
//                    sliders.moveSliders(GlobalVars.sliderState.DOWN);
//                })
//
//                .addTemporalMarker(0.4, ()->{
//                    intake.goCoaxialToPos(GlobalVars.coaxialIntakePos);
//                    extendo.moveExtendo(GlobalVars.extendoState.EXTEND);
//                    intake.pivotauto();
//                })
//                .build();
//
//        TrajectorySequence wing = drive.trajectorySequenceBuilder(drum.end())
//                .lineToLinearHeading(new Pose2d(44 ,-48.5,Math.toRadians(-240)))
//                .addTemporalMarker(10, ()->{
//                    intake.clawIntake(GlobalVars.ClawState.OPEN_AUTO);
//                })
//                .build();
//
//        TrajectorySequence drum1 = drive.trajectorySequenceBuilder(wing.end())
//                .lineToLinearHeading(new Pose2d(43 ,-45.2,Math.toRadians(-136)))
//
//                .addTemporalMarker(0.01, ()->{
//                    extendo.moveExtendo(GlobalVars.extendoState.EXTEND);
//                    intake.pivotauto();
//                })
//
//                .build();
//
//        TrajectorySequence wing1 = drive.trajectorySequenceBuilder(drum1.end())
//                .lineToLinearHeading(new Pose2d(39.5 ,-48,Math.toRadians(-240)))
//                .addTemporalMarker(10, ()->{
//
//                    intake.clawIntake(GlobalVars.ClawState.OPEN_AUTO);
//                })
//                .build();
//
//        ///TO DO: NU MAI IEI A AL TREILEA SAMPLE SI INCEPI CICLURILE
//
////        TrajectorySequence drum2 = drive.trajectorySequenceBuilder(wing1.end())
////                .lineToLinearHeading(new Pose2d(51 ,-44,Math.toRadians(-135)))
////                .addTemporalMarker(0.01, ()->{
////
////                    extendo.moveExtendo(GlobalVars.extendoState.EXTEND);
////                    intake.pivotauto();
////
////
////
////                })
////
////                .build();
////
////        TrajectorySequence wing2 = drive.trajectorySequenceBuilder(drum1.end())
////                .lineToLinearHeading(new Pose2d(41 ,-45,Math.toRadians(-240)))
////                .addTemporalMarker(10, ()->{
////                    intake.clawIntake(GlobalVars.ClawState.OPEN);
////                })
////                .build();
//
//        TrajectorySequence prescore = drive.trajectorySequenceBuilder(wing1.end())
//                .lineToLinearHeading(new Pose2d(36.8,-60,Math.toRadians(-90)))
//                .addTemporalMarker(0.01, ()->{
//                    intake.clawIntake(GlobalVars.ClawState.MORE_CLOSED);
//                    extendo.moveExtendo(GlobalVars.extendoState.RETRACT);
//
//                    outtake.moveArm(GlobalVars.armIntakePos+0.03);
//                    outtake.clawOuttake(GlobalVars.ClawState.OPEN);
//
//                })
//                .addTemporalMarker(0.1, ()->{
//                    intake.goCoaxialToPos(GlobalVars.coaxialTransferPos);
//                    intake.goIntakeToPos(GlobalVars.intakeUpPos);
//                })
//                .lineToLinearHeading(new Pose2d(36.8,-63,Math.toRadians(-90)))
//
//                .addTemporalMarker(10, ()->{
//
//                    outtake.clawOuttake(GlobalVars.ClawState.CLOSE);
//
//                })
//
//                .build();
//
//        TrajectorySequence score = drive.trajectorySequenceBuilder(prescore.end())
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(1,-33, Math.toRadians(83)),Math.toRadians(90))
//                .lineToLinearHeading(new Pose2d(1,-22.5,Math.toRadians(83)))
//
//                .addTemporalMarker(0.01, ()->{
//                    outtake.moveArm(GlobalVars.armOuttakePos+0.02);
//                    sliders.moveSliders(GlobalVars.sliderState.UP);
//                })
//
//                .addTemporalMarker(10, ()->{
//                    outtake.clawOuttake(GlobalVars.ClawState.OPEN);
//                })
//                .build();
//
//        TrajectorySequence prescore1 = drive.trajectorySequenceBuilder(score.end())
//                .addTemporalMarker(0.2, ()->{
//                    sliders.moveSliders(GlobalVars.sliderState.DOWN);
//                })
//                .addTemporalMarker(0.3, ()->{
//                    outtake.moveArm(GlobalVars.armIntakePos+0.03);
//                })
//                .setReversed(true)
//                .splineToSplineHeading(new Pose2d(35,-52,Math.toRadians(-102)),Math.toRadians(-90))
//                .lineToLinearHeading(new Pose2d(35,-68,Math.toRadians(-102)))
//                .addTemporalMarker(10, ()->{
//                    outtake.clawOuttake(GlobalVars.ClawState.MORE_CLOSED);
//                })
//                .build();
//
//        TrajectorySequence score1 = drive.trajectorySequenceBuilder(prescore1.end())
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(7,-33, Math.toRadians(80)),Math.toRadians(90))
//                .lineToLinearHeading(new Pose2d(7,-22,Math.toRadians(80)))
//
//                .addTemporalMarker(0.01, ()->{
//                    outtake.moveArm(GlobalVars.armOuttakePos+0.01);
//                    sliders.moveSliders(GlobalVars.sliderState.UP);
//                })
//
//                .addTemporalMarker(10, ()->{
//                    outtake.clawOuttake(GlobalVars.ClawState.OPEN);
//                })
//                .build();
//
//        TrajectorySequence prescore2 = drive.trajectorySequenceBuilder(score1.end())
//
//                .addTemporalMarker(0.2, ()->{
//                    sliders.moveSliders(GlobalVars.sliderState.DOWN);
//                })
//                .addTemporalMarker(0.3, ()->{
//                    outtake.moveArm(GlobalVars.armIntakePos+0.02);
//
//                })
//                .setReversed(true)
//                .splineToSplineHeading(new Pose2d(31,-52,Math.toRadians(-112)),Math.toRadians(-90))
//                .lineToLinearHeading(new Pose2d(31,-68,Math.toRadians(-112)))
//
//
//                .addTemporalMarker(10, ()->{
//
//                    outtake.clawOuttake(GlobalVars.ClawState.CLOSE);
//
//                })
//                .build();
//
//        TrajectorySequence score2 = drive.trajectorySequenceBuilder(prescore2.end())
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(9,-33, Math.toRadians(86)),Math.toRadians(90))
//                .lineToLinearHeading(new Pose2d(9,-21,Math.toRadians(86)))
//
//                .addTemporalMarker(0.01, ()->{
//                    outtake.moveArm(GlobalVars.armOuttakePos+0.02);
//                    sliders.moveSliders(GlobalVars.sliderState.UP);
//
//                })
//
//                .addTemporalMarker(10, ()->{
//                    outtake.clawOuttake(GlobalVars.ClawState.OPEN);
//                })
//
//
//                .build();
////
////        TrajectorySequence parcare = drive.trajectorySequenceBuilder(score2.end())
////                .lineToLinearHeading(new Pose2d(28,-50,Math.toRadians(-220)))
////
////
////                .addTemporalMarker(0.02, ()->{
////                    sliders.moveSliders(GlobalVars.sliderState.DOWN);
////
////
////
////
////                })
////                .addTemporalMarker(0.3, ()->{
////                    outtake.moveArm(GlobalVars.armTransferPos);
////                    extendo.moveExtendo(GlobalVars.extendoState.EXTEND);
////                    intake.goIntakeToPos(GlobalVars.intakeDownPos);
////                    intake.goCoaxialToPos(GlobalVars.coaxialIntakePos);
////
////                })
////
////
////                .build();
//
//
//        while (!isStarted() && !isStopRequested()) {
//
//            outtake.clawOuttake((GlobalVars.ClawState.MORE_CLOSED));
//            intake.clawIntake(GlobalVars.ClawState.OPEN_AUTO);
//            intake.goIntakeToPos(GlobalVars.intakeUpPos);
//            intake.goCoaxialToPos(GlobalVars.coaxialTransferPos);
//            intake.moveAngular(GlobalVars.angularState.VERTICAL);
//            extendo.moveExtendo(GlobalVars.extendoState.RETRACT);
//
//
//            loop = System.nanoTime();
//            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
//            loopTime = loop;
//            telemetry.addData("sldier: ", sliders.sliderRight.getCurrentPosition());
//            telemetry.update();
//        }
//
//
//        outtake.moveArm(GlobalVars.armOuttakePos+0.02);
//        extendo.moveExtendo(GlobalVars.extendoState.RETRACT);
//        sliders.moveSliders(GlobalVars.sliderState.UP);
//        drive.followTrajectorySequence(preload);
//        sleep(50);
//        drive.followTrajectorySequence(drum);
//        sleep(100);
//        intake.clawIntake(GlobalVars.ClawState.OPEN_AUTO);
//        sleep(50);
//        intake.goIntakeToPos(GlobalVars.intakeDownPos);
//        sleep(100);
//        intake.clawIntake(GlobalVars.ClawState.CLOSE);
//        sleep(200);
//        intake.goIntakeToPos(GlobalVars.intakeUpPos);
//        sleep(50);
//        drive.followTrajectorySequence(wing);
//        sleep(50);
//
//        drive.followTrajectorySequence(drum1);
//        sleep(100);
//        intake.clawIntake(GlobalVars.ClawState.OPEN_AUTO);
//        sleep(50);
//        intake.goIntakeToPos(GlobalVars.intakeDownPos);
//        sleep(100);
//        intake.clawIntake(GlobalVars.ClawState.CLOSE);
//        sleep(200);
//        intake.goIntakeToPos(GlobalVars.intakeUpPos);
//        sleep(100);
//        drive.followTrajectorySequence(wing1);
//        sleep(50);
//
////        drive.followTrajectorySequence(drum2);
////        sleep(100);
////        intake.clawIntake(GlobalVars.ClawState.OPEN_AUTO);
////        sleep(50);
////        intake.goIntakeToPos(GlobalVars.intakeDownPos);
////        sleep(100);
////        intake.clawIntake(GlobalVars.ClawState.CLOSE);
////        sleep(200);
////        intake.goIntakeToPos(GlobalVars.intakeUpPos);
//
//
//        drive.followTrajectorySequence(prescore);
//        drive.followTrajectorySequence(score);
//        drive.followTrajectorySequence(prescore1);
//        drive.followTrajectorySequence(score1);
//        drive.followTrajectorySequence(prescore2);
//        drive.followTrajectorySequence(score2);
////        sleep(100);
////        drive.followTrajectorySequence(parcare);
//
//
//
//
//
//
//
//        sleep(1000000);
//        if (!opModeIsActive()) return;
//
//    }
//}
