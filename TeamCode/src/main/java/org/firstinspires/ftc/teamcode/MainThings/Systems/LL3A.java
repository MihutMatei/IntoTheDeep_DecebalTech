package org.firstinspires.ftc.teamcode.MainThings.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MainThings.utils.GlobalVars;
import org.firstinspires.ftc.teamcode.MainThings.utils.SampleMecanumDrive;
import java.util.List;


@Config
public class LL3A {
    // Constants for thresholds and scaling factors
    public static double TX_POS_THRESHOLD = 1.0;
    public static double TX_NEG_THRESHOLD = -7.8;
    public static double CHASIS_POWER = 0.15;
    public static double TARGET_Y_DEGREES_THRESHOLD = -7.5;
    public static double Y_DEGREES_OFFSET = 20.0;
    public static double Y_DEGREES_DIVISOR = 14.0;
    public static double coef = 0;

    public Limelight3A limelight;
    // Instance variables (no longer static) to store results
    public List<LLResultTypes.ColorResult> colorResults;
    public List<List<Double>> targetCorners;

    public double maxXDelta;
    public double maxYDelta;
    public double computedVal;
    public double chasisPow;
    public Intake intake;
    public Extendo extendo;
    public Outtake outtake;

    public LL3A(HardwareMap hardwareMap, int pipelineIndex) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipelineIndex);
        // 0 red - pipelineIndex
        // 1 blue
        limelight.start();
        intake = new Intake(hardwareMap);
        extendo = new Extendo(hardwareMap);
        outtake = new Outtake(hardwareMap);
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public boolean alignLimeLight(SampleMecanumDrive drive) {
        int idx = 0;
        ElapsedTime timer = new ElapsedTime();

        while(idx < 350) {
            LLResult result = limelight.getLatestResult();
            if (result == null || !result.isValid()) {
                idx++;
                continue;
            }
            if (result.getTx() > -1) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                0,
                                0.2,
                                0
                        )
                );
            } else if (result.getTx() < -4) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                0,
                                -0.2,
                                0
                        )
                );
            }
            else
            {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                0,
                                0,
                                0
                        )
                );
                break;
            }

            sleep(5);
            idx++;

        }
        return true;
    }

    public boolean takeSample() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return false;
        }

        colorResults = result.getColorResults();

        for (LLResultTypes.ColorResult cr : colorResults) {
            List<List<Double>> targetCorners = cr.getTargetCorners();
            if (targetCorners != null && !targetCorners.isEmpty()) {
                // Initialize variables to track the maximum differences
                double maxXDelta = 0;
                double maxYDelta = 0;

                // Iterate over all pairs of corners to find the maximum differences
                for (int i = 0; i < targetCorners.size(); i++) {
                    for (int j = i + 1; j < targetCorners.size(); j++) {
                        List<Double> corner1 = targetCorners.get(i);
                        List<Double> corner2 = targetCorners.get(j);

                        // Calculate the differences in x and y coordinates
                        double deltaX = Math.abs(corner2.get(0) - corner1.get(0));
                        double deltaY = Math.abs(corner2.get(1) - corner1.get(1));
                        // Update the maximum differences

                        maxXDelta = Math.max(maxXDelta, deltaX);
                        maxYDelta = Math.max(maxYDelta, deltaY);

                    }
                }

                if (cr.getTargetYDegrees() > -6) // daca e prea departe targetu poate trb schimbat daca nu vr sa ia
                    continue;

                double val = (cr.getTargetYDegrees() + 20) / 14; // cea mai mica valoarea sa zicem ca ii - 20
                // trb adaugat + 20 ca sa ajungem la 0.
                // apoi lua cea mai mare valoare care sa zicem ca ii gen -6
                // -6 + 20 = 14 si impartim la 14


                // -8 // -15
                // safety check sa nu futem robotu
                if (val < 0)
                    continue;

                // horizontal => x = 2 * y
                // vertical => y = 1.5 * y
                double ratio = maxXDelta / maxYDelta;
                boolean is_diagonal = Math.abs(1 - ratio) < 0.2;

                if (maxXDelta > maxYDelta) {
                    intake.moveAngular(GlobalVars.angularState.HORIZONTAL);
                } else {
                    intake.moveAngular(GlobalVars.angularState.VERTICAL);
                }

                extendo.moveExtendoToPos(Math.max(0.4, 0.58 - val * 0.15)); // calculu magic explicat sus
                intake.goCoaxialToPos(GlobalVars.coaxialIntakePos);

                if (is_diagonal)
                    intake.clawIntakeSetPos(GlobalVars.intakeClawMoreOpenPos);
                else
                    intake.clawIntakeSetPos(GlobalVars.intakeClawOpenPos);

                intake.goIntakeToPos(GlobalVars.intakeInterPos);
                sleep(600);
                intake.goIntakeToPos(GlobalVars.intakeDownPos);
                sleep(250);
                intake.clawIntakeSetPos(GlobalVars.intakeClawClosePos);
                outtake.moveArm(GlobalVars.armTransferPos, GlobalVars.extendoOuttakeRetractPos);
                outtake.clawOuttake(GlobalVars.ClawState.OPEN);
                sleep(500);
                intake.goCoaxialToPos(GlobalVars.coaxialSlideForTransferPos);

                intake.moveAngular(GlobalVars.angularState.VERTICAL);
                intake.goIntakeToPos(GlobalVars.intakeUpPos);
                outtake.clawOuttake(GlobalVars.ClawState.OPEN);
                outtake.moveArm(GlobalVars.armTransferPos, GlobalVars.extendoOuttakeRetractPos);

                sleep(500);

                extendo.moveExtendo(GlobalVars.extendoState.RETRACT);
                intake.clawIntake(GlobalVars.ClawState.MORE_CLOSED);
                intake.goCoaxialToPos(GlobalVars.coaxialTransferPos);

                sleep(700);

                outtake.clawOuttake(GlobalVars.ClawState.CLOSE);
                sleep(600);
                intake.clawIntake(GlobalVars.ClawState.OPEN);
                sleep(300);
                outtake.moveArm(GlobalVars.armOuttakePos, GlobalVars.extendoOuttakeRetractPos);
                sleep(500);
                outtake.clawOuttake(GlobalVars.ClawState.OPEN);
            }
        }
        return true;
    }
}
