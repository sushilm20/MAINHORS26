package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.List;

/**
 * Holds the calibration data and regression model for adaptive flywheel RPM.
 *
 * Usage (in your TeleOp):
 * <pre>
 *   ShooterCalibration cal = new ShooterCalibration();
 *   // calibration points are already populated from the @Configurable statics
 *   // just build the regression:
 *   cal.computeRegression();
 * </pre>
 *
 * All pose / RPM fields are {@code @Configurable} so they can be tuned
 * live in the Panels dashboard without redeploying.
 */
@Configurable
public class ShooterCalibration {

    // ──────────────────────────────────────────────────────────
    //  Blue goal target (field coordinates)
    // ──────────────────────────────────────────────────────────
    @Sorter(sort = 0)  public static double GOAL_X = 12.0;
    @Sorter(sort = 1)  public static double GOAL_Y = 135.0;

    // ──────────────────────────────────────────────────────────
    //  RPM clamp bounds
    // ──────────────────────────────────────────────────────────
    @Sorter(sort = 2)  public static double MIN_RPM = 2300.0;
    @Sorter(sort = 3)  public static double MAX_RPM = 4000.0;

    // ──────────────────────────────────────────────────────────
    //  Calibration point 1
    // ──────────────────────────────────────────────────────────
    @Sorter(sort = 4)  public static double CAL1_X   = 48.0;
    @Sorter(sort = 5)  public static double CAL1_Y   = 96.0;
    @Sorter(sort = 6)  public static double CAL1_HDG = 135.0;
    @Sorter(sort = 7)  public static double CAL1_RPM = 2300.0;

    // ──────────────────────────────────────────────────────────
    //  Calibration point 2
    // ──────────────────────────────────────────────────────────
    @Sorter(sort = 8)  public static double CAL2_X   = 60.0;
    @Sorter(sort = 9)  public static double CAL2_Y   = 125.0;
    @Sorter(sort = 10) public static double CAL2_HDG = 135.0;
    @Sorter(sort = 11) public static double CAL2_RPM = 2400.0;

    // ──────────────────────────────────────────────────────────
    //  Calibration point 3
    // ──────────────────────────────────────────────────────────
    @Sorter(sort = 12) public static double CAL3_X   = 60.0;
    @Sorter(sort = 13) public static double CAL3_Y   = 82.0;
    @Sorter(sort = 14) public static double CAL3_HDG = 135.0;
    @Sorter(sort = 15) public static double CAL3_RPM = 2500.0;

    // ──────────────────────────────────────────────────────────
    //  Calibration point 4
    // ──────────────────────────────────────────────────────────
    @Sorter(sort = 16) public static double CAL4_X   = 72.0;
    @Sorter(sort = 17) public static double CAL4_Y   = 72.0;
    @Sorter(sort = 18) public static double CAL4_HDG = 135.0;
    @Sorter(sort = 19) public static double CAL4_RPM = 2650.0;

    // ──────────────────────────────────────────────────────────
    //  Calibration point 5
    // ──────────────────────────────────────────────────────────
    @Sorter(sort = 20) public static double CAL5_X   = 72.0;
    @Sorter(sort = 21) public static double CAL5_Y   = 120.0;
    @Sorter(sort = 22) public static double CAL5_HDG = 167.0;
    @Sorter(sort = 23) public static double CAL5_RPM = 2550.0;

    // ──────────────────────────────────────────────────────────
    //  Calibration point 6
    // ──────────────────────────────────────────────────────────
    @Sorter(sort = 24) public static double CAL6_X   = 95.0;
    @Sorter(sort = 25) public static double CAL6_Y   = 120.0;
    @Sorter(sort = 26) public static double CAL6_HDG = 135.0;
    @Sorter(sort = 27) public static double CAL6_RPM = 2850.0;

    // ──────────────────────────────────────────────────────────
    //  Calibration point 7
    // ──────────────────────────────────────────────────────────
    @Sorter(sort = 28) public static double CAL7_X   = 52.0;
    @Sorter(sort = 29) public static double CAL7_Y   = 14.0;
    @Sorter(sort = 30) public static double CAL7_HDG = 135.0;
    @Sorter(sort = 31) public static double CAL7_RPM = 3750.0;

    // ──────────────────────────────────────────────────────────
    //  Regression coefficients  (computed — not user-edited)
    // ──────────────────────────────────────────────────────────
    private double slope     = 0.0;
    private double intercept = 0.0;

    /**
     * Builds a {@code List<CalibrationPoint>} from the current static fields.
     * Called internally by {@link #computeRegression()} and can also be used
     * externally if you want to inspect the points.
     */
    public List<CalibrationPoint> getCalibrationPoints() {
        List<CalibrationPoint> pts = new ArrayList<>();
        pts.add(new CalibrationPoint(new Pose(CAL1_X, CAL1_Y, Math.toRadians(CAL1_HDG)), CAL1_RPM));
        pts.add(new CalibrationPoint(new Pose(CAL2_X, CAL2_Y, Math.toRadians(CAL2_HDG)), CAL2_RPM));
        pts.add(new CalibrationPoint(new Pose(CAL3_X, CAL3_Y, Math.toRadians(CAL3_HDG)), CAL3_RPM));
        pts.add(new CalibrationPoint(new Pose(CAL4_X, CAL4_Y, Math.toRadians(CAL4_HDG)), CAL4_RPM));
        pts.add(new CalibrationPoint(new Pose(CAL5_X, CAL5_Y, Math.toRadians(CAL5_HDG)), CAL5_RPM));
        pts.add(new CalibrationPoint(new Pose(CAL6_X, CAL6_Y, Math.toRadians(CAL6_HDG)), CAL6_RPM));
        pts.add(new CalibrationPoint(new Pose(CAL7_X, CAL7_Y, Math.toRadians(CAL7_HDG)), CAL7_RPM));
        return pts;
    }

    /**
     * Runs least-squares linear regression on the calibration points.
     * <p>
     * Model:  {@code RPM = slope × distance + intercept}
     * <p>
     * Call once at init and again any time a calibration field changes in Panels.
     */
    public void computeRegression() {
        List<CalibrationPoint> pts = getCalibrationPoints();
        int n = pts.size();
        if (n < 2) {
            slope = 0;
            intercept = MIN_RPM;
            return;
        }

        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
        for (CalibrationPoint cp : pts) {
            double d = distanceToGoal(cp.pose.getX(), cp.pose.getY());
            double r = cp.rpm;
            sumX  += d;
            sumY  += r;
            sumXY += d * r;
            sumX2 += d * d;
        }

        double denom = n * sumX2 - sumX * sumX;
        if (Math.abs(denom) < 1e-9) {
            slope     = 0.0;
            intercept = sumY / n;
        } else {
            slope     = (n * sumXY - sumX * sumY) / denom;
            intercept = (sumY - slope * sumX) / n;
        }
    }

    /**
     * Returns the RPM the flywheel should target for the given distance,
     * clamped to [{@link #MIN_RPM}, {@link #MAX_RPM}].
     */
    public double rpmForDistance(double distance) {
        double rpm = slope * distance + intercept;
        return Math.max(MIN_RPM, Math.min(MAX_RPM, rpm));
    }

    /** Euclidean distance from an arbitrary field point to the goal. */
    public static double distanceToGoal(double x, double y) {
        double dx = x - GOAL_X;
        double dy = y - GOAL_Y;
        return Math.hypot(dx, dy);
    }

    // ── Telemetry / debug getters ─────────────────────────────
    public double getSlope()     { return slope; }
    public double getIntercept() { return intercept; }
}