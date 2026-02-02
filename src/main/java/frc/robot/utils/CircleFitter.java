package frc.robot.utils;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

/**
 * A utility class to fit a circle to a set of 2D points.
 *
 * The method uses a linear least-squares algorithm to find the circle's center.
 * The equation of a circle is: (x - a)^2 + (y - b)^2 = R^2, where (a, b) is the
 * center.
 * This can be rewritten as: 2ax + 2by + (R^2 - a^2 - b^2) = x^2 + y^2.
 * This is a linear equation in terms of the parameters a, b, and c = R^2 - a^2
 * - b^2.
 * We can solve the system Ax = B for the parameters [a, b, c]^T.
 */
public final class CircleFitter {
    private CircleFitter() {
    }

    /**
     * Fits a circle to a list of 2D points.
     *
     * @param points The list of {@link Translation2d} points to fit.
     * @return An {@link Optional#of(Translation2d)} containing the center of the
     *         fitted circle.
     *         Returns an {@link Optional#empty()} if a fit cannot be determined
     *         (fewer than 3 points or a solver error).
     */
    public static Optional<Translation2d> fit(List<Translation2d> points) {
        if (points == null || points.size() < 3) {
            System.err.println("CircleFitter: Not enough points to fit a circle (requires at least 3).");
            return Optional.empty();
        }

        int numPoints = points.size();

        DMatrixRMaj A = new DMatrixRMaj(numPoints, 3);
        DMatrixRMaj B = new DMatrixRMaj(numPoints, 1);

        for (int i = 0; i < numPoints; i++) {
            double x = points.get(i).getX();
            double y = points.get(i).getY();

            A.set(i, 0, 2.0 * x);
            A.set(i, 1, 2.0 * y);
            A.set(i, 2, 1.0);
            B.set(i, 0, x * x + y * y);
        }

        LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.leastSquares(numPoints, 3);

        // Critical to check if setA is true
        if (!solver.setA(A)) {
            System.err.println("CircleFitter: EJML solver failed. Matrix may be singular.");
            return Optional.empty();
        }

        DMatrixRMaj solution = new DMatrixRMaj(3, 1);
        solver.solve(B, solution);

        // Solution vector contains [a, b, c]
        double centerX = solution.get(0, 0);
        double centerY = solution.get(1, 0);

        return Optional.of(new Translation2d(centerX, centerY));
    }
}
