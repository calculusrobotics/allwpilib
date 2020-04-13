/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpiutil.math;

import java.util.function.BiFunction;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.NormOps_DDRM;
import org.ejml.dense.row.decomposition.qr.QRDecompositionHouseholder_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.CholeskyDecomposition_F64;
import org.ejml.simple.SimpleBase;
import org.ejml.simple.SimpleMatrix;

public class SimpleMatrixUtils {
  private SimpleMatrixUtils() {
  }

  /**
   * Compute the matrix exponential, e^M of the given matrix.
   *
   * @param matrix The matrix to compute the exponential of.
   * @return The resultant matrix.
   */
  @SuppressWarnings({"LocalVariableName", "LineLength"})
  public static SimpleMatrix expm(SimpleMatrix matrix) {
    BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> solveProvider = SimpleBase::solve;
    SimpleMatrix A = matrix;
    double A_L1 = NormOps_DDRM.inducedP1(matrix.getDDRM());
    int n_squarings = 0;

    if (A_L1 < 1.495585217958292e-002) {
      Pair<SimpleMatrix, SimpleMatrix> pair = _pade3(A);
      return dispatchPade(pair.getFirst(), pair.getSecond(), n_squarings, solveProvider);
    } else if (A_L1 < 2.539398330063230e-001) {
      Pair<SimpleMatrix, SimpleMatrix> pair = _pade5(A);
      return dispatchPade(pair.getFirst(), pair.getSecond(), n_squarings, solveProvider);
    } else if (A_L1 < 9.504178996162932e-001) {
      Pair<SimpleMatrix, SimpleMatrix> pair = _pade7(A);
      return dispatchPade(pair.getFirst(), pair.getSecond(), n_squarings, solveProvider);
    } else if (A_L1 < 2.097847961257068e+000) {
      Pair<SimpleMatrix, SimpleMatrix> pair = _pade9(A);
      return dispatchPade(pair.getFirst(), pair.getSecond(), n_squarings, solveProvider);
    } else {
      double maxNorm = 5.371920351148152;
      double log = Math.log(A_L1 / maxNorm) / Math.log(2); // logb(2, arg)
      n_squarings = (int) Math.max(0, Math.ceil(log));
      A = A.divide(Math.pow(2.0, n_squarings));
      Pair<SimpleMatrix, SimpleMatrix> pair = _pade13(A);
      return dispatchPade(pair.getFirst(), pair.getSecond(), n_squarings, solveProvider);
    }
  }

  @SuppressWarnings({"LocalVariableName", "ParameterName", "LineLength"})
  private static SimpleMatrix dispatchPade(SimpleMatrix U, SimpleMatrix V,
                                           int nSquarings, BiFunction<SimpleMatrix, SimpleMatrix, SimpleMatrix> solveProvider) {
    SimpleMatrix P = U.plus(V);
    SimpleMatrix Q = U.negative().plus(V);

    SimpleMatrix R = solveProvider.apply(Q, P);

    for (int i = 0; i < nSquarings; i++) {
      R = R.mult(R);
    }

    return R;
  }

  @SuppressWarnings({"MethodName", "LocalVariableName", "ParameterName"})
  private static Pair<SimpleMatrix, SimpleMatrix> _pade3(SimpleMatrix A) {
    double[] b = new double[]{120, 60, 12, 1};
    SimpleMatrix ident = eye(A.numRows(), A.numCols());

    SimpleMatrix A2 = A.mult(A);
    SimpleMatrix U = A.mult(A2.mult(ident.scale(b[1]).plus(b[3])));
    SimpleMatrix V = A2.scale(b[2]).plus(ident.scale(b[0]));
    return new Pair<>(U, V);
  }

  @SuppressWarnings({"MethodName", "LocalVariableName", "ParameterName"})
  private static Pair<SimpleMatrix, SimpleMatrix> _pade5(SimpleMatrix A) {
    double[] b = new double[]{30240, 15120, 3360, 420, 30, 1};
    SimpleMatrix ident = eye(A.numRows(), A.numCols());
    SimpleMatrix A2 = A.mult(A);
    SimpleMatrix A4 = A2.mult(A2);

    SimpleMatrix U = A.mult(A4.scale(b[5]).plus(A2.scale(b[3])).plus(ident.scale(b[1])));
    SimpleMatrix V = A4.scale(b[4]).plus(A2.scale(b[2])).plus(ident.scale(b[0]));

    return new Pair<>(U, V);
  }

  @SuppressWarnings({"MethodName", "LocalVariableName", "LineLength", "ParameterName"})
  private static Pair<SimpleMatrix, SimpleMatrix> _pade7(SimpleMatrix A) {
    double[] b = new double[]{17297280, 8648640, 1995840, 277200, 25200, 1512, 56, 1};
    SimpleMatrix ident = eye(A.numRows(), A.numCols());
    SimpleMatrix A2 = A.mult(A);
    SimpleMatrix A4 = A2.mult(A2);
    SimpleMatrix A6 = A4.mult(A2);

    SimpleMatrix U =
            A.mult(A6.scale(b[7]).plus(A4.scale(b[5])).plus(A2.scale(b[3])).plus(ident.scale(b[1])));
    SimpleMatrix V =
            A6.scale(b[6]).plus(A4.scale(b[4])).plus(A2.scale(b[2])).plus(ident.scale(b[0]));

    return new Pair<>(U, V);
  }

  @SuppressWarnings({"MethodName", "LocalVariableName", "ParameterName", "LineLength"})
  private static Pair<SimpleMatrix, SimpleMatrix> _pade9(SimpleMatrix A) {
    double[] b = new double[]{17643225600.0, 8821612800.0, 2075673600, 302702400, 30270240,
        2162160, 110880, 3960, 90, 1};
    SimpleMatrix ident = eye(A.numRows(), A.numCols());
    SimpleMatrix A2 = A.mult(A);
    SimpleMatrix A4 = A2.mult(A2);
    SimpleMatrix A6 = A4.mult(A2);
    SimpleMatrix A8 = A6.mult(A2);

    SimpleMatrix U =
            A.mult(A8.scale(b[9]).plus(A6.scale(b[7])).plus(A4.scale(b[5])).plus(A2.scale(b[3])).plus(ident.scale(b[1])));
    SimpleMatrix V =
            A8.scale(b[8]).plus(A6.scale(b[6])).plus(A4.scale(b[4])).plus(A2.scale(b[2])).plus(ident.scale(b[0]));

    return new Pair<>(U, V);
  }

  @SuppressWarnings({"MethodName", "LocalVariableName", "LineLength", "ParameterName"})
  private static Pair<SimpleMatrix, SimpleMatrix> _pade13(SimpleMatrix A) {
    double[] b = new double[]{64764752532480000.0, 32382376266240000.0, 7771770303897600.0,
        1187353796428800.0, 129060195264000.0, 10559470521600.0, 670442572800.0,
        33522128640.0, 1323241920, 40840800, 960960, 16380, 182, 1};
    SimpleMatrix ident = eye(A.numRows(), A.numCols());

    SimpleMatrix A2 = A.mult(A);
    SimpleMatrix A4 = A2.mult(A2);
    SimpleMatrix A6 = A4.mult(A2);

    SimpleMatrix U =
            A.mult(A6.scale(b[13]).plus(A4.scale(b[11])).plus(A2.scale(b[9])).plus(A6.scale(b[7])).plus(A4.scale(b[5])).plus(A2.scale(b[3])).plus(ident.scale(b[1])));
    SimpleMatrix V =
            A6.mult(A6.scale(b[12]).plus(A4.scale(b[10])).plus(A2.scale(b[8]))).plus(A6.scale(b[6]).plus(A4.scale(b[4])).plus(A2.scale(b[2])).plus(ident.scale(b[0])));

    return new Pair<>(U, V);
  }

  private static SimpleMatrix eye(int rows, int cols) {
    return SimpleMatrix.identity(Math.min(rows, cols));
  }

  /**
   * The identy of a square matrix.
   *
   * @param rows the number of rows (and columns)
   * @return the identiy matrix, rows x rows.
   */
  public static SimpleMatrix eye(int rows) {
    return SimpleMatrix.identity(rows);
  }

  /**
   * Decompose the given matrix using Cholesky Decomposition and return a view of the upper
   * triangular matrix (if you want lower triangular see the other overload of this method.) If the
   * input matrix is zeros, this will return the zero matrix.
   *
   * @param src The matrix to decompose.
   * @return The decomposed matrix.
   * @throws RuntimeException if the matrix could not be decomposed (ie. is not positive
   *                          semidefinite).
   */
  public static SimpleMatrix lltDecompose(SimpleMatrix src) {
    return lltDecompose(src, false);
  }

  /**
   * Decompose the given matrix using Cholesky Decomposition. If the input matrix is zeros, this
   * will return the zero matrix.
   *
   * @param src The matrix to decompose.
   * @param lowerTriangular if we want to decompose to the lower triangular Cholesky matrix.
   * @return The decomposed matrix.
   * @throws RuntimeException if the matrix could not be decomposed (ie. is not positive
   *                          semidefinite).
   */
  public static SimpleMatrix lltDecompose(SimpleMatrix src, boolean lowerTriangular) {
    SimpleMatrix temp = src.copy();

    CholeskyDecomposition_F64<DMatrixRMaj> chol =
            DecompositionFactory_DDRM.chol(temp.numRows(), lowerTriangular);
    if (!chol.decompose(temp.getMatrix())) {
      // check that the input is not all zeros -- if they are, we special case and return all
      // zeros.
      var matData = temp.getDDRM().data;
      var isZeros = true;
      for (double matDatum : matData) {
        isZeros = isZeros && Math.abs(matDatum) < 1e-6;
      }
      if (isZeros) {
        return new SimpleMatrix(temp.numRows(), temp.numCols());
      }

      throw new RuntimeException("Cholesky decomposition failed! Input matrix:\n" + src.toString());
    }

    return SimpleMatrix.wrap(chol.getT(null));
  }

  /**
   * Compute the Hoseholder QR decomposition of a matrix.
   * @param src the source matrix.
   * @return the decomposed matrix.
   * @throws RuntimeException if the decomposition fails.
   */
  public static SimpleMatrix householderQrDecompose(SimpleMatrix src) {
    var temp = src.copy();

    var decomposer = new QRDecompositionHouseholder_DDRM();
    if (!(decomposer.decompose(temp.getDDRM()))) {
      throw new RuntimeException("householder decomposition failed!");
    }

    return temp;
  }

  public static class Pair<A, B> {
    private final A m_first;
    private final B m_second;

    public Pair(A first, B second) {
      m_first = first;
      m_second = second;
    }

    public A getFirst() {
      return m_first;
    }

    public B getSecond() {
      return m_second;
    }

    @SuppressWarnings("ParameterName")
    public static <A, B> Pair<A, B> of(A a, B b) {
      return new Pair<>(a, b);
    }
  }

}
