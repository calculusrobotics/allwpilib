/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.sim;

import edu.wpi.first.wpilibj.math.StateSpaceUtil;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Num;
import edu.wpi.first.wpiutil.math.numbers.N1;
import org.ejml.simple.SimpleMatrix;

public class LinearSystemSimulator<S extends Num, I extends Num,
      O extends Num> {

  private final LinearSystem<S, I, O> m_plant;

  private boolean m_shouldAddNoise;

  private Matrix<S, N1> m_trueXhat;
  @SuppressWarnings("MemberName")
  private Matrix<O, N1> m_y;
  @SuppressWarnings("MemberName")
  private Matrix<I, N1> m_u;
  private final Matrix<O, N1> m_measurementStdDevs;

  /**
   * Create a LinearSystemSimulator. This simulator uses an {@link LinearSystem} to simulate
   * the state of the system. In simulationPeriodic, users should first set inputs from motors, update
   * the simulation and write simulated outputs to sensors.
   *
   * @param system               The system being controlled.
   * @param addNoise             If we should add noise to the measurement vector
   * @param m_measurementStdDevs Standard deviations of measurements. Can be null if addNoise is false.
   */
  public LinearSystemSimulator(LinearSystem<S, I, O> system, boolean addNoise, Matrix<O, N1> m_measurementStdDevs) {
    this.m_plant = system;
    this.m_shouldAddNoise = addNoise;

    m_trueXhat = new Matrix<>(new SimpleMatrix(system.getA().getNumRows(), 1));
    this.m_measurementStdDevs = m_measurementStdDevs;
  }

  public boolean getShouldAddNoise() {
    return m_shouldAddNoise;
  }

  public void setShouldAddNoise(boolean m_shouldAddNoise) {
    this.m_shouldAddNoise = m_shouldAddNoise;
  }

  @SuppressWarnings("LocalVariableName")
  public void update(double dtSeconds) {

    // x = ax + bu
    m_trueXhat = m_plant.calculateX(m_trueXhat, m_u, dtSeconds);

    // y = cx + du
    m_y = m_plant.calculateY(m_trueXhat, m_u);
    if (m_shouldAddNoise) {
      m_y = m_y.plus(StateSpaceUtil.makeWhiteNoiseVector(m_measurementStdDevs));
    }
  }

  public Matrix<O, N1> getY() {
    return m_y;
  }

  public double getY(int row) {
    return m_y.get(row, 0);
  }

  public void setInput(Matrix<I, N1> u) {
    this.m_u = u;
  }

  public void setInput(int row, double value) {
    m_u.set(row, 0, value);
  }

  public Matrix<O, N1> getOutput() {
    return m_y;
  }

  public double getOutput(int row) {
    return m_y.get(row, 0);
  }
}
