// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Derivative {
    private int clockCycles;
    private double[] rates;
    int currentIndex = 0;
    int oldestIndex = 0;
    private double oldestRate = 0;
    private double currentRate = 0;

    public Derivative(int clockCycles){
        this.clockCycles = clockCycles;

        rates = new double[clockCycles];
    }

  public void periodic(double value) {
    oldestRate = rates[currentIndex];
    rates[currentIndex] = value;
    currentRate = rates[currentIndex];
    currentIndex++;
    if (currentIndex >= clockCycles){
        currentIndex = 0;
    }
    oldestRate = rates[currentIndex];
  }

  public double getChangeInRate(){ 
    double changeInRate = currentRate - oldestRate;
    return changeInRate;
  }

  public double getNetAcceleration(){
    return getChangeInRate()/clockCycles;
  }
  
}
