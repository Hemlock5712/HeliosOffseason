// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightState;

public class Limelight extends SubsystemBase {

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tshort = table.getEntry("tshort");
  NetworkTableEntry tlong = table.getEntry("tlong");
  NetworkTableEntry thor = table.getEntry("thor");
  NetworkTableEntry tvert = table.getEntry("tvert");
  NetworkTableEntry ledMode = table.getEntry("ledMode");

  public Limelight() {
  }

  public double getY() {
    return ty.getDouble(0.0);
  }

  public double getX() {
    return tx.getDouble(0.0);
  }

  public double getHorizontalWidth() {
    return thor.getDouble(0.0);
  }

  public void setLimelightState(LimelightState state) {
    ledMode.setNumber(state.ordinal());
  }

  public void limelightOff() {
    setLimelightState(LimelightState.OFF);
  }

  public void limelightOn() {
    setLimelightState(LimelightState.ON);
  }

  public void blink() {
    setLimelightState(LimelightState.BLINK);
  }

  public boolean hasValidTarget() {
    return tv.getDouble(0) == 1;
  }

  public boolean alignGood() {
    return Math.abs(getX()) < 2;
  }

  public double[] calcHoodAndRPM() {
    double arr[][] = {
        { 155, 13000, -6 },
        { 160, 12500, -6.5 },
        { 180, 12225, -5.5 },
        { 190, 12200, -5 },
        { 200, 11900, -4.3 },
        { 220, 11500, -3.25 }

    };

    int start = 0, end = arr.length - 1;

    double yoffset = getHorizontalWidth();
    int ans = 0;
    while (start <= end) {
      int mid = (start + end) / 2;

      if (arr[mid][0] < yoffset) {
        start = mid + 1;
      }

      else {
        ans = mid;
        end = mid - 1;
      }
    }

    double[] output = { arr[ans][1], arr[ans][2] };
    // return new double[] { 2000, -4 };
    return output;
  }

  @Override
  public void periodic() {
  }
}
