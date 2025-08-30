// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

// does not include swerve constants

package frc.robot.constants;

// import frc.robot.util.LoggedTunableNumber; - Part of Example

public final class SubsystemConstants {
  public static boolean coralStuckMode = false;
  public static final String CANIVORE_ID_STRING = "CAN Bus 2";
  public static final double LOOP_PERIOD_SECONDS = 0.02;
  public static final boolean tuningMode = true;

  // ----- EXAMPLE -----

  // // public static final double NEAR_FAR_AT_REEF_OFFSET = -0.47;
  // public static final LoggedTunableNumber NEAR_FAR_AT_REEF_OFFSET =
  //     new LoggedTunableNumber("Auto Align/NEAR_FAR_AT_REEF_OFFSET", -0.525); // -0.482;
  // public static final double NEAR_FAR_AWAY_REEF_OFFSET = -0.9;
  // // more neg moves it to the right
  // // public static final double LEFT_RIGHT_BRANCH_OFFSET = -0.015;
  // public static final LoggedTunableNumber CORRECTION_LEFT_BRANCH_OFFSET =
  //     new LoggedTunableNumber("Auto Align/LEFT_BRANCH_OFFSET", -0.045);
  // // new LoggedTunableNumber("Auto Align/LEFT_BRANCH_OFFSET", -0.023);
  // public static final LoggedTunableNumber CORRECTION_RIGHT_BRANCH_OFFSET =
  //     new LoggedTunableNumber("Auto Align/RIGHT_BRANCH_OFFSET", -0.045);
  // // new LoggedTunableNumber("Auto Align/RIGHT_BRANCH_OFFSET", -0.023);
  // public static final double APPROACH_OFFSET_LEFT_RIGHT_OFFSET = -0.2;
  // public static final double ADJUST_OFFSET_LEFT_RIGHT_OFFSET = -0.07;

  // public static class IntakeConstants {
  //   public static final double CURRENT_LIMIT = 40.0;
  //   public static final boolean CURRENT_LIMIT_ENABLED = true;
  // }
}
