# LittleSwerve - Team 999 Swerve Example project in Java

This example is intended to be a teaching/training tools for the teams that want to understand how does the swerve work exactly.
While it requires Java knowledge, and is not necessarily designed to get up and running with swerve in 30 minutes, we tried to make a program that can be easily understood and followed by students on a programming team.

Out project has the following goals:

0. We believe that this should be a primarily student-driven project, so absolute majority of the code is designed by students, and created under mentor supervision. Our example is heavy on documentation. We tried explaining what the individual line of code do, why are they there etc.

1. Keeping CAN utilization low. With all 8 encoders real-time telemetry on SmartDashboard we do not exceed 50% CAN utilization. This is partially done by requiring all encoders to be physically connected to the corresponding motors, so hardware PID routines do not put anything on CAN.

2. Keeping CPU utilization low. When extra debug telemetry is disabled (real-time swerve states printed), CPU utilization on Rio1 is around 20%. We also update odometry only when running trajectories (though the method that updates/prints it is available for troubleshooting and other purposes)

3. Keeping the code compatible with many IMU and motor types. All motors and IMUs are exposed via interfaces, which will allow one to change the hardware with relatively small code modifications. Our current code implements CTR TalonSRX (the test chassis uses 775 for turn motors and mini-CIM for drive motors), and we plan to add NEO implementation as well. The NavX implementation is added, but not tested yet.

4. Keeping the code consistent with WPI programming suggestions for Command Robot. We really tried to adhere to the rules, though probably would need more changes. In any case, we have pretty much no logic code in Robot.java, and most of it in RobotContainer.

5. Perform both teleop and trajectory navigation (latter - using PathPlanner)

## Hardware configuration

We use certain assumptions in our code that may or may not apply to your robot. These assumptions are designed to make the robot perform better, faster and more reliably.

1. We highly recommend connecting the encoders directly to the matching motor controllers without CAN. That would allow them operating on 1ms loop using Hardware PID. It would make turns and trajectory driving more precise.

2. As many of you noted, swerve behaves better if the center of gravity is closer to the center, especially in holonomic driving.

3. We do not require the robot chassis to be a square. However, we noted that non-square chassies has increased angular drift and slippage during holonomic turns, especially when combined with high-velocity limear motion.

4. The angilar motor speed matters. Remember that Falcons/NEOs have around 6k RPM, NEO 550 is around 11k RPM, and 775 is around 20k RPM (all no-load RPMs). With 12.8:1 gearbox (an example for MK4 SDS) for the steer motor it's 2.8k degrees per second for the NEOs, meaning 2.8 degrees per 1ms peak speed. With NEO 550 it's 5.1 degrees per 1ms and 775 - 9.3 degrees. The "slower" RPM motors will work Ok with the 20ms PID loop. These numbers, of course, will be slower under load, and will have lead time that is different for different motor types. However, faster motors would greatly benefit from more frequent adjustments to prevent occilations. Faster motors also will make trajectories MUCH smoother.

### IMU and motor controllers

We expose IMU and motor controllers in our code via interfaces. That would allow one to easily add a specific implementation for another motor controller and IMU, including simulation objects. Currently we provided implementations for TalonSRX motor controller and Pigeon2 IMU. We also created code for NavX IMU, but did not test it yet.

Because of the assumption #1 above, we expose encoders via corresponding motor controllers' methods rather than via separate objects. That allowed us to implement a common behavior of the swerve chassis regardless of the specific motor-related hardware used in a drivetrain.

## Controls configuration

We made a custom Controller class that allows one to use joysticks, game controllers etc to control your robot. We implemented handling of deadbands and optional cube driving that can be configured via controller settings in Constants.

The controller inputs are provided to the teleop and trajectory driving routines via supplier rather than directly exposed controller objects, which allows a quick switch to a different controller type by modifying just a few lines of code.

## Programming progress

We record and publish videos of our individual programming sessions/lessons. Here is a link to our [Swerve programming lessons](https://cheshirerobotics.org/index.php/2023/06/06/swerve-programming/). There you can see our code created, one line at a time.

## Acknowledgement:

The code was developed using two main projects as examples:

Team 125 (Nutron) - 2023 Season code
Team 3039 (Wildcat Robotics) - 2023 Quicksilver code

We also used ideas from other code we observed.

Our code is continually improving, and we're adding features to it based on the feedback we collect.

Feel free to contact the Team 999 mentors on Chiefdelphi, or via links on CheshireRobotics web site.
