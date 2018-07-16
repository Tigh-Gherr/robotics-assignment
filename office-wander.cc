#include <iostream>
#include <signal.h>
#include <math.h>

#include <libplayerc++/playerc++.h>

#include "Vector2D.hh"

using namespace std;
using namespace PlayerCc;

bool programRunning = true;
void programShutdown(int );

bool doorDetection(const RangerProxy &, Vector2D & );
double reachDoorV(const Vector2D &);
double reachDoorW(const Vector2D &);
void wander(const RangerProxy &, double &, double &);
double computeStimulus(const RangerProxy &, int, int);

double rMin = 0.5, rMax = 3.0, vMax = 0.5; // Constants for wandering
double sigma = 30 * M_PI / 180; // Sigma constant, used for wandering

// Computes the stimulus for Braitenberg Vehicle, Collision Avoidance
double computeStimulus(const RangerProxy & lasers, int start, int end) {

  double reading = 0.0, normFactor = 0.0; // Stimulus total and normalisation total
  // From the starting scan to the ending
  for(int i = start; i < end; i++) {
    double angle = lasers.GetMinAngle() + i * lasers.GetAngularRes(); // Retreive angle
    double laser = lasers[i]; // Retreive reading value

    /* Calculate weighting of value based on its angle,
     * so readings to the left and right have less effect
     * on movement
     */
    double weight = exp(-(angle * angle) / (sigma * sigma));

    // Normalise reading to a value between 1 and 0
    if(rMax <= laser) {
      laser = 1;
    } else if(rMin <= laser && laser <= rMax) {
      laser = (laser - rMin) / (rMax - rMin);
    } else if(laser <= rMin) {
      laser = 0;
    }

    // Apply weighting to normalised reading, add to running total
    reading += laser * weight;
    // Add weighting to running total
    normFactor += weight;
  }

  // Normalise total stimulus with total weighting
  return reading / normFactor;
}

// Compute the left hand stimulus
double computeStimulusLeft(const RangerProxy & lasers) {
  int count = lasers.GetRangeCount();
  // The second half of the laser scan is on the left of the robot
  return computeStimulus(lasers, count / 2, count);
}

// Compute the right hand stimulus
double computeStimulusRight(const RangerProxy & lasers) {
  // The first half of the laser scan is on the right of the robot
  return computeStimulus(lasers, 0, lasers.GetRangeCount() / 2);
}

// Calculate the velocity, based on the left/right stimulus
double wanderV(double sLeft, double sRight) {
  return vMax * ((sLeft + sRight) / 2);
}

// Caluclate the turning speed, based on the left/right stimulus
double wanderW(double sLeft, double sRight) {
  return vMax * ((sLeft - sRight) / 0.4);
}

// Calculate whether or not the robot is in deadlock
double deadlocked(const RangerProxy & lasers) {
  // Get front most scan distance, if too short, the robot is too close to a wall
  double scan = lasers[lasers.GetRangeCount() / 2];
  return scan < 1.1;
}

// Wander using collision avoidance
void wander(const RangerProxy & lasers, Position2dProxy & base) {
  // If the robot is deadlocked, turn away
  if(deadlocked(lasers)) {
    double w;
    // Turn in the direction of the furthest wall
    if(lasers[0] > lasers[lasers.GetRangeCount() - 1]) {
      w = -0.4;
    } else {
      w = 0.4;
    }

    base.SetSpeed(0, 0, w);
  } else {
    // Calculate stimulus
    double stimulusLeft = computeStimulusLeft(lasers);
    double stimulusRight = computeStimulusRight(lasers);

    // Calculate velocity and turning rate
    double v = wanderV(stimulusLeft, stimulusRight);
    double w = wanderW(stimulusLeft, stimulusRight);

    // Apply calculated values
    base.SetSpeed(v, 0, w);
  }
}

// Calculate velocity for reaching a door
// Implementation uses a Proportional controller
double reachDoorV(const Vector2D & target) {
  // Maximum values
  double maxV = 0.9;
  double maxD = 1.5;

  // Constant
  double K = maxV / maxD;
  // If target is greater than max distance, move at maximum feed
  if(target.Length() > maxD) {
    return maxV;
  }
  // Else, move at a speed proportional to the distance
  else {
    return maxV * target.Length() / maxD;
  }
}

// Calculate turning speed for reaching a door
// Implementation uses a Proportional Controller
double reachDoorW(const Vector2D & target) {
  // Constant used to control the turning speed
  double K = 0.7;
  // Theta
  double thT = target.Angle();

  // Ensure the robot turns in the direction requiring the least
  // movement
  if(thT > M_PI) {
    thT -= (2 * M_PI);
  }
  if(thT < -(M_PI)) {
    thT += (2 * M_PI);
  }
  return K * thT;
}

// Determine whether a door exists within the current laser scan
// If one exists, calculate its midpoint
bool doorDetection(const RangerProxy & laser, Vector2D & midpoint) {
  bool doorFound = false;
  double dth = 1; // Minimum delta between two scans to be considered a gap

  bool foundLeft = false, foundRight = false; // Flags for door discovered
  int rightIdx, leftIdx; // Index of the door within the laser scan

  Vector2D Pr, Pl; // Caclulted vector coordinates of the door sides

  // From the second laser scan to the last, until a door is found
  for(int i = 1; !doorFound && i < laser.GetRangeCount(); i++) {
    // Difference between current scan and the previous
    double diff = laser[i] - laser[i - 1];

    // If the diff is greater than the delta
    if(diff > dth) {
      foundRight = true; // Set righthand flag
      rightIdx = i - 1; // Set righthand index


      // Acquire angle in terms of the robots reference system
      double thRight = laser.GetMinAngle() + rightIdx * laser.GetAngularRes();
      // Calculate coordinates in terms of the robots reference system
      double x = laser[rightIdx] * cos(thRight);
      double y = laser[rightIdx] * sin(thRight);

      // Set the coordinates for the righthand point
      Pr.X(x);
      Pr.Y(y);
    }

    // If the diff is less than negative delta, and the rightside
    // has been discovered
    if(diff < -dth && foundRight) {
      doorFound = true; // Set doorfound flag
      leftIdx = i; // Set lefthand index

      // Acquire angle in terms of the robots reference system
      double thLeft = laser.GetMinAngle() + leftIdx * laser.GetAngularRes();
      // Calculate coordinates in terms of the robots reference system
      double x = laser[leftIdx] * cos(thLeft);
      double y = laser[leftIdx] * sin(thLeft);

      // Set the coordinates for the lefthand point
      Pl.X(x);
      Pl.Y(y);
    }
  }

  midpoint = 0.5 * (Pr + Pl); // Calculate midpoint between right and left sides of door
  return doorFound; // Return doorfound flag
}


int
main(int argn, char *argv[])
{
  PlayerClient robotClient("localhost");

  RangerProxy laser(&robotClient, 1);
  Position2dProxy base(&robotClient, 0);

  // for a clean shutdown
  signal(SIGINT, programShutdown);

  // Robot pose, position and heading.
  Vector2D robot;
  double robotHeading;

  // Wait until we get the pose of the robot from the server.
  do {
    robotClient.Read();
  }
  while (!laser.IsFresh());

  // Enable robot motors (else the robot will not move).
  base.SetMotorEnable(true);
  while (programRunning)
    {
      robotClient.Read();

      // Midpoint of the found door
      Vector2D midPoint;
      // If a door is detected
      if(doorDetection(laser, midPoint)) {
        // Calculate velocity proportional to the midpoint
        double v = reachDoorV(midPoint);
        // Calculate turning speed proportional to the midpoint
        double w = reachDoorW(midPoint);

        // Apply calculated values
        base.SetSpeed(v, 0, w);
      } else {
        // Wander using collision avoidance
        wander(laser, base);
      }

    }

  // Disable robot motors.
  base.SetMotorEnable(false);

  return 0;
}

// Shutdown the program
void programShutdown(int s)
{
  programRunning = false;
}
