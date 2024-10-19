#include "utility/Path.h"


Path::Path(std::vector<Pose> waypoints) {
  //constructor for path object

  this->waypoints = waypoints;
}


int Path::closestIndex(Pose pose) {
  //returns the index of the closest point on the path to the pose

  int closestIndex = 0;
  float closestDist = 100000;

  for (int i = 0; i < waypoints.size(); i++) {
    if (pose.distance(waypoints.at(i)) < closestDist) {
      closestIndex = i;
      closestDist = pose.distance(waypoints.at(i));
    }
  }

  return closestIndex;
}


int Path::lookaheadIndex(Pose pose, int startIndex, float lookahead) {
  //returns index of first point on path ahead of pose more than x distance away

  for (int i = startIndex; i < waypoints.size(); i++) {
    if (pose.distance(waypoints.at(i)) >= lookahead) return i;
  }

  return lastIndex();
}


int Path::lastIndex() {
  //returns the last path point's index

  return waypoints.size() - 1;
}


Pose Path::closestPoint(Pose pose) {
  //returns the closest point on the path to the pose

  int closestIndex = 0;
  float closestDist = 100000;

  for (int i = 0; i < waypoints.size(); i++) {
    if (pose.distance(waypoints.at(i)) < closestDist) {
      closestIndex = i;
      closestDist = pose.distance(waypoints.at(i));
    }
  }

  return waypoints.at(closestIndex);
}


Pose Path::lookaheadPoint(Pose pose, int startIndex, float lookahead) {
  //returns first point on path ahead of pose more than x distance away

  for (int i = startIndex; i < waypoints.size(); i++) {
    if (pose.distance(waypoints.at(i)) >= lookahead) return waypoints.at(i);
  }

  return waypoints.at(lastIndex());
}


Pose Path::lastPoint() {
  //returns the last path point

  return waypoints.at(lastIndex());
}


Pose Path::at(int index) {
  //returns the path point at an index

  index = Misc::clamp(index, 0, lastIndex());
  return waypoints.at(index);
}


bool Path::nearEnd(int index) {
  //returns whether or not an index is near the end of the path

  return (lastIndex() - index <= 5);
}