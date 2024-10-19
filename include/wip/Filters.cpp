#include "utility/Filters.h"


float StaticEMAFilter::tick(float input) {
  //updates the queue and returns the new average

  value = input * Ka + value * (1.0 - Ka);
  queue.push_back(value);
  if (queue.size() > sampleSize) queue.erase(queue.begin());

  return std::accumulate(queue.begin(), queue.end(), 0.0) / queue.size();
}


float DynamicEMAFilter::tick(float input, float prev) {
  //updates the queue and returns the new average

  float Ka = Misc::clamp(fabs(input - prev) / accelFactor, minKa, maxKa);

  value = input * Ka + value * (1.0 - Ka);
  queue.push_back(value);
  if (queue.size() > sampleSize) queue.erase(queue.begin());

  return std::accumulate(queue.begin(), queue.end(), 0.0) / queue.size();
}


float SlewFilter::tick(float input) {
  //restricts the rate of change of a set of numbers

  if (maxChange < 0) return input;

  float change = fabs(input - lastInput);
  if (change > maxChange) {
    lastInput += std::copysign(maxChange, input - lastInput);
    return lastInput;
  }
  lastInput = input;
  return input;
}


void SlewFilter::reset() {
  //resets the slew filter between uses

  lastInput = 0;
}