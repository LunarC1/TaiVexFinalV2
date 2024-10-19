#include "components/PistonGroup.h"


PistonGroup::PistonGroup(std::vector<Piston> pistons) {
  //constructor: initializes from vector of piston structs

  for (auto &piston : pistons) pistonGroup.insert(std::pair<std::string, Piston>(piston.name, piston));
}


void PistonGroup::add(Piston piston) {
  //adds a piston to the group

  pistonGroup.insert(std::pair<std::string, Piston>(piston.name, piston));
}


void PistonGroup::remove(std::string name) {
  //removes a piston from the group

  pistonGroup.erase(name);
}


void PistonGroup::enable(std::string name) {
  //enables a piston within the group

  pistonGroup.at(name).enabled = true;
}


void PistonGroup::disable(std::string name) {
  //disables a piston within the group

  pistonGroup.at(name).enabled = false;
}


void PistonGroup::on() {
  //contracts all enabled pistons within the group

  for (auto &piston : pistonGroup) if (piston.second.enabled) piston.second.piston->set(true);
}


void PistonGroup::off() {
  //retracts all enabled pistons within the group

  for (auto &piston : pistonGroup) if (piston.second.enabled) piston.second.piston->set(false);
}


void PistonGroup::toggle() {
  //toggles all enabled pistons within the group

  for (auto &piston : pistonGroup) if (piston.second.enabled) piston.second.piston->set(!piston.second.piston);
}


bool PistonGroup::state() {
  //returns the toggle state of the group

  return pistonGroup.begin()->second.piston;
}