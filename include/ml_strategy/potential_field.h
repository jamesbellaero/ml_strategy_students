// PotentialField.h
#ifndef POTENTIAL_FIELD_H
#define POTENTIAL_FIELD_H

#include <string>
#include <Eigen/Dense>
#include <set>
#include <iostream>

class FieldGen{
public:
  std::string name;
  mutable bool attractor;
  mutable Eigen::Vector3d pos;
  mutable double strength, radius;
  FieldGen();
  FieldGen(std::string name);
  FieldGen(std::string name, bool attactor, Eigen::Vector3d pos, double strength, double radius);
  Eigen::Vector3d getAcc(Eigen::Vector3d refPos, bool invert) const;
  bool operator<(const FieldGen& other) const {
    int compareResult = name.compare(other.name);
    return (compareResult < 0);
  }
};


class PotentialField{
public:
  std::set<FieldGen> fieldGens;
  PotentialField();
  PotentialField(std::set<FieldGen> fieldGens);
  bool contains(std::string name);
  void add(FieldGen& fg);
  void updatePos(std::string name, Eigen::Vector3d pos);
  void updateStrength(std::string name, double strength);
  void updateRadius(std::string name, double radius);
  Eigen::Vector3d getAcc(Eigen::Vector3d pos, bool invert);
};


#endif
