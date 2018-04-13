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
  FieldGen(const std::string &name);
  FieldGen(const std::string &name,const bool& attactor, const Eigen::Vector3d &pos,const double& strength, const double& radius);
  Eigen::Vector3d getAcc(const Eigen::Vector3d &refPos,const bool& invert) const;
  bool operator<(const FieldGen& other) const {
    int compareResult = name.compare(other.name);
    return (compareResult < 0);
  }
};


class PotentialField{
public:
  std::set<FieldGen> fieldGens;
  PotentialField();
  PotentialField(const std::set<FieldGen>& fieldGens);
  bool contains(const std::string& name);
  void add(const FieldGen& fg);
  void updatePos(const std::string& name, const Eigen::Vector3d& pos);
  void updateStrength(const std::string &name, const double& strength);
  void updateRadius(const std::string &name, const double& radius);
  Eigen::Vector3d getAcc(const Eigen::Vector3d& pos, const std::string& name, const bool& invert);
};


#endif
