//potential_field.cpp
// References: Spong, "Robotic Dynamics and Control, Second Edition", pg 159-160

#include "ml_strategy/potential_field.h"

FieldGen::FieldGen(){
  this.name = "";
  this.pos << 0,0,0;
  this.strength = 0;
  this.radius = 1;
}

FieldGen::FieldGen(std::string name, bool attractor, Eigen::Vector3d pos, double strength, double radius){
  this.name = name;
  this.attractor = attractor;
  this.pos = pos;
  this.strength = strength;
  this.radius = radius;
}

FieldGen::getAcc(Eigen::Vector3d refPos, bool invert){
  Eigen::Vector3d acc;
  double rho = (pos - refPos).norm(); //Euclidean distance between the two points

  if(invert^it->attractor){// it is an attractor
    if(rho>this.radius)
      acc = -this.strength*(refPos - pos);
    else
      acc = -this.radius*this.strength*(refPos - pos)/rho;
  }
  else{// Repulsor
    if(rho>this.radius)
      acc << 0,0,0;
    else
      acc =  this.strength*(1/rho - 1/this.radius)*1/rho/rho*(refPos-pos)/((refPos-pos).norm());
  }
}

PotentialField::PotentialField(){
}
PotentialField::PotentialField(std::vector<FieldGen> fieldGens){
  this.fieldGens = fieldGens;
}
bool PotentialField::contains(std:string name){
  std::set<FieldGen>::iterator it;
  it = this.fieldGens.find(name);
  return it != this.fieldGens.end();
}
void PotentialField::add(FieldGen fg){
  this.fieldGens.insert(fg);
}
void PotentialField::updatePos(std::string name, Eigen::Vector3d pos){
  std::set<FieldGen>::iterator it = this.fieldGens.find(name);
  it->pos = pos;
}
void PotentialField::updateStrength(std::string name, double strength){
  std::set<FieldGen>::iterator it = this.fieldGens.find(name);
  it->strength = strength;
}
void PotentialField::updateRadius(std::string name, double radius){
  std::set<FieldGen>::iterator it = this.fieldGens.find(name);
  it->radius = radius;
}

Eigen::Vector3d getAcc(Eigen::Vector3d pos, bool invert){
  Eigen::Vector3d acc;
  acc << 0,0,0;
  
  std::set<FieldGen>::iterator it;
  for(it = this.fieldGens.begin(); it<this.fieldGens.end();it++){
    acc += it->getAcc(pos,invert);
  }
}






