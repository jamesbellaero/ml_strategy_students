//potential_field.cpp
// References: Spong, "Robotic Dynamics and Control, Second Edition", pg 159-160

#include "ml_strategy/potential_field.h"

FieldGen::FieldGen(){
  this->name = "";
  this->pos << 0,0,0;
  this->strength = 0;
  this->radius = 1;
}

FieldGen::FieldGen(const std::string& name){
  this->name = name;
  this->pos << 0,0,0;
  this->strength = 0;
  this->radius = 1;
}

FieldGen::FieldGen(const std::string &name, const bool& attractor, const Eigen::Vector3d& pos, const double& strength, const double& radius){
  this->name = name;
  this->attractor = attractor;
  this->pos = pos;
  this->strength = strength;
  this->radius = radius;
}

Eigen::Vector3d FieldGen::getAcc(const Eigen::Vector3d& refPos, const bool& invert) const{
  Eigen::Vector3d acc;
  double rho = (pos - refPos).norm(); //Euclidean distance between the two points
    
  if(invert^this->attractor){// it is an attractor
    if(rho>this->radius)
      acc = -this->strength*(refPos - pos);
    else
      acc = -this->radius*this->strength*(refPos - pos)/rho;
  }
  else{// Repulsor
    if(rho>this->radius)
      acc << 0,0,0;
    else
      acc =  this->strength*(1/rho - 1/this->radius)*1/rho/rho*(refPos-pos)/((refPos-pos).norm());
  }
  //std::cout<<"\n\n"<<"Difference of "<<rho<<" for attractor="<<attractor<<"yields accel of "<<acc<<std::endl;
  //std::cout<<"\n\n"<<this->name<<": "<<pos<<"\n"<<"Strength: "<<strength<<"\tRadius: "<<radius<<"\t Attractor: "<<attractor<<"\t Rho"<<rho<<"\nRefPos: "<<refPos<<std::endl;
  return acc;
}

PotentialField::PotentialField(){
}
PotentialField::PotentialField(const std::set<FieldGen>& fieldGens){
  this->fieldGens = fieldGens;
}
bool PotentialField::contains(const std::string& name){
  FieldGen named(name);

  std::set<FieldGen>::iterator it;
  it = this->fieldGens.find(named);
  return it != this->fieldGens.end();
}
void PotentialField::add(const FieldGen& fg){
  this->fieldGens.insert(fg);
  //std::cout<<"Added "<<fg.name<<std::endl;
}
void PotentialField::updatePos(const std::string& name, const Eigen::Vector3d& pos){
  FieldGen named(name);
  std::set<FieldGen>::iterator it = this->fieldGens.find(named);
  it->pos << pos;
  //std::cout<<"Updated "<<name<<std::endl;
}
void PotentialField::updateStrength(const std::string& name,const double& strength){
  FieldGen named(name);
  std::set<FieldGen>::iterator it = this->fieldGens.find(named);
  it->strength = strength;
}
void PotentialField::updateRadius(const std::string& name, const double& radius){
  FieldGen named(name);
  std::set<FieldGen>::iterator it = this->fieldGens.find(named);
  it->radius = radius;
}

Eigen::Vector3d PotentialField::getAcc(const Eigen::Vector3d& pos, const std::string& name, const bool& invert){
  Eigen::Vector3d acc;
  acc << 0,0,0;
  
  std::set<FieldGen>::iterator it;
  for(it = this->fieldGens.begin(); it!=this->fieldGens.end();it++){
    if(name.compare(it->name)!=0){
      acc += it->getAcc(pos,invert);
      //std::cout<<it->name<<" and "<<name<<": "<<it->getAcc(pos,invert)<<std::endl;
    }
  }
  return acc;
}






