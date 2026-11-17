// SPDX-License-Identifier: MIT
// Export real occupied OcTree leaves, retaining each leaf's actual side length.
#include <octomap/OcTree.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>
int main(int argc,char**argv){try{if(argc!=4)throw std::runtime_error("raw_file resolution binary_flag required");std::ifstream in(argv[1],std::ios::binary);if(!in)throw std::runtime_error("Cannot read OctoMap data");octomap::OcTree tree(std::stod(argv[2]));if(std::string(argv[3])=="1")tree.readBinaryData(in);else tree.readData(in);if(in.fail())throw std::runtime_error("Invalid OctoMap payload");std::cout<<std::setprecision(8)<<"[";bool first=true;for(auto it=tree.begin_leafs();it!=tree.end_leafs();++it){if(!tree.isNodeOccupied(*it))continue;if(!first)std::cout<<",";first=false;std::cout<<"["<<it.getX()<<","<<it.getY()<<","<<it.getZ()<<","<<it.getSize()<<"]";}std::cout<<"]\n";return 0;}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
