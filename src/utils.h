//
// Created by cjq on 23-7-25.
//

#pragma once

#include <iostream>
#include <filesystem>
#include <fstream>
#include <regex>

#include <eigen3/Eigen/Eigen>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <spdlog/spdlog.h>

#include "def.h"
#include "pose.h"


std::vector<PoseStamped> ReadPosesFromTxt(const std::string& txt_path);

void WritePosesToTxt(const std::string& txt_path,const std::vector<PoseStamped> &poses);

void WriteOnePoseToTxt(const std::string& txt_path,const Pose *pose);


vector<double> StringLineToVector(const string& line);


