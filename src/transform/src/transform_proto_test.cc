/**
 * Copyright (c) 2022 XiaoMi
 * 
 * Author: Feixiang Zeng <zengfeixiang@xiaomi.com>
 * 
 */
#include "transform/transform.h"

int main(int argc, char** argv) {
   protos::transform::proto::Rigid2d proto;
   cartographer::transform::Rigid2d pose;
   proto = cartographer::transform::ToProto(pose);
   std::cout<< "proto go" << std::endl;
   return 0;
}