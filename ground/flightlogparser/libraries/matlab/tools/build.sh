#!/bin/bash
g++ ukf_predict2/*.cpp ukf_track_h_ins/*.cpp ukf_update2/*.cpp main.cpp -o main --std=c++11
