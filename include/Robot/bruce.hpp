#pragma once

#define DOF3        3
#define DOF4        4
#define DOF6        6
#define DOF7        7

constexpr int ACTIVE_DOF = 16;
constexpr int DOF_BASEBODY = 6;	               		    //	Floating-base Body DoF
constexpr int DOF_BASEBODY_QUAT = 7;               		//	Floating-base Body DoF w/ quaternion

constexpr int TOTAL_DOF = (DOF_BASEBODY + ACTIVE_DOF);	//	= mjModel->nv
constexpr int TOTAL_DOF_QUAT = (TOTAL_DOF + 1);			//	Orientation is quaternion

constexpr int NO_OF_EE = 2;	                            //	Number of end-effectors