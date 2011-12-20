// -*-C++-*-
/*!
 * @file  GraspControllerSVC_impl.cpp
 * @brief Service implementation code of GraspController.idl
 *
 */
#include <iostream>

#include "GraspControllerSVC_impl.h"

using namespace std;

/*
 * Example implementational code for IDL interface GraspPlanStart
 */
GraspPlanStartSVC_impl::GraspPlanStartSVC_impl()
{
  // Please add extra constructor code here.
}


GraspPlanStartSVC_impl::~GraspPlanStartSVC_impl()
{
  // Please add extra destructor code here.
}





// End of example implementational code

/*
 * Example implementational code for IDL interface GraspPlanResult
 */
GraspPlanResultSVC_impl::GraspPlanResultSVC_impl()
{
  // Please add extra constructor code here.
}


GraspPlanResultSVC_impl::~GraspPlanResultSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
void GraspPlanResultSVC_impl::GraspPlanningResult(const GraspPlanResult::DblSequence3& GraspPos, const GraspPlanResult::DblSequence9& GraspOri, const GraspPlanResult::DblSequence3& ApproachPos, const GraspPlanResult::DblSequence9& ApproachOri, CORBA::Double angle, CORBA::ULong state, CORBA::ULong& isContinue)
{
  // Please insert your code here and remove the following warning pragma
	if(state > 0){
		isContinue=0;
		std::cout << "Grasp Plan failed" << std::endl;	
		return;
	}
	cout << GraspPos[0] << GraspPos[1] << GraspPos[2] << endl;
	
	bool flag=true;
	if(flag){
		isContinue=0;
		std::cout << "success" << endl;
	}else{
		isContinue=1;
		std::cout << "fail" << endl;
	}
	return;
}

void GraspPlanResultSVC_impl::ReleasePlanningResult(const GraspPlanResult::DblSequence3& GraspPos, const GraspPlanResult::DblSequence9& GraspOri, const GraspPlanResult::DblSequence3& ApproachPos, const GraspPlanResult::DblSequence9& ApproachOri, CORBA::Double angle, CORBA::ULong state, CORBA::ULong& isContinue)
{
	std::cout << "Release Plan" << std::endl;	
	GraspPlanningResult(GraspPos, GraspOri, ApproachPos, ApproachOri, angle, state, isContinue);
	return ;

}


// End of example implementational code



