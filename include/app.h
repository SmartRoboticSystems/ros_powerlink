
#ifndef _INC_app_H_
#define _INC_app_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/oplk.h>
#include <ros/ros.h>

#ifdef __cplusplus
extern "C"
{
#endif

tOplkError initApp(void);
void shutdownApp(void);
tOplkError processSync(void);

void node_id(ros::NodeHandle *node);

#ifdef __cplusplus
}
#endif

#endif /* _INC_app_H_ */
