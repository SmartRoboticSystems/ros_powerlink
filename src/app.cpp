//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stddef.h>

#include <oplk/oplk.h>
#include <oplk/debugstr.h>

#include <eventlog/eventlog.h>

#include <app.h>

#include <ros_powerlink/openPowerlinkMsg.h>

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/* structure for input process image */

typedef struct
{
   BYTE    digitalInAU8[4];
   BYTE    analogueInAI8[4];
   INT16   analogueInAI16[2];
   INT32   analogueInAI32;
} PI_IN;

/* structure for output process image */
typedef struct
{
    BYTE    digitalOutAU8[4];
    BYTE    analogueOutAI8[4];
    INT16   analogueOutI16[2];
    INT32   analogueOutAI32;
} PI_OUT;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
/* process image */
static PI_IN*   pProcessImageIn_l;
static PI_OUT*  pProcessImageOut_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError initProcessImage(void);

ros::NodeHandle *node_oplk;
ros::Publisher data_publisher;
ros_powerlink::openPowerlinkMsg mgs_input;
ros::Subscriber data_subscriber;
ros_powerlink::openPowerlinkMsg msg_output;

void node_id(ros::NodeHandle *node)
{
    node_oplk = node;
}

void readDataToPLC(const ros_powerlink::openPowerlinkMsgPtr& msg)
{
    char i;

    for(i = 0; i < 4; i++ )
    {
         pProcessImageIn_l->digitalInAU8[i] = msg->digital_array_AU8[i];
         pProcessImageIn_l->analogueInAI8[i] = msg->analogue_array_AI8[i];
    }
    pProcessImageIn_l->analogueInAI16[0] = msg->analogue_array_AI16[0];
    pProcessImageIn_l->analogueInAI16[1] = msg->analogue_array_AI16[1];
    pProcessImageIn_l->analogueInAI32 = msg->analogue_value_AI32;
}


//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize the synchronous data application

The function initializes the synchronous data application

\return The function returns a tOplkError error code.

\ingroup module_demo_cn_console
*/
//------------------------------------------------------------------------------
tOplkError initApp(void)
{
    tOplkError ret = kErrorOk;

    data_subscriber = node_oplk->subscribe("/powerlink/data_to_plc",1,&readDataToPLC);
    data_publisher = node_oplk->advertise<ros_powerlink::openPowerlinkMsg>("/powerlink/data_from_plc",1);

    ret = initProcessImage();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown the synchronous data application

The function shuts down the synchronous data application

\return The function returns a tOplkError error code.

\ingroup module_demo_cn_console
*/
//------------------------------------------------------------------------------
void shutdownApp(void)
{
    oplk_freeProcessImage();
}

//------------------------------------------------------------------------------
/**
\brief  Synchronous data handler

The function implements the synchronous data handler.

\return The function returns a tOplkError error code.

\ingroup module_demo_cn_console
*/
//------------------------------------------------------------------------------
tOplkError processSync(void)
{
    int i;

    tOplkError      ret = kErrorOk;

    if (oplk_waitSyncEvent(100000) != kErrorOk)
        return ret;

    ret = oplk_exchangeProcessImageOut();
    if (ret != kErrorOk)
        return ret;

    /* read input image - digital outputs */

    for(i = 0; i < 4; i++)
    {
         msg_output.digital_array_AU8[i] = pProcessImageOut_l->digitalOutAU8[i];
         msg_output.analogue_array_AI8[i] = pProcessImageOut_l->analogueOutAI8[i];
    }
    msg_output.analogue_array_AI16[0] = pProcessImageOut_l->analogueOutI16[0];
    msg_output.analogue_array_AI16[1] = pProcessImageOut_l->analogueOutI16[1];
    msg_output.analogue_value_AI32 = pProcessImageOut_l->analogueOutAI32;

    ret = oplk_exchangeProcessImageIn();

    /* publish topic */

    data_publisher.publish(msg_output);

    ros::spinOnce();

    return ret;
}

//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Initialize process image

The function initializes the process image of the application.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initProcessImage(void)
{
    tOplkError      ret = kErrorOk;
    UINT            varEntries;
    tObdSize        obdSize;

    /* Allocate process image */
    printf("Initializing process image...\n");
    printf("Size of process image: Input = %ld Output = %ld\n", sizeof(PI_IN), sizeof(PI_OUT));
    eventlog_printMessage(kEventlogLevelInfo, kEventlogCategoryGeneric,
                          "Allocating process image: Input:%d Output:%d", sizeof(PI_IN), sizeof(PI_OUT));

    ret = oplk_allocProcessImage(sizeof(PI_IN), sizeof(PI_OUT));
    if (ret != kErrorOk)
    {
        return ret;
    }

    pProcessImageIn_l = (PI_IN*)oplk_getProcessImageIn();
    pProcessImageOut_l = (PI_OUT*)oplk_getProcessImageOut();

    /* link process variables used by CN to object dictionary */
    fprintf(stderr, "Linking process image vars:\n");

    // RPDO

    obdSize = sizeof(pProcessImageIn_l->digitalInAU8[0]);
    varEntries = 4;
    ret = oplk_linkProcessImageObject(0x6000, 0x01, offsetof(PI_IN, digitalInAU8), FALSE, obdSize, &varEntries);
    if (ret != kErrorOk)
    {
        fprintf(stderr, "linking process vars failed with \"%s\" (0x%04x)\n", debugstr_getRetValStr(ret), ret);
        return ret;
    }

    obdSize = sizeof(pProcessImageIn_l->analogueInAI8[0]);
    varEntries = 4;
    ret = oplk_linkProcessImageObject(0x6400, 0x01, offsetof(PI_IN, analogueInAI8), FALSE, obdSize, &varEntries);
    if (ret != kErrorOk)
    {
        fprintf(stderr, "linking process vars failed with \"%s\" (0x%04x)\n", debugstr_getRetValStr(ret), ret);
        return ret;
    }

    obdSize = sizeof(pProcessImageIn_l->analogueInAI16[0]);
    varEntries = 2;
    ret = oplk_linkProcessImageObject(0x6401, 0x01, offsetof(PI_IN, analogueInAI16), FALSE, obdSize, &varEntries);
    if (ret != kErrorOk)
    {
        fprintf(stderr, "linking process vars failed with \"%s\" (0x%04x)\n", debugstr_getRetValStr(ret), ret);
        return ret;
    }

    obdSize = sizeof(pProcessImageIn_l->analogueInAI32);
    varEntries = 1;
    ret = oplk_linkProcessImageObject(0x6402, 0x01, offsetof(PI_IN, analogueInAI32), FALSE, obdSize, &varEntries);
    if (ret != kErrorOk)
    {
        fprintf(stderr, "linking process vars failed with \"%s\" (0x%04x)\n", debugstr_getRetValStr(ret), ret);
        return ret;
    }

    // TPDO

    obdSize = sizeof(pProcessImageOut_l->digitalOutAU8[0]);
    varEntries = 4;
    ret = oplk_linkProcessImageObject(0x6200, 0x01, offsetof(PI_OUT, digitalOutAU8), TRUE, obdSize, &varEntries);
    if (ret != kErrorOk)
    {
        fprintf(stderr, "linking process vars failed with \"%s\" (0x%04x)\n", debugstr_getRetValStr(ret), ret);
        return ret;
    }

    obdSize = sizeof(pProcessImageOut_l->analogueOutAI8[0]);
    varEntries = 4;
    ret = oplk_linkProcessImageObject(0x6410, 0x01, offsetof(PI_OUT, analogueOutAI8), TRUE, obdSize, &varEntries);
    if (ret != kErrorOk)
    {
        fprintf(stderr, "linking process vars failed with \"%s\" (0x%04x)\n", debugstr_getRetValStr(ret), ret);
        return ret;
    }

    obdSize = sizeof(pProcessImageOut_l->analogueOutI16[0]);
    varEntries = 2;
    ret = oplk_linkProcessImageObject(0x6411, 0x01, offsetof(PI_OUT, analogueOutI16), TRUE, obdSize, &varEntries);
    if (ret != kErrorOk)
    {
        fprintf(stderr, "linking process vars failed with \"%s\" (0x%04x)\n", debugstr_getRetValStr(ret), ret);
        return ret;
    }

    obdSize = sizeof(pProcessImageOut_l->analogueOutAI32);
    varEntries = 1;
    ret = oplk_linkProcessImageObject(0x6412, 0x01, offsetof(PI_OUT, analogueOutAI32), TRUE, obdSize, &varEntries);
    if (ret != kErrorOk)
    {
        fprintf(stderr, "linking process vars failed with \"%s\" (0x%04x)\n", debugstr_getRetValStr(ret), ret);
        return ret;
    }

    fprintf(stderr, "Linking process vars... ok\n\n");

    return kErrorOk;
}
