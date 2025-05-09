#include "uxr/client/client.h"
#include <AP_DDS/AP_DDS_config.h>


#define AP_DDS_ARMMOTOR_SRV(NAME) "rs/" NAME "/arm_motorsService"
#define AP_DDS_ARMMOTOR_SRV_RQ(NAME) "rq/" NAME "/arm_motorsRequest"
#define AP_DDS_ARMMOTOR_SRV_RR(NAME) "rr/" NAME "/arm_motorsReply"

#define AP_DDS_MODESW_SRV(NAME) "rs/" NAME "/mode_switchService"
#define AP_DDS_MODESW_SRV_RQ(NAME) "rq/" NAME "/mode_switchRequest"
#define AP_DDS_MODESW_SRV_RR(NAME) "rr/" NAME "/mode_switchReply"


#define AP_DDS_PARAMCHECH_SRV(NAME) "rs/" NAME "/prearm_checkService"
#define AP_DDS_PARAMCHECH_SRV_RQ(NAME) "rq/" NAME "/prearm_checkRequest"
#define AP_DDS_PARAMCHECH_SRV_RR(NAME) "rr/" NAME "/prearm_checkReply"

#define AP_DDS_TKOFF_SRV(NAME) "rs/" NAME "/experimental/takeoffService"
#define AP_DDS_TKOFF_SRV_RQ(NAME) "rq/" NAME "/experimental/takeoffRequest"
#define AP_DDS_TKOFF_SRV_RR(NAME) "rr/" NAME "/experimental/takeoffReply"

#define AP_DDS_SETPARAM_SRV(NAME) "rs/" NAME "/set_parametersService"
#define AP_DDS_SETPARAM_SRV_RQ(NAME) "rq/" NAME "/set_parametersRequest"
#define AP_DDS_SETPARAM_SRV_RR(NAME) "rr/" NAME "/set_parametersReply"

#define AP_DDS_GETPARAM_SRV(NAME) "rs/" NAME "/get_parametersService"
#define AP_DDS_GETPARAM_SRV_RQ(NAME) "rq/" NAME "/get_parametersRequest"
#define AP_DDS_GETPARAM_SRV_RR(NAME) "rr/" NAME "/get_parametersReply"

enum class ServiceIndex: uint8_t {
#if AP_DDS_ARM_SERVER_ENABLED
    ARMING_MOTORS,
#endif // #if AP_DDS_ARM_SERVER_ENABLED
#if AP_DDS_MODE_SWITCH_SERVER_ENABLED
    MODE_SWITCH,
#endif // AP_DDS_MODE_SWITCH_SERVER_ENABLED
#if AP_DDS_ARM_CHECK_SERVER_ENABLED
    PREARM_CHECK,
#endif // AP_DDS_ARM_CHECK_SERVER_ENABLED
#if AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED
    TAKEOFF,
#endif // AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED
#if AP_DDS_PARAMETER_SERVER_ENABLED
    SET_PARAMETERS,
    GET_PARAMETERS
#endif // AP_DDS_PARAMETER_SERVER_ENABLED
};

static inline constexpr uint8_t to_underlying(const ServiceIndex index)
{
    static_assert(sizeof(index) == sizeof(uint8_t));
    return static_cast<uint8_t>(index);
}

constexpr struct AP_DDS_Client::Service_table AP_DDS_Client::services[] = {
#if AP_DDS_ARM_SERVER_ENABLED
    {
        .req_id = to_underlying(ServiceIndex::ARMING_MOTORS),
        .rep_id = to_underlying(ServiceIndex::ARMING_MOTORS),
        .service_rr = Service_rr::Replier,
        .service_name =  AP_DDS_ARMMOTOR_SRV(AP_DDS_PARTICIPANT_NAME),
        .request_type = "ardupilot_msgs::srv::dds_::ArmMotors_Request_",
        .reply_type = "ardupilot_msgs::srv::dds_::ArmMotors_Response_",
        .request_topic_name = AP_DDS_ARMMOTOR_SRV_RQ(AP_DDS_PARTICIPANT_NAME),
        .reply_topic_name = AP_DDS_ARMMOTOR_SRV_RR(AP_DDS_PARTICIPANT_NAME),
        .qos = {
            .durability = UXR_DURABILITY_TRANSIENT_LOCAL,
            .reliability = UXR_RELIABILITY_RELIABLE,
            .history = UXR_HISTORY_KEEP_LAST,
            .depth = 5,
        },
    },
#endif // AP_DDS_ARM_SERVER_ENABLED
#if AP_DDS_MODE_SWITCH_SERVER_ENABLED
    {
        .req_id = to_underlying(ServiceIndex::MODE_SWITCH),
        .rep_id = to_underlying(ServiceIndex::MODE_SWITCH),
        .service_rr = Service_rr::Replier,
        .service_name = AP_DDS_MODESW_SRV(AP_DDS_PARTICIPANT_NAME),
        .request_type = "ardupilot_msgs::srv::dds_::ModeSwitch_Request_",
        .reply_type = "ardupilot_msgs::srv::dds_::ModeSwitch_Response_",
        .request_topic_name = AP_DDS_MODESW_SRV_RQ(AP_DDS_PARTICIPANT_NAME),
        .reply_topic_name = AP_DDS_MODESW_SRV_RR(AP_DDS_PARTICIPANT_NAME),
    },
#endif // AP_DDS_MODE_SWITCH_SERVER_ENABLED
#if AP_DDS_ARM_CHECK_SERVER_ENABLED
    {
        .req_id = to_underlying(ServiceIndex::PREARM_CHECK),
        .rep_id = to_underlying(ServiceIndex::PREARM_CHECK),
        .service_rr = Service_rr::Replier,
        .service_name = AP_DDS_PARAMCHECH_SRV(AP_DDS_PARTICIPANT_NAME), 
        .request_type = "std_srvs::srv::dds_::Trigger_Request_",
        .reply_type = "std_srvs::srv::dds_::Trigger_Response_",
        .request_topic_name = AP_DDS_PARAMCHECH_SRV_RQ(AP_DDS_PARTICIPANT_NAME),
        .reply_topic_name = AP_DDS_PARAMCHECH_SRV_RR(AP_DDS_PARTICIPANT_NAME),
    },
#endif // AP_DDS_ARM_CHECK_SERVER_ENABLED    
#if AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED
    {
        .req_id = to_underlying(ServiceIndex::TAKEOFF),
        .rep_id = to_underlying(ServiceIndex::TAKEOFF),
        .service_rr = Service_rr::Replier,
        .service_name = AP_DDS_TKOFF_SRV(AP_DDS_PARTICIPANT_NAME),
        .request_type = "ardupilot_msgs::srv::dds_::Takeoff_Request_",
        .reply_type = "ardupilot_msgs::srv::dds_::Takeoff_Response_",
        .request_topic_name = AP_DDS_TKOFF_SRV_RQ(AP_DDS_PARTICIPANT_NAME),
        .reply_topic_name = AP_DDS_TKOFF_SRV_RR(AP_DDS_PARTICIPANT_NAME),
    },
#endif // AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED
#if AP_DDS_PARAMETER_SERVER_ENABLED
    {
        .req_id = to_underlying(ServiceIndex::SET_PARAMETERS),
        .rep_id = to_underlying(ServiceIndex::SET_PARAMETERS),
        .service_rr = Service_rr::Replier,
        .service_name = AP_DDS_SETPARAM_SRV(AP_DDS_PARTICIPANT_NAME),
        .request_type = "rcl_interfaces::srv::dds_::SetParameters_Request_",
        .reply_type = "rcl_interfaces::srv::dds_::SetParameters_Response_",
        .request_topic_name = AP_DDS_SETPARAM_SRV_RQ(AP_DDS_PARTICIPANT_NAME),
        .reply_topic_name = AP_DDS_SETPARAM_SRV_RR(AP_DDS_PARTICIPANT_NAME),
    },
    {
        .req_id = to_underlying(ServiceIndex::GET_PARAMETERS),
        .rep_id = to_underlying(ServiceIndex::GET_PARAMETERS),
        .service_rr = Service_rr::Replier,
        .service_name = AP_DDS_GETPARAM_SRV(AP_DDS_PARTICIPANT_NAME),
        .request_type = "rcl_interfaces::srv::dds_::GetParameters_Request_",
        .reply_type = "rcl_interfaces::srv::dds_::GetParameters_Response_",
        .request_topic_name = AP_DDS_GETPARAM_SRV_RQ(AP_DDS_PARTICIPANT_NAME),
        .reply_topic_name = AP_DDS_GETPARAM_SRV_RR(AP_DDS_PARTICIPANT_NAME),
    },
#endif // AP_DDS_PARAMETER_SERVER_ENABLED
};
