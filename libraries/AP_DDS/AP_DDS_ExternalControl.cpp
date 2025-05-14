#include "AP_DDS_config.h"

#if AP_DDS_ENABLED

#include "AP_DDS_ExternalControl.h"
#include "AP_DDS_Frames.h"
#include <AP_AHRS/AP_AHRS.h>

#include <AP_ExternalControl/AP_ExternalControl.h>

// These are the Goal Interface constants. Because microxrceddsgen does not expose
// them in the generated code, they are manually maintained
// Position ignore flags
static constexpr uint16_t TYPE_MASK_IGNORE_LATITUDE = 1;
static constexpr uint16_t TYPE_MASK_IGNORE_LONGITUDE = 2;
static constexpr uint16_t TYPE_MASK_IGNORE_ALTITUDE = 4;

bool AP_DDS_External_Control::handle_global_position_control(ardupilot_msgs_msg_GlobalPosition& cmd_pos)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    if (strcmp(cmd_pos.header.frame_id, MAP_FRAME) == 0) {
        // Narrow the altitude
        const int32_t alt_cm  = static_cast<int32_t>(cmd_pos.altitude * 100);

        Location::AltFrame alt_frame;
        if (!convert_alt_frame(cmd_pos.coordinate_frame, alt_frame)) {
            return false;
        }

        constexpr uint32_t MASK_POS_IGNORE =
            TYPE_MASK_IGNORE_LATITUDE |
            TYPE_MASK_IGNORE_LONGITUDE |
            TYPE_MASK_IGNORE_ALTITUDE;

        if (!(cmd_pos.type_mask & MASK_POS_IGNORE)) {
            Location loc(cmd_pos.latitude * 1E7, cmd_pos.longitude * 1E7, alt_cm, alt_frame);
            if (!external_control->set_global_position(loc)) {
                return false; // Don't try sending other commands if this fails
            }
        }

        // TODO add velocity and accel handling

        return true;
    }

    return false;
}

bool AP_DDS_External_Control::handle_velocity_control(geometry_msgs_msg_TwistStamped& cmd_vel)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    if (strcmp(cmd_vel.header.frame_id, BASE_LINK_FRAME_ID) == 0) {
        // Convert commands from body frame (x-forward, y-left, z-up) to NED.
        Vector3f linear_velocity;
        Vector3f linear_velocity_base_link {
            float(cmd_vel.twist.linear.x),
            float(cmd_vel.twist.linear.y),
            float(-cmd_vel.twist.linear.z) };
        const float yaw_rate = -cmd_vel.twist.angular.z;

        auto &ahrs = AP::ahrs();
        linear_velocity = ahrs.body_to_earth(linear_velocity_base_link);
        return external_control->set_linear_velocity_and_yaw_rate(linear_velocity, yaw_rate);
    }

    else if (strcmp(cmd_vel.header.frame_id, MAP_FRAME) == 0) {
        // Convert commands from ENU to NED frame
        Vector3f linear_velocity {
            float(cmd_vel.twist.linear.y),
            float(cmd_vel.twist.linear.x),
            float(-cmd_vel.twist.linear.z) };
        const float yaw_rate = -cmd_vel.twist.angular.z;
        return external_control->set_linear_velocity_and_yaw_rate(linear_velocity, yaw_rate);
    }

    return false;
}
bool AP_DDS_External_Control::handle_accel_control(ardupilot_msgs_msg_Accel& cmd_acc)
{

    //gcs().send_text(MAV_SEVERITY_CRITICAL, "veri: %f", (double)cmd_acc.accel_mig.linear.x);
    
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    
        // commands from ENU frame
        Vector3f accel_mig {
            float(cmd_acc.accel_mig.linear.y),
            float(cmd_acc.accel_mig.linear.x),
            float(cmd_acc.accel_mig.linear.z)};
         Vector3f accel_sep {
            float(cmd_acc.accel_sep.linear.y),
            float(cmd_acc.accel_sep.linear.x),
            float(cmd_acc.accel_sep.linear.z)};   
        Vector3f accel_coh {
            float(cmd_acc.accel_coh.linear.y),
            float(cmd_acc.accel_coh.linear.x),
            float(cmd_acc.accel_coh.linear.z)};
        Vector3f accel_alig {
            float(cmd_acc.accel_alig.linear.y),
            float(cmd_acc.accel_alig.linear.x),
            float(cmd_acc.accel_alig.linear.z)};
    
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "mig_x: %f", (double)cmd_acc.accel_mig.linear.x);

    gcs().send_text(MAV_SEVERITY_CRITICAL, "mig_z: %f", (double)cmd_acc.accel_mig.linear.z);
    
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "sep_x: %f", (double)cmd_acc.accel_sep.linear.x);
    
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "sep_y: %f", (double)cmd_acc.accel_sep.linear.y);
    
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "coh_x: %f", (double)cmd_acc.accel_coh.linear.x);
    
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "coh_y: %f", (double)cmd_acc.accel_coh.linear.y);
    
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "alig_x: %f", (double)cmd_acc.accel_alig.linear.x);
    
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "alig_y: %f", (double)cmd_acc.accel_alig.linear.y);

    return external_control->set_linear_acceleration(accel_mig, accel_sep, accel_coh, accel_alig);

}
bool AP_DDS_External_Control::arm(AP_Arming::Method method, bool do_arming_checks)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    return external_control->arm(method, do_arming_checks);
}

bool AP_DDS_External_Control::disarm(AP_Arming::Method method, bool do_disarm_checks)
{
    auto *external_control = AP::externalcontrol();
    if (external_control == nullptr) {
        return false;
    }

    return external_control->disarm(method, do_disarm_checks);
}

bool AP_DDS_External_Control::convert_alt_frame(const uint8_t frame_in,  Location::AltFrame& frame_out)
{

    // Specified in ROS REP-147; only some are supported.
    switch (frame_in) {
    case 5: // FRAME_GLOBAL_INT
        frame_out = Location::AltFrame::ABSOLUTE;
        break;
    case 6: // FRAME_GLOBAL_REL_ALT
        frame_out = Location::AltFrame::ABOVE_HOME;
        break;
    case 11: // FRAME_GLOBAL_TERRAIN_ALT
        frame_out = Location::AltFrame::ABOVE_TERRAIN;
        break;
    default:
        return false;
    }
    return true;
}


#endif // AP_DDS_ENABLED