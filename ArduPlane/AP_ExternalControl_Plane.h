
/*
  external control library for plane
 */
#pragma once

#include <AP_ExternalControl/AP_ExternalControl.h>

#if AP_EXTERNAL_CONTROL_ENABLED

#include <AP_Common/Location.h>

class AP_ExternalControl_Plane : public AP_ExternalControl {
public:
    /*
        Sets the target global position for a loiter point.
    */
    bool set_global_position(const Location& loc) override WARN_IF_UNUSED;
    bool set_linear_acceleration(const Vector3f &accel_mig,const Vector3f &accel_sep,const Vector3f &accel_coh,const Vector3f &accel_alig) override WARN_IF_UNUSED;

};

#endif // AP_EXTERNAL_CONTROL_ENABLED
