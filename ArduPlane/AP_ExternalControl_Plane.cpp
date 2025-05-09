/*
  external control library for plane
 */


#include "AP_ExternalControl_Plane.h"
#if AP_EXTERNAL_CONTROL_ENABLED

#include "Plane.h"

/*
  Sets the target global position for a loiter point.
*/
bool AP_ExternalControl_Plane::set_global_position(const Location& loc)
{

    // set_target_location already checks if plane is ready for external control.
    // It doesn't check if flying or armed, just that it's in guided mode.
    return plane.set_target_location(loc);
}



bool AP_ExternalControl_Plane::set_linear_acceleration(const Vector3f &accel_mig,const Vector3f &accel_sep,const Vector3f &accel_coh,const Vector3f &accel_alig)
{

  Vector3f body_accel_mig;
  Vector3f body_accel_sep;
  Vector3f body_accel_coh;
  Vector3f body_accel_alig;
  auto &ahrs = AP::ahrs();

  body_accel_mig = ahrs.earth_to_body(accel_mig);

  body_accel_sep = ahrs.earth_to_body(accel_sep);
  
  body_accel_coh = ahrs.earth_to_body(accel_coh);
  
  body_accel_alig = ahrs.earth_to_body(accel_alig);

  Vector3f total_accel;



  Vector2f body_accel_mig_converted; // migration için y ekseni bazlı kontrol geri yönde z ekseni yok daha
    if(body_accel_mig.x < 0 && body_accel_mig.y < 0){

      body_accel_mig_converted = {0,-sqrt(body_accel_mig.x*body_accel_mig.x + body_accel_mig.y*body_accel_mig.y)};

    }
    else if(body_accel_mig.x < 0 && body_accel_mig.y > 0){
      body_accel_mig_converted = {0,sqrt(body_accel_mig.x*body_accel_mig.x + body_accel_mig.y*body_accel_mig.y)};
    }
    else{
      body_accel_mig_converted = {body_accel_mig.x,body_accel_mig.y};
    } 
  //body_accel_mig_converted = {body_accel_mig.x,body_accel_mig.y}; // üstteki kısım test edilcek
  total_accel.x = body_accel_sep.x + body_accel_coh.x;
  total_accel.y = body_accel_mig_converted.y + body_accel_sep.y + body_accel_coh.y + body_accel_alig.y;
  total_accel.z = body_accel_sep.z + body_accel_coh.z + body_accel_alig.z; // body_accel_mig_converted 2D o yüzden yok burada şuan
  
  uint32_t now = AP_HAL::millis();
  
  float body_add_x_velocity = total_accel.x*((float)now-(float)plane.last_roll_override_ms)/1000.0f; // ivmenin delta t katkısı


  plane.new_airspeed_cm = (float)plane.new_airspeed_cm + body_add_x_velocity*100.0f;
  plane.new_airspeed_cm = constrain_float((float)plane.new_airspeed_cm,plane.aparm.airspeed_min*100.0f + 300.0f,plane.aparm.airspeed_max*100.0f - 300.0f); // 3 m/s offset sonra parametrik yap, sadece uzak olan uçaklar hızlanıp yavaşlasın
  plane.new_nav_roll_cd = atanf(total_accel.y/GRAVITY_MSS)*(180.0f/M_PI)*100; // cd 100 için 100 ile çarp
  plane.use_roll_override = 1;

  plane.last_roll_override_ms = now;

  gcs().send_text(MAV_SEVERITY_CRITICAL, "body mig x  %f",  (float)body_accel_mig.x );

  plane.set_mode_by_number(Mode::Number::GUIDED,ModeReason::DDS_COMMAND); // bunu komut gelmezse yaparsın 




  return true;

  
}


#endif // AP_EXTERNAL_CONTROL_ENABLED
