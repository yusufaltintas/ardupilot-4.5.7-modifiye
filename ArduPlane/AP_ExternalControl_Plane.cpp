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



  Vector3f body_accel_mig_converted; // migration için y ekseni bazlı kontrol
    if(body_accel_mig.x < 0 && body_accel_mig.y < 0){

      body_accel_mig_converted = {0,-sqrt(body_accel_mig.x*body_accel_mig.x + body_accel_mig.y*body_accel_mig.y), body_accel_mig.z}; 
    }
    else if(body_accel_mig.x < 0 && body_accel_mig.y > 0){
      body_accel_mig_converted = {0,sqrt(body_accel_mig.x*body_accel_mig.x + body_accel_mig.y*body_accel_mig.y), body_accel_mig.z};
    }
    else{
      body_accel_mig_converted = {body_accel_mig.x,body_accel_mig.y, body_accel_mig.z};
    } 
  //body_accel_mig_converted = {body_accel_mig.x,body_accel_mig.y}; // üstteki kısım test edilcek
  total_accel.x = body_accel_sep.x + body_accel_coh.x;
  total_accel.y = body_accel_mig_converted.y + body_accel_sep.y + body_accel_coh.y + body_accel_alig.y;
  
  uint32_t now = AP_HAL::millis();
  
  float dt = ((float)now-(float)plane.last_roll_override_ms)/1000.0f;

  float body_add_x_velocity = total_accel.x*dt; // ivmenin delta t katkısı
  float earth_z_acc = accel_mig.z + accel_coh.z + accel_sep.z; // earth framede irtifa hesapları
  float body_add_z_height = earth_z_acc*dt*dt*0.5f; // verilen ivmeye karşılık gelen o anlık hız değişimi ayrıca irtifa değişimi earth framedeki ivmeye göre olmalı

  //gcs().send_text(MAV_SEVERITY_CRITICAL, "change_target_altitude_cm  %d", (int)(body_add_z_height*100.0f));
  gcs().send_text(MAV_SEVERITY_CRITICAL, "target_altitude  %d",  plane.target_altitude.amsl_cm);
  //gcs().send_text(MAV_SEVERITY_CRITICAL, "body_accel_sep.z  %f",  body_accel_sep.z);
  //gcs().send_text(MAV_SEVERITY_CRITICAL, "body_accel_coh.z   %f",  body_accel_coh.z);
  //gcs().send_text(MAV_SEVERITY_CRITICAL, "body_accel_mig_converted.z  %f",  body_accel_mig_converted.z);
  //gcs().send_text(MAV_SEVERITY_CRITICAL, "total_accel.z  %f",  total_accel.z);

  
  
  if (abs(body_add_z_height*100.0f) < 0.01f){ // büyüklüğü x değerinden küçük değerler için değişiklik yapma irtifada, ileride bunun yerine lowpass kullanabiliriz
  
  body_add_z_height = 0.0f; // TODO : üst limit koy bu değere
  //plane.set_target_altitude_current(); // yükselmeyi engelliyor

  }
  
  plane.change_target_altitude((int32_t)(15.0f*body_add_z_height*100.0f)); // int cm birimine geçirilip irtifa değişimi olarak verildi


  plane.new_airspeed_cm = (float)plane.new_airspeed_cm + body_add_x_velocity*100.0f; // * 100 m to cm çevirisi için
  plane.new_airspeed_cm = constrain_float((float)plane.new_airspeed_cm,plane.aparm.airspeed_min*100.0f + 600.0f,plane.aparm.airspeed_max*100.0f - 100.0f); // offsetleri sonra parametrik yap, sadece uzak olan uçaklar hızlanıp yavaşlasın
  plane.new_nav_roll_cd = atanf(total_accel.y/GRAVITY_MSS)*(180.0f/M_PI)*100; // cd 100 için 100 ile çarp
  plane.use_roll_override = 1;
  plane.acc_dds_overriding = 1;

  plane.last_roll_override_ms = now;
  plane.last_acc_dds_ms = now; // TODO : aynı yerde kullanıyorlar last_roll_override_ms ile last_acc_dds_ms, last_roll_override_ms olanı kaldır.

  plane.set_mode_by_number(Mode::Number::GUIDED,ModeReason::DDS_COMMAND); // bunu komut gelmezse yaparsın 




  return true;

  
}


#endif // AP_EXTERNAL_CONTROL_ENABLED
