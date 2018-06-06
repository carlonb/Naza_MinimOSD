
//
// NAZA_Calc_Home_Altit
//
//------------------ Home Distance and Direction Calculation ----------------------------------
// Set Home pos, Altitude Over Ground, Dir to Home, Dist to Home, Heading


void Naza_setHomeAltitudeDirHome()
{
  float dstlon, dstlat;
  long bearing;

  if (osd_got_home == true) { // Home fatto ? Si
    // shrinking factor for longitude going to poles direction
    float rads = fabs(osd_home_lat) * 0.0174532925;
    double scaleLongDown = cos(rads);
    double scaleLongUp   = 1.0f/cos(rads);

    //DST to Home
    dstlat = fabs(osd_home_lat - GPS_latitude) * 111319.5;
    dstlon = fabs(osd_home_lon - GPS_longitude) * 111319.5 * scaleLongDown;
    GPS_distanceToHome = (sqrt(sq(dstlat) + sq(dstlon)));   // in mt

    MwAltitude = GPS_alt - osd_home_alt;  // Altitudine dal livello del suolo in cm
    
    //DIR to Home
    dstlat = (osd_home_lat - GPS_latitude);                 // OffSet Y
    dstlon = (osd_home_lon - GPS_longitude) * scaleLongUp;  // OffSet X
    bearing = 90 + (atan2(dstlat, - dstlon) * 57.295775);   // absolut home direction
    if (bearing < 0) bearing += 360;          // normalization
    bearing = bearing - 180;                  // absolut return direction
    if (bearing < 0) bearing += 360;          // normalization
    bearing = bearing - MwHeading;            // relative home direction (MwHeading dal GPS NAZA)
    if (bearing < 0) bearing += 360;          // normalization   
    GPS_directionToHome = bearing;            // Dato passato in screen.ino per Dir to Home

  } else { // Home fatto ? No   
    // criteria for a stable home position and stable ground altitude:
    //  - GPS fix
    //  - with at least 9 satellites
    //  - osd_alt stable for 100 * 50ms = 5s (timed routine di 50ms)
    //  - osd_alt stable means the delta is lower 100 cm
    if ((GPS_fix == 2 || GPS_fix == 3) && GPS_numSat >= 9 && osd_alt_cnt < 100) {
      if (fabs(osd_alt_prev - GPS_alt) > 100) {
        osd_alt_cnt = 0;
        osd_alt_prev = GPS_alt;
      } else {
        if (++osd_alt_cnt >= 100) {
          osd_home_lat = GPS_latitude;    // take this osd_lat as osd_home_lat
          osd_home_lon = GPS_longitude;   // take this osd_lon as osd_home_lon
          osd_home_alt = GPS_alt;         // take this stable osd_alt as osd_home_alt in cm
          osd_got_home = true;
        }
      }
    }
  }
}

