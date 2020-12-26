#include <offboard/offboard.h>

#include <geometry_msgs/Point.h>
#include <geographic_msgs/GeoPoint.h>

/***********************  CONSTANTS  ***********************/
const double a = 6378137.0;         // WGS-84 Earth semimajor axis (m)
const double b = 6356752.314245;     // Derived Earth semiminor axis (m)
const double f = (a - b) / a;           // Ellipsoid Flatness
const double f_inv = 1.0 / f;       // Inverse flattening

//const double f_inv = 298.257223563; // WGS-84 Flattening Factor of the Earth 
//const double b = a - a / f_inv;
//const double f = 1.0 / f_inv;

const double a_sq = a * a;
const double b_sq = b * b;
const double e_sq = f * (2 - f);    // Square of Eccentricity

/***********************               FUNCTIONS              ***********************/
/* WGS84ToECEF: Converts the WGS-84 Geodetic point (latitude, longitude, altitude) **
** to Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z)                      **/
geometry_msgs::Point WGS84ToECEF(double, double, double);

/* ECEFToWGS84: Converts the Earth-Centered Earth-Fixed coordinates (x, y, z) to   **
** WGS-84 Geodetic point (latitude, longitude, altitude)                           **/
geographic_msgs::GeoPoint ECEFToWGS84(double, double, double);

/* ECEFToENU: Converts the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z) **
** to East-North-Up coordinates in a Local Tangent Plane that is centered at the   **
** (WGS-84) Geodetic point (lat0, lon0, alt0)                                      **/
geometry_msgs::Point ECEFToENU(double, double, double, double, double, double);

/* ENUToECEF: Converts East-North-Up coordinates (xEast, yNorth, zUp) in a Local   **
** Tangent Plane that is centered at  (WGS-84) Geodetic point (lat0, lon0, alt0)   **
** to the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z)                  **/
geometry_msgs::Point ENUToECEF(double, double, double, double, double, double);

/* WGS84ToENU: Converts the geodetic WGS-84 coordinated (lat, lon, alt) to         **
** East-North-Up coordinates in a Local Tangent Plane that is centered at the      **
** (WGS-84) Geodetic point (lat0, lon0, alt0)                                      **/
geometry_msgs::Point WGS84ToENU(double, double, double, double, double, double);

/* ENUToWGS84: Converts the East-North-Up coordinates in a Local Tangent Plane to  **
** geodetic WGS-84 coordinated (lat, lon, alt) that is centered at the (WGS-84)    **
** Geodetic point (lat0, lon0, alt0)                                               **/
geographic_msgs::GeoPoint ENUToWGS84(double, double, double, double, double, double);

// VARIANTS 
geometry_msgs::PoseStamped goal_pose; //Local goal position setpoint
geometry_msgs::Point enu_goal, enu_curr; //Local ENU points: converted from GPS goal and current
geographic_msgs::GeoPoint refpoint; //Reference point to convert ECEF to ENU and vice versa

geometry_msgs::Point WGS84ToECEF(double lat, double lon, double alt)
{
    geometry_msgs::Point ecef;
    double lambda = radian(lat);
    double phi = radian(lon);
    double s = sin(lambda);
    double N = a / sqrt(1 - e_sq * s * s);

    double sin_lambda = sin(lambda);
    double cos_lambda = cos(lambda);
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);

    ecef.x = (alt + N) * cos_lambda * cos_phi;
    ecef.y = (alt + N) * cos_lambda * sin_phi;
    ecef.z = (alt + (1 - e_sq) * N) * sin_lambda;

    return ecef;
}

geographic_msgs::GeoPoint ECEFToWGS84(double x, double y, double z)
{
    geographic_msgs::GeoPoint wgs84;
    double eps = e_sq / (1.0 - e_sq);
    double p = sqrt(x * x + y * y);
    double q = atan2((z * a), (p * b));
    double sin_q = sin(q);
    double cos_q = cos(q);
    double sin_q_3 = sin_q * sin_q * sin_q;
    double cos_q_3 = cos_q * cos_q * cos_q;
    double phi = atan2((z + eps * b * sin_q_3), (p - e_sq * a * cos_q_3));
    double lambda = atan2(y, x);
    double v = a / sqrt(1.0 - e_sq * sin(phi) * sin(phi));
    
    wgs84.altitude = (p / cos(phi)) - v;

    wgs84.latitude = degree(phi);
    wgs84.longitude = degree(lambda);

    return wgs84;
}

geometry_msgs::Point ECEFToENU(double x, double y, double z,
                               double lat0, double lon0, double alt0)
{
    geometry_msgs::Point enu;
    double lambda = radian(lat0);
    double phi = radian(lon0);
    double s = sin(lambda);
    double N = a / sqrt(1 - e_sq * s * s);

    double sin_lambda = sin(lambda);
    double cos_lambda = cos(lambda);
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);

    double x0 = (alt0 + N) * cos_lambda * cos_phi;
    double y0 = (alt0 + N) * cos_lambda * sin_phi;
    double z0 = (alt0 + (1 - e_sq) * N) * sin_lambda;

    double xd, yd, zd;
    xd = x - x0;
    yd = y - y0;
    zd = z - z0;

    enu.x = -sin_phi * xd + cos_phi * yd;
    enu.y = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
    enu.z = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;

    return enu;
}

geometry_msgs::Point ENUToECEF(double xEast, double yNorth, double zUp,
                                double lat0, double lon0, double alt0)
{
    geometry_msgs::Point ecef;
    double lambda = radian(lat0);
    double phi = radian(lon0);
    double s = sin(lambda);
    double N = a / sqrt(1 - e_sq * s * s);

    double sin_lambda = sin(lambda);
    double cos_lambda = cos(lambda);
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);

    double x0 = (alt0 + N) * cos_lambda * cos_phi;
    double y0 = (alt0 + N) * cos_lambda * sin_phi;
    double z0 = (alt0 + (1 - e_sq) * N) * sin_lambda;

    double xd = -sin_phi * xEast - cos_phi * sin_lambda * yNorth + cos_lambda * cos_phi * zUp;
    double yd = cos_phi * xEast - sin_lambda * sin_phi * yNorth + cos_lambda * sin_phi * zUp;
    double zd = cos_lambda * yNorth + sin_lambda * zUp;

    ecef.x = xd + x0;
    ecef.y = yd + y0;
    ecef.z = zd + z0;

    return ecef;
}

geometry_msgs::Point WGS84ToENU(double lat, double lon, double alt,
                                double lat0, double lon0, double alt0)
{
    geometry_msgs::Point ecef = WGS84ToECEF(lat, lon, alt);
    geometry_msgs::Point enu = ECEFToENU(ecef.x, ecef.y, ecef.z,
                                        lat0, lon0, alt0);
    return enu;
}

geographic_msgs::GeoPoint ENUToWGS84(double xEast, double yNorth, double zUp,
                                      double lat0, double lon0, double alt0)
{
    geometry_msgs::Point ecef = ENUToECEF(xEast, yNorth, zUp,
                                          lat0, lon0, alt0);
    geographic_msgs::GeoPoint wgs84 = ECEFToWGS84(ecef.x, ecef.y, ecef.z);

    return wgs84;
}