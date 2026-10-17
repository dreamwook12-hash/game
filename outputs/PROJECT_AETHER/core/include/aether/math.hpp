#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aether {
inline constexpr double pi = 3.141592653589793238462643383279502884;
inline constexpr double gravity = 9.80665;

// Plain SI value type. Validate at API boundaries; arithmetic follows IEEE 754.
struct Vec3 {
    double x=0, y=0, z=0;
    constexpr Vec3 operator+(Vec3 b) const { return {x+b.x,y+b.y,z+b.z}; }
    constexpr Vec3 operator-(Vec3 b) const { return {x-b.x,y-b.y,z-b.z}; }
    constexpr Vec3 operator-() const { return {-x,-y,-z}; }
    constexpr Vec3 operator*(double s) const { return {x*s,y*s,z*s}; }
    constexpr Vec3 operator/(double s) const { return {x/s,y/s,z/s}; }
    constexpr Vec3& operator+=(Vec3 b) { x+=b.x; y+=b.y; z+=b.z; return *this; }
    constexpr Vec3& operator-=(Vec3 b) { x-=b.x; y-=b.y; z-=b.z; return *this; }
    constexpr Vec3& operator*=(double s) { x*=s; y*=s; z*=s; return *this; }
};
inline constexpr Vec3 operator*(double s, Vec3 v) { return v*s; }
inline constexpr double dot(Vec3 a, Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline constexpr Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
inline double norm(Vec3 v) { return std::hypot(v.x,v.y,v.z); }
inline bool finite(Vec3 v) { return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z); }

struct Quaternion {
    double w=1, x=0, y=0, z=0;
    Quaternion normalized() const {
        if (!std::isfinite(w)||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z))
            throw std::invalid_argument("Quaternion must be finite");
        const double scale=std::max({std::abs(w),std::abs(x),std::abs(y),std::abs(z)});
        if (scale==0) throw std::invalid_argument("Zero quaternion has no orientation");
        const double a=w/scale,b=x/scale,c=y/scale,d=z/scale;
        const double length=std::sqrt(a*a+b*b+c*c+d*d);
        return {a/length,b/length,c/length,d/length};
    }
    constexpr Quaternion conjugate() const { return {w,-x,-y,-z}; }
    constexpr Quaternion operator*(Quaternion b) const {
        return {w*b.w-x*b.x-y*b.y-z*b.z,
                w*b.x+x*b.w+y*b.z-z*b.y,
                w*b.y-x*b.z+y*b.w+z*b.x,
                w*b.z+x*b.y-y*b.x+z*b.w};
    }
    Vec3 rotate(Vec3 v) const {
        if (!finite(v)) throw std::invalid_argument("Rotation vector must be finite");
        const Quaternion q=normalized();
        const Vec3 qv{q.x,q.y,q.z};
        const Vec3 t=2.0*cross(qv,v);
        const Vec3 result=v+q.w*t+cross(qv,t);
        if (!finite(result)) throw std::overflow_error("Rotation result overflow");
        return result;
    }
    // Intrinsic roll/pitch/yaw in radians: Rz(yaw) Ry(pitch) Rx(roll).
    // Maps body forward/right/down vectors into local north/east/down.
    static Quaternion fromEuler(double roll, double pitch, double yaw) {
        if (!std::isfinite(roll)||!std::isfinite(pitch)||!std::isfinite(yaw))
            throw std::invalid_argument("Euler angles must be finite");
        const double cr=std::cos(roll/2),sr=std::sin(roll/2);
        const double cp=std::cos(pitch/2),sp=std::sin(pitch/2);
        const double cy=std::cos(yaw/2),sy=std::sin(yaw/2);
        return Quaternion{cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,
                          cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy}.normalized();
    }
};
inline bool finite(Quaternion q) {
    return std::isfinite(q.w)&&std::isfinite(q.x)&&std::isfinite(q.y)&&std::isfinite(q.z);
}
} // namespace aether
