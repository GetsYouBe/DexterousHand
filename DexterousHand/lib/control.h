#ifndef _CONTROL_
#define _CONTROL_

#include <Arduino.h>
#include <Adafruit_BNO055.h>
//#include <cmath>

#define XZY 2


imu::Quaternion  diffQuaterniopn(imu::Quaternion A, imu::Quaternion B){//差分を求める
    imu::Quaternion diff;
    A.normalize();
    diff = A.conjugate() * B;
    diff.normalize();
    return diff;
}

imu::Vector<3> convertEuler(imu::Quaternion qua){
    double xx = qua.x() * qua.x();
    double xy = qua.x() * qua.y();
    double xz = qua.x() * qua.z();
    double xw = qua.x() * qua.w();

    double yy = qua.y() * qua.y();
    double yz = qua.y() * qua.z();
    double yw = qua.y() * qua.w();

    double zz = qua.z() * qua.z();
    double zw = qua.z() * qua.w();

    double ww = qua.w() * qua.w();

    double m[3][3];//回転行列
    // 回転行列へ変換
    m[0][0] = 2.0 * (ww + xx) - 1.0;
    m[0][1] = 2.0 * (xy - zw);
    m[0][2] = 2.0 * (xz + yw);
    m[1][0] = 2.0 * (xy + zw);
    m[1][1] = 2.0 * (ww + yy) - 1.0;
    m[1][2] = 2.0 * (yz - xw);
    m[2][0] = 2.0 * (xz - yw);
    m[2][1] = 2.0 * (yz + xw);
    m[2][2] = 2.0 * (ww + zz) - 1.0;

    imu::Vector<3> result;
    return result;
}

imu::Vector<3> convert(imu::Quaternion q){
    imu::Vector<3> result;

    float m00 = 2.0 * (q.w() * q.w() + q.x() * q.x()) - 1.0;
    float m01 = 2.0 * (q.x() * q.y() - q.z() * q.w());
    float m02 = 2.0 * (q.x() * q.z() + q.y() * q.w());
    float m11 = 2.0 * (q.w() * q.w() + q.y() * q.y()) - 1.0;
    float m12 = 2.0 * (q.y() * q.z() - q.x() * q.w());
    float m21 = 2.0 * (q.y() * q.z() + q.x() * q.w());
    float m22 = 2.0 * (q.w() * q.w() + q.z() * q.z()) - 1.0;

    if(m01 > 1) m01 = 1.0;
    else if(m01 < -1) m01 = -1.0;

    result.z() = asin(-m01);

    if(result.z() != 0){
        result.x() = atan2(m21,m11);
        result.y() = atan2(m02,m00);
    }else{
        result.x() = atan2(-m12,m22);
        result.y() = 0;
    }

    return result;
}   




#endif