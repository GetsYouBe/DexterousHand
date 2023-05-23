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
    double m[3][3];//回転行列
    // 回転行列へ変換
    
    double tmp1;
    double tmp2;

    tmp1 = qua.w() * qua.w();//ww
    tmp2 = qua.x() * qua.x();//xx

    m[0][0] = 2.0 * (tmp1 + tmp2) - 1.0;//2.0 * (ww + xx) - 1.0;

    tmp2 = qua.y() * qua.y();//yy
    
    m[1][1] = 2.0 * (tmp1 + tmp2) - 1.0;//2.0 * (ww + yy) - 1.0;

    tmp2 = qua.z() * qua.z();//zz

    m[2][2] = 2.0 * (tmp1 + tmp2) - 1.0;//2.0 * (ww + zz) - 1.0;

    tmp1 = qua.x() * qua.y();//xy
    tmp2 = qua.z() * qua.w();//zw

    m[0][1] = 2.0 * (tmp1 - tmp2);//2.0 * (xy - zw);
    m[1][0] = 2.0 * (tmp1 + tmp2);//2.0 * (xy + zw);

    tmp1 = qua.x() * qua.z();//xz
    tmp2 = qua.y() * qua.w();//yw

    m[0][2] = 2.0 * (tmp1 + tmp2);//2.0 * (xz + yw);
    m[2][0] = 2.0 * (tmp1 - tmp2);//2.0 * (xz - yw);

    tmp1 = qua.y() * qua.z();//yz
    tmp2 = qua.x() * qua.w();//xw

    m[1][2] = 2.0 * (tmp1 - tmp2);//2.0 * (yz - xw);
    m[2][1] = 2.0 * (tmp1 + tmp2);//2.0 * (yz + xw);

    // m[0][0] = 2.0 * (ww + xx) - 1.0;
    // m[0][1] = 2.0 * (xy - zw);
    // m[0][2] = 2.0 * (xz + yw);
    // m[1][0] = 2.0 * (xy + zw);
    // m[1][1] = 2.0 * (ww + yy) - 1.0;
    // m[1][2] = 2.0 * (yz - xw);
    // m[2][0] = 2.0 * (xz - yw);
    // m[2][1] = 2.0 * (yz + xw);
    // m[2][2] = 2.0 * (ww + zz) - 1.0;

    imu::Vector<3> result;//返り値
    
    //回転行列からオイラー角に変換　回転順：XZY
    result.z() = asin(-m[0][1]);
    
    if(cos(result.z()) != 0.0){
        result.x() = atan2(m[2][1],m[1][1]);
        result.y() = atan2(m[0][2],m[0][0]);
    }else{
        result.x() = atan2(-m[1][2],m[2][2]);
        result.y() = 0;
    }
    
    return result;//返す
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