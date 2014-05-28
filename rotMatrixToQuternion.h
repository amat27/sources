#include <math.h>
#include <iostream>

struct Quaternion
{
    double x;
    double y;
    double z;
    double w;
};

struct rotMatrix
{
    double m11, m12, m13;
    double m21, m22, m23;
    double m31, m32, m33;
};


//fuction:旋转矩阵到四元数的转换
void rotMatrixToQuternion(Quaternion &q, rotMatrix &r)
{
    double tr = r.m11 + r.m22 +r.m33;
    double temp = 0.0;
    if(tr > 0.0)
    {
        temp = 0.5f / sqrtf(tr+1);
        q.w = 0.25f / temp;
        q.x = (r.m23 - r.m32) * temp;
        q.y = (r.m31 - r.m13) * temp;
        q.z = (r.m12 - r.m21) * temp;
    }
    else
    {
        if(r.m11 > r.m22 && r.m11 > r.m33)
        {
            temp = 2.0f * sqrtf(1.0f + r.m11 - r.m22 - r.m33);
            q.w = (r.m32 - r.m23) / temp;
            q.x = 0.25f * temp;
            q.y = (r.m12 + r.m21) / temp;
            q.z = (r.m13 + r.m31) / temp;
        }
        else if( r.m22 > r.m33)
        {
            temp = 2.0f * sqrtf(1.0f + r.m22 - r.m11 - r.m33);
            q.w = (r.m13 - r.m31) / temp;
            q.x = (r.m12 + r.m21) / temp;
            q.y =  0.25f * temp;
            q.z = (r.m23 + r.m32) / temp;
        }
        else
        {
            temp = 2.0f * sqrtf(1.0f + r.m33 - r.m11 - r.m22);
            q.w = (r.m21 - r.m12) / temp;
            q.x = (r.m13 + r.m31) / temp;
            q.y = (r.m23 + r.m32) / temp;
            q.z = 0.25f * temp;
        }
    }
}

