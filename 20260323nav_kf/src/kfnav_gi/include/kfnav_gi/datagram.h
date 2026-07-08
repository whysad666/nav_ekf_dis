
#pragma once
#pragma pack(1)

using pvat_t = struct {
    int year;
    int month;
    int day;
    float second;

    float q0;
    float q1;
    float q2;
    float q3;

    float roll;
    float pitch;
    float yaw;

    float vel_n;
    float vel_e;
    float vel_d;

    float pos_x;
    float pos_y;
    float pos_z;

    float longitude;
    float latitude;
    float altitude;

    int navstate;
};

#pragma pack()
