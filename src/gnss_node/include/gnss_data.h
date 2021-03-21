//
// Created by eagleflag on 2021/3/20.
//

#ifndef MULTIMODALEVNPERCPTION_GNSS_DATA_H
#define MULTIMODALEVNPERCPTION_GNSS_DATA_H

struct gnss_data
{
    double time = 0.0;
    double longitude = 0.0;
    double latitude = 0.0;
    double altitude = 0.0;
    double local_E = 0.0;
    double local_N = 0.0;
    double local_U = 0.0;
    int status = 0;
    int service = 0;

};

#endif //MULTIMODALEVNPERCPTION_GNSS_DATA_H
