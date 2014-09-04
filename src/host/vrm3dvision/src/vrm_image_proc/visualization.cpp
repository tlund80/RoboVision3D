#include "visualization.hpp"

void val2rgb(double val, int bit_level, int& r, int& g, int& b)
{
    double      hh, q, t, ff;
    long        i;
    double max_val = pow(2.0,bit_level);

    if(val < 0.0) {
        r = 0;
        g = 0;
        b = 0;
        return;
    }
    hh = val;
    if(hh >= max_val) hh = 0.0;
    hh /= max_val/5.0;
    i = (long)hh;
    ff = (hh - i) * 255.0;
    q = 255.0 - ff;
    t = 255.0 - (255.0 - ff);

    switch((i+1)) {
    case 0:
        r = 255;
        g = t;
        b = 0;
        break;
    case 1:
        r = q;
        g = 255;
        b = 0;
        break;
    case 2:
        r = 0;
        g = 255;
        b = t;
        break;

    case 3:
        r = 0;
        g = q;
        b = 255;
        break;
    case 4:
        r = t;
        g = 0;
        b = 255;
        break;
    case 5:
    default:
        r = 255;
        g = 0;
        b = q;
        break;
    }
    return;
}
