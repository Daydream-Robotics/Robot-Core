#ifndef _RNG_HPP_
#define _RNG_HPP_

#include <cstdint>
#include <cmath>

namespace rng {

class xoshiro128plus {
private:
    uint32_t s[4];
    bool has_spare;
    double spare;

    static inline uint32_t rotl(uint32_t x, int k) {
        return (x << k) | (x >> (32 - k));
    }

public:
    xoshiro128plus(uint32_t seed) : has_spare(false), spare(0.0) {
        s[0] = seed;
        s[1] = seed * 1812433253U + 1;
        s[2] = seed * 1812433253U + 2;
        s[3] = seed * 1812433253U + 3;
    }
    
    uint32_t next() {
        const uint32_t result = s[0] + s[3];
        const uint32_t t = s[1] << 9;
        
        s[2] ^= s[0];
        s[3] ^= s[1];
        s[1] ^= s[2];
        s[0] ^= s[3];
        
        s[2] ^= t;
        s[3] = rotl(s[3], 11);
        
        return result;
    }
    
    double uniform() {
        return (next() >> 8) * 0x1.0p-24;
    }
    
    double normal(double mean, double stddev) {
        if (has_spare) {
            has_spare = false;
            return mean + stddev * spare;
        }

        has_spare = true;
        double u, v, s;
        do {
            u = uniform() * 2.0 - 1.0;
            v = uniform() * 2.0 - 1.0;
            s = u * u + v * v;
        } while (s >= 1.0 || s == 0.0);

        s = std::sqrt(-2.0 * std::log(s) / s);
        spare = v * s;
        return mean + stddev * u * s;
    }
};

}

#endif

