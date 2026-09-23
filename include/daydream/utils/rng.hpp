#ifndef _RNG_HPP_
#define _RNG_HPP_

//Inclusions
#include <cstdint>
#include <cmath>

//Created namespace rng
namespace rng {

//fast, small-state pseudo random number generator (xoshiro128+)
//used by MCL for particle noise and resampling, much cheaper than std::mt19937 on the brain
//not cryptographically secure, only meant for simulation/noise
class Xoshiro128Plus {
private:
    //128-bit generator state split into four 32-bit words
    uint32_t m_state[4];
    //true if a second normal sample from the last Box-Muller pair is cached
    bool m_hasSpare;
    //cached second normal sample (standard normal, mean 0 stddev 1)
    double m_spare;

    //rotates x left by k bits
    static inline uint32_t rotl(uint32_t x, int k) {
        return (x << k) | (x >> (32 - k));
    }

public:
    //seeds the generator state from a single 32-bit value
    //each state word is derived from the seed so the state is never all zeros
    Xoshiro128Plus(uint32_t seed) : m_hasSpare(false), m_spare(0.0) {
        m_state[0] = seed;
        m_state[1] = seed * 1812433253U + 1;
        m_state[2] = seed * 1812433253U + 2;
        m_state[3] = seed * 1812433253U + 3;
    }

    //advances the state and returns the next raw 32-bit random number
    uint32_t next() {
        //output is the sum of the first and last state words
        const uint32_t result = m_state[0] + m_state[3];
        const uint32_t t = m_state[1] << 9;

        //mix the state words together
        m_state[2] ^= m_state[0];
        m_state[3] ^= m_state[1];
        m_state[1] ^= m_state[2];
        m_state[0] ^= m_state[3];

        m_state[2] ^= t;
        m_state[3] = rotl(m_state[3], 11);

        return result;
    }

    //returns a uniform random double in [0, 1)
    //uses the top 24 bits (lowest bits of xoshiro128+ are weaker) scaled by 2^-24
    double uniform() {
        return (next() >> 8) * 0x1.0p-24;
    }

    //returns a normally distributed random double with the given mean and stddev
    //uses the Marsaglia polar method (Box-Muller variant), which makes two samples at once
    //the second sample is cached and returned on the next call
    double normal(double mean, double stddev) {
        //use the cached sample from the previous call if there is one
        if (m_hasSpare) {
            m_hasSpare = false;
            return mean + stddev * m_spare;
        }

        m_hasSpare = true;
        //s is the squared radius of the random point
        double u, v, s;
        //pick a random point inside the unit circle (reject points outside or at the center)
        do {
            u = uniform() * 2.0 - 1.0;
            v = uniform() * 2.0 - 1.0;
            s = u * u + v * v;
        } while (s >= 1.0 || s == 0.0);

        //scale the point to get two independent standard normal samples (u*s and v*s)
        s = std::sqrt(-2.0 * std::log(s) / s);
        //cache one sample for the next call, return the other
        m_spare = v * s;
        return mean + stddev * u * s;
    }
};

}

#endif
