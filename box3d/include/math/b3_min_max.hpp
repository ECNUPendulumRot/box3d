// This file is same as the one of bullet3
// for more information, please access:
// http://bulletphysics.org
#ifndef BOX3D_B3_MIN_MAX_HPP
#define BOX3D_B3_MIN_MAX_HPP


template <class T>
inline const T& b3_min(const T& a, const T& b)
{
    return a < b ? a : b;
}

template <class T>
inline const T& b3_max(const T& a, const T& b)
{
    return a > b ? a : b;
}

template <class T>
inline const T& b3_clamp(const T& a, const T& lb, const T& ub)
{
    return a < lb ? lb : (ub < a ? ub : a);
}


#endif //BOX3D_B3_MIN_MAX_HPP
