/*################################################################################
  ##
  ##   Copyright (C) 2016-2023 Keith O'Hara
  ##
  ##   This file is part of the OptimLib C++ library.
  ##
  ##   Licensed under the Apache License, Version 2.0 (the "License");
  ##   you may not use this file except in compliance with the License.
  ##   You may obtain a copy of the License at
  ##
  ##       http://www.apache.org/licenses/LICENSE-2.0
  ##
  ##   Unless required by applicable law or agreed to in writing, software
  ##   distributed under the License is distributed on an "AS IS" BASIS,
  ##   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  ##   See the License for the specific language governing permissions and
  ##   limitations under the License.
  ##
  ################################################################################*/

#pragma once

#include <algorithm>
#include <numeric>
#include <random>
#include <vector>

// version

#ifndef OPTIM_VERSION_MAJOR
    #define OPTIM_VERSION_MAJOR 3
#endif

#ifndef OPTIM_VERSION_MINOR
    #define OPTIM_VERSION_MINOR 1
#endif

#ifndef OPTIM_VERSION_PATCH
    #define OPTIM_VERSION_PATCH 3
#endif

//

//

#if defined(_OPENMP) && !defined(OPTIM_DONT_USE_OPENMP)
    #undef OPTIM_USE_OPENMP
    #define OPTIM_USE_OPENMP
#endif

#if !defined(_OPENMP) && defined(OPTIM_USE_OPENMP)
    #undef OPTIM_USE_OPENMP

    #undef OPTIM_DONT_USE_OPENMP
    #define OPTIM_DONT_USE_OPENMP
#endif

// #ifdef OPTIM_USE_OPENMP
    // #include "omp.h" //  OpenMP
// #endif

#ifdef OPTIM_DONT_USE_OPENMP
    #ifdef OPTIM_USE_OPENMP
        #undef OPTIM_USE_OPENMP
    #endif
#endif

//

#ifndef optimlib_inline
    #define optimlib_inline 
#endif

// floating point number type

#define OPTIM_FPN_TYPE double
#define OPTIM_FPN_SMALL_NUMBER fp_t(1e-08)


//

namespace optim
{
    using uint_t = unsigned int;
    using fp_t = OPTIM_FPN_TYPE;

    using rand_engine_t = std::mt19937_64;

    static const double eps_dbl = std::numeric_limits<fp_t>::epsilon();
    static const double inf = std::numeric_limits<fp_t>::infinity();
}

//


#include <iostream>
#include <Eigen/Dense>

// template<typename eT, int iTr, int iTc>
// using EigenMat = Eigen::Matrix<eT,iTr,iTc>;

namespace optim
{
    using Mat_t = Eigen::Matrix<fp_t, Eigen::Dynamic, Eigen::Dynamic>;

    using ColVec_t = Eigen::Matrix<fp_t, Eigen::Dynamic, 1>;
    using RowVec_t = Eigen::Matrix<fp_t, 1, Eigen::Dynamic>;

    using ColVecInt_t = Eigen::Matrix<int, Eigen::Dynamic, 1>;
    using RowVecInt_t = Eigen::Matrix<int, 1, Eigen::Dynamic>;

    using ColVecUInt_t = Eigen::Matrix<size_t, Eigen::Dynamic, 1>;
    using RowVecUInt_t = Eigen::Matrix<size_t, 1, Eigen::Dynamic>;
}


//

#ifndef BMO_ENABLE_EXTRA_FEATURES
    #define BMO_ENABLE_EXTRA_FEATURES
#endif

#ifndef BMO_ENABLE_STATS_FEATURES
    #define BMO_ENABLE_STATS_FEATURES
#endif

#ifndef BMO_RNG_ENGINE_TYPE
    #define BMO_RNG_ENGINE_TYPE optim::rand_engine_t
#endif

#ifndef BMO_CORE_TYPES
    #define BMO_CORE_TYPES

    namespace bmo
    {
        using fp_t = OPTIM_FPN_TYPE;

        using ColVec_t = optim::ColVec_t;
        using RowVec_t = optim::RowVec_t;
        using ColVecInt_t = optim::ColVecInt_t;
        using RowVecInt_t = optim::RowVecInt_t;
        using ColVecUInt_t = optim::ColVecUInt_t;

        using Mat_t = optim::Mat_t;
    }
#endif

#define BMO_MATOPS_SET_VALUES_SCALAR(x,a) (x).setConstant(a)
#define BMO_MATOPS_COUT std::cout
#define BMO_MATOPS_TRANSPOSE_INPLACE(x) (x).transpose()
#define BMO_MATOPS_SIZE(x) static_cast<size_t>((x).size())
#define BMO_MATOPS_EYE(n) bmo::Mat_t::Identity(n,n)
#define BMO_MATOPS_CONSTANT_COLVEC(n,a) bmo::ColVec_t::Constant(n,a)
#define BMO_MATOPS_ABS(x) (x).cwiseAbs()
#define BMO_MATOPS_MAX(x,y) (x).array().max((y).array())
#define BMO_MATOPS_ZERO_COLVEC(n) bmo::ColVec_t::Zero(n)
#define BMO_MATOPS_ONE_COLVEC(n) bmo::ColVec_t::Ones(n)
#define BMO_MATOPS_ZERO_MAT(n,k) bmo::Mat_t::Zero(n,k)
#define BMO_MATOPS_DOT_PROD(x,y) (x).dot(y)
#define BMO_MATOPS_EXTRACT_DIAG(x) (x).diagonal()
#define BMO_MATOPS_HADAMARD_PROD(x,y) (x).cwiseProduct(y)
#define BMO_MATOPS_IS_FINITE(x) static_cast<bool>((( (x - (x)).array() == (x - (x)).array() )).all())
#define BMO_MATOPS_L1NORM(x) (x).array().abs().sum()
#define BMO_MATOPS_L2NORM(x) (x).norm()
#define BMO_MATOPS_ARRAY_ADD_SCALAR(x,a) ((x).array() + (a)).matrix()
#define BMO_MATOPS_ARRAY_DIV_ARRAY(x,y)  ((x).array() / (y).array()).matrix()
#define BMO_MATOPS_TRANSPOSE(x) (x).transpose()
#define BMO_MATOPS_TRANSPOSE_INPLACE(x) (x).transpose()
