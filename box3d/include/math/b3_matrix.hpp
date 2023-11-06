// Algorithms used in this file is part of libigl
// just for easier compiling.
// For more information, please access:
// https://github.com/libigl/libigl

#ifndef BOX3D_B3_MATRIX_HPP
#define BOX3D_B3_MATRIX_HPP


#include <vector>
#include <Eigen/Sparse>

template <typename T>
inline int b3_max_size(const std::vector<T> & V)
{
    int max_size = -1;
    for(
            typename std::vector<T>::const_iterator iter = V.begin();
            iter != V.end();
            iter++)
    {
        int size = (int)iter->size();
        max_size = (max_size > size ? max_size : size);
    }
    return max_size;
}


template <typename T>
inline int b3_min_size(const std::vector<T> & V)
{
    int min_size = -1;
    for(
            typename std::vector<T>::const_iterator iter = V.begin();
            iter != V.end();
            iter++)
    {
        int size = (int)iter->size();
        // have to handle base case
        if(min_size == -1)
        {
            min_size = size;
        }else{
            min_size = (min_size < size ? min_size : size);
        }
    }
    return min_size;
}


template <typename T, typename Derived>
inline bool b3_list_to_matrix(const std::vector<std::vector<T > > & V,Eigen::PlainObjectBase<Derived>& M)
{
    // number of rows
    int m = V.size();
    if(m == 0) {
        M.resize(Derived::RowsAtCompileTime>=0?Derived::RowsAtCompileTime:0,
                 Derived::ColsAtCompileTime>=0?Derived::ColsAtCompileTime:0);
        return true;
    }
    // number of columns
    int n = b3_min_size(V);
    if(n != b3_max_size(V))
    {
        return false;
    }
    assert(n != -1);
    // Resize output
    M.resize(m,n);

    // Loop over rows
    for(int i = 0;i < m; i++) {
        // Loop over cols
        for(int j = 0; j < n;j++) {
            M(i,j) = V[i][j];
        }
    }

    return true;
}


template <typename DerivedF, typename T>
void b3_adjacency_matrix(
        const Eigen::MatrixBase<DerivedF> & F,
        Eigen::SparseMatrix<T>& A)
{
    using namespace std;
    using namespace Eigen;
    typedef typename DerivedF::Scalar Index;

    typedef Triplet<T> IJV;
    vector<IJV > ijv;
    ijv.reserve(F.size()*2);
    // Loop over **simplex** (i.e., **not quad**)
    for(int i = 0;i<F.rows();i++)
    {
        // Loop over this **simplex**
        for(int j = 0;j<F.cols();j++)
            for(int k = j+1;k<F.cols();k++)
            {
                // Get indices of edge: s --> d
                Index s = F(i,j);
                Index d = F(i,k);
                ijv.push_back(IJV(s,d,1));
                ijv.push_back(IJV(d,s,1));
            }
    }

    const Index n = F.maxCoeff()+1;
    A.resize(n,n);
    switch(F.cols())
    {
        case 3:
            A.reserve(6*(F.maxCoeff()+1));
            break;
        case 4:
            A.reserve(26*(F.maxCoeff()+1));
            break;
    }
    A.setFromTriplets(ijv.begin(),ijv.end());

    // Force all non-zeros to be one

    // Iterate over outside
    for(int k=0; k<A.outerSize(); ++k)
    {
        // Iterate over inside
        for(typename Eigen::SparseMatrix<T>::InnerIterator it (A,k); it; ++it)
        {
            assert(it.value() != 0);
            A.coeffRef(it.row(),it.col()) = 1;
        }
    }
}


#endif //BOX3D_B3_MATRIX_HPP
