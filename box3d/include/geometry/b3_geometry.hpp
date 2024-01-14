// Algorithms used in this file is part of libigl
// just for easier compiling.
// For more information, please access:
// https://github.com/libigl/libigl

#ifndef BOX3D_B3_GEOMETRY_HPP
#define BOX3D_B3_GEOMETRY_HPP


#include "math/b3_matrix_operation.hpp"


template <typename DerivedF, typename DerivedE>
void b3_edges(const Eigen::MatrixBase<DerivedF> & F,
              Eigen::PlainObjectBase<DerivedE> & E);

template <typename DerivedI, typename DerivedC, typename DerivedE>
void b3_edges(const Eigen::MatrixBase<DerivedI> & I,
              const Eigen::MatrixBase<DerivedC> & C,
              Eigen::PlainObjectBase<DerivedE> & E);

template <typename T, typename DerivedE>
void b3_edges(const Eigen::SparseMatrix<T> & A,
              Eigen::PlainObjectBase<DerivedE> & E);


template <typename DerivedF, typename DerivedE>
void b3_edges(
        const Eigen::MatrixBase<DerivedF> & F,
        Eigen::PlainObjectBase<DerivedE> & E)
{
    // build adjacency matrix
    typedef typename DerivedF::Scalar Index;
    Eigen::SparseMatrix<Index> A;
    b3_adjacency_matrix(F,A);
    b3_edges(A,E);
}


template <typename DerivedI, typename DerivedC, typename DerivedE>
void b3_edges(
        const Eigen::MatrixBase<DerivedI> & I,
        const Eigen::MatrixBase<DerivedC> & C,
        Eigen::PlainObjectBase<DerivedE> & E)
{
    typedef typename DerivedE::Scalar Index;
    Eigen::SparseMatrix<Index> A;
    b3_adjacency_matrix(I,C,A);
    b3_edges(A,E);
}


template <typename T, typename DerivedE>
void b3_edges(
        const Eigen::SparseMatrix<T> & A,
        Eigen::PlainObjectBase<DerivedE> & E)
{
    // Number of non zeros should be twice number of edges
    assert(A.nonZeros()%2 == 0);
    // Resize to fit edges
    E.resize(A.nonZeros()/2,2);
    int i = 0;
    // Iterate over outside
    for(int k=0; k<A.outerSize(); ++k)
    {
        // Iterate over inside
        for(typename Eigen::SparseMatrix<T>::InnerIterator it (A,k); it; ++it)
        {
            // only add edge in one direction
            if(it.row()<it.col())
            {
                E(i,0) = it.row();
                E(i,1) = it.col();
                i++;
            }
        }
    }
    assert(i == E.rows() && "A should be symmetric");
}


#endif //BOX3D_B3_GEOMETRY_HPP
