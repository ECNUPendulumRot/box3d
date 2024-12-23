
#ifndef B3_VORONOI_SIMPLEX_SOLVER_HPP
#define B3_VORONOI_SIMPLEX_SOLVER_HPP

#include "math/b3_vec3.hpp"

#define VORONOI_SIMPLEX_MAX_VERTS 5

#define VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD 0.0001f

struct b3UsageBitfield
{
    b3UsageBitfield()
    {
        reset();
    }

    void reset()
    {
        usedVertexA = false;
        usedVertexB = false;
        usedVertexC = false;
        usedVertexD = false;
    }
    unsigned short usedVertexA : 1;
    unsigned short usedVertexB : 1;
    unsigned short usedVertexC : 1;
    unsigned short usedVertexD : 1;
    unsigned short unused1 : 1;
    unsigned short unused2 : 1;
    unsigned short unused3 : 1;
    unsigned short unused4 : 1;
};

struct b3SubSimplexClosestResult
{
    b3Vec3r m_closestPointOnSimplex;
    //MASK for m_usedVertices
    //stores the simplex vertex-usage, using the MASK,
    // if m_usedVertices & MASK then the related vertex is used
    b3UsageBitfield m_usedVertices;
    real m_barycentricCoords[4];
    bool m_degenerate;

    void reset()
    {
        m_degenerate = false;
        set_barycentric_coordinates();
        m_usedVertices.reset();
    }
    bool is_valid()
    {
        bool valid = (m_barycentricCoords[0] >= real(0.)) &&
                     (m_barycentricCoords[1] >= real(0.)) &&
                     (m_barycentricCoords[2] >= real(0.)) &&
                     (m_barycentricCoords[3] >= real(0.));

        return valid;
    }
    void set_barycentric_coordinates(real a = 0, real b = 0, real c = 0, real d = 0)
    {
        m_barycentricCoords[0] = a;
        m_barycentricCoords[1] = b;
        m_barycentricCoords[2] = c;
        m_barycentricCoords[3] = d;
    }
};

class b3VoronoiSimplexSolver {
public:
    
    int m_numVertices;

    b3Vec3r m_simplexVectorW[VORONOI_SIMPLEX_MAX_VERTS];
    b3Vec3r m_simplexPointsP[VORONOI_SIMPLEX_MAX_VERTS];
    b3Vec3r m_simplexPointsQ[VORONOI_SIMPLEX_MAX_VERTS];

    b3Vec3r m_cachedP1;
    b3Vec3r m_cachedP2;
    b3Vec3r m_cachedV;
    b3Vec3r m_lastW;

    real m_equalVertexThreshold;
    bool m_cachedValidClosest;

    b3SubSimplexClosestResult m_cachedBC;

    bool m_needsUpdate;

    void remove_vertex(int index);
    void reduce_vertices(const b3UsageBitfield& usedVerts);
    bool update_closest_vector_and_points();

    bool closest_pt_point_tetrahedron(const b3Vec3r& p, const b3Vec3r& a, const b3Vec3r& b, const b3Vec3r& c, const b3Vec3r& d, b3SubSimplexClosestResult& finalResult);
    int point_outside_of_plane(const b3Vec3r& p, const b3Vec3r& a, const b3Vec3r& b, const b3Vec3r& c, const b3Vec3r& d);
    bool closest_pt_point_triangle(const b3Vec3r& p, const b3Vec3r& a, const b3Vec3r& b, const b3Vec3r& c, b3SubSimplexClosestResult& result);
    
    b3VoronoiSimplexSolver()
        : m_equalVertexThreshold(VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD)
    {
    }
    void reset();

    void add_vertex(const b3Vec3r& w, const b3Vec3r& p, const b3Vec3r& q);

    bool closest(b3Vec3r & v);

    real max_vertex();

    bool full_simplex() const
    {
        return (m_numVertices == 4);
    }

    int get_simplex(b3Vec3r * pBuf, b3Vec3r * qBuf, b3Vec3r * yBuf) const;

    bool in_simplex(const b3Vec3r& w);

    void backup_closest(b3Vec3r & v);

    bool empty_simplex() const;

    void compute_points(b3Vec3r & p1, b3Vec3r & p2);

    int num_vertices() const
    {
        return m_numVertices;
    }
};


#endif