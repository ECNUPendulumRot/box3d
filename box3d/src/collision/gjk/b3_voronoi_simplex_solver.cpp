
#include "collision/gjk/b3_voronoi_simplex_solver.hpp"

#define VERTA 0
#define VERTB 1
#define VERTC 2
#define VERTD 3

#define CATCH_DEGENERATE_TETRAHEDRON 1
void b3VoronoiSimplexSolver::remove_vertex(int index)
{
    b3_assert(m_numVertices > 0);
    m_numVertices--;
    m_simplexVectorW[index] = m_simplexVectorW[m_numVertices];
    m_simplexPointsP[index] = m_simplexPointsP[m_numVertices];
    m_simplexPointsQ[index] = m_simplexPointsQ[m_numVertices];
}

void b3VoronoiSimplexSolver::reduce_vertices(const b3UsageBitfield &usedVerts) {
    if ((num_vertices() >= 4) && (!usedVerts.usedVertexD))
        remove_vertex(3);

    if ((num_vertices() >= 3) && (!usedVerts.usedVertexC))
        remove_vertex(2);

    if ((num_vertices() >= 2) && (!usedVerts.usedVertexB))
        remove_vertex(1);

    if ((num_vertices() >= 1) && (!usedVerts.usedVertexA))
        remove_vertex(0);
}

//clear the simplex, remove all the vertices
void b3VoronoiSimplexSolver::reset()
{
    m_cachedValidClosest = false;
    m_numVertices = 0;
    m_needsUpdate = true;
    m_lastW = b3Vec3r(b3_real_max, b3_real_max, b3_real_max);
    m_cachedBC.reset();
}

//add a vertex
void b3VoronoiSimplexSolver::add_vertex(const b3Vec3r& w, const b3Vec3r& p, const b3Vec3r& q)
{
    m_lastW = w;
    m_needsUpdate = true;

    m_simplexVectorW[m_numVertices] = w;
    m_simplexPointsP[m_numVertices] = p;
    m_simplexPointsQ[m_numVertices] = q;

    m_numVertices++;
}

bool b3VoronoiSimplexSolver::update_closest_vector_and_points() {
    if (m_needsUpdate)
    {
        m_cachedBC.reset();

        m_needsUpdate = false;

        switch (num_vertices())
        {
            case 0:
                m_cachedValidClosest = false;
                break;
            case 1:
            {
                m_cachedP1 = m_simplexPointsP[0];
                m_cachedP2 = m_simplexPointsQ[0];
                m_cachedV = m_cachedP1 - m_cachedP2;  //== m_simplexVectorW[0]
                m_cachedBC.reset();
                m_cachedBC.set_barycentric_coordinates(1, 0, 0, 0);
                m_cachedValidClosest = m_cachedBC.is_valid();
                break;
            };
            case 2:
            {
                //closest point origin from line segment
                const b3Vec3r& from = m_simplexVectorW[0];
                const b3Vec3r& to = m_simplexVectorW[1];
                b3Vec3r nearest;

                b3Vec3r p(0, 0, 0);
                b3Vec3r diff = p - from;
                b3Vec3r v = to - from;
                real t = v.dot(diff);

                if (t > 0)
                {
                    real dotVV = v.dot(v);
                    if (t < dotVV)
                    {
                        t /= dotVV;
                        diff -= t * v;
                        m_cachedBC.m_usedVertices.usedVertexA = true;
                        m_cachedBC.m_usedVertices.usedVertexB = true;
                    }
                    else
                    {
                        t = 1;
                        diff -= v;
                        //reduce to 1 point
                        m_cachedBC.m_usedVertices.usedVertexB = true;
                    }
                }
                else
                {
                    t = 0;
                    //reduce to 1 point
                    m_cachedBC.m_usedVertices.usedVertexA = true;
                }
                m_cachedBC.set_barycentric_coordinates(1 - t, t);
                nearest = from + t * v;

                m_cachedP1 = m_simplexPointsP[0] + t * (m_simplexPointsP[1] - m_simplexPointsP[0]);
                m_cachedP2 = m_simplexPointsQ[0] + t * (m_simplexPointsQ[1] - m_simplexPointsQ[0]);
                m_cachedV = m_cachedP1 - m_cachedP2;

                reduce_vertices(m_cachedBC.m_usedVertices);

                m_cachedValidClosest = m_cachedBC.is_valid();
                break;
            }
            case 3:
            {
                //closest point origin from triangle
                b3Vec3r p(real(0.), real(0.), real(0.));

                const b3Vec3r& a = m_simplexVectorW[0];
                const b3Vec3r& b = m_simplexVectorW[1];
                const b3Vec3r& c = m_simplexVectorW[2];

                closest_pt_point_triangle(p, a, b, c, m_cachedBC);
                m_cachedP1 = m_simplexPointsP[0] * m_cachedBC.m_barycentricCoords[0] +
                             m_simplexPointsP[1] * m_cachedBC.m_barycentricCoords[1] +
                             m_simplexPointsP[2] * m_cachedBC.m_barycentricCoords[2];

                m_cachedP2 = m_simplexPointsQ[0] * m_cachedBC.m_barycentricCoords[0] +
                             m_simplexPointsQ[1] * m_cachedBC.m_barycentricCoords[1] +
                             m_simplexPointsQ[2] * m_cachedBC.m_barycentricCoords[2];

                m_cachedV = m_cachedP1 - m_cachedP2;

                reduce_vertices(m_cachedBC.m_usedVertices);
                m_cachedValidClosest = m_cachedBC.is_valid();

                break;
            }
            case 4:
            {
                b3Vec3r p(real(0.), real(0.), real(0.));

                const b3Vec3r& a = m_simplexVectorW[0];
                const b3Vec3r& b = m_simplexVectorW[1];
                const b3Vec3r& c = m_simplexVectorW[2];
                const b3Vec3r& d = m_simplexVectorW[3];

                bool hasSeparation = closest_pt_point_tetrahedron(p, a, b, c, d, m_cachedBC);

                if (hasSeparation)
                {
                    m_cachedP1 = m_simplexPointsP[0] * m_cachedBC.m_barycentricCoords[0] +
                                 m_simplexPointsP[1] * m_cachedBC.m_barycentricCoords[1] +
                                 m_simplexPointsP[2] * m_cachedBC.m_barycentricCoords[2] +
                                 m_simplexPointsP[3] * m_cachedBC.m_barycentricCoords[3];

                    m_cachedP2 = m_simplexPointsQ[0] * m_cachedBC.m_barycentricCoords[0] +
                                 m_simplexPointsQ[1] * m_cachedBC.m_barycentricCoords[1] +
                                 m_simplexPointsQ[2] * m_cachedBC.m_barycentricCoords[2] +
                                 m_simplexPointsQ[3] * m_cachedBC.m_barycentricCoords[3];

                    m_cachedV = m_cachedP1 - m_cachedP2;
                    reduce_vertices(m_cachedBC.m_usedVertices);
                }
                else
                {
                    //					printf("sub distance got penetration\n");

                    if (m_cachedBC.m_degenerate)
                    {
                        m_cachedValidClosest = false;
                    }
                    else
                    {
                        m_cachedValidClosest = true;
                        //degenerate case == false, penetration = true + zero
                        m_cachedV.set_zero();
                    }
                    break;
                }

                m_cachedValidClosest = m_cachedBC.is_valid();

                //closest point origin from tetrahedron
                break;
            }
            default:
            {
                m_cachedValidClosest = false;
            }
        };
    }

    return m_cachedValidClosest;
}

//return/calculate the closest vertex
bool b3VoronoiSimplexSolver::closest(b3Vec3r& v)
{
    bool succes = update_closest_vector_and_points();
    v = m_cachedV;
    return succes;
}

real b3VoronoiSimplexSolver::max_vertex()
{
    int i, numverts = num_vertices();
    real maxV = real(0.);
    for (i = 0; i < numverts; i++)
    {
        real curLen2 = m_simplexVectorW[i].length2();
        if (maxV < curLen2)
            maxV = curLen2;
    }
    return maxV;
}

//return the current simplex
int b3VoronoiSimplexSolver::get_simplex(b3Vec3r* pBuf, b3Vec3r* qBuf, b3Vec3r* yBuf) const
{
    int i;
    for (i = 0; i < num_vertices(); i++)
    {
        yBuf[i] = m_simplexVectorW[i];
        pBuf[i] = m_simplexPointsP[i];
        qBuf[i] = m_simplexPointsQ[i];
    }
    return num_vertices();
}

bool b3VoronoiSimplexSolver::in_simplex(const b3Vec3r& w)
{
    bool found = false;
    int i, numverts = num_vertices();
    //real maxV = real(0.);

    //w is in the current (reduced) simplex
    for (i = 0; i < numverts; i++)
    {
#ifdef BT_USE_EQUAL_VERTEX_THRESHOLD
        if (m_simplexVectorW[i].distance2(w) <= m_equalVertexThreshold)
#else
        if (m_simplexVectorW[i] == w)
#endif
        {
            found = true;
            break;
        }
    }

    //check in case lastW is already removed
    if (w == m_lastW)
        return true;

    return found;
}

void b3VoronoiSimplexSolver::backup_closest(b3Vec3r& v)
{
    v = m_cachedV;
}

bool b3VoronoiSimplexSolver::empty_simplex() const
{
    return (num_vertices() == 0);
}

void b3VoronoiSimplexSolver::compute_points(b3Vec3r& p1, b3Vec3r& p2)
{
    update_closest_vector_and_points();
    p1 = m_cachedP1;
    p2 = m_cachedP2;
}

bool b3VoronoiSimplexSolver::closest_pt_point_triangle(const b3Vec3r &p, const b3Vec3r &a, const b3Vec3r &b, const b3Vec3r &c, b3SubSimplexClosestResult &result) {
    result.m_usedVertices.reset();

    // Check if P in vertex region outside A
    b3Vec3r ab = b - a;
    b3Vec3r ac = c - a;
    b3Vec3r ap = p - a;
    real d1 = ab.dot(ap);
    real d2 = ac.dot(ap);
    if (d1 <= real(0.0) && d2 <= real(0.0))
    {
        result.m_closestPointOnSimplex = a;
        result.m_usedVertices.usedVertexA = true;
        result.set_barycentric_coordinates(1, 0, 0);
        return true;  // a; // barycentric coordinates (1,0,0)
    }

    // Check if P in vertex region outside B
    b3Vec3r bp = p - b;
    real d3 = ab.dot(bp);
    real d4 = ac.dot(bp);
    if (d3 >= real(0.0) && d4 <= d3)
    {
        result.m_closestPointOnSimplex = b;
        result.m_usedVertices.usedVertexB = true;
        result.set_barycentric_coordinates(0, 1, 0);

        return true;  // b; // barycentric coordinates (0,1,0)
    }
    // Check if P in edge region of AB, if so return projection of P onto AB
    real vc = d1 * d4 - d3 * d2;
    if (vc <= real(0.0) && d1 >= real(0.0) && d3 <= real(0.0))
    {
        real v = d1 / (d1 - d3);
        result.m_closestPointOnSimplex = a + v * ab;
        result.m_usedVertices.usedVertexA = true;
        result.m_usedVertices.usedVertexB = true;
        result.set_barycentric_coordinates(1 - v, v, 0);
        return true;
        //return a + v * ab; // barycentric coordinates (1-v,v,0)
    }

    // Check if P in vertex region outside C
    b3Vec3r cp = p - c;
    real d5 = ab.dot(cp);
    real d6 = ac.dot(cp);
    if (d6 >= real(0.0) && d5 <= d6)
    {
        result.m_closestPointOnSimplex = c;
        result.m_usedVertices.usedVertexC = true;
        result.set_barycentric_coordinates(0, 0, 1);
        return true;  //c; // barycentric coordinates (0,0,1)
    }

    // Check if P in edge region of AC, if so return projection of P onto AC
    real vb = d5 * d2 - d1 * d6;
    if (vb <= real(0.0) && d2 >= real(0.0) && d6 <= real(0.0))
    {
        real w = d2 / (d2 - d6);
        result.m_closestPointOnSimplex = a + w * ac;
        result.m_usedVertices.usedVertexA = true;
        result.m_usedVertices.usedVertexC = true;
        result.set_barycentric_coordinates(1 - w, 0, w);
        return true;
        //return a + w * ac; // barycentric coordinates (1-w,0,w)
    }

    // Check if P in edge region of BC, if so return projection of P onto BC
    real va = d3 * d6 - d5 * d4;
    if (va <= real(0.0) && (d4 - d3) >= real(0.0) && (d5 - d6) >= real(0.0))
    {
        real w = (d4 - d3) / ((d4 - d3) + (d5 - d6));

        result.m_closestPointOnSimplex = b + w * (c - b);
        result.m_usedVertices.usedVertexB = true;
        result.m_usedVertices.usedVertexC = true;
        result.set_barycentric_coordinates(0, 1 - w, w);
        return true;
        // return b + w * (c - b); // barycentric coordinates (0,1-w,w)
    }

    // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
    real denom = real(1.0) / (va + vb + vc);
    real v = vb * denom;
    real w = vc * denom;

    result.m_closestPointOnSimplex = a + ab * v + ac * w;
    result.m_usedVertices.usedVertexA = true;
    result.m_usedVertices.usedVertexB = true;
    result.m_usedVertices.usedVertexC = true;
    result.set_barycentric_coordinates(1 - v - w, v, w);

    return true;
    //	return a + ab * v + ac * w; // = u*a + v*b + w*c, u = va * denom = real(1.0) - v - w
}

/// Test if point p and d lie on opposite sides of plane through abc
int b3VoronoiSimplexSolver::point_outside_of_plane(const b3Vec3r& p, const b3Vec3r& a, const b3Vec3r& b, const b3Vec3r& c, const b3Vec3r& d)
{
    b3Vec3r normal = (b - a).cross(c - a);

    real signp = (p - a).dot(normal);  // [AP AB AC]
    real signd = (d - a).dot(normal);  // [AD AB AC]

#ifdef CATCH_DEGENERATE_TETRAHEDRON
#ifdef BT_USE_DOUBLE_PRECISION
    if (signd * signd < (real(1e-8) * real(1e-8)))
	{
		return -1;
	}
#else
    if (signd * signd < (real(1e-4) * real(1e-4)))
    {
        //		printf("affine dependent/degenerate\n");//
        return -1;
    }
#endif

#endif
    // Points on opposite sides if expression signs are opposite
    return signp * signd < real(0.);
}

bool b3VoronoiSimplexSolver::closest_pt_point_tetrahedron(const b3Vec3r& p, const b3Vec3r& a, const b3Vec3r& b, const b3Vec3r& c, const b3Vec3r& d, b3SubSimplexClosestResult& finalResult)
{
    b3SubSimplexClosestResult tempResult;

    // Start out assuming point inside all halfspaces, so closest to itself
    finalResult.m_closestPointOnSimplex = p;
    finalResult.m_usedVertices.reset();
    finalResult.m_usedVertices.usedVertexA = true;
    finalResult.m_usedVertices.usedVertexB = true;
    finalResult.m_usedVertices.usedVertexC = true;
    finalResult.m_usedVertices.usedVertexD = true;

    int pointOutsideABC = point_outside_of_plane(p, a, b, c, d);
    int pointOutsideACD = point_outside_of_plane(p, a, c, d, b);
    int pointOutsideADB = point_outside_of_plane(p, a, d, b, c);
    int pointOutsideBDC = point_outside_of_plane(p, b, d, c, a);

    if (pointOutsideABC < 0 || pointOutsideACD < 0 || pointOutsideADB < 0 || pointOutsideBDC < 0)
    {
        finalResult.m_degenerate = true;
        return false;
    }

    if (!pointOutsideABC && !pointOutsideACD && !pointOutsideADB && !pointOutsideBDC)
    {
        return false;
    }

    real bestSqDist = FLT_MAX;
    // If point outside face abc then compute closest point on abc
    if (pointOutsideABC)
    {
        closest_pt_point_triangle(p, a, b, c, tempResult);
        b3Vec3r q = tempResult.m_closestPointOnSimplex;

        real sqDist = (q - p).dot(q - p);
        // Update best closest point if (squared) distance is less than current best
        if (sqDist < bestSqDist)
        {
            bestSqDist = sqDist;
            finalResult.m_closestPointOnSimplex = q;
            //convert result bitmask!
            finalResult.m_usedVertices.reset();
            finalResult.m_usedVertices.usedVertexA = tempResult.m_usedVertices.usedVertexA;
            finalResult.m_usedVertices.usedVertexB = tempResult.m_usedVertices.usedVertexB;
            finalResult.m_usedVertices.usedVertexC = tempResult.m_usedVertices.usedVertexC;
            finalResult.set_barycentric_coordinates(
                tempResult.m_barycentricCoords[VERTA],
                tempResult.m_barycentricCoords[VERTB],
                tempResult.m_barycentricCoords[VERTC],
                0);
        }
    }

    // Repeat test for face acd
    if (pointOutsideACD)
    {
        closest_pt_point_triangle(p, a, c, d, tempResult);
        b3Vec3r q = tempResult.m_closestPointOnSimplex;
        //convert result bitmask!

        real sqDist = (q - p).dot(q - p);
        if (sqDist < bestSqDist)
        {
            bestSqDist = sqDist;
            finalResult.m_closestPointOnSimplex = q;
            finalResult.m_usedVertices.reset();
            finalResult.m_usedVertices.usedVertexA = tempResult.m_usedVertices.usedVertexA;

            finalResult.m_usedVertices.usedVertexC = tempResult.m_usedVertices.usedVertexB;
            finalResult.m_usedVertices.usedVertexD = tempResult.m_usedVertices.usedVertexC;
            finalResult.set_barycentric_coordinates(
                tempResult.m_barycentricCoords[VERTA],
                0,
                tempResult.m_barycentricCoords[VERTB],
                tempResult.m_barycentricCoords[VERTC]);
        }
    }
    // Repeat test for face adb

    if (pointOutsideADB)
    {
        closest_pt_point_triangle(p, a, d, b, tempResult);
        b3Vec3r q = tempResult.m_closestPointOnSimplex;
        //convert result bitmask!

        real sqDist = (q - p).dot(q - p);
        if (sqDist < bestSqDist)
        {
            bestSqDist = sqDist;
            finalResult.m_closestPointOnSimplex = q;
            finalResult.m_usedVertices.reset();
            finalResult.m_usedVertices.usedVertexA = tempResult.m_usedVertices.usedVertexA;
            finalResult.m_usedVertices.usedVertexB = tempResult.m_usedVertices.usedVertexC;

            finalResult.m_usedVertices.usedVertexD = tempResult.m_usedVertices.usedVertexB;
            finalResult.set_barycentric_coordinates(
                tempResult.m_barycentricCoords[VERTA],
                tempResult.m_barycentricCoords[VERTC],
                0,
                tempResult.m_barycentricCoords[VERTB]);
        }
    }
    // Repeat test for face bdc

    if (pointOutsideBDC)
    {
        closest_pt_point_triangle(p, b, d, c, tempResult);
        b3Vec3r q = tempResult.m_closestPointOnSimplex;
        //convert result bitmask!
        real sqDist = (q - p).dot(q - p);
        if (sqDist < bestSqDist)
        {
            bestSqDist = sqDist;
            finalResult.m_closestPointOnSimplex = q;
            finalResult.m_usedVertices.reset();
            //
            finalResult.m_usedVertices.usedVertexB = tempResult.m_usedVertices.usedVertexA;
            finalResult.m_usedVertices.usedVertexC = tempResult.m_usedVertices.usedVertexC;
            finalResult.m_usedVertices.usedVertexD = tempResult.m_usedVertices.usedVertexB;

            finalResult.set_barycentric_coordinates(
                0,
                tempResult.m_barycentricCoords[VERTA],
                tempResult.m_barycentricCoords[VERTC],
                tempResult.m_barycentricCoords[VERTB]);
        }
    }

    //help! we ended up full !

    if (finalResult.m_usedVertices.usedVertexA &&
        finalResult.m_usedVertices.usedVertexB &&
        finalResult.m_usedVertices.usedVertexC &&
        finalResult.m_usedVertices.usedVertexD)
    {
        return true;
    }

    return true;
}
