
#include "collision/b3_collision.hpp"

#include "geometry/b3_cube_shape.hpp"

struct dContactGemo;


void b3_line_closest_approach(
    const b3Vec3r& pa, const b3Vec3r& ua,
    const b3Vec3r& pb, const b3Vec3r& ub,
    real* alpha, real* beta)
{
    b3Vec3r p = pb - pa;
    real uaub = ua.dot(ub);
    real q1 = ua.dot(p);
    real q2 = -ub.dot(p);
    real d = 1 - uaub * uaub;
    if (d <= real(1e-4)) {
        *alpha = 0;
        *beta = 0;
    } else {
        d = real(1) / d;
        *alpha = (q1 + uaub * q2) * d;
        *beta = (uaub * q1 + q2) *d;
    }
}

// find all the intersection points between the 2D rectangle with vertices
// at (+/-h[0],+/-h[1]) and the 2D quadrilateral with vertices (p[0],p[1]),
// (p[2],p[3]),(p[4],p[5]),(p[6],p[7]).
//
// the intersection points are returned as x,y pairs in the 'ret' array.
// the number of intersection points is returned by the function (this will
// be in the range 0 to 8).
static int intersectRectQuad2(real h[2], real p[8], real ret[16])
{
    // q (and r) contain nq (and nr) coordinate points for the current (and
    // chopped) polygons
    int nq = 4, nr = 0;
    real buffer[16];
    real* q = p;
    real* r = ret;
    for (int dir = 0; dir <= 1; dir++)
    {
        // direction notation: xy[0] = x axis, xy[1] = y axis
        for (int sign = -1; sign <= 1; sign += 2)
        {
            // chop q along the line xy[dir] = sign*h[dir]
            real* pq = q;
            real* pr = r;
            nr = 0;
            for (int i = nq; i > 0; i--)
            {
                // go through all points in q and all lines between adjacent points
                if (sign * pq[dir] < h[dir])
                {
                    // this point is inside the chopping line
                    pr[0] = pq[0];
                    pr[1] = pq[1];
                    pr += 2;
                    nr++;
                    if (nr & 8)
                    {
                        q = r;
                        goto done;
                    }
                }
                real* nextq = (i > 1) ? pq + 2 : q;
                if ((sign * pq[dir] < h[dir]) ^ (sign * nextq[dir] < h[dir]))
                {
                    // this line crosses the chopping line
                    pr[1 - dir] = pq[1 - dir] + (nextq[1 - dir] - pq[1 - dir]) /
                                                (nextq[dir] - pq[dir]) * (sign * h[dir] - pq[dir]);
                    pr[dir] = sign * h[dir];
                    pr += 2;
                    nr++;
                    if (nr & 8)
                    {
                        q = r;
                        goto done;
                    }
                }
                pq += 2;
            }
            q = r;
            r = (q == ret) ? buffer : ret;
            nq = nr;
        }
    }
done:
    if (q != ret) {
        memcpy(ret, q, nr * 2 * sizeof(real));
    }
    return nr;
}


#define M__PI 3.14159265f

// given n points in the plane (array p, of size 2*n), generate m points that
// best represent the whole set. the definition of 'best' here is not
// predetermined - the idea is to select points that give good box-box
// collision detection behavior. the chosen point indexes are returned in the
// array iret (of size m). 'i0' is always the first entry in the array.
// n must be in the range [1..8]. m must be in the range [1..n]. i0 must be
// in the range [0..n-1].

void cullPoints2(int n, real p[], int m, int i0, int iret[])
{
    // compute the centroid of the polygon in cx,cy
    int i, j;
    real a, cx, cy, q;
    if (n == 1) {
        cx = p[0];
        cy = p[1];
    } else if (n == 2) {
        cx = real(0.5) * (p[0] + p[2]);
        cy = real(0.5) * (p[1] + p[3]);
    } else {
        a = 0;
        cx = 0;
        cy = 0;
        for (i = 0; i < (n - 1); i++) {
            q = p[i * 2] * p[i * 2 + 3] - p[i * 2 + 2] * p[i * 2 + 1];
            a += q;
            cx += q * (p[i * 2] + p[i * 2 + 2]);
            cy += q * (p[i * 2 + 1] + p[i * 2 + 3]);
        }
        q = p[n * 2 - 2] * p[1] - p[0] * p[n * 2 - 1];
        if (b3_abs(a + q) > b3_real_epsilon) {
            a = 1.f / (real(3.0) * (a + q));
        } else {
            a = b3_real_max;
        }
        cx = a * (cx + q * (p[n * 2 - 2] + p[0]));
        cy = a * (cy + q * (p[n * 2 - 1] + p[1]));
    }

    // compute the angle of each point w.r.t. the centroid
    real A[8];
    for (i = 0; i < n; i++) {
        A[i] = atan2(p[i * 2 + 1] - cy, p[i * 2] - cx);
    }

    // search for points that have angles closest to A[i0] + i*(2*pi/m).
    int avail[8];
    for (i = 0; i < n; i++) avail[i] = 1;
    avail[i0] = 0;
    iret[0] = i0;
    iret++;
    for (j = 1; j < m; j++) {
        a = real(j) * (2 * M__PI / m) + A[i0];
        if (a > M__PI)
            a -= 2 * M__PI;
        real maxdiff = 1e9, diff;

        *iret = i0;  // iret is not allowed to keep this value, but it sometimes does, when diff=#QNAN0

        for (i = 0; i < n; i++) {
            if (avail[i]) {
                diff = b3_abs(A[i] - a);
                if (diff > M__PI) diff = 2 * M__PI - diff;
                if (diff < maxdiff) {
                    maxdiff = diff;
                    *iret = i;
                }
            }
        }
        avail[*iret] = 0;
        iret++;
    }
}


int b3_cube_cube(
    const b3Transformr& xf_a, const b3Vec3r& half_xyz_a,
    const b3Transformr& xf_b, const b3Vec3r& half_xyz_b,
    b3Vec3r& normal, real* depth, int* return_code,
    int maxc, dContactGemo* contact, int skip, b3Manifold* output)
{
    // get vector from centers of boxA to boxB, in the world
    b3Vec3r p = xf_b.position() - xf_a.position();

    // a vector in A, the world vector is p = R_a^T * local_a
    // a vector in B, the world vector is p = R_b^T * local_b
    // a vector in B, relative to A is local_a = R_a * R_b^T * local_b
    const b3Mat33r Ra = xf_a.rotation_matrix().transpose();
    const b3Mat33r Rb = xf_b.rotation_matrix().transpose();
    b3Mat33r R = Ra * Rb.transpose();
    b3Mat33r absR;
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            absR(i, j) = b3_abs(R(i, j));
        }
    }

    // for all 15 possible separating axes:
    //   * see if the axis separates the boxes. if so, return 0.
    //   * find the depth of the penetration along the separating axis (s2)
    //   * if this is the largest depth so far, record it.
    // the normal vector will be set to the separating axis with the smallest
    // depth. note: normalR is set to point to a column of R1 or R2 if that is
    // the smallest depth normal so far. otherwise normalR is 0 and normalC is
    // set to a vector relative to body 1. invert_normal is 1 if the sign of
    // the normal should be flipped.

    // test boxA's axes
    real s = -b3_real_min;
    bool invert_normal = false;
    int code = 0;
    // return_code = 0;

    // local_p is p relative to boxA
    // xf_a.rotation_matrix() is [u1, u2, u3]^T, in world coordinates
    b3Vec3r local_a = Ra * p;
    for (int i = 0; i < 3; i++) {
        real total_length = half_xyz_a[i] + absR.row(i).dot(half_xyz_b);
        real gap = b3_abs(local_a[i]) - total_length;
        if (gap > 0) {
            return 0;
        }
        if (gap > s) {
            s = gap;
            normal = Ra.col(i);
            invert_normal = local_a[i] < 0;
            code = i + 1;
        }
    }
    // local_p is p relative to boxB
    // xf_b.rotation_matrix() is [v1, v2, v3]^T, in world coordinates
    b3Vec3r local_b = Rb * p;
    for (int i = 0; i < 3; i++) {
        real total_length = half_xyz_b[i] + absR.col(i).dot(half_xyz_a);
        real gap = b3_abs(local_b[i]) - total_length;
        if (gap > 0) {
            return 0;
        }
        if (gap > s) {
            s = gap;
            normal = Rb.col(i);
            invert_normal = local_b[i] < 0;
            code = i + 4;
        }
    }

    // note: cross product axes need to be scaled when s is computed.
    // normal (n1,n2,n3) is relative to box 1.

    // separating axis = u1 x (v1, v2, v3)
    // separating axis = u2 x (v1, v2, v3)
    // separating axis = u3 x (v1, v2, v3)
    for (int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            b3Vec3r axis = Ra.col(i).cross(Rb.col(j));
            b3Vec3r abs_axis = axis.abs();
            real total_length = (half_xyz_a + half_xyz_b).dot(abs_axis);
            real distance = p.dot(axis);
            real gap = total_length - distance;
            if (gap > 0) {
                return 0;
            }
            if (gap > s) {
                s = gap;
                normal = axis.normalized();
                invert_normal = distance < 0;
                code = 3 * i + j + 7;
            }
        }
    }

    if (code == 0) {
        return 0;
    }

    if (invert_normal) {
        normal = -normal;
    }

    *depth = s;

    // compute contact points
    if (code > 6) {
        // an edge from boxA touches an edge from boxB
        // find a point pa on the intersecting edge of boxA
        b3Vec3r pa = xf_a.position();
        real sign;
        for(int i = 0; i < 3; i++) {
            sign = (normal.dot(Ra.col(i))) > 0 ? real(1.0) : real(-1.0);
            pa += sign * half_xyz_a[i] * Ra.col(i);
        }

        // find a point pb on the intersecting edge of boxB
        b3Vec3r pb = xf_b.position();
        for(int i = 0; i < 3; i++) {
            sign = (normal.dot(Rb.col(i)) > 0) ? real(1.0) : real(-1.0);
            pb += sign * half_xyz_b[i] * Rb.col(i);
        }

        real alpha, beta;
        b3Vec3r ua, ub;
        // Get direction of first edge
        ua = Ra.col((code - 7) / 3);
        // Get direction of second edge
        ub = Rb.col((code - 7) % 3);
        // Get closet points between edges(one at each)
        b3_line_closest_approach(pa, ua, pb, ub, &alpha, &beta);
        pa += ua * alpha;
        pb += ub * beta;

        // TODO: add contact point to manifold

        return 1;
    }

    // okay, we have a face-something intersection (because the separating
    // axis is perpendicular to a face). define face 1 to be the reference
    // face (i.e. the normal vector is perpendicular to this) and face 2 to be
    // the incident face (the closest face of the other box).
    b3Mat33r R1, R2;
    b3Vec3r p1, p2;
    b3Vec3r half_xyz_1, half_xyz_2;
    if(code <= 3) {
        R1 = Ra;
        R2 = Rb;
        p1 = xf_a.position();
        p2 = xf_b.position();
        half_xyz_1 = half_xyz_a;
        half_xyz_2 = half_xyz_b;
    } else {
        R1 = Rb;
        R2 = Ra;
        p1 = xf_b.position();
        p2 = xf_a.position();
        half_xyz_1 = half_xyz_b;
        half_xyz_2 = half_xyz_a;
    }

    // nr = normal vector of reference face dotted with axes of incident box.
    // anr = absolute values of nr.
    b3Vec3r nr, anr;
    if (code > 3) {
        normal = -normal;
    }
    nr = R2 * normal;
    anr = nr.abs();

    // find the largest compontent of anr: this corresponds to the normal
    // for the indident face. the other axis numbers of the indicent face
    // are stored in a1,a2.
    int lanr, a1, a2;
    if (anr[1] > anr[0]) {
        a1 = 0;
        if(anr[1] > anr[2]) {
            lanr = 1;
            a2 = 2;
        } else {
            lanr = 2;
            a2 = 1;
        }
    } else {
        if(anr[0] > anr[2]) {
            lanr = 0;
            a1 = 1;
            a2 = 2;
        } else {
            a1 = 0;
            a2 = 1;
            lanr = 2;
        }
    }

    // compute center point of incident face, in reference-face coordinates
    b3Vec3r center;
    if(nr[lanr] < 0) {
        center = p2 - p1 + half_xyz_2[lanr] * R2.col(lanr);
    } else {
        center = p2 - p1 - half_xyz_2[lanr] * R2.col(lanr);
    }

    // find the normal and non-normal axis numbers of the reference box
    int codeN, code1, code2;
    if (code <= 3)
        codeN = code - 1;
    else
        codeN = code - 4;
    if (codeN == 0) {
        code1 = 1;
        code2 = 2;
    } else if (codeN == 1) {
        code1 = 0;
        code2 = 2;
    } else {
        code1 = 0;
        code2 = 1;
    }

    // find the four corners of the incident face, in reference-face coordinates
    real quad[8];  // 2D coordinate of incident face (x,y pairs)
    real c1, c2, m11, m12, m21, m22;
    c1 = center.dot(R1.col(code1));
    c2 = center.dot(R1.col(code2));
    // optimize this? - we have already computed this data above, but it is not
    // stored in an easy-to-index format. for now it's quicker just to recompute
    // the four dot products.
    m11 = R1.col(code1).dot(R2.col(a1));
    m12 = R1.col(code1).dot(R2.col(a2));
    m21 = R1.col(code2).dot(R2.col(a1));
    m22 = R1.col(code2).dot(R2.col(a2));

    {
        real k1 = m11 * half_xyz_2[a1];
        real k2 = m21 * half_xyz_2[a1];
        real k3 = m12 * half_xyz_2[a2];
        real k4 = m22 * half_xyz_2[a2];
        quad[0] = c1 - k1 - k3;
        quad[1] = c2 - k2 - k4;
        quad[2] = c1 - k1 + k3;
        quad[3] = c2 - k2 + k4;
        quad[4] = c1 + k1 + k3;
        quad[5] = c2 + k2 + k4;
        quad[6] = c1 + k1 - k3;
        quad[7] = c2 + k2 - k4;
    }

    // find the size of the reference face
    real rect[2];
    rect[0] = half_xyz_1[code1];
    rect[1] = half_xyz_1[code2];

    // intersect the incident and reference faces
    real ret[16];
    int n = intersectRectQuad2(rect, quad, ret);
    if (n < 1) return 0;  // this should never happen

    // convert the intersection points into reference-face coordinates,
    // and compute the contact position and depth for each point. only keep
    // those points that have a positive (penetrating) depth. delete points in
    // the 'ret' array as necessary so that 'point' and 'ret' correspond.
    b3Vec3r point[8];  // penetrating contact points
    real dep[8];        // depths for those points
    real det1 = 1.f / (m11 * m22 - m12 * m21);
    m11 *= det1;
    m12 *= det1;
    m21 *= det1;
    m22 *= det1;
    int cnum = 0;  // number of penetrating contact points found
    for (int j = 0; j < n; j++)
    {
        real k1 = m22 * (ret[j * 2] - c1) - m12 * (ret[j * 2 + 1] - c2);
        real k2 = -m21 * (ret[j * 2] - c1) + m11 * (ret[j * 2 + 1] - c2);
        for (int i = 0; i < 3; i++) {
            point[cnum] = center + k1 * R2.col(a1) + k2 * R2.col(a2);
        }
        dep[cnum] = half_xyz_1[codeN] - normal.dot(point[cnum]);
        if (dep[cnum] >= 0)
        {
            ret[cnum * 2] = ret[j * 2];
            ret[cnum * 2 + 1] = ret[j * 2 + 1];
            cnum++;
        }
    }
    if (cnum < 1) return 0;  // this should never happen

    // we can't generate more contacts than we actually have
    if (maxc > cnum) maxc = cnum;
    if (maxc < 1) maxc = 1;

    if (cnum <= maxc)
    {
        if (code < 4) {
            // we have less contacts than we need, so we use them all
            for (int j = 0; j < cnum; j++) {
                b3Vec3r pointInWorld = point[j] + p1;
                // TODO: add Contact point
            }
        } else {
            // we have less contacts than we need, so we use them all
            for (int j = 0; j < cnum; j++) {
                b3Vec3r pointInWorld = point[j] + p1 - normal * dep[j];
                // TODO: add Contact point
            }
        }
    } else {
        // we have more contacts than are wanted, some of them must be culled.
        // find the deepest point, it is always the first contact.
        int i1 = 0;
        real maxdepth = dep[0];
        for (int i = 1; i < cnum; i++) {
            if (dep[i] > maxdepth) {
                maxdepth = dep[i];
                i1 = i;
            }
        }

        int iret[8];
        cullPoints2(cnum, ret, maxc, i1, iret);

        for (int j = 0; j < maxc; j++) {
            //      dContactGeom *con = CONTACT(contact,skip*j);
            //    for (i=0; i<3; i++) con->pos[i] = point[iret[j]*3+i] + pa[i];
            //  con->depth = dep[iret[j]];

            b3Vec3r posInWorld = point[iret[j]] + p1;
            if (code < 4) {
                // TODO: add Contact point
            } else {
                // TODO: add Contact point
            }
        }
        cnum = maxc;
    }

    *return_code = code;
    return cnum;
}

void b3_collide_cube(
    b3Manifold *manifold,
    const b3CubeShape *cube_A, const b3Transformr &xf_A,
    const b3CubeShape *cube_B, const b3Transformr &xf_B)
{
    int skip = 0;
    dContactGemo* contact = nullptr;

    b3Vec3r normal;
    real depth;
    int return_code;
    int maxc = 4;

    b3_cube_cube(xf_B, cube_A->m_h_xyz, xf_B, cube_B->m_h_xyz, normal, &depth, &return_code,
                 maxc, contact, skip, manifold);
}