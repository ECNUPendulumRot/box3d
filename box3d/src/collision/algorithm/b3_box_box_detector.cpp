
#include "collision/algorithm/b3_box_box_detector.hpp"
#include "geometry/b3_cube_shape.hpp"

#include "collision/b3_fixture.hpp"
#include "dynamics/b3_body.hpp"
#include "collision/b3_manifold_result.hpp"


b3BoxBoxDetector::b3BoxBoxDetector(const b3Fixture* fixtureA, const b3Fixture* fixtureB)
    : m_fixtureA(fixtureA), m_fixtureB(fixtureB)
{
}

// given two boxes (p1,R1,side1) and (p2,R2,side2), collide them together and
// generate contact points. this returns 0 if there is no contact otherwise
// it returns the number of contacts generated.
// `normal' returns the contact normal.
// `depth' returns the maximum penetration depth along that normal.
// `return_code' returns a number indicating the type of contact that was
// detected:
//        1,2,3 = box 2 intersects with a face of box 1
//        4,5,6 = box 1 intersects with a face of box 2
//        7..15 = edge-edge contact
// `maxc' is the maximum number of contacts allowed to be generated, i.e.
// the size of the `contact' array.
// `contact' and `skip' are the contact array information provided to the
// collision functions. this function only fills in the position and depth
// fields.

void dLineClosestApproach(const b3Vec3r& pa, const b3Vec3r& ua,
						  const b3Vec3r& pb, const b3Vec3r& ub,
						  real* alpha, real* beta);
void dLineClosestApproach(const b3Vec3r& pa, const b3Vec3r& ua,
						  const b3Vec3r& pb, const b3Vec3r& ub,
						  real* alpha, real* beta)
{
	b3Vec3r p;
	p[0] = pb[0] - pa[0];
	p[1] = pb[1] - pa[1];
	p[2] = pb[2] - pa[2];
	real uaub = ua.dot(ub);
	real q1 = ua.dot(p);
	real q2 = -ub.dot(p);
	real d = 1 - uaub * uaub;
	if (d <= real(0.0001f))
	{
		// @@@ this needs to be made more robust
		*alpha = 0;
		*beta = 0;
	}
	else
	{
		d = 1.f / d;
		*alpha = (q1 + uaub * q2) * d;
		*beta = (uaub * q1 + q2) * d;
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
	if (q != ret) memcpy(ret, q, nr * 2 * sizeof(real));
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

#define MAX_CONTACT_POINT_COUNT 4

void cullPoints2(int n, real p[], int i0, int iret[]);
void cullPoints2(int n, real p[], int i0, int iret[])
{
	// compute the centroid of the polygon in cx,cy
	real total_area = 0;
    real cx = 0;
    real cy = 0;

    // Divide the polygon into triangles，
    // calculate the area and centroid of each triangle
    // the centroid of a triangle is ( (x1 + x2 + x3) / 3, (y1 + y2 + y3) / 3)
    for (int i = 0; i < (n - 1); i++) {
        real area = p[i * 2] * p[i * 2 + 3] - p[i * 2 + 2] * p[i * 2 + 1];
        total_area += area;
        cx += area * (p[i * 2] + p[i * 2 + 2]);
        cy += area * (p[i * 2 + 1] + p[i * 2 + 3]);
    }
    real area = p[n * 2 - 2] * p[1] - p[0] * p[n * 2 - 1];
    cx += area * (p[n * 2 - 2] + p[0]);
    cy += area * (p[n * 2 - 1] + p[1]);
    total_area += area;
    // compute 1 / total_area
    if (b3_abs(total_area) > b3_real_epsilon) {
        total_area = 1.f / (real(3.0) * total_area);
    } else {
        total_area = b3_real_max;
    }
    cx = total_area * cx;
    cy = total_area * cy;


	// compute the angle of each point w.r.t. the centroid
	real A[8];
	for (int i = 0; i < n; i++) {
        // tan (theta) = (y - cy) / (x - cx)
        A[i] = atan2f(p[i * 2 + 1] - cy, p[i * 2] - cx);
    }

	// search for points that have angles closest to A[i0] + i*(2*pi/m).
	int avail[8];
	for (int i = 0; i < n; i++) {
        avail[i] = 1;
    }
    avail[i0] = 0;
	iret[0] = i0;
	iret++;
	for (int j = 1; j < MAX_CONTACT_POINT_COUNT; j++) {
		real theta = real(j) * (2 * M__PI / MAX_CONTACT_POINT_COUNT) + A[i0];
		if (theta > M__PI) {
            theta -= 2 * M__PI;
        }

		real max_diff = 1e9, diff;

		*iret = i0;  // iret is not allowed to keep this value, but it sometimes does, when diff=#QNAN0

		for (int i = 0; i < n; i++) {
			if (avail[i]) {
				diff = b3_abs(A[i] - theta);
				if (diff > M__PI) {
                    diff = 2 * M__PI - diff;
                }
                if (diff < max_diff) {
					max_diff = diff;
					*iret = i;
				}
			}
		}
		avail[*iret] = 0;
		iret++;
	}
}

int dBoxBox2(const b3Transformr& xfA, const b3Vec3r& side1,
             const b3Transformr& xfB, const b3Vec3r& side2,
			 b3Vec3r& normal, real* depth, b3ManifoldResult& output);

int dBoxBox2(const b3Transformr& xfA, const b3Vec3r& side1,
             const b3Transformr& xfB, const b3Vec3r& side2,
			 b3Vec3r& normal, real* depth, b3ManifoldResult& output)
{
	const real fudge_factor = real(1.05);
	b3Vec3r p, pp, normalC;
	real R11, R12, R13, R21, R22, R23, R31, R32, R33,
		Q11, Q12, Q13, Q21, Q22, Q23, Q31, Q32, Q33, s, s2, l;
	int i, j, invert_normal, code;

    const b3Mat33r& R1 = xfA.rotation_matrix();
    const b3Mat33r& R2 = xfB.rotation_matrix();
    const b3Vec3r& p1 = xfA.position();
    const b3Vec3r& p2 = xfB.position();

	// get vector from centers of box 1 to box 2, relative to box 1
	p = p2 - p1;
    // get pp = p relative to body 1
    pp = R1.transpose() * p;

    b3Vec3r A, B;
    // get side lengths / 2
    A = side1 * real(0.5);
    B = side2 * real(0.5);

	// Rij is R1'*R2, i.e. the relative rotation between R1 and R2
    b3Mat33r R = R1.transpose() * R2;
    R11 = R(0, 0);
    R12 = R(0, 1);
    R13 = R(0, 2);
    R21 = R(1, 0);
    R22 = R(1, 1);
    R23 = R(1, 2);
    R31 = R(2, 0);
    R32 = R(2, 1);
    R33 = R(2, 2);

	Q11 = b3_abs(R11);
	Q12 = b3_abs(R12);
	Q13 = b3_abs(R13);
	Q21 = b3_abs(R21);
	Q22 = b3_abs(R22);
	Q23 = b3_abs(R23);
	Q31 = b3_abs(R31);
	Q32 = b3_abs(R32);
	Q33 = b3_abs(R33);

	// for all 15 possible separating axes:
	//   * see if the axis separates the boxes. if so, return 0.
	//   * find the depth of the penetration along the separating axis (s2)
	//   * if this is the largest depth so far, record it.
	// the normal vector will be set to the separating axis with the smallest
	// depth. note: normalR is set to point to a column of R1 or R2 if that is
	// the smallest depth normal so far. otherwise normalR is 0 and normalC is
	// set to a vector relative to body 1. invert_normal is 1 if the sign of
	// the normal should be flipped.

    b3Vec3r normalR;

#define TST(expr1, expr2, norm, cc)    \
	s2 = b3_abs(expr1) - (expr2);      \
	if (s2 > 0) return 0;              \
	if (s2 > s) {                      \
		s = s2;                        \
		normalR = norm;                \
		invert_normal = ((expr1) < 0); \
		code = (cc);                   \
	}

	s = -b3_real_max;
	invert_normal = 0;
	code = 0;

	// separating axis = u1,u2,u3
	TST(pp[0], (A[0] + B[0] * Q11 + B[1] * Q12 + B[2] * Q13), R1.col(0), 1);
	TST(pp[1], (A[1] + B[0] * Q21 + B[1] * Q22 + B[2] * Q23), R1.col(1), 2);
	TST(pp[2], (A[2] + B[0] * Q31 + B[1] * Q32 + B[2] * Q33), R1.col(2), 3);

	// separating axis = v1,v2,v3
    b3Vec3r pp2 = R2.transpose() * p;
	TST(pp2[0], (A[0] * Q11 + A[1] * Q21 + A[2] * Q31 + B[0]), R2.col(0), 4);
	TST(pp2[1], (A[0] * Q12 + A[1] * Q22 + A[2] * Q32 + B[1]), R2.col(1), 5);
	TST(pp2[2], (A[0] * Q13 + A[1] * Q23 + A[2] * Q33 + B[2]), R2.col(2), 6);

	// note: cross product axes need to be scaled when s is computed.
	// normal (n1,n2,n3) is relative to box 1.
#undef TST
#define TST(expr1, expr2, n1, n2, n3, cc)                \
	s2 = b3_abs(expr1) - (expr2);                        \
	if (s2 > b3_real_epsilon) return 0;                     \
	l = b3_sqrt((n1) * (n1) + (n2) * (n2) + (n3) * (n3)); \
	if (l > b3_real_epsilon) {                           \
		s2 /= l;                                         \
		if (s2 * fudge_factor > s) {                     \
			s = s2;                                      \
			normalR.set_zero();                          \
			normalC[0] = (n1) / l;                       \
			normalC[1] = (n2) / l;                       \
			normalC[2] = (n3) / l;                       \
			invert_normal = ((expr1) < 0);               \
			code = (cc);                                 \
		}                                                \
	}

	real fudge2(1.0e-5f);

	Q11 += fudge2;
	Q12 += fudge2;
	Q13 += fudge2;

	Q21 += fudge2;
	Q22 += fudge2;
	Q23 += fudge2;

	Q31 += fudge2;
	Q32 += fudge2;
	Q33 += fudge2;

	// separating axis = u1 x (v1,v2,v3)
	TST(pp[2] * R21 - pp[1] * R31, (A[1] * Q31 + A[2] * Q21 + B[1] * Q13 + B[2] * Q12), 0, -R31, R21, 7);
	TST(pp[2] * R22 - pp[1] * R32, (A[1] * Q32 + A[2] * Q22 + B[0] * Q13 + B[2] * Q11), 0, -R32, R22, 8);
	TST(pp[2] * R23 - pp[1] * R33, (A[1] * Q33 + A[2] * Q23 + B[0] * Q12 + B[1] * Q11), 0, -R33, R23, 9);

	// separating axis = u2 x (v1,v2,v3)
	TST(pp[0] * R31 - pp[2] * R11, (A[0] * Q31 + A[2] * Q11 + B[1] * Q23 + B[2] * Q22), R31, 0, -R11, 10);
	TST(pp[0] * R32 - pp[2] * R12, (A[0] * Q32 + A[2] * Q12 + B[0] * Q23 + B[2] * Q21), R32, 0, -R12, 11);
	TST(pp[0] * R33 - pp[2] * R13, (A[0] * Q33 + A[2] * Q13 + B[0] * Q22 + B[1] * Q21), R33, 0, -R13, 12);

	// separating axis = u3 x (v1,v2,v3)
	TST(pp[1] * R11 - pp[0] * R21, (A[0] * Q21 + A[1] * Q11 + B[1] * Q33 + B[2] * Q32), -R21, R11, 0, 13);
	TST(pp[1] * R12 - pp[0] * R22, (A[0] * Q22 + A[1] * Q12 + B[0] * Q33 + B[2] * Q31), -R22, R12, 0, 14);
	TST(pp[1] * R13 - pp[0] * R23, (A[0] * Q23 + A[1] * Q13 + B[0] * Q32 + B[1] * Q31), -R23, R13, 0, 15);

#undef TST

	if (!code) return 0;

	// if we get to this point, the boxes interpenetrate. compute the normal
	// in global coordinates.
	if (!normalR.is_zero()) {
        normal = normalR;
	} else {
        normal = R1 * normalC;
	}
	if (invert_normal) {
		normal = -normal;
	}
	*depth = -s;

	// compute contact point(s)
	if (code > 6) {
		// an edge from box 1 touches an edge from box 2.
		// find a point pa on the intersecting edge of box 1
		b3Vec3r pa{p1};
		real sign;
		for (j = 0; j < 3; j++) {
            const b3Vec3r& v = R1.col(j);
			sign = (normal.dot(v) > 0) ? real(1.0) : real(-1.0);
            pa += sign * A[j] * v;
		}

		// find a point pb on the intersecting edge of box 2
		b3Vec3r pb{p2};
		for (j = 0; j < 3; j++) {
            const b3Vec3r& v = R2.col(j);
			sign = (normal.dot(v) > 0) ? real(-1.0) : real(1.0);
            pb += sign * B[j] * v;
		}

		real alpha, beta;
        b3Vec3r ua = R1.col((code - 7) / 3);
        b3Vec3r ub = R2.col((code - 7) % 3);

		dLineClosestApproach(pa, ua, pb, ub, &alpha, &beta);

        pa += ua * alpha;
        output.add_contact_point(normal, pa, -*depth);
		return 1;
	}

	// okay, we have a face-something intersection (because the separating
	// axis is perpendicular to a face). define face 'a' to be the reference
	// face (i.e. the normal vector is perpendicular to this) and face 'b' to be
	// the incident face (the closest face of the other box).

    b3Vec3r pa, pb, Sa, Sb;
    b3Mat33r Ra, Rb;
	if (code <= 3) {
		Ra = R1;
		Rb = R2;
		pa = p1;
		pb = p2;
		Sa = A;
		Sb = B;
	} else {
		Ra = R2;
		Rb = R1;
		pa = p2;
		pb = p1;
		Sa = B;
		Sb = A;
	}

	// nr = normal vector of reference face dotted with axes of incident box.
	// anr = absolute values of nr.
	b3Vec3r normal2, nr, anr;
	if (code <= 3) {
        normal2 = normal;
	} else {
        normal2 = -normal;
	}
    nr = Rb.transpose() * normal2;
    anr = nr.abs();

	// find the largest compontent of anr: this corresponds to the normal
	// for the indident face. the other axis numbers of the indicent face
	// are stored in a1,a2.
	int lanr, a1, a2;
	if (anr[1] > anr[0]) {
		if (anr[1] > anr[2]) {
			a1 = 0;
			lanr = 1;
			a2 = 2;
		} else {
			a1 = 0;
			a2 = 1;
			lanr = 2;
		}
	} else {
		if (anr[0] > anr[2]) {
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
	if (nr[lanr] < 0) {
        center = pb - pa + Sb[lanr] * Rb.col(lanr);
	} else {
        center = pb - pa - Sb[lanr] * Rb.col(lanr);
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
    c1 = center.dot(Ra.col(code1));
    c2 = center.dot(Ra.col(code2));
	// optimize this? - we have already computed this data above, but it is not
	// stored in an easy-to-index format. for now it's quicker just to recompute
	// the four dot products.
    m11 = Ra.col(code1).dot(Rb.col(a1));
    m12 = Ra.col(code1).dot(Rb.col(a2));
    m21 = Ra.col(code2).dot(Rb.col(a1));
    m22 = Ra.col(code2).dot(Rb.col(a2));
	{
		real k1 = m11 * Sb[a1];
		real k2 = m21 * Sb[a1];
		real k3 = m12 * Sb[a2];
		real k4 = m22 * Sb[a2];
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
	rect[0] = Sa[code1];
	rect[1] = Sa[code2];

	// intersect the incident and reference faces
	real ret[16];
	int n = intersectRectQuad2(rect, quad, ret);
	if (n < 1) return 0;  // this should never happen

	// convert the intersection points into reference-face coordinates,
	// and compute the contact position and depth for each point. only keep
	// those points that have a positive (penetrating) depth. delete points in
	// the 'ret' array as necessary so that 'point' and 'ret' correspond.
	real point[3 * 8];  // penetrating contact points
	real dep[8];        // depths for those points
	real det1 = 1.f / (m11 * m22 - m12 * m21);
	m11 *= det1;
	m12 *= det1;
	m21 *= det1;
	m22 *= det1;
	int cnum = 0;  // number of penetrating contact points found
	for (j = 0; j < n; j++) {
		real k1 = m22 * (ret[j * 2] - c1) - m12 * (ret[j * 2 + 1] - c2);
		real k2 = -m21 * (ret[j * 2] - c1) + m11 * (ret[j * 2 + 1] - c2);
		for (i = 0; i < 3; i++) {
            point[cnum * 3 + i] = center[i] + k1 * Rb(i, a1) + k2 * Rb(i, a2);
        }
		dep[cnum] = Sa[codeN];
        for (int k = 0; k < 3; k++) {
            dep[cnum] -= normal2[k] * point[cnum * 3 + k];
        }
		if (dep[cnum] >= 0) {
			ret[cnum * 2] = ret[j * 2];
			ret[cnum * 2 + 1] = ret[j * 2 + 1];
			cnum++;
		}
	}
    if (cnum < 1) return 0;  // this should never happen

	if (cnum <= 4) {
		if (code < 4) {
			// we have less contacts than we need, so we use them all
			for (j = 0; j < cnum; j++) {
				b3Vec3r pointInWorld;
				for (i = 0; i < 3; i++)
					pointInWorld[i] = point[j * 3 + i] + pa[i] + normal[i] * dep[j];
				output.add_contact_point(normal, pointInWorld, -dep[j]);
			}
		} else {
			// we have less contacts than we need, so we use them all
			for (j = 0; j < cnum; j++) {
				b3Vec3r pointInWorld;
				for (i = 0; i < 3; i++)
					pointInWorld[i] = point[j * 3 + i] + pa[i];
				output.add_contact_point(normal, pointInWorld, -dep[j]);
			}
		}
	} else {
		// we have more contacts than are wanted, some of them must be culled.
		// find the deepest point, it is always the first contact.
		int i1 = 0;
		real max_depth = dep[0];
		for (i = 1; i < cnum; i++) {
			if (dep[i] > max_depth) {
				max_depth = dep[i];
				i1 = i;
			}
		}

        // cull to 4 points
		int iret[MAX_CONTACT_POINT_COUNT];
		cullPoints2(cnum, ret, i1, iret);

		for (j = 0; j < MAX_CONTACT_POINT_COUNT; j++) {
			b3Vec3r posInWorld;
			for (i = 0; i < 3; i++)
				posInWorld[i] = point[iret[j] * 3 + i] + pa[i];
			if (code < 4) {
				output.add_contact_point(normal, posInWorld + normal * dep[iret[j]], -dep[iret[j]]);
			} else {
				output.add_contact_point(normal, posInWorld, -dep[iret[j]]);
			}
		}
		cnum = 4;
	}

	return cnum;
}

void b3BoxBoxDetector::get_closest_points(b3ManifoldResult &output)
{
    b3Body* bodyA = m_fixtureA->get_body();
    b3Body* bodyB = m_fixtureB->get_body();

    b3Transformr xfA, xfB;
    xfA.set(bodyA->get_position(), bodyA->get_quaternion());
    xfB.set(bodyB->get_position(), bodyB->get_quaternion());

	b3Vec3r normal;
	real depth;

    auto boxA = (b3CubeShape*)m_fixtureA->get_shape();
    auto boxB = (b3CubeShape*)m_fixtureB->get_shape();

	dBoxBox2(xfA, boxA->m_xyz,
			 xfB, boxB->m_xyz,
			 normal, &depth, output);
}
