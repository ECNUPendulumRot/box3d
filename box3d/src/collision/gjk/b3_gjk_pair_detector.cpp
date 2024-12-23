
#include "collision/gjk/b3_gjk_pair_detector.hpp"

#include "geometry/b3_shape.hpp"
#include "collision/b3_manifold_result.hpp"

#include "collision/gjk/b3_penetration_depth_solver.hpp"

#include <stdio.h>

#include <iostream>

b3GjkPairDetector::b3GjkPairDetector(const b3Shape *shapeA, const b3Shape *shapeB, b3VoronoiSimplexSolver *simplex_solver, b3PenetrationDepthSolver *penetration_depth_solver)
    : m_penetration_depth_solver(penetration_depth_solver),
      m_simplex_solver(simplex_solver),
      m_minkowskiA(shapeA),
      m_minkowskiB(shapeB),
      m_marginA(shapeA->get_margin()),
      m_marginB(shapeB->get_margin()),
      m_cached_separating_distance(0.f),
      m_cur_iter(0),
      m_cached_separating_axis(0, 1, 0),
      m_degenerate_simplex(0) {

}

void b3GjkPairDetector::get_closest_points(const b3GjkPairDetector::ClosestPointInput &input, b3ManifoldResult &result) {
    real distance = real(0);
    b3Vec3r normalInB = b3Vec3r::zero();

    b3Transformr local_transA = input.m_transformA;
    b3Transformr local_transB = input.m_transformB;
    b3Vec3r position_offset = (local_transA.position() + local_transB.position()) * real(0.5);
    local_transA.set_position(local_transA.position() - position_offset);
    local_transB.set_position(local_transB.position() - position_offset);

    int gGjk_max_iter = 1000;

    bool is_valid = false;
    bool check_simplex = false;
    bool check_penetration = true;

    b3Vec3r org_normal_inB = b3Vec3r::zero();
    real margin = m_marginA + m_marginB;

    real squared_distance = b3_real_max;
    real delta = 0;

    m_simplex_solver->reset();
    b3Mat33r RA_T = local_transA.rotation_matrix().transpose();
    b3Mat33r RB_T = local_transB.rotation_matrix().transpose();
    b3Vec3r pointOnA, pointOnB;
    while (true) {
        b3Vec3r separating_axis_inA = RA_T * (-m_cached_separating_axis);
        b3Vec3r separating_axis_inB = RB_T * m_cached_separating_axis;

//        std::cout << "m_cachedSeparatingAxis = " << m_cached_separating_axis.x << ", " << m_cached_separating_axis.y << ", " << m_cached_separating_axis.z << std::endl;
//        std::cout << "separatingAxisInA = " << separating_axis_inA.x << ", " << separating_axis_inA.y << ", " << separating_axis_inA.z << std::endl;
//        std::cout << "separatingAxisInB = " << separating_axis_inB.x << ", " << separating_axis_inB.y << ", " << separating_axis_inB.z << std::endl;

        b3Vec3r p_inA = m_minkowskiA->local_get_support_vertex(separating_axis_inA);
        b3Vec3r q_inB = m_minkowskiB->local_get_support_vertex(separating_axis_inB);

//        std::cout << "pInA = " << p_inA.x << ", " << p_inA.y << ", " << p_inA.z << std::endl;
//        std::cout << "qInB = " << q_inB.x << ", " << q_inB.y << ", " << q_inB.z << std::endl;

        b3Vec3r p_world = local_transA.transform(p_inA);
        b3Vec3r q_world = local_transB.transform(q_inB);

//        std::cout << "pWorld = " << p_world.x << ", " << p_world.y << ", " << p_world.z << std::endl;
//        std::cout << "qWorld = " << q_world.x << ", " << q_world.y << ", " << q_world.z << std::endl;

        b3Vec3r w = p_world - q_world;
        delta = m_cached_separating_axis.dot(w);

        // potential exit, they don't overlap
        if (delta > 0 && (delta * delta > squared_distance * input.m_maximum_distance_squared)) {
            m_degenerate_simplex = 10;
            check_simplex = true;
            break;
        }

        //exit 0: the new point is already in the simplex, or we didn't come any closer
        if (m_simplex_solver->in_simplex(w)) {
            m_degenerate_simplex = 1;
            check_simplex = true;
            break;
        }

        real f0 = squared_distance - delta;
        real f1 = squared_distance * b3_rel_error2;

        if (f0 <= f1) {
            if (f0 <= 0) {
                m_degenerate_simplex = 2;
            } else {
                m_degenerate_simplex = 12;
            }
            check_simplex = true;
            break;
        }

        //add current vertex to simplex
        m_simplex_solver->add_vertex(w, p_world, q_world);
        b3Vec3r new_cached_separating_axis;

        //calculate the closest point to the origin (update vector v)
        if (!m_simplex_solver->closest(new_cached_separating_axis)) {
            m_degenerate_simplex = 3;
            check_simplex = true;
            break;
        }

        if (new_cached_separating_axis.length2() < b3_rel_error2) {
            m_cached_separating_axis = new_cached_separating_axis;
            m_degenerate_simplex = 6;
            check_simplex = true;
            break;
        }

        real previous_squared_distance = squared_distance;
        squared_distance = new_cached_separating_axis.length2();

        //are we getting any closer ?
        if (previous_squared_distance - squared_distance <= b3_real_epsilon * previous_squared_distance) {
            check_simplex = true;
            m_degenerate_simplex = 12;
            break;
        }

        m_cached_separating_axis = new_cached_separating_axis;

        if (m_cur_iter++ > gGjk_max_iter) {
            printf("Gjk Pair Detector maxIter exceeded:%i\n", m_cur_iter);
            break;
        }
        if (m_simplex_solver->full_simplex()) {
            m_degenerate_simplex = 13;
            break;
        }
    }

    if (check_simplex) {
        m_simplex_solver->compute_points(pointOnA, pointOnB);
        normalInB = m_cached_separating_axis;

        real len_sqr = m_cached_separating_axis.length2();

        // valid normal
        if (len_sqr < b3_rel_error2) {
            m_degenerate_simplex = 5;
        }
        if (len_sqr > b3_real_epsilon * b3_real_epsilon) {
            real rlen = real(1.) / b3_sqrt(len_sqr);
            normalInB *= rlen;  // normalize

            real s = b3_sqrt(squared_distance);

            b3_assert(s > real(0.0));
            pointOnA -= m_cached_separating_axis * (m_marginA / s);
            pointOnB += m_cached_separating_axis * (m_marginB / s);
            distance = ((real(1.) / rlen) - margin);
            is_valid = true;
            org_normal_inB = normalInB;
        }
    }

    bool catch_degenerate_penetration_case = m_penetration_depth_solver && m_degenerate_simplex && ((distance + margin) < 0.001);

    if ((check_penetration && (!is_valid || catch_degenerate_penetration_case))) {
        // penetration case

        if (m_penetration_depth_solver) {
            // Penetration depth case.
            b3Vec3r tmp_point_onA, tmp_point_onB;

            m_cached_separating_axis.set_zero();

            bool isValid2 = m_penetration_depth_solver->calcPenDepth(
                *m_simplex_solver,
                m_minkowskiA, m_minkowskiB,
                local_transA, local_transB,
                m_cached_separating_axis, tmp_point_onA, tmp_point_onB);

            if (m_cached_separating_axis.length2()) {
                if (isValid2) {
                    b3Vec3r tmp_normal_inB = tmp_point_onB - tmp_point_onA;
                    real len_sqr = tmp_normal_inB.length2();
                    if (len_sqr <= b3_real_epsilon * b3_real_epsilon) {
                        tmp_normal_inB = m_cached_separating_axis;
                        len_sqr = m_cached_separating_axis.length2();
                    }

                    if (len_sqr > b3_real_epsilon * b3_real_epsilon) {
                        tmp_normal_inB /= b3_sqrt(len_sqr);
                        real distance2 = -(tmp_point_onA - tmp_point_onB).length();
                        //only replace valid penetrations when the result is deeper (check)
                        if (!is_valid || (distance2 < distance)) {
                            distance = distance2;
                            pointOnA = tmp_point_onA;
                            pointOnB = tmp_point_onB;
                            normalInB = tmp_normal_inB;
                            is_valid = true;
                        }
                    }
                } else {
                    ///this is another degenerate case, where the initial GJK calculation reports a degenerate case
                    ///EPA reports no penetration, and the second GJK (using the supporting vector without margin)
                    ///reports a valid positive distance. Use the results of the second GJK instead of failing.
                    ///thanks to Jacob.Langford for the reproduction case
                    ///http://code.google.com/p/bullet/issues/detail?id=250

                    if (m_cached_separating_axis.length2() > real(0.)) {
                        real distance2 = (tmp_point_onA - tmp_point_onB).length() - margin;
                        //only replace valid distances when the distance is less
                        if (!is_valid || (distance2 < distance)) {
                            distance = distance2;
                            pointOnA = tmp_point_onA;
                            pointOnB = tmp_point_onB;
                            pointOnA -= m_cached_separating_axis * m_marginA;
                            pointOnB += m_cached_separating_axis * m_marginB;
                            normalInB = m_cached_separating_axis;
                            normalInB.normalized();

                            is_valid = true;
                        }
                    }
                }
            } else {
                printf("EPA didn't return a valid value\n");
            }
        }
    }

    if (is_valid && (distance < 0 || distance * distance < input.m_maximum_distance_squared)) {
        m_cached_separating_axis = normalInB;
        m_cached_separating_distance = distance;

        b3Vec3r separating_axis_inA = RA_T * (-org_normal_inB);
        b3Vec3r separating_axis_inB = RB_T * org_normal_inB;

        b3Vec3r p_inA = m_minkowskiA->local_get_support_vertex(separating_axis_inA);
        b3Vec3r q_inB = m_minkowskiB->local_get_support_vertex(separating_axis_inB);

        b3Vec3r p_world = local_transA.transform(p_inA);
        b3Vec3r q_world = local_transB.transform(q_inB);
        b3Vec3r w = p_world - q_world;
        real d2 = org_normal_inB.dot(w) - margin;

        separating_axis_inA = RA_T * normalInB;
        separating_axis_inB = RB_T * (-normalInB);

        p_inA = m_minkowskiA->local_get_support_vertex(separating_axis_inA);
        q_inB = m_minkowskiB->local_get_support_vertex(separating_axis_inB);

        p_world = local_transA.transform(p_inA);
        q_world = local_transB.transform(q_inB);
        w = p_world - q_world;
        real d1 = (-normalInB).dot(w) - margin;

        separating_axis_inA = RA_T * (-normalInB);
        separating_axis_inB = RB_T * normalInB;

        p_inA = m_minkowskiA->local_get_support_vertex(separating_axis_inA);
        q_inB = m_minkowskiB->local_get_support_vertex(separating_axis_inB);

        p_world = local_transA.transform(p_inA);
        q_world = local_transB.transform(q_inB);
        w = p_world - q_world;
        real d0 = normalInB.dot(w) - margin;

        if (d1 > d0) {
            normalInB *= -1;
        }
        if (org_normal_inB.length2()) {
            if (d2 > d0 && d2 > d1 && d2 > distance) {
                normalInB = org_normal_inB;
                distance = d2;
            }
        }

        result.add_contact_point(normalInB, pointOnB + position_offset, distance);
    } else {
        printf("invalid gjk query\n");
    }
}

