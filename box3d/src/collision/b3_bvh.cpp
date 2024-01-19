
#include "collision/b3_bvh.hpp"

#include "common/b3_common.hpp"

#include "collision/b3_fixture.hpp"

#include "common/b3_block_allocator.hpp"


b3Node::b3Node():
    m_aabb(b3AABB()),
    m_child1(b3_NULL_NODE),
    m_child2(b3_NULL_NODE),
    m_height(b3_NULL_HEIGHT)
{
    m_next = b3_NULL_NODE;
}


b3DynamicTree::b3DynamicTree()
{
    m_root = b3_NULL_NODE;

    m_node_capacity = 16;
    m_node_count = 0;

    m_free_list = 0;

    // Allocate memory space after the initialization of m_block_allocator
}


void b3DynamicTree::set_block_allocator(b3BlockAllocator* block_allocator) {
    m_block_allocator = block_allocator;

    m_nodes = (b3Node*) m_block_allocator->allocate(m_node_capacity * sizeof(b3Node));
    memset(m_nodes, 0, m_node_capacity * sizeof(b3Node));

    for (int32 i = 0; i < m_node_capacity - 1; ++i) {
        m_nodes[i].m_next = i + 1;
        m_nodes[i].m_height = b3_NULL_HEIGHT;
    }

    m_nodes[m_node_capacity - 1].m_next = b3_NULL_NODE;
    m_nodes[m_node_capacity - 1].m_height = b3_NULL_HEIGHT;
}


b3DynamicTree::~b3DynamicTree()
{
    m_block_allocator->free(m_nodes, m_node_capacity * sizeof(b3Node));
    // In the class b3World delete.
    m_block_allocator = nullptr;
}


int32 b3DynamicTree::create_bvh_proxy(const b3AABB &aabb, b3FixtureProxy* fixture_proxy)
{

    int32 bvh_proxy_id = assign_node();

    b3Vector3d r(b3_aabb_extension, b3_aabb_extension, b3_aabb_extension);
    m_nodes[bvh_proxy_id].m_aabb.m_min = aabb.m_min - r;
    m_nodes[bvh_proxy_id].m_aabb.m_max = aabb.m_max + r;

    m_nodes[bvh_proxy_id].m_height = 0;

    m_nodes[bvh_proxy_id].m_moved = true;

    m_nodes[bvh_proxy_id].fixture_proxy = fixture_proxy;

    insert_to_leaf(bvh_proxy_id);

    return bvh_proxy_id;
}


void b3DynamicTree::destroy_bvh_proxy(int32 proxy_id)
{
    b3_assert(0 <= proxy_id && proxy_id < m_node_capacity);
    b3_assert(m_nodes[proxy_id].is_leaf());


}


int32 b3DynamicTree::assign_node()
{
    // If there is no empty node in allocated m_nodes,
    // then expands the m_nodes
    if (m_free_list == b3_NULL_NODE) {
        expand_node_list(m_node_capacity * 2);
    }

    // assign the first free node to the caller
    // set m_free_list to the next node of the current assigned node
    // also initialize the m_free_list node
    int32 free_node = m_free_list;

    m_free_list = m_nodes[free_node].m_next;
    m_nodes[free_node].m_parent = b3_NULL_NODE;
    m_nodes[free_node].m_child1 = b3_NULL_NODE;
    m_nodes[free_node].m_child2 = b3_NULL_NODE;
    m_nodes[free_node].m_height = 0;
    m_nodes[free_node].m_moved = false;
    ++m_node_count;

    return free_node;
}


void b3DynamicTree::recycle_node(int32 i_node)
{
    b3_assert(0 <= i_node && i_node <= m_node_capacity);
    b3_assert(0 <= m_node_count);

    m_nodes[i_node].m_next = m_free_list;
    m_nodes[i_node].m_height = b3_NULL_HEIGHT;
    m_free_list = i_node;
    --m_node_count;
}


void b3DynamicTree::insert_to_leaf(int32 leaf)
{
    if (m_root == b3_NULL_NODE) {
        m_root = leaf;
        m_nodes[m_root].m_parent = b3_NULL_NODE;
        return;
    }

    // Get this leaf's AABB
    b3AABB node_aabb = m_nodes[leaf].m_aabb;

    // insert this leaf to the leaf
    // find the best sibling to insert
    int32 f_index = m_root;
    while (!m_nodes[f_index].is_leaf()) {

        int32 child1 = m_nodes[f_index].m_child1;
        int32 child2 = m_nodes[f_index].m_child2;

        float f_area = m_nodes[f_index].m_aabb.get_surface_area();

        b3AABB combined_f_aabb;
        combined_f_aabb.combine(m_nodes[f_index].m_aabb, node_aabb);

        float combined_f_sa = combined_f_aabb.get_surface_area();

        // TODO: compare the cost of Erin with Jeffrey Goldsmith's
        // From Zhenyu Xu's perspective, Jeffrey Goldsmith's cost is more reasonable

        // Every time a parent is created,
        // the possibility of hitting the parent is SA(parent)/SA(root).
        // If hit, two children will do hit check.
        float cost_combine_f = 2.0f * combined_f_sa;

        // If the leaf will be inserted to next level,
        // The SA expansion of this level is SA(new) - SA(old).
        // If hit, two children will do hit check.
        float cost_inheritance = 2.0f * (combined_f_sa - f_area);

        // Firstly, insert into child1
        float cost1;
        if (m_nodes[child1].is_leaf()) {
            b3AABB combined_c1_aabb;
            combined_c1_aabb.combine(m_nodes[child1].m_aabb, node_aabb);

            // If insert into the child,
            // The combination with the child will generate a new leaf
            // This leaf will have the possibility of being hit SA(leaf)/SA(root)
            // If hit, two leaf will do hit check.
            cost1 = 2.0f * combined_c1_aabb.get_surface_area() + cost_inheritance;
        } else {
            b3AABB combined_c1_aabb;
            combined_c1_aabb.combine(m_nodes[child1].m_aabb, node_aabb);

            // If child1 is not a leaf,
            // Combining to it will continue to descending.
            // So the cost contains two parts.
            float old_c1_sa = m_nodes[child1].m_aabb.get_surface_area();
            float new_c1_sa = combined_c1_aabb.get_surface_area();
            cost1 = 2 * (new_c1_sa - old_c1_sa) + cost_inheritance;
        }

        // Next, insert into child2
        float cost2;
        if (m_nodes[child2].is_leaf()) {
            b3AABB combined_c2_aabb;
            combined_c2_aabb.combine(m_nodes[child2].m_aabb, node_aabb);
            cost2 = 2.0f * combined_c2_aabb.get_surface_area() + cost_inheritance;
        } else {
            b3AABB combined_c2_aabb;
            combined_c2_aabb.combine(m_nodes[child2].m_aabb, node_aabb);

            float old_c2_sa = m_nodes[child2].m_aabb.get_surface_area();
            float new_c2_sa = combined_c2_aabb.get_surface_area();
            cost2 = 2 * (new_c2_sa - old_c2_sa) + cost_inheritance;
        }

        if (cost_combine_f < cost1 && cost_combine_f < cost2)
            break;

        if (cost1 < cost2)
            f_index = child1;
        else
            f_index = child2;
    }

    int32 sibling = f_index;

    // Create A new parent for the sibling and the insertion leaf
    int32 old_parent = m_nodes[sibling].m_parent;
    int32 new_parent = assign_node();
    m_nodes[new_parent].m_parent = old_parent;
    m_nodes[new_parent].m_aabb.combine(node_aabb, m_nodes[sibling].m_aabb);
    m_nodes[new_parent].m_height = m_nodes[sibling].m_height + 1;

    if (old_parent != b3_NULL_NODE) {
        // The sibling is not the root
        if (m_nodes[old_parent].m_child1 == sibling)
            // If the left one is the sibling,
            // then the new parent is the parent's left child
            m_nodes[old_parent].m_child1 = new_parent;
        else
            m_nodes[old_parent].m_child2 = new_parent;

        // Setup relationship between new parent and two children
        m_nodes[new_parent].m_child1 = sibling;
        m_nodes[new_parent].m_child2 = leaf;
        m_nodes[sibling].m_parent = new_parent;
        m_nodes[leaf].m_parent = new_parent;
    } else {
        // The sibling is the root
        // After insertion, the new parent is the root
        m_nodes[new_parent].m_child1 = sibling;
        m_nodes[new_parent].m_child2 = leaf;
        m_nodes[sibling].m_parent = new_parent;
        m_nodes[leaf].m_parent = new_parent;
        m_root = new_parent;

    }

    // After insertion, adjust all parents through the pass
    f_index = m_nodes[leaf].m_parent;
    while (f_index != b3_NULL_NODE) {
        f_index = balance(f_index);

        int32 child1 = m_nodes[f_index].m_child1;
        int32 child2 = m_nodes[f_index].m_child2;

        b3_assert(child1 != b3_NULL_NODE);
        b3_assert(child2 != b3_NULL_NODE);

        m_nodes[f_index].m_height = 1 + b3_max(m_nodes[child1].m_height, m_nodes[child2].m_height);
        m_nodes[f_index].m_aabb.combine(m_nodes[child1].m_aabb, m_nodes[child2].m_aabb);

        f_index = m_nodes[f_index].m_parent;
    }
}

void b3DynamicTree::remove_leaf(int32 leaf)
{
    if (leaf == m_root) {
        m_root = b3_NULL_NODE;
        return;
    }

    int32 i_parent = m_nodes[leaf].m_parent;
    int32 i_grand_parent = m_nodes[i_parent].m_parent;
    int32 i_sibling;

    // find the sibling, whether a subtree or leaf
    if (m_nodes[i_parent].m_child1 == leaf)
        i_sibling = m_nodes[i_parent].m_child2;
    else
        i_sibling = m_nodes[i_parent].m_child1;

    // If grandparent of this leaf exists
    if (i_grand_parent != b3_NULL_NODE) {
        // Destroy the parent because only this parent has only one child
        if (m_nodes[i_grand_parent].m_child1 == i_parent)
            m_nodes[i_grand_parent].m_child1 = i_sibling;
        else
            m_nodes[i_grand_parent].m_child2 = i_sibling;

        m_nodes[i_sibling].m_parent = i_grand_parent;
        recycle_node(i_parent);

        int32 f_index = i_grand_parent;
        while (f_index != b3_NULL_NODE) {
            f_index = balance(f_index);

            int32 child1 = m_nodes[f_index].m_child1;
            int32 child2 = m_nodes[f_index].m_child2;

            m_nodes[f_index].m_aabb.combine(m_nodes[child1].m_aabb, m_nodes[child2].m_aabb);
            m_nodes[f_index].m_height = 1 + b3_max(m_nodes[child1].m_height, m_nodes[child2].m_height);

            f_index = m_nodes[f_index].m_parent;
        }
    } else {
        // If grandparent of this leaf does not exist
        // The sibling becomes the root
        m_root = i_sibling;
        m_nodes[i_sibling].m_parent = b3_NULL_NODE;
        recycle_node(i_parent);
    }
}


void b3DynamicTree::expand_node_list(int32 count)
{

    // Check whether the list is full
    b3_assert(m_node_count == m_node_capacity);

    m_node_capacity = count;

    // TODO: check if this is the best way to expand the list
    // Copy old nodes to a larger list of nodes
    b3Node* old_nodes = m_nodes;
    m_nodes = (b3Node*) m_block_allocator->allocate(m_node_capacity * sizeof(b3Node));
    memcpy(m_nodes, old_nodes, m_node_count * sizeof(b3Node));
    m_block_allocator->free(old_nodes, m_node_count * sizeof(b3Node));

    // Establish relationship between nodes in the list
    for (int32 i = m_node_count; i < m_node_capacity - 1; ++i) {
        m_nodes[i].m_next = i + 1;
        m_nodes[i].m_height = b3_NULL_HEIGHT;
    }
    m_nodes[m_node_capacity - 1].m_next = b3_NULL_NODE;
    m_nodes[m_node_capacity - 1].m_height = b3_NULL_HEIGHT;

    // The first free node is at m_node_count
    m_free_list = m_node_count;
}


int32 b3DynamicTree::balance(int32 i_A)
{
    b3_assert(i_A != b3_NULL_NODE);

    // pointer to the i_A proxy
    b3Node* p_A = m_nodes + i_A;

    // The subtree on i_A is already balanced
    if (p_A->is_leaf() || p_A->m_height < 2)
        return i_A;

    //              A
    //            /   \
    //           B     C
    //          / \   / \
    //         D   E F   G
    int32 i_B = p_A->m_child1;
    int32 i_C = p_A->m_child2;

    b3_assert(0 <= i_B && i_B < m_node_capacity);
    b3_assert(0 <= i_C && i_C < m_node_capacity);

    b3Node* p_B = m_nodes + i_B;
    b3Node* p_C = m_nodes + i_C;

    int32 balance = p_C->m_height - p_B->m_height;

    if (balance > 1) {
        int32 i_F = p_C->m_child1;
        int32 i_G = p_C->m_child2;

        b3Node* p_F = m_nodes + i_F;
        b3Node* p_G = m_nodes + i_G;

        p_C->m_child1 = i_A;
        p_C->m_parent = p_A->m_parent;
        p_A->m_parent = i_C;

        // Setup i_A's parent to C
        if (p_C->m_parent != b3_NULL_NODE) {
            if (m_nodes[p_C->m_parent].m_child1 == i_A) {
                m_nodes[p_C->m_parent].m_child1 = i_C;
            } else {
                b3_assert(m_nodes[p_C->m_parent].m_child2 == i_A);
                m_nodes[p_C->m_parent].m_child2 = i_C;
            }
        } else {
            m_root = i_C;
        }

        if (p_F->m_height > p_G->m_height) {
            p_C->m_child2 = i_F;
            p_A->m_child2 = i_G;
            p_G->m_parent = i_A;

            p_A->m_aabb.combine(p_B->m_aabb, p_G->m_aabb);
            p_C->m_aabb.combine(p_A->m_aabb, p_F->m_aabb);

            p_A->m_height = 1 + b3_max(p_B->m_height, p_G->m_height);
            p_C->m_height = 1 + b3_max(p_A->m_height, p_F->m_height);
        } else {
            p_C->m_child2 = i_G;
            p_A->m_child2 = i_F;
            p_F->m_parent = i_A;

            p_A->m_aabb.combine(p_B->m_aabb, p_F->m_aabb);
            p_C->m_aabb.combine(p_A->m_aabb, p_G->m_aabb);

            p_A->m_height = 1 + b3_max(p_B->m_height, p_F->m_height);
            p_C->m_height = 1 + b3_max(p_A->m_height, p_G->m_height);
        }
        return i_C;
    }

    if (balance < -1) {
        int32 i_D = p_B->m_child1;
        int32 i_E = p_B->m_child2;

        b3Node* p_D = m_nodes + i_D;
        b3Node* p_E = m_nodes + i_E;

        p_B->m_child1 = i_A;
        p_B->m_parent = p_A->m_parent;
        p_A->m_parent = i_B;

        // Setup i_A's parent to B
        if (p_B->m_parent != b3_NULL_NODE) {
            if (m_nodes[p_B->m_parent].m_child1 == i_A) {
                m_nodes[p_B->m_parent].m_child1 = i_B;
            } else {
                b3_assert(m_nodes[p_B->m_parent].m_child2 == i_A);
                m_nodes[p_B->m_parent].m_child2 = i_B;
            }
        } else {
            m_root = i_B;
        }


        if (p_D->m_height > p_E->m_height) {
            p_B->m_child2 = i_D;
            p_A->m_child1 = i_E;
            p_E->m_parent = i_A;

            p_A->m_aabb.combine(p_C->m_aabb, p_E->m_aabb);
            p_B->m_aabb.combine(p_A->m_aabb, p_D->m_aabb);

            p_A->m_height = 1 + b3_max(p_C->m_height, p_E->m_height);
            p_B->m_height = 1 + b3_max(p_A->m_height, p_D->m_height);
        } else {
            p_B->m_child2 = i_E;
            p_A->m_child1 = i_D;
            p_D->m_parent = i_A;

            p_A->m_aabb.combine(p_C->m_aabb, p_D->m_aabb);
            p_B->m_aabb.combine(p_A->m_aabb, p_E->m_aabb);

            p_A->m_height = 1 + b3_max(p_C->m_height, p_D->m_height);
            p_B->m_height = 1 + b3_max(p_A->m_height, p_E->m_height);
        }

        return i_B;
    }
    return i_A;
}

bool b3DynamicTree::move_proxy(int32 proxy_id, const b3AABB &aabb)
{
    b3_assert(0 <= proxy_id && proxy_id < m_node_capacity);

    b3_assert(m_nodes[proxy_id].is_leaf());

    const b3AABB& tree_aabb = m_nodes[proxy_id].m_aabb;

    if (tree_aabb.contains(aabb)) {
        return false;
    }

    remove_leaf(proxy_id);

    m_nodes[proxy_id].m_aabb = aabb;
    insert_to_leaf(proxy_id);

    return true;
}









