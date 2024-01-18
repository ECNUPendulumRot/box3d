
#ifndef BOX3D_B3_IO_HPP
#define BOX3D_B3_IO_HPP


#include <string>
#include <vector>
#include <Eigen/Core>


bool read_obj(const std::string& obj_file_name,
              std::vector<std::vector<double>>& V,
              std::vector<std::vector<double>>& TC,
              std::vector<std::vector<double>>& N,
              std::vector<std::vector<int>>& F,
              std::vector<std::vector<int>>& FTC,
              std::vector<std::vector<int>>& FN,
              std::vector<std::vector<int>>& L);

/// @brief Read a mesh from an ascii obj file
///
/// Fills in vertex positions, normals and texture coordinates. Mesh may
/// have faces of any number of degree.
///
/// @param[in] obj_file_name  path to .obj file
/// @param[out] V             double matrix of vertex positions
/// @param[out] TC            double matrix of texture coordinates
/// @param[out] N             double matrix of corner normals #N by 3
/// @param[out] F             #F list of face indices into vertex positions
/// @param[out] FTC           #F list of face indices into vertex texture
///                           coordinates
/// @param[out] FN            #F list of face indices into vertex normals
/// @param[out] L             list of polyline indices into vertex positions
///
/// @returns true on success, false on errors
bool read_obj(FILE* obj_file,
              std::vector<std::vector<double>>& V,
              std::vector<std::vector<double>>& TC,
              std::vector<std::vector<double>>& N,
              std::vector<std::vector<int>>& F,
              std::vector<std::vector<int>>& FTC,
              std::vector<std::vector<int>>& FN,
              std::vector<std::vector<int>>& L);


#endif //BOX3D_B3_IO_HPP
