
#include "utils/b3_io.hpp"

#include "utils/b3_log.hpp"


std::string remove_newline(std::string s) {
    s.erase(std::remove(s.begin(), s.end(), '\n'), s.end());
    return s;
}


bool read_obj(const std::string& obj_file_name,
              std::vector<std::vector<double>> &V,
              std::vector<std::vector<double>> &TC,
              std::vector<std::vector<double>> &N,
              std::vector<std::vector<int>> &F,
              std::vector<std::vector<int>> &FTC,
              std::vector<std::vector<int>> &FN,
              std::vector<std::vector<int>> &L) {
    // Open file, and check for error
    FILE* obj_file = fopen(obj_file_name.c_str(), "r");
    if (obj_file == nullptr) {
        spdlog::error("read_obj: {:s} could not be opened!", obj_file_name);
        return false;
    }
    return read_obj(obj_file, V, TC, N, F, FTC, FN, L);
}


// This file is related to rigid-ipc and igl
// For more information, please access:
// https://github.com/libigl/libigl/blob/main/include/igl/readOBJ.h and
// https://github.com/ipc-sim/rigid-ipc/blob/main/src/io/read_obj.hpp
bool read_obj(FILE *obj_file,
              std::vector<std::vector<double>> &V,
              std::vector<std::vector<double>> &TC,
              std::vector<std::vector<double>> &N,
              std::vector<std::vector<int>> &F,
              std::vector<std::vector<int>> &FTC,
              std::vector<std::vector<int>> &FN,
              std::vector<std::vector<int>> &L)
{
    // File open was successful so clear outputs
    V.clear();
    TC.clear();
    N.clear();
    F.clear();
    FTC.clear();
    FN.clear();
    L.clear();

    // variables and constants to assist parsing the .obj file
    // Constant strings to compare against
    std::string v("v");
    std::string vn("vn");
    std::string vt("vt");
    std::string f("f");
    std::string l("l");
    std::string tic_tac_toe("#");

    const int LINE_MAX_LEN = 2048;

    char line[LINE_MAX_LEN];
    int line_no = 1;
    while (fgets(line, LINE_MAX_LEN, obj_file) != nullptr) {
        char type[LINE_MAX_LEN];
        // Read first word containing type
        if (sscanf(line, "%s", type) == 1) {
            // Get pointer to rest of line right after type
            char* rest_of_line = &line[strlen(type)];
            if (type == v) {
                std::istringstream ls(&line[1]);
                std::vector<double> vertex { std::istream_iterator<double>(ls),
                                             std::istream_iterator<double>() };

                // if (vertex.size() < 3) {
                //     spdlog::error(
                //         "read_obj: vertex on line {:d} should have at "
                //         "least 3 coordinates",
                //         line_no);
                //     fclose(obj_file);
                //     return false;
                // }

                V.push_back(vertex);
            } else if (type == vn) {
                double x[3];
                int count =
                        sscanf(rest_of_line, "%lf %lf %lf\n", &x[0], &x[1], &x[2]);
                if (count != 3) {
                    spdlog::error(
                            "read_obj: normal on line {:d} should have 3 "
                            "coordinates",
                            line_no);
                    fclose(obj_file);
                    return false;
                }
                std::vector<double> normal(count);
                for (int i = 0; i < count; i++) {
                    normal[i] = x[i];
                }
                N.push_back(normal);
            } else if (type == vt) {
                double x[3];
                int count =
                        sscanf(rest_of_line, "%lf %lf %lf\n", &x[0], &x[1], &x[2]);
                if (count != 2 && count != 3) {
                    spdlog::error(
                            "read_obj: texture coords on line {:d} should have "
                            "2 or 3 coordinates (has {:d})",
                            line_no, count);
                    fclose(obj_file);
                    return false;
                }
                std::vector<double> tex(count);
                for (int i = 0; i < count; i++) {
                    tex[i] = x[i];
                }
                TC.push_back(tex);
            } else if (type == f) {
                const auto& shift = [&V](const int i) -> int {
                    return i < 0 ? i + V.size() : i - 1;
                };
                const auto& shift_t = [&TC](const int i) -> int {
                    return i < 0 ? i + TC.size() : i - 1;
                };
                const auto& shift_n = [&N](const int i) -> int {
                    return i < 0 ? i + N.size() : i - 1;
                };
                std::vector<int> f;
                std::vector<int> ftc;
                std::vector<int> fn;
                // Read each "word" after type
                char word[LINE_MAX_LEN];
                int offset;
                while (sscanf(rest_of_line, "%s%n", word, &offset) == 1) {
                    // adjust offset
                    rest_of_line += offset;
                    // Process word
                    long int i, it, in;
                    if (sscanf(word, "%ld/%ld/%ld", &i, &it, &in) == 3) {
                        f.push_back(shift(i));
                        ftc.push_back(shift_t(it));
                        fn.push_back(shift_n(in));
                    } else if (sscanf(word, "%ld/%ld", &i, &it) == 2) {
                        f.push_back(shift(i));
                        ftc.push_back(shift_t(it));
                    } else if (sscanf(word, "%ld//%ld", &i, &in) == 2) {
                        f.push_back(shift(i));
                        fn.push_back(shift_n(in));
                    } else if (sscanf(word, "%ld", &i) == 1) {
                        f.push_back(shift(i));
                    } else {
                        spdlog::error(
                                "read_obj: face on line {:d} has invalid "
                                "element format",
                                line_no);
                        fclose(obj_file);
                        return false;
                    }
                }
                if ((f.size() > 0 && fn.size() == 0 && ftc.size() == 0)
                    || (f.size() > 0 && fn.size() == f.size()
                        && ftc.size() == 0)
                    || (f.size() > 0 && fn.size() == 0
                        && ftc.size() == f.size())
                    || (f.size() > 0 && fn.size() == f.size()
                        && ftc.size() == f.size())) {
                    // No matter what add each type to lists so that lists
                    // are the correct lengths
                    F.push_back(f);
                    FTC.push_back(ftc);
                    FN.push_back(fn);
                } else {
                    spdlog::error(
                            "read_obj: face on line {:d} has invalid format",
                            line_no);
                    fclose(obj_file);
                    return false;
                }

            } else if (type == l) {
                std::istringstream ls(&line[1]);
                std::vector<int> polyline { std::istream_iterator<int>(ls),
                                            std::istream_iterator<int>() };

                if (polyline.size() < 2) {
                    spdlog::error(
                            "read_obj: line element on line {:d} should have "
                            "at least 2 vertices",
                            line_no);
                    fclose(obj_file);
                    return false;
                }

                for (int i = 0; i < polyline.size(); i++) {
                    polyline[i] = polyline[i] < 0 ? polyline[i] + V.size()
                                                  : polyline[i] - 1;
                }

                L.push_back(polyline);
            } else if (
                    strlen(type) >= 1
                    && (type[0] == '#' || type[0] == 'g' || type[0] == 's'
                        || strcmp("usemtl", type) == 0
                        || strcmp("mtllib", type) == 0)) {
                // ignore comments or other stuff
            } else {
                // ignore any other lines
                std::string line_no_newline = remove_newline(line);
                spdlog::warn(
                        "read_obj: ignored non-comment line {:d}: {:s}", line_no,
                        line_no_newline);
            }
        } else {
            // ignore empty line
        }
        line_no++;
    }
    fclose(obj_file);

    assert(F.size() == FN.size());
    assert(F.size() == FTC.size());

    return true;
}


