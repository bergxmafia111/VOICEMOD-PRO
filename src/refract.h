#pragma once

#include <string>
#include <vector>
#include <Eigen/Core>


/**
 * @brief Parse an .obj file to extract vertices and normals.
 * 
 * @param[in]  objFilePath  Path to the .obj file.
 * @param[out] vertices     Populated with (x, y, z) vertex positions.
 * @param[out] normals      Populated with (nx, ny, nz) normal vectors.
 * 
 * @throws std::runtime_error if the file cannot be opened or parsed correctly.
 */
void ParseOBJ(const std::string &objFilePath,
              std::vector<Eigen::Vector3d> &vertices,
              std::vector<Eigen::Vector3d> &normals);


/**
 * @brief Compute refracted directions from the given normal vectors.
 * 
 * Incident direction is assumed to be (0, 0, 1). If total internal reflection 
 * occurs for a particular normal, that direction is substituted by a 
 * near-horizontal TIR direction.
 * 
 * @param[in]  normals      Normal vectors (one per vertex).
 * @param[out] refracteds   Refracted ray directions (same size as normals).
 * @param[in]  eta          Ratio of refractive indices (eta1 / eta2).
 */
void Refract(const std::vector<Eigen::Vector3d> &normals,
             std::vector<Eigen::Vector3d> &refracteds,
             double eta);


/**
 * @brief Calculate intersection points on a plane located at z = receiverDistance.
 * 
 * Each ray is treated as: vertex + t * refractedDir. The plane is parallel to X-Y.
 * 
 * @param[in]  vertices       Vertex positions (one per refracted direction).
 * @param[in]  refracteds     Refracted directions.
 * @param[out] intersections  2D positions where each ray intersects the plane,
 *                            scaled so the final output is in a [0..256]^2 range.
 * @param[in]  receiverDistance  The z-coordinate of the plane to intersect.
 */
void CalculateIntersections(const std::vector<Eigen::Vector3d> &vertices,
                            const std::vector<Eigen::Vector3d> &refracteds,
                            std::vector<Eigen::Vector2d> &intersections,
                            double receiverDistance);
