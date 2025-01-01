#include "refract.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

// Helper function for string-splitting
static std::vector<std::string> SplitLine(const std::string &s, const std::string &delimiter) {
    std::vector<std::string> tokens;
    size_t pos_start = 0;
    const size_t delim_len = delimiter.length();
    while (true) {
        size_t pos_end = s.find(delimiter, pos_start);
        if (pos_end == std::string::npos) {
            // No more delimiters found; push last token
            tokens.emplace_back(s.substr(pos_start));
            break;
        }
        // Extract token
        tokens.emplace_back(s.substr(pos_start, pos_end - pos_start));
        pos_start = pos_end + delim_len;
    }
    return tokens;
}

// Reads an OBJ file and populates `vertices` and `normals` from "v" and "vn" lines.
// Ignores "vt" lines and everything else not relevant.
void ParseOBJ(const std::string &objFilePath,
              std::vector<Eigen::Vector3d> *vertices,
              std::vector<Eigen::Vector3d> *normals) {
    if (!vertices || !normals) {
        throw std::runtime_error("ParseOBJ: `vertices` or `normals` pointers are null.");
    }

    std::ifstream file(objFilePath);
    if (!file.is_open()) {
        std::cerr << "Could not open OBJ file: " << objFilePath << std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        // Trim leading/trailing whitespace (optional improvement)
        if (line.size() < 2) {
            continue;
        }

        // Check vertex line
        if (line.rfind("v ", 0) == 0) {
            auto lineContents = SplitLine(line, " ");
            if (lineContents.size() < 4) {
                continue; // skip malformed lines
            }
            // lineContents: ["v", "x", "y", "z", ...]
            double x = std::stod(lineContents[1]);
            double y = std::stod(lineContents[2]);
            double z = std::stod(lineContents[3]);
            vertices->emplace_back(x, y, z);
        }
        // Check normal line
        else if (line.rfind("vn", 0) == 0) {
            auto lineContents = SplitLine(line, " ");
            if (lineContents.size() < 4) {
                continue; // skip malformed lines
            }
            // lineContents: ["vn", "nx", "ny", "nz", ...]
            double nx = std::stod(lineContents[1]);
            double ny = std::stod(lineContents[2]);
            double nz = std::stod(lineContents[3]);
            normals->emplace_back(nx, ny, nz);
        }
        // Check texture line (ignore / break if you prefer)
        else if (line.rfind("vt", 0) == 0) {
            // We don't need texture coords; skip
            continue;
        }
        // Otherwise, skip
    }

    file.close();

    // Optional sanity check
    if (vertices->empty() || normals->empty()) {
        std::cerr << "Warning: OBJ parsing found no vertices or normals in " << objFilePath << std::endl;
    }
}

// Given vectors of normals (parallel to the same indexing as vertices), computes the refracted
// ray directions for incidence from (0,0,1) (i.e. light traveling along +Z). 
// If total internal reflection occurs, we store a large off-axis TIR vector for that index.
void Refract(const std::vector<Eigen::Vector3d> &normals,
             std::vector<Eigen::Vector3d> *refracteds,
             double eta) {
    if (!refracteds) {
        throw std::runtime_error("Refract: `refracteds` pointer is null.");
    }

    refracteds->clear();
    refracteds->reserve(normals.size());

    // We assume an incident vector (0,0,1). The lens has index = eta, next medium is air=1.0.
    // TIR direction is an arbitrary large deflection that won't show up in the 256 x 256 image.
    const Eigen::Vector3d incident(0.0, 0.0, 1.0);
    const Eigen::Vector3d TIR(0.9999, 0.0, 0.0141418); // used for "off to the side"

    for (const auto &N : normals) {
        // cosIncidenceAngle = incident dot N = (0,0,1).dot( Nx, Ny, Nz ) = Nz
        double cosIncidenceAngle = N.z();
        double sin2RefractedAngle = eta * eta * (1.0 - (cosIncidenceAngle * cosIncidenceAngle));

        // If not TIR
        if (sin2RefractedAngle <= 1.0) {
            // formula T = (eta * incident) - [eta * cosI - sqrt(1 - sin2RefAngle)] * N
            double sqrtTerm = std::sqrt(1.0 - sin2RefractedAngle);
            Eigen::Vector3d r = eta * incident
                              - (eta * cosIncidenceAngle - sqrtTerm) * N;
            refracteds->push_back(r);
        } else {
            // TIR
            refracteds->push_back(TIR);
        }
    }
}

// Calculates intersection points for each vertex + its corresponding refracted ray on a 
// plane at z=receiver_plane, returning them in 2D (x,y). Then scales them to [0..256] range.
void CalculateIntersections(const std::vector<Eigen::Vector3d> &vertices,
                            const std::vector<Eigen::Vector3d> &refracteds,
                            std::vector<Eigen::Vector2d> *intersections,
                            double receiver_plane) {
    if (!intersections) {
        throw std::runtime_error("CalculateIntersections: `intersections` pointer is null.");
    }
    intersections->clear();
    if (vertices.size() != refracteds.size()) {
        std::cerr << "Warning: Mismatch in size of vertices (" 
                  << vertices.size() << ") and refracteds (" 
                  << refracteds.size() << ")." << std::endl;
    }

    // We'll clamp iteration to the smaller of the two arrays
    size_t nPoints = std::min(vertices.size(), refracteds.size());
    intersections->reserve(nPoints);

    for (size_t i = 0; i < nPoints; i++) {
        // Solve for t in (vertex.z + refract.z * t) = receiver_plane
        double denom = refracteds[i].z();
        if (std::abs(denom) < 1e-9) {
            // If direction is nearly parallel to plane, skip or push back fallback
            intersections->push_back(Eigen::Vector2d(-9999, -9999));
            continue;
        }
        double t = (receiver_plane - vertices[i].z()) / denom;

        // Intersection in x,y
        double ix = vertices[i].x() + refracteds[i].x() * t;
        double iy = vertices[i].y() + refracteds[i].y() * t;

        // Scale from [-1,1] to [0..256]. If actual range is unknown, you can adjust
        double scaledX = ix * 128.0 + 128.0;
        double scaledY = iy * 128.0 + 128.0;

        intersections->push_back(Eigen::Vector2d(scaledX, scaledY));
    }
}
