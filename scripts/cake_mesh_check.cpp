// Python이 계산한 각 자세의 충돌 형상을 FCL로 검사하는 오프라인 도구.
#include <assimp/Importer.hpp>
#include <assimp/config.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <fcl/fcl.h>

#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using Object = fcl::CollisionObjectd;
using Geometry = fcl::CollisionGeometryd;

// URDF 메시의 단위를 적용하고, 서브메시의 변환과 삼각형을 하나의 BVH로 합친다.
std::shared_ptr<Geometry> load_mesh(const std::string& path, const fcl::Vector3d& scale) {
    Assimp::Importer importer;
    importer.SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, true);
    const auto* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_PreTransformVertices);
    if (!scene || !scene->HasMeshes()) {
        throw std::runtime_error("메시 로드 실패: " + path + " " + importer.GetErrorString());
    }

    std::vector<fcl::Vector3d> vertices;
    std::vector<fcl::Triangle> triangles;
    for (unsigned i = 0; i < scene->mNumMeshes; ++i) {
        const auto* mesh = scene->mMeshes[i];
        const auto offset = vertices.size();
        for (unsigned j = 0; j < mesh->mNumVertices; ++j) {
            const auto& point = mesh->mVertices[j];
            vertices.emplace_back(point.x * scale.x(), point.y * scale.y(), point.z * scale.z());
        }
        for (unsigned j = 0; j < mesh->mNumFaces; ++j) {
            const auto& face = mesh->mFaces[j];
            if (face.mNumIndices == 3) {
                triangles.emplace_back(offset + face.mIndices[0],
                                       offset + face.mIndices[1],
                                       offset + face.mIndices[2]);
            }
        }
    }
    if (triangles.empty()) {
        throw std::runtime_error("삼각형이 없는 메시: " + path);
    }

    auto geometry = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
    geometry->beginModel();
    geometry->addSubModel(vertices, triangles);
    geometry->endModel();
    return geometry;
}

// 형상과 검사 쌍을 한 번 읽은 뒤 각 샘플의 변환만 갱신한다.
int main() {
    try {
        std::size_t count;
        if (!(std::cin >> count)) {
            throw std::runtime_error("충돌 형상 입력 없음");
        }

        std::vector<std::unique_ptr<Object>> objects;
        for (std::size_t i = 0; i < count; ++i) {
            std::string type;
            double x, y, z;
            std::cin >> type >> x >> y >> z;

            std::shared_ptr<Geometry> geometry;
            if (type == "mesh") {
                std::string path;
                std::cin >> std::quoted(path);
                geometry = load_mesh(path, {x, y, z});
            } else if (type == "box") {
                geometry = std::make_shared<fcl::Boxd>(x, y, z);
            } else if (type == "cylinder") {
                geometry = std::make_shared<fcl::Cylinderd>(x, y);
            } else {
                throw std::runtime_error("지원하지 않는 충돌 형상: " + type);
            }
            objects.push_back(std::make_unique<Object>(std::move(geometry)));
        }

        std::size_t pair_count;
        std::cin >> pair_count;
        std::vector<std::pair<std::size_t, std::size_t>> pairs(pair_count);
        for (auto& pair : pairs) {
            std::cin >> pair.first >> pair.second;
        }

        std::size_t samples;
        std::cin >> samples;
        for (std::size_t sample = 0; sample < samples; ++sample) {
            for (auto& object : objects) {
                auto transform = fcl::Transform3d::Identity();
                for (int row = 0; row < 3; ++row) {
                    for (int column = 0; column < 4; ++column) {
                        if (!(std::cin >> transform.matrix()(row, column))) {
                            throw std::runtime_error("자세 데이터가 잘렸습니다");
                        }
                    }
                }
                object->setTransform(transform);
                object->computeAABB();
            }
            for (const auto& [a, b] : pairs) {
                if (!objects.at(a)->getAABB().overlap(objects.at(b)->getAABB())) {
                    continue;
                }
                fcl::CollisionResultd result;
                fcl::collide(objects[a].get(), objects[b].get(), fcl::CollisionRequestd(), result);
                if (result.isCollision()) {
                    std::cout << sample << ' ' << a << ' ' << b << '\n';
                }
            }
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
