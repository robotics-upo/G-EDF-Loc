#ifndef GAUSSIAN_MAP_HPP
#define GAUSSIAN_MAP_HPP

#include <vector>
#include <string>
#include <tuple>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <limits>
#include <algorithm> 
#include <cstring> 
#include <unordered_map> // [OPTIMIZACIÓN 1] Hash Map

#include <Eigen/Core>
#include <Eigen/Dense>

// [IMPORTANTE] Librería VDT requerida. 
#include "vdt/exp.h"

namespace gaussian_map {

// ---------------------------------------------------------
// OPTIMIZACIÓN MATEMÁTICA (Usando VDT)
// ---------------------------------------------------------
inline double fast_exp(double x) {
    // Corte para valores despreciables (optimización de cola)
    if (x < -20.0) return 0.0;
    return vdt::fast_exp(x);
}

// =========================================================
// ESTRUCTURAS DE DATOS
// =========================================================

struct GaussianStored {
    double mean_x, mean_y, mean_z;
    // Escala inversa (1/sigma^4). Diagonal.
    double inv_s_xx, inv_s_yy, inv_s_zz; 
    double weight;
};

struct MapResult {
    double value;             // Valor suavizado
    Eigen::Vector3d gradient; // Gradiente local
};

// Definición de la clave y su Hash
using CubeKey = std::tuple<int, int, int>;

struct CubeKeyHash {
    std::size_t operator()(const CubeKey& k) const {
        std::size_t seed = 0;
        auto hash_combine = [&seed](int v) {
            seed ^= std::hash<int>{}(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        };
        hash_combine(std::get<0>(k));
        hash_combine(std::get<1>(k));
        hash_combine(std::get<2>(k));
        return seed;
    }
};

// =========================================================
// CLASE PRINCIPAL
// =========================================================
class GaussianMap {
public:
    GaussianMap() : cube_size_(1.0), blending_margin_(0.0) {
        resetBounds();
    }
    ~GaussianMap() {}

    // ---------------------------------------------------------
    // 1. CARGA GDF1 BINARIO
    // ---------------------------------------------------------
    bool loadMapFromBin(const std::string& filename) {
        FILE* fp = fopen(filename.c_str(), "rb");
        if (!fp) {
            std::cerr << "[GaussianMap] Error al abrir " << filename << std::endl;
            return false;
        }

        MapHeader header;
        if (fread(&header, sizeof(MapHeader), 1, fp) != 1) {
            fclose(fp); return false;
        }
        if (std::strncmp(header.magic, "GDF1", 4) != 0) {
            std::cerr << "[GaussianMap] Error: Formato incorrecto (Magic != GDF1)" << std::endl;
            fclose(fp); return false;
        }

        cube_size_ = static_cast<double>(header.cube_size);
        blending_margin_ = static_cast<double>(header.cube_margin);

        std::cout << "[GDF1] Cargado. Cubos: " << header.num_cubes 
                  << " | Size: " << cube_size_ 
                  << "m | Margin: " << blending_margin_ << "m" << std::endl;

        // Optimización: Reservar buckets para evitar rehashing
        grid_map_.reserve(header.num_cubes); 

        for (uint32_t i = 0; i < header.num_cubes; ++i) {
            CubeHeader ch;
            if (fread(&ch, sizeof(CubeHeader), 1, fp) != 1) break;

            // Cálculo de índice con offset de seguridad
            int idx_x = static_cast<int>(std::floor((ch.origin[0] + 0.01 * cube_size_) / cube_size_));
            int idx_y = static_cast<int>(std::floor((ch.origin[1] + 0.01 * cube_size_) / cube_size_));
            int idx_z = static_cast<int>(std::floor((ch.origin[2] + 0.01 * cube_size_) / cube_size_));
            CubeKey key = std::make_tuple(idx_x, idx_y, idx_z);

            auto& voxel_gaussians = grid_map_[key];
            voxel_gaussians.reserve(ch.num_gaussians);

            std::vector<GaussianData> buffer(ch.num_gaussians);
            size_t read_count = fread(buffer.data(), sizeof(GaussianData), ch.num_gaussians, fp);
            
            if (read_count != ch.num_gaussians) {
                std::cerr << "[GaussianMap] Error leyendo gaussianas en cubo " << i << std::endl;
                break;
            }

            for (const auto& gd : buffer) {
                float w = gd.weight;
                if (std::isnan(w) || std::isinf(w)) w = 0.0f;
                
                GaussianStored g;
                g.mean_x = gd.mean[0]; 
                g.mean_y = gd.mean[1]; 
                g.mean_z = gd.mean[2];
                g.weight = w;
                
                // NOTA: Usamos 1/sigma^4 según implementación binaria (aunque spec diga cuadrado)
                float s0 = std::abs(gd.sigma[0]) < 1e-4f ? 1e-4f : gd.sigma[0];
                float s1 = std::abs(gd.sigma[1]) < 1e-4f ? 1e-4f : gd.sigma[1];
                float s2 = std::abs(gd.sigma[2]) < 1e-4f ? 1e-4f : gd.sigma[2];
                
                g.inv_s_xx = 1.0 / (double(s0) * s0 * s0 * s0);
                g.inv_s_yy = 1.0 / (double(s1) * s1 * s1 * s1);
                g.inv_s_zz = 1.0 / (double(s2) * s2 * s2 * s2);

                voxel_gaussians.push_back(g);
                updateBounds(g.mean_x, g.mean_y, g.mean_z);
            }
        }
        fclose(fp);
        return true;
    }

    // ---------------------------------------------------------
    // 2. EVALUACIÓN CON GRADIENTE (OPTIMIZADA V3 - Unificada)
    // ---------------------------------------------------------
    // ---------------------------------------------------------
    // 2. EVALUACIÓN CON GRADIENTE (OPTIMIZADA Y SUAVE)
    // ---------------------------------------------------------
    MapResult evaluateWithGradient(double x, double y, double z) const {
        
        int cx = static_cast<int>(std::floor(x / cube_size_));
        int cy = static_cast<int>(std::floor(y / cube_size_));
        int cz = static_cast<int>(std::floor(z / cube_size_));

        // --- 1. LÓGICA DE EFICIENCIA (FAST PATH) ---
        // Calculamos coordenadas locales para ver si estamos seguros en el centro
        double cube_x0 = cx * cube_size_;
        double cube_y0 = cy * cube_size_;
        double cube_z0 = cz * cube_size_;
        
        double lx = x - cube_x0;
        double ly = y - cube_y0;
        double lz = z - cube_z0;

        // Si estamos lejos de TODOS los bordes, no necesitamos vecinos.
        bool fast_path = (lx >= blending_margin_ && lx <= cube_size_ - blending_margin_ &&
                          ly >= blending_margin_ && ly <= cube_size_ - blending_margin_ &&
                          lz >= blending_margin_ && lz <= cube_size_ - blending_margin_);

        // CASO A: CENTRO DEL CUBO (RÁPIDO)
        if (fast_path) {
            auto it = grid_map_.find(std::make_tuple(cx, cy, cz));
            
            // Si el cubo central está vacío, devolvemos distancia máxima y gradiente cero
            if (it == grid_map_.end()) {
                return {20.0, Eigen::Vector3d::Zero()}; 
            }

            double val_acc = 0.0;
            Eigen::Vector3d grad_acc = Eigen::Vector3d::Zero();

            // Solo iteramos las gaussianas de ESTE cubo
            for (const auto& g : it->second) {
                double dx = x - g.mean_x;
                double dy = y - g.mean_y;
                double dz = z - g.mean_z;
                
                double dsq = (dx*dx * g.inv_s_xx) + (dy*dy * g.inv_s_yy) + (dz*dz * g.inv_s_zz);
                // Optimización: Si está muy lejos de la gaussiana, exp() es 0.
                if (dsq > 20.0) continue; 

                double val = g.weight * fast_exp(-0.5 * dsq);
                
                val_acc += val;
                // El gradiente se acumula aquí mismo (reducido)
                grad_acc.x() += -val * (g.inv_s_xx * dx);
                grad_acc.y() += -val * (g.inv_s_yy * dy);
                grad_acc.z() += -val * (g.inv_s_zz * dz);
            }
            return {val_acc, grad_acc};
        }

        // --- 2. LÓGICA DE SUAVIDAD (SLOW PATH / BLENDING) ---
        // Estamos en un borde. Tenemos que preguntar a los vecinos.
        
        // Optimizamos rangos: solo miramos a la izquierda (-1) si estamos en borde izquierdo, etc.
        int sx = (lx < blending_margin_) ? -1 : 0;       
        int ex = (lx > cube_size_ - blending_margin_) ? 1 : 0; 
        int sy = (ly < blending_margin_) ? -1 : 0;
        int ey = (ly > cube_size_ - blending_margin_) ? 1 : 0;
        int sz = (lz < blending_margin_) ? -1 : 0;
        int ez = (lz > cube_size_ - blending_margin_) ? 1 : 0;

        double total_val = 0.0;
        Eigen::Vector3d total_grad = Eigen::Vector3d::Zero();
        double sum_w = 0.0;

        // Bucles pequeños (máximo 2x2x2 = 8 iteraciones)
        for (int i = sx; i <= ex; ++i) {
            for (int j = sy; j <= ey; ++j) {
                for (int k = sz; k <= ez; ++k) {
                    
                    int nx = cx + i;
                    int ny = cy + j;
                    int nz = cz + k;
                    
                    // Calculamos peso antes de buscar (si peso es 0, nos ahorramos la búsqueda)
                    double w_blend = calculateBlendWeight(x, y, z, nx, ny, nz);
                    if (w_blend < 1e-6) continue;

                    auto it = grid_map_.find(std::make_tuple(nx, ny, nz));
                    if (it == grid_map_.end()) continue;

                    // Acumulamos contribución de este vecino
                    double val_local = 0.0;
                    Eigen::Vector3d grad_local = Eigen::Vector3d::Zero();

                    for (const auto& g : it->second) {
                        double dx = x - g.mean_x;
                        double dy = y - g.mean_y;
                        double dz = z - g.mean_z;
                        double dsq = (dx*dx * g.inv_s_xx) + (dy*dy * g.inv_s_yy) + (dz*dz * g.inv_s_zz);
                        if (dsq > 20.0) continue;

                        double val = g.weight * fast_exp(-0.5 * dsq);
                        
                        val_local += val;
                        grad_local.x() += -val * (g.inv_s_xx * dx);
                        grad_local.y() += -val * (g.inv_s_yy * dy);
                        grad_local.z() += -val * (g.inv_s_zz * dz);
                    }

                    // Suma ponderada (Blending)
                    total_val += w_blend * val_local;
                    total_grad += w_blend * grad_local;
                    sum_w += w_blend;
                }
            }
        }

        if (sum_w > 1e-6) {
            // Normalizamos por la suma de pesos para mantener la escala correcta
            return {total_val / sum_w, total_grad / sum_w};
        } else {
            return {20.0, Eigen::Vector3d::Zero()}; // Zona desconocida
        }
    }

    // ---------------------------------------------------------
    // 3. CONSULTA DE DISTANCIA (OPTIMIZADA V2)
    // ---------------------------------------------------------
    double getDistanceAt(double x, double y, double z) const {
        int cx = static_cast<int>(std::floor(x / cube_size_));
        int cy = static_cast<int>(std::floor(y / cube_size_));
        int cz = static_cast<int>(std::floor(z / cube_size_));
        
        double cube_x0 = cx * cube_size_;
        double cube_y0 = cy * cube_size_;
        double cube_z0 = cz * cube_size_;

        double lx = x - cube_x0;
        double ly = y - cube_y0;
        double lz = z - cube_z0;

        // Smart Neighbor Search: Solo activar cubos necesarios
        int sx = 0, ex = 0;
        if (lx < blending_margin_) sx = -1;       
        else if (lx > cube_size_ - blending_margin_) ex = 1; 

        int sy = 0, ey = 0;
        if (ly < blending_margin_) sy = -1;
        else if (ly > cube_size_ - blending_margin_) ey = 1;

        int sz = 0, ez = 0;
        if (lz < blending_margin_) sz = -1;
        else if (lz > cube_size_ - blending_margin_) ez = 1;

        // Caso Rápido: Sin vecinos
        if (sx == 0 && ex == 0 && sy == 0 && ey == 0 && sz == 0 && ez == 0) {
             CubeKey key = std::make_tuple(cx, cy, cz);
             auto it = grid_map_.find(key);
             if (it == grid_map_.end()) return 0.0;
             
             double val_acc = 0.0;
             for (const auto& g : it->second) {
                double dx = x - g.mean_x;
                double dy = y - g.mean_y;
                double dz = z - g.mean_z;
                double dsq = (dx*dx * g.inv_s_xx) + (dy*dy * g.inv_s_yy) + (dz*dz * g.inv_s_zz);
                val_acc += g.weight * fast_exp(-0.5 * dsq);
             }
             return val_acc;
        }

        // Caso Lento: Blending con vecinos
        double total_val = 0.0;
        double sum_w = 0.0;

        for (int i = sx; i <= ex; ++i) {
            for (int j = sy; j <= ey; ++j) {
                for (int k = sz; k <= ez; ++k) {
                    
                    int nx = cx + i;
                    int ny = cy + j;
                    int nz = cz + k;
                    
                    double w_blend = calculateBlendWeight(x, y, z, nx, ny, nz);
                    if (w_blend < 1e-6) continue;

                    CubeKey key = std::make_tuple(nx, ny, nz);
                    auto it = grid_map_.find(key);
                    if (it == grid_map_.end()) continue;

                    // Cálculo IN-LINE para evitar rellamadas a find()
                    double val_local = 0.0;
                    for (const auto& g : it->second) {
                        double dx = x - g.mean_x;
                        double dy = y - g.mean_y;
                        double dz = z - g.mean_z;
                        double dsq = (dx*dx * g.inv_s_xx) + (dy*dy * g.inv_s_yy) + (dz*dz * g.inv_s_zz);
                        val_local += g.weight * fast_exp(-0.5 * dsq);
                    }

                    total_val += w_blend * val_local;
                    sum_w += w_blend;
                }
            }
        }
        return (sum_w > 0.0) ? (total_val / sum_w) : 0.0;
    }

    // --- UTILS ---
    bool isIntoGrid(double x, double y, double z) const {
        int cx = static_cast<int>(std::floor(x / cube_size_));
        int cy = static_cast<int>(std::floor(y / cube_size_));
        int cz = static_cast<int>(std::floor(z / cube_size_));
        return grid_map_.find(std::make_tuple(cx, cy, cz)) != grid_map_.end();
    }
    double getMinX() const { return min_x_; }
    double getMaxX() const { return max_x_; }
    double getCubeMargin() const { return blending_margin_; }

private:
    std::unordered_map<CubeKey, std::vector<GaussianStored>, CubeKeyHash> grid_map_;    
    
    double cube_size_;
    double blending_margin_;
    double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;

    #pragma pack(push, 1)
    struct MapHeader {
        char magic[4]; 
        uint32_t version;
        uint32_t num_cubes;
        float avg_mae;
        float std_dev;
        float bounds_min[3];
        float bounds_max[3];
        float cube_size;
        float empty_search_margin;
        float cube_margin;
        uint8_t padding[64];
    };

    struct CubeHeader {
        float origin[3];
        float mae;
        float std_dev;
        uint32_t num_gaussians;
    };

    struct GaussianData {
        uint32_t id;
        float mean[3];
        float sigma[3];
        float weight;
    };
    #pragma pack(pop)

    inline double calculateBlendWeight(double px, double py, double pz, int cx, int cy, int cz) const {
        double cube_x0 = cx * cube_size_;
        double cube_y0 = cy * cube_size_;
        double cube_z0 = cz * cube_size_;

        double dx = std::min(px - cube_x0, (cube_x0 + cube_size_) - px);
        double dy = std::min(py - cube_y0, (cube_y0 + cube_size_) - py);
        double dz = std::min(pz - cube_z0, (cube_z0 + cube_size_) - pz);

        double min_dist = std::min({dx, dy, dz});

        if (min_dist >= 0.0) return 1.0; 
        if (min_dist <= -blending_margin_) return 0.0;

        double t = 1.0 + (min_dist / blending_margin_);
        return t * t * (3.0 - 2.0 * t);
    }
    
    void resetBounds() {
        min_x_ = std::numeric_limits<double>::max();
        max_x_ = std::numeric_limits<double>::lowest();
        min_y_ = std::numeric_limits<double>::max();
        max_y_ = std::numeric_limits<double>::lowest();
        min_z_ = std::numeric_limits<double>::max();
        max_z_ = std::numeric_limits<double>::lowest();
    }

    void updateBounds(double x, double y, double z) {
        if (x < min_x_) min_x_ = x;
        if (x > max_x_) max_x_ = x;
        if (y < min_y_) min_y_ = y;
        if (y > max_y_) max_y_ = y;
        if (z < min_z_) min_z_ = z;
        if (z > max_z_) max_z_ = z;
    }
};

} // namespace gaussian_map

#endif