#include "VoxelData.h"
#include <algorithm>
#include <fstream>

using namespace std;

// --- VoxelGrid Implementation ---

// ボクセルグリッドを指定解像度でリサイズし、データを初期化
void VoxelGrid::Resize(int res) {
    resolution = res;
    data.assign(res * res * res, 0.0f);
}

// グリッドデータを全てゼロでクリア
void VoxelGrid::Clear() {
    std::fill(data.begin(), data.end(), 0.0f);
}

// 指定座標のボクセル値への参照を取得（範囲外の場合はダミー値を返す）
float& VoxelGrid::At(int x, int y, int z) {
    if (x < 0 || x >= resolution || y < 0 || y >= resolution || z < 0 || z >= resolution) {
        static float dummy = 0.0f; 
        return dummy;
    }
    return data[z * resolution * resolution + y * resolution + x];
}

// 指定座標のボクセル値を取得（範囲外の場合は0を返す）
float VoxelGrid::Get(int x, int y, int z) const {
    if (x < 0 || x >= resolution || y < 0 || y >= resolution || z < 0 || z >= resolution)
        return 0.0f;
    return data[static_cast<size_t>(z) * resolution * resolution + y * resolution + x];
}

// ボクセルグリッドと基準姿勢情報をバイナリファイルへ保存
bool VoxelGrid::SaveToFile(const char* filename) const {
    ofstream ofs(filename, ios::binary);
    if (!ofs) 
        return false;
    
    int version = 2;
    ofs.write(reinterpret_cast<const char*>(&version), sizeof(int));
    ofs.write(reinterpret_cast<const char*>(&resolution), sizeof(int));
    
    int data_size = data.size();
    ofs.write(reinterpret_cast<const char*>(&data_size), sizeof(int));
    ofs.write(reinterpret_cast<const char*>(data.data()), data_size * sizeof(float));
    
    // 基準姿勢情報を書き込み
    ofs.write(reinterpret_cast<const char*>(&has_reference), sizeof(bool));
    if (has_reference) {
        ofs.write(reinterpret_cast<const char*>(&reference_root_pos.x), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&reference_root_pos.y), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&reference_root_pos.z), sizeof(float));
        
        float m[9] = {
            reference_root_ori.m00, reference_root_ori.m01, reference_root_ori.m02,
            reference_root_ori.m10, reference_root_ori.m11, reference_root_ori.m12,
            reference_root_ori.m20, reference_root_ori.m21, reference_root_ori.m22
        };
        ofs.write(reinterpret_cast<const char*>(m), sizeof(m));
    }
    
    return ofs.good();
}

// バイナリファイルからボクセルグリッドと基準姿勢情報を読み込み（バージョン対応）
bool VoxelGrid::LoadFromFile(const char* filename) {
    ifstream ifs(filename, ios::binary);
    if (!ifs) 
        return false;
    
    int version;
    ifs.read(reinterpret_cast<char*>(&version), sizeof(int));
    
    int res;
    ifs.read(reinterpret_cast<char*>(&res), sizeof(int));
    
    int data_size;
    ifs.read(reinterpret_cast<char*>(&data_size), sizeof(int));
    
    Resize(res);
    ifs.read(reinterpret_cast<char*>(data.data()), data_size * sizeof(float));
    
    // バージョン2以降は基準姿勢情報を読み込み
    if (version >= 2) {
        ifs.read(reinterpret_cast<char*>(&has_reference), sizeof(bool));
        if (has_reference) {
            ifs.read(reinterpret_cast<char*>(&reference_root_pos.x), sizeof(float));
            ifs.read(reinterpret_cast<char*>(&reference_root_pos.y), sizeof(float));
            ifs.read(reinterpret_cast<char*>(&reference_root_pos.z), sizeof(float));
            
            float m[9];
            ifs.read(reinterpret_cast<char*>(m), sizeof(m));
            reference_root_ori.set(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
        }
    } else {
        has_reference = false;
    }
    
    return ifs.good();
}

// 部位ごとのボクセルデータをバイナリファイルへ保存
bool SegmentVoxelData::SaveToFile(const char* filename) const {
    ofstream ofs(filename, ios::binary);
    if (!ofs) 
        return false;
    
    ofs.write(reinterpret_cast<const char*>(&num_segments), sizeof(int));
    
    for (int i = 0; i < num_segments; ++i) {
        const VoxelGrid& grid = segment_grids[i];
        ofs.write(reinterpret_cast<const char*>(&grid.resolution), sizeof(int));
        
        int data_size = grid.data.size();
        ofs.write(reinterpret_cast<const char*>(&data_size), sizeof(int));
        ofs.write(reinterpret_cast<const char*>(grid.data.data()), data_size * sizeof(float));
    }
    
    return ofs.good();
}

// バイナリファイルから部位ごとのボクセルデータを読み込み
bool SegmentVoxelData::LoadFromFile(const char* filename) {
    ifstream ifs(filename, ios::binary);
    if (!ifs) 
        return false;
    
    int num_seg;
    ifs.read(reinterpret_cast<char*>(&num_seg), sizeof(int));
    
    num_segments = num_seg;
    segment_grids.resize(num_seg);
    
    for (int i = 0; i < num_segments; ++i) {
        VoxelGrid& grid = segment_grids[i];
        
        int res;
        ifs.read(reinterpret_cast<char*>(&res), sizeof(int));
        
        int data_size;
        ifs.read(reinterpret_cast<char*>(&data_size), sizeof(int));
        
        grid.Resize(res);
        ifs.read(reinterpret_cast<char*>(grid.data.data()), data_size * sizeof(float));
    }
    
    return ifs.good();
}
