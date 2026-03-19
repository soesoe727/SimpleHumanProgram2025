#pragma once
#include <vector>
#include <Point3.h>
#include <Matrix3.h>

struct VoxelGrid {
    int resolution;
	std::vector<float> data; //data数=resolution^3
    
    // 基準姿勢情報（腰の位置・回転）
    Point3f reference_root_pos;
    Matrix3f reference_root_ori;
    bool has_reference;
    
    VoxelGrid() : resolution(0), has_reference(false) {
        reference_root_pos.set(0, 0, 0);
        reference_root_ori.setIdentity();
    }
    
    void Resize(int res);
    void Clear();
    float& At(int x, int y, int z);
    float Get(int x, int y, int z) const;
    
    // 基準姿勢の設定
    void SetReference(const Point3f& root_pos, const Matrix3f& root_ori) {
        reference_root_pos = root_pos;
        reference_root_ori = root_ori;
        has_reference = true;
    }
    
    // ファイル保存・読み込み
    bool SaveToFile(const char* filename) const;
    bool LoadFromFile(const char* filename);
};

// フレーム単位の疎ボクセル（occupancy専用）
struct SparseVoxel {
    int index;
    float value;

    SparseVoxel() : index(0), value(0.0f) {}
    SparseVoxel(int idx, float v) : index(idx), value(v) {}
};

struct SegmentFrameSparseVoxelData {
    std::vector< std::vector<SparseVoxel> > segment_sparse_voxels;
    int num_segments;

    // このフレームの基準姿勢（ルート）
    Point3f reference_root_pos;
    Matrix3f reference_root_ori;
    bool has_reference;

    SegmentFrameSparseVoxelData() : num_segments(0), has_reference(false) {
        reference_root_pos.set(0, 0, 0);
        reference_root_ori.setIdentity();
    }

    void Resize(int num_seg) {
        num_segments = num_seg;
        segment_sparse_voxels.clear();
        segment_sparse_voxels.resize(num_seg);
    }

    void Clear() {
        for (size_t i = 0; i < segment_sparse_voxels.size(); ++i)
            segment_sparse_voxels[i].clear();
    }

    void SetReference(const Point3f& root_pos, const Matrix3f& root_ori) {
        reference_root_pos = root_pos;
        reference_root_ori = root_ori;
        has_reference = true;
    }
};

struct MotionFrameSparseVoxelCache {
    int resolution;
    int num_segments;
    std::vector<SegmentFrameSparseVoxelData> frames;

    MotionFrameSparseVoxelCache() : resolution(0), num_segments(0) {}

    void Resize(int num_frames, int num_seg, int res) {
        resolution = res;
        num_segments = num_seg;
        frames.clear();
        frames.resize(num_frames);
        for (int i = 0; i < num_frames; ++i)
            frames[i].Resize(num_seg);
    }

    void Clear() {
        frames.clear();
        resolution = 0;
        num_segments = 0;
    }
};

// 部位ごとのボクセルグリッドコレクション
struct SegmentVoxelData {
	std::vector<VoxelGrid> segment_grids; // 各部位のボクセルグリッド
	int num_segments; // 部位数
    
    SegmentVoxelData() : num_segments(0) {}
    
	// 部位数と各グリッドの解像度を設定
    void Resize(int num_seg, int resolution) {
        num_segments = num_seg;
        segment_grids.resize(num_seg);
        for (int i = 0; i < num_seg; ++i) {
            segment_grids[i].Resize(resolution);
        }
    }
    
    void Clear() {
        for (auto& grid : segment_grids) {
            grid.Clear();
        }
    }
    
	// 指定部位のボクセルグリッドを取得
    VoxelGrid& GetSegmentGrid(int segment_index) {
        if (segment_index >= 0 && segment_index < num_segments) {
            return segment_grids[segment_index];
        }
        static VoxelGrid dummy;
        return dummy;
    }
    
    // ファイル保存・読み込み
    bool SaveToFile(const char* filename) const;
    bool LoadFromFile(const char* filename);
};
