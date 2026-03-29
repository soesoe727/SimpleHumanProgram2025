# Copilot Instructions

## プロジェクト ガイドライン
- User wants model gizmo to show only required handles simultaneously (red X arrow + blue Z arrow + green Y rotation circle), and Y rotation should be in-place rather than world-origin dependent.
- User ultimately wants to remove `VoxelGrid` and `SegmentVoxelData`, and migrate fully to per-frame × per-segment sparse voxel cache structures with on-demand accumulation.