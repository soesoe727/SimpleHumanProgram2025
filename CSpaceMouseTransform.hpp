#ifndef CSPACEMOUSETRANSFORM_HPP_INCLUDED
#define CSPACEMOUSETRANSFORM_HPP_INCLUDED

// vecmath for matrix operations
#include <Matrix4.h>
#include <Vector3.h>

// standard library
#include <mutex>

namespace TDx {
namespace TraceNL {

/// <summary>
/// Manages a transformation matrix (Matrix4f) that can be controlled by SpaceMouse input.
/// This class provides thread-safe access to a transformation matrix and methods to update it.
/// </summary>
class CSpaceMouseTransform {
public:
  /// <summary>
  /// Initializes a new instance of the CSpaceMouseTransform class.
  /// </summary>
  CSpaceMouseTransform();

  /// <summary>
  /// Gets the current transformation matrix.
  /// </summary>
  /// <param name="matrix">Output matrix to store the transformation.</param>
  void GetTransformMatrix(Matrix4f &matrix) const;

  /// <summary>
  /// Applies a translation to the current transformation.
  /// </summary>
  /// <param name="dx">Translation along X axis.</param>
  /// <param name="dy">Translation along Y axis.</param>
  /// <param name="dz">Translation along Z axis.</param>
  void ApplyTranslation(float dx, float dy, float dz);

  /// <summary>
  /// Applies a rotation to the current transformation (world coordinate system).
  /// </summary>
  /// <param name="angle_rad">Rotation angle in radians.</param>
  /// <param name="axis_x">X component of rotation axis.</param>
  /// <param name="axis_y">Y component of rotation axis.</param>
  /// <param name="axis_z">Z component of rotation axis.</param>
  void ApplyRotation(float angle_rad, float axis_x, float axis_y, float axis_z);

  /// <summary>
  /// Applies a rotation to the current transformation (local coordinate system).
  /// </summary>
  /// <param name="angle_rad">Rotation angle in radians.</param>
  /// <param name="axis_x">X component of rotation axis in local space.</param>
  /// <param name="axis_y">Y component of rotation axis in local space.</param>
  /// <param name="axis_z">Z component of rotation axis in local space.</param>
  void ApplyLocalRotation(float angle_rad, float axis_x, float axis_y, float axis_z);

  /// <summary>
  /// Resets the transformation matrix to identity.
  /// </summary>
  void Reset();

  /// <summary>
  /// Sets the transformation matrix directly.
  /// </summary>
  /// <param name="matrix">The new transformation matrix.</param>
  void SetTransformMatrix(const Matrix4f &matrix);

  /// <summary>
  /// Applies incremental transformation from SpaceMouse motion data (world coordinate system).
  /// </summary>
  /// <param name="translation">Translation vector (x, y, z).</param>
  /// <param name="rotation">Rotation vector (rx, ry, rz) - angle is magnitude, axis is direction.</param>
  /// <param name="scale">Scale factor for the motion (default 1.0).</param>
  void ApplyMotion(const Vector3f &translation, const Vector3f &rotation, float scale = 1.0f);

  /// <summary>
  /// Applies incremental transformation from SpaceMouse motion data (local coordinate system).
  /// </summary>
  /// <param name="translation">Translation vector (x, y, z).</param>
  /// <param name="rotation">Rotation vector (rx, ry, rz) - angle is magnitude, axis is direction.</param>
  /// <param name="scale">Scale factor for the motion (default 1.0).</param>
  void ApplyLocalMotion(const Vector3f &translation, const Vector3f &rotation, float scale = 1.0f);

private:
  mutable std::mutex m_mutex;
  Matrix4f m_transform;

  /// <summary>
  /// Helper function to create a rotation matrix from axis-angle representation.
  /// </summary>
  static Matrix4f CreateRotationMatrix(float angle_rad, float axis_x, float axis_y, float axis_z);
  
  /// <summary>
  /// Helper function to create a translation matrix.
  /// </summary>
  static Matrix4f CreateTranslationMatrix(float dx, float dy, float dz);
  
  /// <summary>
  /// Orthonormalizes the rotation part of a transformation matrix.
  /// Prevents accumulation of numerical errors.
  /// </summary>
  static void OrthoNormalizeRotation(Matrix4f &matrix);
};

} // namespace TraceNL
} // namespace TDx

#endif // CSPACEMOUSETRANSFORM_HPP_INCLUDED
