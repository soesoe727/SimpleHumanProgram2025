#include "CSpaceMouseTransform.hpp"
#include <cmath>

namespace TDx {
namespace TraceNL {

/// <summary>
/// Initializes a new instance of the CSpaceMouseTransform class.
/// </summary>
CSpaceMouseTransform::CSpaceMouseTransform() {
  m_transform.setIdentity();
}

/// <summary>
/// Gets the current transformation matrix.
/// </summary>
void CSpaceMouseTransform::GetTransformMatrix(Matrix4f &matrix) const {
  std::lock_guard<std::mutex> lock(m_mutex);
  matrix = m_transform;
}

/// <summary>
/// Applies a translation to the current transformation.
/// </summary>
void CSpaceMouseTransform::ApplyTranslation(float dx, float dy, float dz) {
  std::lock_guard<std::mutex> lock(m_mutex);
  Matrix4f translation = CreateTranslationMatrix(dx, dy, dz);
  m_transform = translation * m_transform;
}

/// <summary>
/// Applies a rotation to the current transformation (world coordinate system).
/// </summary>
void CSpaceMouseTransform::ApplyRotation(float angle_rad, float axis_x, float axis_y, float axis_z) {
  std::lock_guard<std::mutex> lock(m_mutex);
  Matrix4f rotation = CreateRotationMatrix(angle_rad, axis_x, axis_y, axis_z);
  m_transform = rotation * m_transform;
}

/// <summary>
/// Applies a rotation to the current transformation (local coordinate system).
/// </summary>
void CSpaceMouseTransform::ApplyLocalRotation(float angle_rad, float axis_x, float axis_y, float axis_z) {
  std::lock_guard<std::mutex> lock(m_mutex);
  Matrix4f rotation = CreateRotationMatrix(angle_rad, axis_x, axis_y, axis_z);
  m_transform = m_transform * rotation;
}

/// <summary>
/// Resets the transformation matrix to identity.
/// </summary>
void CSpaceMouseTransform::Reset() {
  std::lock_guard<std::mutex> lock(m_mutex);
  m_transform.setIdentity();
}

/// <summary>
/// Sets the transformation matrix directly.
/// </summary>
void CSpaceMouseTransform::SetTransformMatrix(const Matrix4f &matrix) {
  std::lock_guard<std::mutex> lock(m_mutex);
  m_transform = matrix;
}

/// <summary>
/// Applies incremental transformation from SpaceMouse motion data (world coordinate system).
/// </summary>
void CSpaceMouseTransform::ApplyMotion(const Vector3f &translation, const Vector3f &rotation, float scale) {
  std::lock_guard<std::mutex> lock(m_mutex);
  
  // Apply translation
  if (translation.lengthSquared() > 1e-6f) {
    Matrix4f trans = CreateTranslationMatrix(
      translation.x * scale,
      translation.y * scale,
      translation.z * scale
    );
    m_transform = trans * m_transform;
  }
  
  // Apply rotation (world coordinate system)
  float rotation_magnitude = rotation.length();
  if (rotation_magnitude > 1e-6f) {
    // Rotation vector represents axis-angle: direction is axis, magnitude is angle
    float angle = rotation_magnitude * scale;
    Vector3f axis = rotation;
    axis.normalize();
    
    Matrix4f rot = CreateRotationMatrix(angle, axis.x, axis.y, axis.z);
    m_transform = rot * m_transform;
  }
}

/// <summary>
/// Applies incremental transformation from SpaceMouse motion data.
/// Rotation: World coordinate system with pivot at object center
/// Translation: World coordinate system (along world X/Y/Z axes)
/// This keeps the rotation axes world-aligned while rotating around the object center.
/// </summary>
void CSpaceMouseTransform::ApplyLocalMotion(const Vector3f &translation, const Vector3f &rotation, float scale) {
std::lock_guard<std::mutex> lock(m_mutex);
  
  // Build rotation transformation (world axes, pivot at object center)
  Matrix4f rotationTransform;
  rotationTransform.setIdentity();
  
  // Apply rotations around each axis separately (world space)
  // The order matters: we apply in Z-Y-X order (Yaw-Pitch-Roll convention)
  
  float rx = rotation.x * scale;  // Pitch (around local X axis)
  float ry = rotation.y * scale;  // Yaw   (around local Y axis)
  float rz = -rotation.z * scale;  // Roll  (around local Z axis)
  
  // Only build rotation matrix if there's significant rotation
  if (std::abs(rx) > 1e-6f || std::abs(ry) > 1e-6f || std::abs(rz) > 1e-6f) {
    // Apply rotations in order: Z (Roll), Y (Yaw), X (Pitch)
    // This is the standard Euler angle order for local rotations
    
    if (std::abs(rz) > 1e-6f) {
      Matrix4f rollMat = CreateRotationMatrix(rz, 0.0f, 0.0f, 1.0f);
      rotationTransform = rotationTransform * rollMat;
    }
    
    if (std::abs(ry) > 1e-6f) {
      Matrix4f yawMat = CreateRotationMatrix(ry, 0.0f, 1.0f, 0.0f);
      rotationTransform = rotationTransform * yawMat;
    }
    
    if (std::abs(rx) > 1e-6f) {
      Matrix4f pitchMat = CreateRotationMatrix(rx, 1.0f, 0.0f, 0.0f);
      rotationTransform = rotationTransform * pitchMat;
    }
  }
  
  // Build translation transformation (world space)
  Matrix4f translationTransform;
  translationTransform.setIdentity();
  
  if (translation.lengthSquared() > 1e-6f) {
    translationTransform = CreateTranslationMatrix(
      translation.x * scale,
      translation.y * scale,
      translation.z * scale
    );
  }
  
  // Apply transformations:
  // 1. Apply WORLD rotation around object center (left multiply with pivot)
  // 2. Apply WORLD translation (left multiply)
  //
  // Formula: m_transform = T_world * (T_center * R_world * T_center^-1) * m_transform

  if (std::abs(rx) > 1e-6f || std::abs(ry) > 1e-6f || std::abs(rz) > 1e-6f) {
    Matrix4f centerTransform;
    centerTransform.setIdentity();
    centerTransform.m03 = m_transform.m03;
    centerTransform.m13 = m_transform.m13;
    centerTransform.m23 = m_transform.m23;

    Matrix4f inverseCenterTransform;
    inverseCenterTransform.setIdentity();
    inverseCenterTransform.m03 = -m_transform.m03;
    inverseCenterTransform.m13 = -m_transform.m13;
    inverseCenterTransform.m23 = -m_transform.m23;

    rotationTransform = centerTransform * (rotationTransform * inverseCenterTransform);
  }

  m_transform = translationTransform * (rotationTransform * m_transform);
  
  // Orthonormalize the rotation part to prevent drift
  if (std::abs(rx) > 1e-6f || std::abs(ry) > 1e-6f || std::abs(rz) > 1e-6f) {
    OrthoNormalizeRotation(m_transform);
  }
}

/// <summary>
/// Orthonormalizes the rotation part of a transformation matrix.
/// This prevents accumulation of numerical errors that can cause drift.
/// </summary>
void CSpaceMouseTransform::OrthoNormalizeRotation(Matrix4f &matrix) {
  // Extract the 3x3 rotation part
  Vector3f x(matrix.m00, matrix.m10, matrix.m20);
  Vector3f y(matrix.m01, matrix.m11, matrix.m21);
  Vector3f z(matrix.m02, matrix.m12, matrix.m22);
  
  // Gram-Schmidt orthonormalization
  // Normalize X axis
  x.normalize();
  
  // Make Y perpendicular to X
  float dot_xy = x.x * y.x + x.y * y.y + x.z * y.z;
  y.x -= dot_xy * x.x;
  y.y -= dot_xy * x.y;
  y.z -= dot_xy * x.z;
  y.normalize();
  
  // Make Z perpendicular to both X and Y using cross product
  z.x = x.y * y.z - x.z * y.y;
  z.y = x.z * y.x - x.x * y.z;
  z.z = x.x * y.y - x.y * y.x;
  z.normalize();
  
  // Write back the orthonormalized axes
  matrix.m00 = x.x; matrix.m10 = x.y; matrix.m20 = x.z;
  matrix.m01 = y.x; matrix.m11 = y.y; matrix.m21 = y.z;
  matrix.m02 = z.x; matrix.m12 = z.y; matrix.m22 = z.z;
  
  // Translation part (m03, m13, m23) and homogeneous coordinate (m30-m33) remain unchanged
}

/// <summary>
/// Helper function to create a rotation matrix from axis-angle representation.
/// </summary>
Matrix4f CSpaceMouseTransform::CreateRotationMatrix(float angle_rad, float axis_x, float axis_y, float axis_z) {
  // Normalize axis
  float length = std::sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z);
  if (length < 1e-6f) {
    Matrix4f identity;
    identity.setIdentity();
    return identity;
  }
  
  float x = axis_x / length;
  float y = axis_y / length;
  float z = axis_z / length;
  
  float c = std::cos(angle_rad);
  float s = std::sin(angle_rad);
  float t = 1.0f - c;
  
  // Rodrigues' rotation formula
  Matrix4f result;
  result.m00 = t * x * x + c;
  result.m01 = t * x * y - s * z;
  result.m02 = t * x * z + s * y;
  result.m03 = 0.0f;
  
  result.m10 = t * x * y + s * z;
  result.m11 = t * y * y + c;
  result.m12 = t * y * z - s * x;
  result.m13 = 0.0f;
  
  result.m20 = t * x * z - s * y;
  result.m21 = t * y * z + s * x;
  result.m22 = t * z * z + c;
  result.m23 = 0.0f;
  
  result.m30 = 0.0f;
  result.m31 = 0.0f;
  result.m32 = 0.0f;
  result.m33 = 1.0f;
  
  return result;
}

/// <summary>
/// Helper function to create a translation matrix.
/// </summary>
Matrix4f CSpaceMouseTransform::CreateTranslationMatrix(float dx, float dy, float dz) {
  Matrix4f result;
  result.setIdentity();
  result.m03 = dx;
  result.m13 = dy;
  result.m23 = dz;
  return result;
}

} // namespace TraceNL
} // namespace TDx
