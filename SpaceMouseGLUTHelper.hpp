#ifndef SPACEMOUSE_GLUT_HELPER_HPP_INCLUDED
#define SPACEMOUSE_GLUT_HELPER_HPP_INCLUDED

// vecmath for compatibility with SimpleHumanGLUT
#include <Matrix4.h>

namespace SpaceMouseHelper {

/// <summary>
/// Simple interface for integrating SpaceMouse with GLUT applications.
/// This provides a minimal API that can be used without depending on the full SDK.
/// </summary>
class CSpaceMouseGLUT {
public:
  /// <summary>
  /// Gets the singleton instance.
  /// </summary>
  static CSpaceMouseGLUT& GetInstance();

  /// <summary>
  /// Initializes SpaceMouse support.
  /// Call this once during application initialization.
  /// </summary>
  /// <param name="applicationName">Name of your application.</param>
  /// <returns>True if successful, false otherwise.</returns>
  bool Initialize(const char* applicationName);

  /// <summary>
  /// Shuts down SpaceMouse support.
  /// Call this during application cleanup.
  /// </summary>
  void Shutdown();

  /// <summary>
  /// Gets the current transformation matrix for an object.
  /// </summary>
  /// <param name="matrix">Output parameter to receive the matrix.</param>
  void GetTransformMatrix(Matrix4f& matrix) const;

  /// <summary>
  /// Applies the transformation matrix to the current OpenGL matrix.
  /// Call this in your display function after setting up your modelview matrix.
  /// </summary>
  void ApplyToGLMatrix() const;

  /// <summary>
  /// Resets the transformation to identity.
  /// </summary>
  void Reset();

  /// <summary>
  /// Sets the motion sensitivity (default is 1.0).
  /// Higher values make the SpaceMouse more sensitive.
  /// </summary>
  /// <param name="scale">Sensitivity scale factor.</param>
  void SetSensitivity(float scale);

  /// <summary>
  /// Manual update for translation (useful for testing or keyboard simulation).
  /// </summary>
  void ApplyTranslation(float dx, float dy, float dz);

  /// <summary>
  /// Manual update for rotation in world coordinates (useful for testing or keyboard simulation).
  /// </summary>
  /// <param name="angle_rad">Rotation angle in radians.</param>
  /// <param name="axis_x">X component of rotation axis.</param>
  /// <param name="axis_y">Y component of rotation axis.</param>
  /// <param name="axis_z">Z component of rotation axis.</param>
  void ApplyRotation(float angle_rad, float axis_x, float axis_y, float axis_z);

  /// <summary>
  /// Manual update for rotation in local coordinates (useful for testing or keyboard simulation).
  /// </summary>
  /// <param name="angle_rad">Rotation angle in radians.</param>
  /// <param name="axis_x">X component of rotation axis in local space.</param>
  /// <param name="axis_y">Y component of rotation axis in local space.</param>
  /// <param name="axis_z">Z component of rotation axis in local space.</param>
  void ApplyLocalRotation(float angle_rad, float axis_x, float axis_y, float axis_z);

  /// <summary>
  /// Applies motion input from SpaceMouse device.
  /// This method uses local coordinate system for intuitive control.
  /// Call this when motion events are received from the SpaceMouse.
  /// </summary>
  /// <param name="tx">Translation X component.</param>
  /// <param name="ty">Translation Y component.</param>
  /// <param name="tz">Translation Z component.</param>
  /// <param name="rx">Rotation X component (radians).</param>
  /// <param name="ry">Rotation Y component (radians).</param>
  /// <param name="rz">Rotation Z component (radians).</param>
  void ApplyMotionInput(float tx, float ty, float tz, float rx, float ry, float rz);

  /// <summary>
  /// Updates the SpaceMouse state. Call this regularly from your main loop.
  /// This polls for events from the SpaceMouse driver.
  /// </summary>
  void Update();

private:
  CSpaceMouseGLUT();
  ~CSpaceMouseGLUT();
  
  CSpaceMouseGLUT(const CSpaceMouseGLUT&) = delete;
  CSpaceMouseGLUT& operator=(const CSpaceMouseGLUT&) = delete;

  class Impl;
  Impl* m_impl;
};

} // namespace SpaceMouseHelper

#endif // SPACEMOUSE_GLUT_HELPER_HPP_INCLUDED
