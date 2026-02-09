#ifndef CSPACEMOUSECONTROLLER_HPP_INCLUDED
#define CSPACEMOUSECONTROLLER_HPP_INCLUDED

#include "CSpaceMouseTransform.hpp"
//#include "CNavigationModel.hpp"
#include <memory>

namespace TDx {
namespace TraceNL {

/// <summary>
/// Controller class that bridges SpaceMouse SDK navigation model and transformation matrix.
/// This class listens to navigation events and updates a transformation matrix accordingly.
/// </summary>
class CSpaceMouseController {
public:
/// <summary>
/// Initializes a new instance of the CSpaceMouseController class.
/// </summary>
CSpaceMouseController();

/// <summary>
/// Destructor.
/// </summary>
~CSpaceMouseController();

/// <summary>
/// Gets the transformation object managed by this controller.
/// </summary>
std::shared_ptr<CSpaceMouseTransform> GetTransform() const { return m_transform; }

  /// <summary>
  /// Enables SpaceMouse navigation and starts listening to input.
  /// </summary>
  /// <param name="profileHint">Application name to display in 3Dconnexion Properties.</param>
  /// <returns>True if successfully enabled, false otherwise.</returns>
  bool Enable(const char* profileHint);

  /// <summary>
  /// Disables SpaceMouse navigation.
  /// </summary>
  void Disable();

  /// <summary>
  /// Checks if the controller is currently enabled.
  /// </summary>
  bool IsEnabled() const { return m_enabled; }

  /// <summary>
  /// Sets the motion sensitivity scale factor.
  /// </summary>
  /// <param name="scale">Scale factor (default 1.0).</param>
  void SetMotionScale(float scale) { m_motionScale = scale; }

  /// <summary>
  /// Gets the current motion sensitivity scale factor.
  /// </summary>
  float GetMotionScale() const { return m_motionScale; }

  /// <summary>
  /// Resets the transformation to identity.
  /// </summary>
  void ResetTransform();

  /// <summary>
  /// Applies motion from SpaceMouse input (uses local coordinate system).
  /// This method should be called when motion events are received from the device.
  /// </summary>
  /// <param name="tx">Translation X component.</param>
  /// <param name="ty">Translation Y component.</param>
  /// <param name="tz">Translation Z component.</param>
  /// <param name="rx">Rotation X component (angle in radians or axis component).</param>
  /// <param name="ry">Rotation Y component.</param>
  /// <param name="rz">Rotation Z component.</param>
  void ApplyMotionInput(float tx, float ty, float tz, float rx, float ry, float rz);

  /// <summary>
  /// Updates the SpaceMouse state (polls for events from navlib).
  /// This should be called regularly from the main application loop.
  /// </summary>
  void Update();

private:
  std::shared_ptr<CSpaceMouseTransform> m_transform;
  bool m_enabled;
  float m_motionScale;

  // Navigation model for SpaceMouse SDK integration
  class Impl;
  Impl* m_impl;
};

} // namespace TraceNL
} // namespace TDx

#endif // CSPACEMOUSECONTROLLER_HPP_INCLUDED
