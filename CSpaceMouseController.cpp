#include "CSpaceMouseController.hpp"
#include "CNavigationModel.hpp"
#include "CViewportViewModel.hpp"
#include "CCamera3D.hpp"
#include <iostream>
#include <system_error>
#include <cmath>

namespace TDx {
namespace TraceNL {

/// <summary>
/// Custom NavigationModel that intercepts camera updates and redirects to object transform
/// </summary>
class CSpaceMouseNavigationModel : public Navigation::CNavigationModel {
public:
  CSpaceMouseNavigationModel(Navigation::IViewModelNavigation* vm, CSpaceMouseController* controller)
    : Navigation::CNavigationModel(vm)
    , m_controller(controller)
    , m_viewModel(vm)
    , m_initialCameraSet(false)
    , m_lastUpdateTime(std::chrono::steady_clock::now())
  {
  }

  long SetMotionFlag(bool motion) override {
    auto result = Navigation::CNavigationModel::SetMotionFlag(motion);
    if (motion) {
      m_initialCameraSet = false;
      m_lastUpdateTime = std::chrono::steady_clock::now();
    }
    return result;
  }

  // Override SetCameraMatrix to intercept SpaceMouse motion
  long SetCameraMatrix(const navlib::matrix_t& matrix) override {
    auto currentTime = std::chrono::steady_clock::now();
    auto timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(
      currentTime - m_lastUpdateTime).count();
    
    // First time, just store the initial state
    if (!m_initialCameraSet) {
      m_lastMatrix = matrix;
      m_initialCameraSet = true;
      m_lastUpdateTime = currentTime;
      return 0;
    }

    // If more than 500ms elapsed since last update, this might be a discontinuity
    // Reset reference instead of applying potentially large delta
    if (timeSinceLastUpdate > 500) {
      m_lastMatrix = matrix;
      m_lastUpdateTime = currentTime;
      return 0;
    }

    // Calculate the delta transformation
    // matrix format: [right.x, right.y, right.z, 0, up.x, up.y, up.z, 0, 
    //                -forward.x, -forward.y, -forward.z, 0, pos.x, pos.y, pos.z, 1]
    
    // Extract translation delta (camera movement)
    float tx = static_cast<float>(matrix.m30 - m_lastMatrix.m30);
    float ty = static_cast<float>(matrix.m31 - m_lastMatrix.m31);
    float tz = static_cast<float>(matrix.m32 - m_lastMatrix.m32);

    // Extract rotation delta using proper differential rotation calculation
    // R_delta = R_new * R_old^T
    // For small rotations, we can extract axis-angle representation
    
    // Get rotation matrices (3x3 part)
    // Old rotation matrix
    float r00_old = static_cast<float>(m_lastMatrix.m00);
    float r01_old = static_cast<float>(m_lastMatrix.m01);
    float r02_old = static_cast<float>(m_lastMatrix.m02);
    float r10_old = static_cast<float>(m_lastMatrix.m10);
    float r11_old = static_cast<float>(m_lastMatrix.m11);
    float r12_old = static_cast<float>(m_lastMatrix.m12);
    float r20_old = static_cast<float>(m_lastMatrix.m20);
    float r21_old = static_cast<float>(m_lastMatrix.m21);
    float r22_old = static_cast<float>(m_lastMatrix.m22);
    
    // New rotation matrix
    float r00_new = static_cast<float>(matrix.m00);
    float r01_new = static_cast<float>(matrix.m01);
    float r02_new = static_cast<float>(matrix.m02);
    float r10_new = static_cast<float>(matrix.m10);
    float r11_new = static_cast<float>(matrix.m11);
    float r12_new = static_cast<float>(matrix.m12);
    float r20_new = static_cast<float>(matrix.m20);
    float r21_new = static_cast<float>(matrix.m21);
    float r22_new = static_cast<float>(matrix.m22);

    // Compute R_delta = R_new * R_old^T (transpose of old)
    float d00 = r00_new * r00_old + r01_new * r01_old + r02_new * r02_old;
    float d01 = r00_new * r10_old + r01_new * r11_old + r02_new * r12_old;
    float d02 = r00_new * r20_old + r01_new * r21_old + r02_new * r22_old;
    float d10 = r10_new * r00_old + r11_new * r01_old + r12_new * r02_old;
    float d11 = r10_new * r10_old + r11_new * r11_old + r12_new * r12_old;
    float d12 = r10_new * r20_old + r11_new * r21_old + r12_new * r22_old;
    float d20 = r20_new * r00_old + r21_new * r01_old + r22_new * r02_old;
    float d21 = r20_new * r10_old + r21_new * r11_old + r22_new * r12_old;
    float d22 = r20_new * r20_old + r21_new * r21_old + r22_new * r22_old;

    // Remove rotation-induced translation (camera orbit around pivot)
    float pivot_x = 0.0f;
    float pivot_y = 0.0f;
    float pivot_z = 0.0f;
    if (m_viewModel) {
      auto pivot = m_viewModel->GetPivotPosition();
      pivot_x = static_cast<float>(pivot.x);
      pivot_y = static_cast<float>(pivot.y);
      pivot_z = static_cast<float>(pivot.z);
    }

    float old_pos_x = static_cast<float>(m_lastMatrix.m30);
    float old_pos_y = static_cast<float>(m_lastMatrix.m31);
    float old_pos_z = static_cast<float>(m_lastMatrix.m32);

    float new_pos_x = static_cast<float>(matrix.m30);
    float new_pos_y = static_cast<float>(matrix.m31);
    float new_pos_z = static_cast<float>(matrix.m32);

    float rel_old_x = old_pos_x - pivot_x;
    float rel_old_y = old_pos_y - pivot_y;
    float rel_old_z = old_pos_z - pivot_z;

    float rot_old_x = d00 * rel_old_x + d01 * rel_old_y + d02 * rel_old_z;
    float rot_old_y = d10 * rel_old_x + d11 * rel_old_y + d12 * rel_old_z;
    float rot_old_z = d20 * rel_old_x + d21 * rel_old_y + d22 * rel_old_z;

    float expected_x = pivot_x + rot_old_x;
    float expected_y = pivot_y + rot_old_y;
    float expected_z = pivot_z + rot_old_z;

    float delta_world_x = new_pos_x - expected_x;
    float delta_world_y = new_pos_y - expected_y;
    float delta_world_z = new_pos_z - expected_z;

    // Check if delta is abnormally large (indicates a reset or discontinuity)
    // This happens when navlib resets the camera after idle time
    float translation_magnitude = std::sqrt(delta_world_x * delta_world_x + delta_world_y * delta_world_y + delta_world_z * delta_world_z);
    const float MAX_TRANSLATION_PER_FRAME = 10.0f;  // 通常の操作では1フレームで10単位以上動かない
    
    if (translation_magnitude > MAX_TRANSLATION_PER_FRAME) {
      // Reset reference instead of applying large delta
      m_lastMatrix = matrix;
      m_lastUpdateTime = currentTime;
      return 0;
    }

    // Re-map translation to world axes (remove camera orientation)
    // navlib provides translation in the camera's local basis; convert it back
    // to world-aligned axes using the previous camera orientation.
    {
      float right_x = static_cast<float>(m_lastMatrix.m00);
      float right_y = static_cast<float>(m_lastMatrix.m01);
      float right_z = static_cast<float>(m_lastMatrix.m02);

      float up_x = static_cast<float>(m_lastMatrix.m10);
      float up_y = static_cast<float>(m_lastMatrix.m11);
      float up_z = static_cast<float>(m_lastMatrix.m12);

      float forward_x = -static_cast<float>(m_lastMatrix.m20);
      float forward_y = -static_cast<float>(m_lastMatrix.m21);
      float forward_z = -static_cast<float>(m_lastMatrix.m22);

      float tx_world = right_x * delta_world_x + right_y * delta_world_y + right_z * delta_world_z;
      float ty_world = up_x * delta_world_x + up_y * delta_world_y + up_z * delta_world_z;
      float tz_world = forward_x * delta_world_x + forward_y * delta_world_y + forward_z * delta_world_z;

      tx = tx_world;
      ty = ty_world;
      tz = tz_world;
    }

    // Extract rotation angles from delta matrix (using small angle approximation)
    // For small rotations, the off-diagonal elements represent the rotation
    float rx = (d21 - d12) * 0.5f;  // Pitch (X軸周り回転)
    float ry = (d02 - d20) * 0.5f;  // Yaw   (Y軸周り回転)
    float rz = (d10 - d01) * 0.5f;  // Roll  (Z軸周り回転)

    // Apply to our transform with axis mapping:
    // SpaceMouse操作 → オブジェクトの動き
    // - マウス左右 → X軸左右移動
    // - マウス上下 → Y軸上下移動  
    // - マウス前後 → Z軸前後移動
    // - ピッチ → X軸周り回転
    // - ヨー → Y軸周り回転
    // - ロール → Z軸周り回転
    
    if (m_controller) {
      // カメラの逆の動きをオブジェクトに適用
      // 符号を反転して、感覚的な操作にする
      m_controller->ApplyMotionInput(
        -tx,  // X軸左右移動
        -ty,  // Y軸上下移動
        -tz,  // Z軸前後移動
        -rx,  // X軸周り回転 (ピッチ)
        -ry,  // Y軸周り回転 (ヨー)
        -rz   // Z軸周り回転 (ロール)
      );
    }

    // Store current as last
    m_lastMatrix = matrix;
    m_lastUpdateTime = currentTime;

    // Don't actually move the camera
    return 0;
  }

private:
  CSpaceMouseController* m_controller;
  Navigation::IViewModelNavigation* m_viewModel;
  bool m_initialCameraSet;
  navlib::matrix_t m_lastMatrix;
  std::chrono::steady_clock::time_point m_lastUpdateTime;
};

/// <summary>
/// Implementation class that integrates with the SpaceMouse SDK
/// </summary>
class CSpaceMouseController::Impl {
public:
  Impl(CSpaceMouseController* owner)
    : m_owner(owner)
    , m_navigationModel(nullptr)
    , m_viewModel(nullptr)
  {
  }

  ~Impl() {
    Cleanup();
  }

  bool Enable(const char* profileHint) {
    try {
      // Create the view model
      m_viewModel = std::make_shared<ViewModels::CViewportViewModel>();
      
      // Setup a default viewport  
      auto viewport = std::make_shared<Media3D::CViewport3D>();
      m_viewModel->SetView(viewport);
      
      // Create our custom navigation model
      m_navigationModel = std::make_unique<CSpaceMouseNavigationModel>(m_viewModel.get(), m_owner);
      
      // Set the profile hint (application name for 3Dconnexion Settings)
      if (profileHint) {
        m_navigationModel->PutProfileHint(profileHint);
      }
      
      // CRITICAL: Enable the navigation connection
      std::error_code ec;
      m_navigationModel->EnableNavigation(true, ec);
      
      if (ec) {
        std::cerr << "[ERROR] Failed to enable navigation!" << std::endl;
        std::cerr << "        Error: " << ec.message() << " (code: " << ec.value() << ")" << std::endl;
        std::cerr << "        Make sure 3Dconnexion driver is running" << std::endl;
        Cleanup();
        return false;
      }
      
      return true;
    }
    catch (const std::exception& e) {
      std::cerr << "\n[ERROR] Exception: " << e.what() << std::endl;
      Cleanup();
      return false;
    }
  }

  void Disable() {
    Cleanup();
  }

  void Cleanup() {
    if (m_navigationModel) {
      // Disable navigation before destroying
      std::error_code ec;
      m_navigationModel->EnableNavigation(false, ec);
      if (ec) {
        std::cerr << "[WARN] Error disabling navigation: " << ec.message() << std::endl;
      }
      m_navigationModel.reset();
    }
    m_viewModel.reset();
  }
  
  // Update method to poll navlib
  void Update() {
    if (m_navigationModel) {
      // The navigation model may need to be updated/polled
      // This allows navlib to process events and call callbacks like SetCameraMatrix
      try {
        // Some implementations may need explicit update calls
        // For now, the multi-threaded navlib should handle this automatically
      }
      catch (...) {
        // Ignore errors
      }
    }
  }

private:
  CSpaceMouseController* m_owner;
  std::unique_ptr<CSpaceMouseNavigationModel> m_navigationModel;
  std::shared_ptr<ViewModels::CViewportViewModel> m_viewModel;
};

/// <summary>
/// Initializes a new instance of the CSpaceMouseController class.
/// </summary>
CSpaceMouseController::CSpaceMouseController()
  : m_transform(std::make_shared<CSpaceMouseTransform>())
  , m_enabled(false)
  , m_motionScale(1.0f)
  , m_impl(nullptr)
{
}

/// <summary>
/// Destructor.
/// </summary>
CSpaceMouseController::~CSpaceMouseController() {
  Disable();
  if (m_impl) {
    delete m_impl;
    m_impl = nullptr;
  }
}

/// <summary>
/// Enables SpaceMouse navigation and starts listening to input.
/// </summary>
bool CSpaceMouseController::Enable(const char* profileHint) {
  if (m_enabled) {
    return true;
  }

  // Create implementation if needed
  if (!m_impl) {
    m_impl = new Impl(this);
  }

  // Initialize the navigation model
  if (!m_impl->Enable(profileHint)) {
    std::cerr << "[ERROR] Failed to initialize SpaceMouse navigation model" << std::endl;
    std::cerr << "        The application will continue without SpaceMouse support" << std::endl;
    return false;
  }
  
  m_enabled = true;
  return true;
}

/// <summary>
/// Disables SpaceMouse navigation.
/// </summary>
void CSpaceMouseController::Disable() {
  if (!m_enabled) {
    return;
  }

  if (m_impl) {
    m_impl->Disable();
  }

  m_enabled = false;
}

/// <summary>
/// Resets the transformation to identity.
/// </summary>
void CSpaceMouseController::ResetTransform() {
  m_transform->Reset();
}

/// <summary>
/// Applies motion from SpaceMouse input (uses local coordinate system).
/// </summary>
void CSpaceMouseController::ApplyMotionInput(float tx, float ty, float tz, float rx, float ry, float rz) {
  if (!m_enabled || !m_transform) {
    return;
  }

  // Create translation and rotation vectors
  Vector3f translation(tx, ty, tz);
  Vector3f rotation(rx, ry, rz);

  // Apply the motion using hybrid coordinate system:
  // - Translation: World coordinate system (always X/Y/Z direction)
  // - Rotation: Local coordinate system (around object's own axes)
  m_transform->ApplyLocalMotion(translation, rotation, m_motionScale);
}

/// <summary>
/// Updates the SpaceMouse state (polls for events from navlib).
/// </summary>
void CSpaceMouseController::Update() {
  if (!m_enabled || !m_impl) {
    return;
  }
  
  m_impl->Update();
}

} // namespace TraceNL
} // namespace TDx
