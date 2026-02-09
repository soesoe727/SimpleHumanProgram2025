#include "SpaceMouseGLUTHelper.hpp"
#include "CSpaceMouseController.hpp"

// OpenGL for matrix operations
#ifdef _WIN32
#include <windows.h>
#endif
#include <GL/gl.h>

namespace SpaceMouseHelper {

/// <summary>
/// Implementation class (Pimpl idiom to hide SDK dependencies)
/// </summary>
class CSpaceMouseGLUT::Impl {
public:
  Impl() : m_controller(nullptr) {}

  ~Impl() {
    if (m_controller) {
      m_controller->Disable();
      delete m_controller;
    }
  }

  bool Initialize(const char* applicationName) {
    if (m_controller) {
      return true;  // Already initialized
    }

    m_controller = new TDx::TraceNL::CSpaceMouseController();
    return m_controller->Enable(applicationName);
  }

  void Shutdown() {
    if (m_controller) {
      m_controller->Disable();
      delete m_controller;
      m_controller = nullptr;
    }
  }

  void GetTransformMatrix(Matrix4f& matrix) const {
    if (m_controller && m_controller->GetTransform()) {
      m_controller->GetTransform()->GetTransformMatrix(matrix);
    } else {
      matrix.setIdentity();
    }
  }

  void ApplyToGLMatrix() const {
    Matrix4f mat;
    GetTransformMatrix(mat);
    
    // Convert Matrix4f to OpenGL format (column-major)
    float glMatrix[16] = {
      mat.m00, mat.m10, mat.m20, mat.m30,
      mat.m01, mat.m11, mat.m21, mat.m31,
      mat.m02, mat.m12, mat.m22, mat.m32,
      mat.m03, mat.m13, mat.m23, mat.m33
    };
    
    glMultMatrixf(glMatrix);
  }

  void Reset() {
    if (m_controller) {
      m_controller->ResetTransform();
    }
  }

  void SetSensitivity(float scale) {
    if (m_controller) {
      m_controller->SetMotionScale(scale);
    }
  }

  void ApplyTranslation(float dx, float dy, float dz) {
    if (m_controller && m_controller->GetTransform()) {
      m_controller->GetTransform()->ApplyTranslation(dx, dy, dz);
    }
  }

  void ApplyRotation(float angle_rad, float axis_x, float axis_y, float axis_z) {
    if (m_controller && m_controller->GetTransform()) {
      m_controller->GetTransform()->ApplyRotation(angle_rad, axis_x, axis_y, axis_z);
    }
  }

  void ApplyLocalRotation(float angle_rad, float axis_x, float axis_y, float axis_z) {
    if (m_controller && m_controller->GetTransform()) {
      m_controller->GetTransform()->ApplyLocalRotation(angle_rad, axis_x, axis_y, axis_z);
    }
  }

  void ApplyMotionInput(float tx, float ty, float tz, float rx, float ry, float rz) {
    if (m_controller) {
      m_controller->ApplyMotionInput(tx, ty, tz, rx, ry, rz);
    }
  }

  void Update() {
    if (m_controller) {
      m_controller->Update();
    }
  }

private:
  TDx::TraceNL::CSpaceMouseController* m_controller;
};

/// <summary>
/// Gets the singleton instance.
/// </summary>
CSpaceMouseGLUT& CSpaceMouseGLUT::GetInstance() {
  static CSpaceMouseGLUT instance;
  return instance;
}

CSpaceMouseGLUT::CSpaceMouseGLUT() : m_impl(new Impl()) {}

CSpaceMouseGLUT::~CSpaceMouseGLUT() {
  delete m_impl;
}

bool CSpaceMouseGLUT::Initialize(const char* applicationName) {
  return m_impl->Initialize(applicationName);
}

void CSpaceMouseGLUT::Shutdown() {
  m_impl->Shutdown();
}

void CSpaceMouseGLUT::GetTransformMatrix(Matrix4f& matrix) const {
  m_impl->GetTransformMatrix(matrix);
}

void CSpaceMouseGLUT::ApplyToGLMatrix() const {
  m_impl->ApplyToGLMatrix();
}

void CSpaceMouseGLUT::Reset() {
  m_impl->Reset();
}

void CSpaceMouseGLUT::SetSensitivity(float scale) {
  m_impl->SetSensitivity(scale);
}

void CSpaceMouseGLUT::ApplyTranslation(float dx, float dy, float dz) {
  m_impl->ApplyTranslation(dx, dy, dz);
}

void CSpaceMouseGLUT::ApplyRotation(float angle_rad, float axis_x, float axis_y, float axis_z) {
  m_impl->ApplyRotation(angle_rad, axis_x, axis_y, axis_z);
}

void CSpaceMouseGLUT::ApplyLocalRotation(float angle_rad, float axis_x, float axis_y, float axis_z) {
  m_impl->ApplyLocalRotation(angle_rad, axis_x, axis_y, axis_z);
}

void CSpaceMouseGLUT::ApplyMotionInput(float tx, float ty, float tz, float rx, float ry, float rz) {
  m_impl->ApplyMotionInput(tx, ty, tz, rx, ry, rz);
}

void CSpaceMouseGLUT::Update() {
  m_impl->Update();
}

} // namespace SpaceMouseHelper
