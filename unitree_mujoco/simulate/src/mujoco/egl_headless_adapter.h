#ifndef MUJOCO_SIMULATE_EGL_HEADLESS_ADAPTER_H_
#define MUJOCO_SIMULATE_EGL_HEADLESS_ADAPTER_H_

#include "platform_ui_adapter.h"

#include <EGL/egl.h>

namespace mujoco {
class EglHeadlessAdapter final : public PlatformUIAdapter {
 public:
  EglHeadlessAdapter();
  ~EglHeadlessAdapter() override;

  std::pair<double, double> GetCursorPosition() const override;
  double GetDisplayPixelsPerInch() const override;
  std::pair<int, int> GetFramebufferSize() const override;
  std::pair<int, int> GetWindowSize() const override;
  bool IsGPUAccelerated() const override;
  void PollEvents() override;
  void SetClipboardString(const char* text) override;
  void SetVSync(bool enabled) override;
  void SetWindowTitle(const char* title) override;
  bool ShouldCloseWindow() const override;
  void SwapBuffers() override;
  void ToggleFullscreen() override;
  bool IsLeftMouseButtonPressed() const override;
  bool IsMiddleMouseButtonPressed() const override;
  bool IsRightMouseButtonPressed() const override;
  bool IsAltKeyPressed() const override;
  bool IsCtrlKeyPressed() const override;
  bool IsShiftKeyPressed() const override;
  bool IsMouseButtonDownEvent(int act) const override;
  bool IsKeyDownEvent(int act) const override;
  int TranslateKeyCode(int key) const override;
  mjtButton TranslateMouseButton(int button) const override;

 private:
  EGLDisplay display_ = EGL_NO_DISPLAY;
  EGLContext context_ = EGL_NO_CONTEXT;
  EGLSurface surface_ = EGL_NO_SURFACE;
};
}  // namespace mujoco

#endif
