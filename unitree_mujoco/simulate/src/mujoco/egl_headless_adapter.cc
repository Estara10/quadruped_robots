#include "egl_headless_adapter.h"

#include <EGL/eglext.h>
#include <GL/gl.h>

#include <cstdio>
#include <cstdlib>

namespace mujoco {
EglHeadlessAdapter::EglHeadlessAdapter() {
  auto get_platform_display = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(
      eglGetProcAddress("eglGetPlatformDisplayEXT"));
  if (!get_platform_display) mju_error("EGL surfaceless platform is unavailable");
  display_ = get_platform_display(EGL_PLATFORM_SURFACELESS_MESA,
                                  EGL_DEFAULT_DISPLAY, nullptr);
  EGLint major = 0, minor = 0;
  if (display_ == EGL_NO_DISPLAY || !eglInitialize(display_, &major, &minor))
    mju_error("could not initialize surfaceless EGL");
  if (!eglBindAPI(EGL_OPENGL_API)) mju_error("could not bind EGL OpenGL API");

  const EGLint config_attributes[] = {
      EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
      EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
      EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8,
      EGL_NONE};
  EGLConfig config = nullptr;
  EGLint config_count = 0;
  if (!eglChooseConfig(display_, config_attributes, &config, 1, &config_count) ||
      config_count != 1)
    mju_error("could not choose EGL pbuffer config");
  const EGLint surface_attributes[] = {EGL_WIDTH, 1280, EGL_HEIGHT, 720, EGL_NONE};
  surface_ = eglCreatePbufferSurface(display_, config, surface_attributes);
  context_ = eglCreateContext(display_, config, EGL_NO_CONTEXT, nullptr);
  if (surface_ == EGL_NO_SURFACE || context_ == EGL_NO_CONTEXT ||
      !eglMakeCurrent(display_, surface_, surface_, context_))
    mju_error("could not create current EGL pbuffer context");
  std::printf("[MUJOCO-RENDER] backend=EGL_SURFACELESS visible=0 renderer=%s version=%s\n",
              glGetString(GL_RENDERER), glGetString(GL_VERSION));
}

EglHeadlessAdapter::~EglHeadlessAdapter() {
  if (display_ != EGL_NO_DISPLAY) {
    if (eglGetCurrentContext() == context_) {
      FreeMjrContext();
      eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    }
    if (surface_ != EGL_NO_SURFACE) eglDestroySurface(display_, surface_);
    if (context_ != EGL_NO_CONTEXT) eglDestroyContext(display_, context_);
    eglTerminate(display_);
  }
}

std::pair<double, double> EglHeadlessAdapter::GetCursorPosition() const { return {0, 0}; }
double EglHeadlessAdapter::GetDisplayPixelsPerInch() const { return 96.0; }
std::pair<int, int> EglHeadlessAdapter::GetFramebufferSize() const { return {1280, 720}; }
std::pair<int, int> EglHeadlessAdapter::GetWindowSize() const { return {1280, 720}; }
bool EglHeadlessAdapter::IsGPUAccelerated() const { return false; }
void EglHeadlessAdapter::PollEvents() {}
void EglHeadlessAdapter::SetClipboardString(const char*) {}
void EglHeadlessAdapter::SetVSync(bool) {}
void EglHeadlessAdapter::SetWindowTitle(const char*) {}
bool EglHeadlessAdapter::ShouldCloseWindow() const { return false; }
void EglHeadlessAdapter::SwapBuffers() { eglSwapBuffers(display_, surface_); }
void EglHeadlessAdapter::ToggleFullscreen() {}
bool EglHeadlessAdapter::IsLeftMouseButtonPressed() const { return false; }
bool EglHeadlessAdapter::IsMiddleMouseButtonPressed() const { return false; }
bool EglHeadlessAdapter::IsRightMouseButtonPressed() const { return false; }
bool EglHeadlessAdapter::IsAltKeyPressed() const { return false; }
bool EglHeadlessAdapter::IsCtrlKeyPressed() const { return false; }
bool EglHeadlessAdapter::IsShiftKeyPressed() const { return false; }
bool EglHeadlessAdapter::IsMouseButtonDownEvent(int) const { return false; }
bool EglHeadlessAdapter::IsKeyDownEvent(int) const { return false; }
int EglHeadlessAdapter::TranslateKeyCode(int key) const { return key; }
mjtButton EglHeadlessAdapter::TranslateMouseButton(int) const { return mjBUTTON_NONE; }
}  // namespace mujoco
