// xrSession.h - minimal OpenXR + OpenGL stereo session for SkinFlaps.
//
// Wraps an OpenXR instance/session/swapchains around the app's existing GLFW/WGL GL context and
// exposes a per-frame loop that renders the scene twice (one FBO per eye) via a supplied callback.
// The physics and the scene are untouched; this is purely a second consumer of the renderer, the
// way the design docs describe a Unity/splat front end would be.
//
// Usage in the frame loop:
//     if (xr.beginFrame()) {                        // false => runtime idle this frame, skip
//         for (int eye = 0; eye < xr.eyeCount(); ++eye) {
//             xr.beginEye(eye);                      // binds this eye's FBO + viewport
//             float view[16], proj[16];
//             xr.eyeMatrices(eye, worldScale, view, proj);
//             gl3w->drawSceneWithMatrices(view, proj);
//             xr.endEye(eye);
//         }
//         xr.endFrame();
//     }
//
// All OpenXR types are hidden behind an opaque impl so this header carries no <openxr.h> dependency.

#ifndef __XR_SESSION_H__
#define __XR_SESSION_H__

struct GLFWwindow;

class xrSession {
public:
	xrSession();
	~xrSession();

	// Create instance/session/swapchains bound to the current GL context of `window`.
	// Returns false (and stays inert) if no runtime is present or init fails - caller falls back
	// to desktop. `reason` receives a human-readable message either way.
	bool initialize(GLFWwindow* window, char* reasonBuf, int reasonBufLen);
	bool active() const { return _active; }
	int  eyeCount() const;

	// Pump OpenXR events; call once per frame before beginFrame(). Sets `exitRequested` if the
	// runtime asked us to quit (headset removed, runtime shutdown).
	void pollEvents(bool& exitRequested);

	// Returns true if the runtime wants rendering this frame. If false, do not draw the eyes.
	bool beginFrame();
	void beginEye(int eye);   // acquire+bind this eye's swapchain image as an FBO, set viewport, clear
	// Fill column-major GL view (world->eye) and projection matrices for this eye.
	// worldScale maps model units to OpenXR meters (scene is ~tens of model units across).
	void eyeMatrices(int eye, float worldScale, float (&view)[16], float (&proj)[16]);
	void endEye(int eye);     // release this eye's swapchain image
	void endFrame();          // submit the composited layer to the compositor

	void shutdown();

private:
	struct Impl;
	Impl* _p;
	bool _active;
};

#endif // __XR_SESSION_H__
