// xrViewer.h — OpenXR head-tracked stereo viewer for SkinFlaps (v1: view-only).
//
// Renders the existing gl3wGraphics scene per-eye into OpenXR swapchains while the desktop
// window keeps working as the mirror with all tools. Enabled by the -xr command-line switch;
// every failure (no runtime, no headset, no GL support) degrades to desktop-only with a
// message on stderr. Interaction stays on the desktop for now — controllers are phase 2.

#ifndef __XR_VIEWER_H__
#define __XR_VIEWER_H__

#include <cstdint>
#include <vector>

class gl3wGraphics;
struct GLFWwindow;

class xrViewer
{
public:
	// Create instance/session/swapchains. Call once, with the GL context current on this thread.
	// Returns false (with stderr diagnostics) if VR is unavailable; the app then runs desktop-