// Dear ImGui: User interface for GLFW + OpenGL 3, using programmable pipeline
// (GLFW is a cross-platform general purpose library for handling windows, inputs, OpenGL/Vulkan/Metal graphics context creation, etc.)
// If you are new to Dear ImGui, read documentation from the docs/ folder + read the top of imgui.cpp.
// Read online: https://github.com/ocornut/imgui/tree/master/docs

#include <stdio.h>
#include <string.h>
#include <vector>
#include <tbb/task_arena.h>
#include <atomic>
#include "surgicalActions.h"
#include <gl3wGraphics.h>
#include "FacialFlapsGui.h"

FacialFlapsGui ffg;

// scripted-mode framebuffer dump: 24-bit BMP straight from glReadPixels (rows are already
// bottom-up, matching BMP layout). For capturing what the compositor won't hand to a screen grab.
static void dumpFramebufferBMP(GLFWwindow* w, const char* path)
{
	int fbw, fbh;
	glfwGetFramebufferSize(w, &fbw, &fbh);
	int rowBytes = (fbw * 3 + 3) & ~3;
	std::vector<unsigned char> px((size_t)rowBytes * fbh, 0);
	glPixelStorei(GL_PACK_ALIGNMENT, 4);
	glReadPixels(0, 0, fbw, fbh, GL_BGR, GL_UNSIGNED_BYTE, px.data());
	unsigned int imgBytes = (unsigned int)px.size(), fileBytes = 54 + imgBytes;
	unsigned char hdr[54] = { 'B','M' };
	memcpy(hdr + 2, &fileBytes, 4); hdr[10] = 54; hdr[14] = 40;
	memcpy(hdr + 18, &fbw, 4); memcpy(hdr + 22, &fbh, 4);
	hdr[26] = 1; hdr[28] = 24; memcpy(hdr + 34, &imgBytes, 4);
	if (FILE* bf = fopen(path, "wb")) {
		fwrite(hdr, 1, 54, bf);
		fwrite(px.data(), 1, px.size(), bf);
		fclose(bf);
	}
}

int main(int argc, char** argv)
{
	if (!ffg.initImguiGlfw()) {
		puts("Failed to open Glfw window.\n");
		return 1;
	}
	if (!ffg.initCleftSim()) {
		puts("Failed to initialize cleft simulator.\n");
		return 1;
	}
	surgicalActions* sa = ffg.getSurgicalActions();
	bccTetScene* bts = sa->getBccTetScene();
	sa->physicsDone = true;
	// Interactive view mode: SkinFlaps.exe --view <history.hst> [modelDir]
	// Loads the scene, starts beating, keeps the window open, and responds to mouse/keys. No
	// auto-advance, no auto-close, no console metrics. Key B toggles the beat. This is the
	// hands-on path; the scripted (unattended, exits) path below is for capture/regression.
	bool interactiveView = argc > 1 && (strcmp(argv[1], "--view") == 0 || strcmp(argv[1], "-i") == 0);
	int beatFrames = (!interactiveView && argc > 3) ? atoi(argv[3]) : 0;  // scripted beat test: frames of beating after replay
	int dumpEvery = (!interactiveView && argc > 4) ? atoi(argv[4]) : 0;   // >0: dump a framebuffer BMP every N beat frames (video capture)
	if (interactiveView) {
		FacialFlapsGui::scriptedReplay = true;  // route the benign missing-.bed message to stderr, not a modal
		if (argc < 3 || !FacialFlapsGui::startScriptedReplay(argv[2], argc > 3 ? argv[3] : "")) {
			fputs("usage: SkinFlaps.exe --view <history.hst> [modelDir]\n", stderr);
			return 2;
		}
		bts->startBeating();  // stands the solver up and begins the beat; toggle later with B
	}
	else if (argc > 1) {  // scripted replay: SkinFlaps.exe <history.hst> [modelDir] [beatFrames] [dumpEvery]
		FacialFlapsGui::scriptedReplay = true;
		if (!FacialFlapsGui::startScriptedReplay(argv[1], argc > 2 ? argv[2] : "")) {
			fputs("Failed to load history file for scripted replay.\n", stderr);
			return 2;
		}
		fprintf(stderr, "Scripted replay of %s: %zu actions.\n", argv[1], sa->historySize());
	}
	int settleFrames = 120;  // scripted replay: extra solve frames after the last action before exit
	long beatFramesRun = 0;
	bool beatStarted = false;
	bool dumpShotThisFrame = false;
	bool updateThrow = false;
	while (!glfwWindowShouldClose(ffg.FFwindow))
	{
		try {
				// Poll and handle events (inputs, window resize, etc.)
			// You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
			// - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application.
			// - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application.
			// Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
			glfwPollEvents();
			// Start the Dear ImGui frame
			ImGui_ImplOpenGL3_NewFrame();
			ImGui_ImplGlfw_NewFrame();
			ImGui::NewFrame();
			if (FacialFlapsGui::physicsDrag)
				ffg.showHourglass();
			ffg.InstanceCleftGui();

			// Rendering
			ImGui::Render();
			ImVec4 clear_color = ImVec4(0.0f, 0.0f, 0.0f, 1.00f);
			glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
			glClear(GL_COLOR_BUFFER_BIT);

			if (sa->taskThreadError) {
				sa->taskThreadError = false;
				std::string err = sa->taskThreadErrorStr;
				ffg.handleThrow(err.c_str());
				throw(std::logic_error(err));
			}

			if (sa->physicsDone) {
				// draw last physics result before starting a new solve
				// Unfortunately all graphics calls must be executed fom the master thread.
				if (sa->newTopology) {
					sa->getSurgGraphics()->setNewTopology();
					sa->getSurgGraphics()->updatePositionsNormalsTangents();
					sa->newTopology = false;
				}
				if (bts->forcesApplied()) {
					sa->getSutures()->updateSutureGraphics();
					if (sa->getSurgGraphics()->getSceneNode()->visible)
						bts->updateSurfaceDraw();
					else {  // draw only tets without the surface
						if (ffg.getgl3wGraphics()->getLines()->getSceneNode() && ffg.getgl3wGraphics()->getLines()->getSceneNode()->visible)
							bts->drawTetLattice();
					}
				}
				if (ffg.physicsDrag)  //  && ffg.loadFile.empty()
					ffg.physicsDrag = false;
				if (FacialFlapsGui::scriptedReplay && !interactiveView && ffg.nextCounter < 1) {
					if (!sa->historyComplete())
						ffg.nextCounter = 1;  // auto-press NEXT
					else if (beatFrames > 0) {  // scripted beat test after the history is done
						if (!beatStarted) {
							bts->startBeating();  // one-time full solver init if nothing else did
							beatStarted = true;
							fprintf(stderr, "beat mode started: %d frames\n", beatFrames);
						}
						else {
							++beatFramesRun;
							if (dumpEvery > 0 && beatFramesRun % dumpEvery == 0)
								dumpShotThisFrame = true;
							if (beatFramesRun % 30 == 0 || beatFramesRun >= beatFrames) {
								// surface bounding-box volume as the beat metric
								std::vector<Vec3f>* px = sa->getSurgGraphics()->getMaterialTriangles()->getPositionArrayPtr();
								float mnv[3]{ 1e30f, 1e30f, 1e30f }, mxv[3]{ -1e30f, -1e30f, -1e30f };
								for (auto& p : *px) for (int c = 0; c < 3; ++c) {
									if (p[c] < mnv[c]) mnv[c] = p[c];
									if (p[c] > mxv[c]) mxv[c] = p[c];
								}
								fprintf(stderr, "beat %ld: bbox vol %.1f (%.2f x %.2f x %.2f)\n", beatFramesRun,
									(mxv[0] - mnv[0]) * (mxv[1] - mnv[1]) * (mxv[2] - mnv[2]),
									mxv[0] - mnv[0], mxv[1] - mnv[1], mxv[2] - mnv[2]);
								dumpShotThisFrame = true;
								if (beatFramesRun >= beatFrames)
									glfwSetWindowShouldClose(ffg.FFwindow, 1);
							}
						}
					}
					else if (--settleFrames < 1)
						glfwSetWindowShouldClose(ffg.FFwindow, 1);
				}
				if (ffg.nextCounter > 0) {
					ffg.getSurgicalActions()->nextHistoryAction();
					--ffg.nextCounter;
					if (FacialFlapsGui::scriptedReplay)
						fprintf(stderr, "action %zu/%zu\n", sa->historyActionsDone(), sa->historySize());
				}
				else{
				// below is from: https://www.intel.com/content/www/us/en/develop/documentation/onetbb-documentation/top/onetbb-developer-guide/design-patterns/gui-thread.html
					if (bts->forcesApplied() && !bts->isPhysicsPaused()) {  // physicsDone recheck necessary since nextHistoryAction() may have spawned a task that this one would collide with
						sa->physicsDone = false;
						tbb::task_arena(tbb::task_arena::attach()).enqueue([&]() {
							try {
								bts->updatePhysics();
								sa->physicsDone = true;
							}
							catch (...) {
								updateThrow = true;
								sa->taskThreadError = true;
								sa->taskThreadErrorStr = "Couldn't update physics after last action.";
							}
							}
						);
					}
				}
			}
			ffg.getgl3wGraphics()->drawAll();

			ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());  // Always do this last so it prints GUI on top of your scene

			if (dumpShotThisFrame) {  // scripted beat mode: dump the finished back buffer
				dumpShotThisFrame = false;
				char shotPath[64];
				snprintf(shotPath, sizeof(shotPath), "beat_%04ld.bmp", beatFramesRun);
				dumpFramebufferBMP(ffg.FFwindow, shotPath);
			}
		}
		catch (const std::runtime_error& re) {
			ffg.nextCounter = 0;
			std::string err = "Program runtime error occurred.\n";
			err += re.what();
			ffg.handleThrow(err.c_str());
		}
		catch (const std::logic_error& le){
			ffg.nextCounter = 0;
			std::string err = "Program logic error occurred.\n";
			err += le.what();
			ffg.handleThrow(err.c_str());
		}
		catch (const std::bad_alloc& ba) {
			ffg.nextCounter = 0;
			std::string err = "Not enough memory in this machine to handle this program.\n";
			err += ba.what();
			ffg.handleThrow(err.c_str());
		}
		catch (...) {
			ffg.nextCounter = 0;
			// catch any other errors
			ffg.handleThrow("Unspecified program error occurred.\n");
		}
		glfwSwapBuffers(ffg.FFwindow);
	}
	while (!updateThrow && !sa->physicsDone)
		;
	ffg.destroyImguiGlfw();
	if (FacialFlapsGui::scriptedReplay) {
		fprintf(stderr, "Scripted replay finished: %zu/%zu actions, exit code %d.\n",
			sa->historyActionsDone(), sa->historySize(), FacialFlapsGui::scriptedExitCode);
		return FacialFlapsGui::scriptedExitCode;
	}
    return 0;
}
