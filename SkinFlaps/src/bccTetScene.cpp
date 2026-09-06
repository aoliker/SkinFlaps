//////////////////////////////////////////////////////////////////
// File: bccTetScene.cpp
// Author: Court Cutting
// Date: 3/4/2019
// Purpose: Bcc tet based projective dynamics physics library interface to surgical simulator code.
//    This version uses the ShapeOp subroutine library of Sofien Bouaziz ( http://ShapeOp.org )
//    to do the physics.  So far have experimented with mass-springs, physX(NVIDIA),
//    physBAM(Fedkiw, Teran, Sifakis, et al.), corotated linear elasticity(Teran, Sifakis, Mitchell) and
//    the shape matching code of Rivers and James.
//    Copyright 2019 - All rights reserved at the present time.
// Revised: 11/1/2024
// Description: This revision uses the multiresolution bcc tet classes to drop tet count.
///////////////////////////////////////////////////////////////////

#include "bccTetScene.h"
#include <string>
#include <fstream>
#include <algorithm>
#include "gl3wGraphics.h"
#include "surgicalActions.h"
#include "boundingBox.h"
#include "json.h"
#include "closestPointOnTriangle.h"
#include "remapTetPhysics.h"
#include <iostream>
#include <cstdint>
#include <chrono>
#include <ctime>

bool bccTetScene::loadScene(const char *dataDirectory, const char *sceneFileName)
{
	_physicsPaused = true;
	std::string path(dataDirectory);
	path.append(sceneFileName);
	std::ifstream istr(path.c_str());
	std::string jsonStr;
	if (!istr.is_open()) {
		path = std::string("Unable to load: ") + path;
		_surgAct->sendUserMessage(path.c_str(), "Error Message");
		istr.close();
		return false;
	}
	else {
		char ch;
		while (istr.get(ch))
			jsonStr.push_back(ch);

	}
	istr.close();
	json::Value my_data = json::Deserialize(jsonStr);  // will trim leading and trailing white space from {} pair
	if (my_data.GetType() != json::ObjectVal) {
		_surgAct->sendUserMessage("Module file not in correct JSON format-", "Error Message");
		return false;
	}
	json::Object scnObj = my_data.ToObject();
	json::Object::ValueMap::iterator oit, suboit, suboit2;
	// get texture files first
	std::map<int, GLuint> txMap;
	std::string nrm, tex;
	if ((oit = scnObj.find("textureFiles")) == scnObj.end()) {
		_surgAct->sendUserMessage("No texture files in scene file-", "Error Message");
		return false;
	}
	else {
		json::Object txObj = oit->second.ToObject();
		for (suboit = txObj.begin(); suboit != txObj.end(); ++suboit) {
			path = dataDirectory + suboit->first;
			GLuint txNow = _gl3w->getTextures()->loadTexture(suboit->second.ToInt(), path.c_str());
			if (txNow > 0xfffffffe) {
				path = "Unable to load bitmap .bmp input file: " + path;
				_surgAct->sendUserMessage(path.c_str(), "Error Message");
				return false;
			}
			int txNum = suboit->second.ToInt();
			txMap.insert(std::make_pair(txNum, txNow));
		}
	}
	if ((oit = scnObj.find("staticObjects")) != scnObj.end()) {
		json::Object statObj = oit->second.ToObject();
		for (suboit = statObj.begin(); suboit != statObj.end(); ++suboit) {
			path = dataDirectory + suboit->first;
			std::map<int, GLuint>::iterator tit;
			std::vector<int> txIds;
			json::Object tmapObj = suboit->second.ToObject();
			for (suboit2 = tmapObj.begin(); suboit2 != tmapObj.end(); ++suboit2) {
				if (suboit2->first == "textureMap")
					txIds.push_back(suboit2->second.ToInt());
				else if (suboit2->first == "normalMap")
					txIds.push_back(suboit2->second.ToInt());
				else {
					_surgAct->sendUserMessage("Incorrect static object section in .smd input file-", "Error Message");
					return false;
				}
			}
			// this is a staticTriangle, not elastic so put on graphics card and clean up
			if ( _gl3w->loadStaticObjFile(path.c_str(), txIds, true) == NULL)
			{
				_surgAct->sendUserMessage("Unable to load fixed triangle .obj input file-", "Error Message");
				return false;
			}
		}
	}
	std::string deepBedFilepath;
	deepBedFilepath.clear();
	if ((oit = scnObj.find("dynamicObjects")) == scnObj.end()) {
		_surgAct->sendUserMessage("No dynamic objects in this scene file-", "Error Message");
		return false;
	}
	else {
		json::Object dynObj = oit->second.ToObject();
		std::vector<int> txIds;
		for (suboit = dynObj.begin(); suboit != dynObj.end(); ++suboit) {
			path = dataDirectory + suboit->first;
			std::map<int, GLuint>::iterator tit;
			json::Object tmapObj = suboit->second.ToObject();
			for (suboit2 = tmapObj.begin(); suboit2 != tmapObj.end(); ++suboit2) {
				if (suboit2->first == "textureMaps") {
					json::Array txArr;
					txArr = suboit2->second.ToArray();
					for (int i = 0; i < txArr.size(); ++i) {
						txIds.push_back(txArr[i].ToInt());
						if (!_gl3w->getTextures()->textureExists(txIds.back())) {
							_surgAct->sendUserMessage("Missing texture or normal map in dynamic triangle section in .smd input file-", "Error Message");
							return false;
						}
					}
				}
			}
			_mt = _surgAct->getSurgGraphics()->getMaterialTriangles();
			if (_mt->readObjFile(path.c_str())) {
				_surgAct->sendUserMessage("Unable to load fixed materialTriangle .obj input file-", "Error Message");
				return false;
			}
			// same material texture seams processed in graphics,
			// may want to create hard texture & normal seams between materials here.
			_surgAct->getSurgGraphics()->setGl3wGraphics(_gl3w);
			std::string vtxShd(dataDirectory), frgShd(dataDirectory);
			vtxShd.append("mtVertexShader.txt");
			frgShd.append("mtFragmentShader.txt");
			_surgAct->getSurgGraphics()->setTextureFilesCreateProgram(txIds, vtxShd.c_str(), frgShd.c_str());  // openGL buffers ceated here
			_surgAct->getSurgGraphics()->setNewTopology();
			_surgAct->getSurgGraphics()->updatePositionsNormalsTangents();
			_surgAct->getSurgGraphics()->computeLocalBounds();
			path = suboit->first;
			size_t pos = path.rfind(".obj");
			path.erase(pos);
			_mt->setName(path.c_str());
			_surgAct->getSurgGraphics()->getSceneNode()->setName(path.c_str());
			// input new deep bed file here
			deepBedFilepath.clear();
			deepBedFilepath.append(dataDirectory);
			deepBedFilepath.append(path);
			deepBedFilepath.append(".bed");
		}
	}
	if ((oit = scnObj.find("fixedGeometry")) != scnObj.end()) {
		// now using fixedCollisionSets instead
		throw(std::logic_error("Model .smd file sent to simulator uses an old fixedGeometry specifier that is no longer supported.\n"));
	}
	if ((oit = scnObj.find("fixedCollisionSets")) != scnObj.end()) {
		json::Object hullObj = oit->second.ToObject();
		std::string lsPath;
		for (suboit = hullObj.begin(); suboit != hullObj.end(); ++suboit) {
			lsPath = dataDirectory + suboit->first;
			json::Array polyArr;
			polyArr = suboit->second.ToArray();
			std::vector<int> vIdx;
			vIdx.reserve(polyArr.size());
			for (int i = 0; i < polyArr.size(); ++i)
				vIdx.push_back( polyArr[i].ToInt());
			_tetCol.addFixedCollisionSet(lsPath, vIdx);
		}
	}
	int nTetSizeLevels = 4, maxDimMegatetSubdivs = 31;  // Multires settings initial tet count 11,587 tets while old single res was0.5 million tets for cleft model.  Now loaded in properties below.
	if ((oit = scnObj.find("tetrahedralProperties")) != scnObj.end()) {
		json::Object hullObj = oit->second.ToObject();
		float lowTetWeight, highTetWeight, TJunctionWeight, strainMin, strainMax, collisionWeight, fixedWeight, periferalWeight, hookWeight, sutureWeight, autoSutureSpacing, selfCollisionWeight;
		for (suboit = hullObj.begin(); suboit != hullObj.end(); ++suboit) {
			if (suboit->first == "minStrain")
				strainMin = suboit->second.ToFloat();
			else if (suboit->first == "maxStrain")
				strainMax = suboit->second.ToFloat();
			else if (suboit->first == "lowTetWeight")
				_lowTetWeight = lowTetWeight = suboit->second.ToFloat();
			else if (suboit->first == "highTetWeight")
				highTetWeight = suboit->second.ToFloat();
			else if (suboit->first == "TJunctionWeight")
				TJunctionWeight = suboit->second.ToFloat();
			else if (suboit->first == "collisionWeight")
				collisionWeight = suboit->second.ToFloat();
			else if (suboit->first == "selfCollisionWeight")
				selfCollisionWeight = suboit->second.ToFloat();
			else if (suboit->first == "fixedWeight")
				fixedWeight = suboit->second.ToFloat();
			else if (suboit->first == "periferalWeight")
				periferalWeight = suboit->second.ToFloat();
			else if (suboit->first == "sutureWeight")
				sutureWeight = suboit->second.ToFloat();
			else if (suboit->first == "hookWeight")
				hookWeight = suboit->second.ToFloat();
			else if (suboit->first == "autoSutureSpacing")
				autoSutureSpacing = suboit->second.ToFloat();
			else if (suboit->first == "maxDimMegatetSubdivs")
				maxDimMegatetSubdivs = suboit->second.ToInt();
			else if (suboit->first == "nTetSizeLevels")
				nTetSizeLevels = suboit->second.ToInt();
			else
				_surgAct->sendUserMessage("Unknown tetrahedral property in scene file-", "File Error Message");
		}
		_ptp.setTetProperties(lowTetWeight, highTetWeight, TJunctionWeight, strainMin, strainMax, collisionWeight, selfCollisionWeight, fixedWeight, periferalWeight);
		_ptp.setHookSutureWeights(hookWeight, sutureWeight, 0.3f);
		_surgAct->getSutures()->setAutoSutureSpacing(autoSutureSpacing);
	}
	struct tetSubset {
		std::string objFile;
		float lowTetWeight;
		float highTetWeight;
		float strainMin;
		float strainMax;
	};
	std::list<tetSubset> tetSubsets;
	if ((oit = scnObj.find("tetrahedralSubsets")) != scnObj.end()) {
		json::Object tetSubObj = oit->second.ToObject();
		tetSubset ts;
		for (suboit = tetSubObj.begin(); suboit != tetSubObj.end(); ++suboit) {
			path = dataDirectory + suboit->first;
			ts.objFile = path;
			json::Object tetSubData = suboit->second.ToObject();
			for (auto dataoit = tetSubData.begin(); dataoit != tetSubData.end(); ++dataoit) {
				if (dataoit->first == "minStrain")
					ts.strainMin = dataoit->second.ToFloat();
				else if (dataoit->first == "maxStrain")
					ts.strainMax = dataoit->second.ToFloat();
				else if (dataoit->first == "lowTetWeight")
					ts.lowTetWeight = dataoit->second.ToFloat();
				else if (dataoit->first == "highTetWeight")
					ts.highTetWeight = dataoit->second.ToFloat();
				else;
			}
			tetSubsets.push_back(ts);
		}
	}
	else
		;
	createNewPhysicsLattice(maxDimMegatetSubdivs, nTetSizeLevels);  // now creating operable lattice on load
	_surgAct->getDeepCutPtr()->setMaterialTriangles(_mt);
	if (!_surgAct->getDeepCutPtr()->setDeepBed(_mt, deepBedFilepath.c_str(), &_vnTets)){
		_surgAct->sendUserMessage("Undermine layer .bed file could not be found-", "Error Message");
	}
	if (!tetSubsets.empty()) {
		for (auto& ts : tetSubsets)
			_tetSubsets.createSubset(&_vnTets, ts.objFile, ts.lowTetWeight, ts.highTetWeight, ts.strainMin, ts.strainMax);
	}
	_gl3w->frameScene(true);  // computes bounding spheres
	return true;
}

void bccTetScene::updateOldPhysicsLattice()
{
	_rtp.getOldPhysicsData(&_vnTets);  // must be done before any new incisions.  Worst case example < 0.02 seconds - not worth multithreading.
	_tc.addNewMultiresIncision();

#ifdef NO_PHYSICS
	_firstSpatialCoords.assign(_vnTets.nodeNumber(), Vec3f());
	_vnTets.setNodeSpatialCoordinatePointer(&_firstSpatialCoords[0]);  // for no physics debug
#else
	std::vector<uint8_t> tetSizeMult;
	tetSizeMult.reserve(_vnTets.tetNumber());
	for (int n = _vnTets.tetNumber(), i = 0; i < n; ++i) {
		uint8_t sizeBit = 1;
		auto& c = _vnTets.tetCentroid(i);
		unsigned short ored = c[0] | c[1] | c[2];
		while (true) {
			if (ored & sizeBit)
					break;
			sizeBit <<= 1;
		}
		tetSizeMult.push_back(sizeBit);
	}
	std::array<float, 3>* nodeSpatialCoords = _ptp.createBccTetStructure_multires(_vnTets.getTetNodeArray(), tetSizeMult, (float)_vnTets.getTetUnitSize());
	_vnTets.setNodeSpatialCoordinatePointer(nodeSpatialCoords);  // vector created in _ptp
#endif
	_rtp.remapNewPhysicsNodePositions(&_vnTets);  // requires node spatial coordinate array pointer. Worst case example < 0.02 seconds - not worth multithreading.
	std::vector<int> subNodes;
	std::vector<std::vector<int> > macroNodes;
	std::vector<std::vector<float> > macroBarys;
	_vnTets.getTJunctionConstraints(subNodes, macroNodes, macroBarys);
	_ptp.addInterNodeConstraints(subNodes, macroNodes, macroBarys);
	_tetSubsets.sendTetSubsets(&_vnTets, _mt, &_ptp);

	if (_forcesApplied) {  // _tetsModified not necessary as implied by calling this routine
		initPdPhysics();
		_tetsModified = true;
	}
	_physicsPaused = false;
}

void bccTetScene::createNewPhysicsLattice(int maxDimMegatetSubdivs, int nTetSizeLevels)
{
	try {
		_tetsModified = false;
		_tc.setRemapTetPhysics(&_rtp);
		_tc.createFirstMacroTets(_mt, &_vnTets, nTetSizeLevels, maxDimMegatetSubdivs);
		_surgAct->getDeepCutPtr()->setVnBccTetrahedra(&_vnTets);
		_surgAct->getDeepCutPtr()->setMaterialTriangles(_mt);

//		std::cout << "Tet number at this time is " << _vnTets.tetNumber() << "\n";

		_surgAct->getHooks()->setSpringConstant(_lowTetWeight * 1.5f);  // COURT fix me after macrotet issue resolved

#ifdef NO_PHYSICS
		_firstSpatialCoords.assign(_vnTets.nodeNumber(), Vec3f());
		_vnTets.setNodeSpatialCoordinatePointer(&_firstSpatialCoords[0]);  // for no physics debug
#else
		std::vector<uint8_t> tetSizeMult;
		tetSizeMult.reserve(_vnTets.tetNumber());
		for (int n = _vnTets.tetNumber(), i = 0; i < n; ++i) {
			// COURT may do faster with just first 2 nodes
			uint8_t sizeBit = 1;
			auto& c = _vnTets.tetCentroid(i);
			while (true) {
				if (c[0] & sizeBit || c[1] & sizeBit || c[2] & sizeBit)
					break;
				sizeBit <<= 1;
			}
			tetSizeMult.push_back(sizeBit);
		}
		std::array<float, 3>* nodeSpatialCoords = _ptp.createBccTetStructure_multires(_vnTets.getTetNodeArray(), tetSizeMult, (float)_vnTets.getTetUnitSize());
		_vnTets.setNodeSpatialCoordinatePointer(nodeSpatialCoords);  // vector created in _ptp
#endif
		_vnTets.materialCoordsToNodeSpatialVector();

		std::vector<int> subNodes;
		std::vector<std::vector<int> > macroNodes;
		std::vector<std::vector<float> > macroBarys;
		_vnTets.getTJunctionConstraints(subNodes, macroNodes, macroBarys);
		_ptp.addInterNodeConstraints(subNodes, macroNodes, macroBarys);

		_tetsModified = false;
		_physicsPaused = false;
	}  // end try block
	catch (...) {
		_surgAct->taskThreadError = true;
		_surgAct->taskThreadErrorStr = "Couldn't create the initial physics lattice. Probable model error.";
	}
}

void bccTetScene::initPdPhysics()
{  // called after each new tet lattice created
	fixPeriostealPeriferalVertices();  // doesn't throw
	if (!_tetCol.empty()) {
		_tetCol.updateFixedCollisions(_mt, &_vnTets);
		_tetCol.initSoftCollisions(_mt, &_vnTets);
	}
#ifndef NO_PHYSICS
//	if (_surgAct->getHooks()->getNumberOfHooks() < 1 && _surgAct->getSutures()->getNumberOfSutures() < 1)
//		throw(std::logic_error("Trying to initialize physics without applying any forces.\n"));
	_surgAct->getHooks()->setGroupPhysicsInit(true);  // instead of individual inits, add all hooks and sutures then initialize physics only once.
	_surgAct->getSutures()->setGroupPhysicsInit(true);
	_surgAct->getHooks()->updateHookPhysics();
	_surgAct->getSutures()->updateSuturePhysics();
	_ptp.initializePhysics();
	_surgAct->getHooks()->setGroupPhysicsInit(false);
	_surgAct->getSutures()->setGroupPhysicsInit(false);
#endif
}

void bccTetScene::generateFiberField()
{  // rule-based helical myocardial fiber field (endo +60 deg -> epi -60 deg about the long axis).
   // Computed in grid space (uniformly scaled from material space, so directions are faithful).
   // See CARDIAC.md for the physiology this approximates.
	int n = _vnTets.tetNumber();
	if (n < 1)
		return;
	std::vector<Vec3f> c(n);
	Vec3f mean(0.f, 0.f, 0.f);
	for (int i = 0; i < n; ++i) {
		const bccTetCentroid& tc = _vnTets.tetCentroid(i);
		c[i].set((float)tc[0] * 0.5f, (float)tc[1] * 0.5f, (float)tc[2] * 0.5f);  // stored centroid is doubled
		mean += c[i];
	}
	mean *= 1.0f / n;
	// covariance (xx,yy,zz,xy,xz,yz) -> dominant eigenvector = long (apex-base) axis, by power iteration
	double cov[6] = { 0,0,0,0,0,0 };
	for (int i = 0; i < n; ++i) {
		Vec3f d = c[i] - mean;
		cov[0] += (double)d.X * d.X; cov[1] += (double)d.Y * d.Y; cov[2] += (double)d.Z * d.Z;
		cov[3] += (double)d.X * d.Y; cov[4] += (double)d.X * d.Z; cov[5] += (double)d.Y * d.Z;
	}
	Vec3f L(0.f, 0.f, 1.f);
	for (int it = 0; it < 64; ++it) {
		Vec3f y((float)(cov[0] * L.X + cov[3] * L.Y + cov[4] * L.Z),
				(float)(cov[3] * L.X + cov[1] * L.Y + cov[5] * L.Z),
				(float)(cov[4] * L.X + cov[5] * L.Y + cov[2] * L.Z));
		if (y.length() < 1e-12f) break;
		y.normalize(); L = y;
	}
	// transmural depth normaliser: max radial distance from the long axis
	std::vector<float> radial(n); std::vector<Vec3f> circ(n);
	float maxR = 1e-6f;
	for (int i = 0; i < n; ++i) {
		Vec3f d = c[i] - mean;
		float along = d * L;                 // dot
		Vec3f r = d - L * along;             // radial component
		float rl = r.length();
		radial[i] = rl;
		Vec3f rhat;
		if (rl > 1e-6f) rhat = r / rl;
		else { Vec3f t = (fabs(L.X) < 0.9f) ? Vec3f(1.f, 0.f, 0.f) : Vec3f(0.f, 1.f, 0.f); rhat = t - L * (t * L); rhat.normalize(); }
		circ[i] = L ^ rhat;                  // circumferential = L x rhat
		if (rl > maxR) maxR = rl;
	}
	std::vector<float> fx(n), fy(n), fz(n);
	const float deg2rad = 3.14159265f / 180.0f;
	for (int i = 0; i < n; ++i) {
		float t = radial[i] / maxR; if (t > 1.f) t = 1.f;      // 0 near axis (endo-ish) -> 1 outer (epi)
		float alpha = (1.0f - 2.0f * t) * 60.0f * deg2rad;     // +60 deg endo -> -60 deg epi
		Vec3f f = circ[i] * cosf(alpha) + L * sinf(alpha);
		if (f.length() < 1e-9f) f = Vec3f(1.f, 0.f, 0.f);
		f.normalize();
		fx[i] = f.X; fy[i] = f.Y; fz[i] = f.Z;
	}
	_ptp.setFiberField(fx, fy, fz);
}

void bccTetScene::startBeating()
{  // stands the solver up if no hook/suture has yet; then activation oscillates in updatePhysics()
	if (_vnTets.empty())
		return;
	generateFiberField();  // must precede the (re)init so the fiber lands in the kernel's blocked arrays
	if (!_forcesApplied)
		_forcesApplied = true;
	initPdPhysics();       // full init - scatters the fiber field and (re)builds the solver
	_tetsModified = true;
	_beatFrame = 0;
	_beating = true;
}

void bccTetScene::updatePhysics()
{
	if (_vnTets.empty())
		return;
//	if (!_tetsModified && _forcesApplied) {  // don't do this here anymore
//		_tetsModified = true;
//		initPdPhysics();
//	}

#ifndef NO_PHYSICS
	if (_tetsModified || _forcesApplied) {
		if (_beating) {  // runs on the physics task thread, before the solve - no race with it
			++_beatFrame;
			// Asymmetric cardiac twitch (CARDIAC.md): fast systolic rise to peak contraction,
			// slower diastolic relaxation, then diastasis at rest. Fractions of one cycle.
			float phase = float(_beatFrame % _beatPeriod) / _beatPeriod;
			const float riseFrac = 0.18f;   // ~150ms / 800ms rise to peak
			const float fallFrac = 0.34f;   // relaxation slower than contraction
			float contract;                 // 0 = relaxed, 1 = peak
			if (phase < riseFrac) { float s = phase / riseFrac; contract = s * s * (3.f - 2.f * s); }
			else if (phase < riseFrac + fallFrac) { float s = (phase - riseFrac) / fallFrac; contract = 1.f - s * s * (3.f - 2.f * s); }
			else contract = 0.f;            // diastole / filling
			float lambda = 1.f - _beatAmplitude * contract;
			_ptp.setUniformActivation(lambda);
		}
		_tetCol.findSoftCollisionPairs();
		_ptp.solve();
	}
#endif

#ifdef WRITE_FOR_RENDER
	RenderHelper<float>::writeMesh(*_mt);
	RenderHelper<float>::frame++;
#endif
}
 
void bccTetScene::setVisability(char surface, char physics)
{  // 0=off, 1=on, 2=don't change
	if (surface < 1)
		_surgAct->getSurgGraphics()->getSceneNode()->visible = false;
	if (surface == 1)
		_surgAct->getSurgGraphics()->getSceneNode()->visible = true;
	if (physics < 1)
		_gl3w->getLines()->setLinesVisible(false);
	if (physics == 1) {
		if (!_gl3w->getLines()->getSceneNode()) {
			createTetLatticeDrawing();
			drawTetLattice();
		}
		else
			_gl3w->getLines()->setLinesVisible(true);
	}
}

void bccTetScene::updateSurfaceDraw()
{
	int nv;
	auto pArr = _mt->getPositionArrayPtr();
	nv = pArr->size();
	for (int i = 0; i < nv; ++i) {
		if (_vnTets.getVertexTetrahedron(i) > -1)  // an excision may have occurred leaving an empty vertex
			_vnTets.getBarycentricTetPosition(_vnTets.getVertexTetrahedron(i), *(_vnTets.getVertexWeight(i)), pArr->at(i));
	}
	_surgAct->getSurgGraphics()->updatePositionsNormalsTangents();
	if (_gl3w->getLines()->linesVisible())
		drawTetLattice();
}

 void bccTetScene::fixPeriostealPeriferalVertices()
{  // this routine should be done only once after original lattice constructed
	struct anchorPoint {
		bool isPeriferal;
		std::array<float, 3> baryWeight, pos;
	}ap;
	std::unordered_map<int, anchorPoint> fixPoints;  // key is tet index. At present only oneper tet.
	// fixed nodes take precedence over periferal nodes
	// Fix all nodes surrounding a periosteal or periferalvertex.
	auto enterFixPoint = [&](int vId, bool periferal) {
		const Vec3f* vp = _vnTets.getVertexWeight(vId);
		ap.baryWeight[0] = vp->X;
		ap.baryWeight[1] = vp->Y;
		ap.baryWeight[2] = vp->Z;
		_vnTets.vertexMaterialCoordinate(vId, ap.pos);
		ap.isPeriferal = periferal;
		fixPoints.insert(std::make_pair(_vnTets.getVertexTetrahedron(vId), ap));
	};
	for (int n = _mt->numberOfTriangles(), i = 0; i < n; ++i) {
		if (_mt->triangleMaterial(i) == 7) {  // periosteal triangle
			for (int k = 0; k < 3; ++k) {
				int vIdx = _mt->triangleVertices(i)[k];
				enterFixPoint(vIdx, false);
			}
		}
		if (_mt->triangleMaterial(i) == 1) {  // periosteal triangle
			for (int k = 0; k < 3; ++k) {
				int vIdx = _mt->triangleVertices(i)[k];
				enterFixPoint(vIdx, true);
			}
		}
	}
	size_t n = fixPoints.size();
	std::vector<int> fixedTets, peripheralTets;
	fixedTets.reserve(n);
	peripheralTets.reserve(n);
	std::vector<std::array<float, 3> > fixedWeights, peripheralWeights, fixedPos, peripheralPos;
	fixedWeights.reserve(n);
	peripheralWeights.reserve(n);
	fixedPos.reserve(n);
	peripheralPos.reserve(n);
	for (auto &fp : fixPoints) {
		if (fp.second.isPeriferal) {
			peripheralTets.push_back(fp.first);
			peripheralWeights.push_back(fp.second.baryWeight);
			peripheralPos.push_back(fp.second.pos);
		}
		else {
			fixedTets.push_back(fp.first);
			fixedWeights.push_back(fp.second.baryWeight);
			fixedPos.push_back(fp.second.pos);
		}
	}
#ifndef NO_PHYSICS
	_ptp.setFixedVertices(fixedTets, fixedWeights, fixedPos, peripheralTets, peripheralWeights, peripheralPos);
#endif
}

void bccTetScene::createTetLatticeDrawing()
{
	if (_nodeGraphicsPositions.size() == _vnTets.nodeNumber())
		return;
	_nodeGraphicsPositions.clear();
	_nodeGraphicsPositions.assign(_vnTets.nodeNumber() << 2, 1.0f);
	GLfloat *ngp = &_nodeGraphicsPositions[0];
	for (int n = _vnTets.nodeNumber(), i = 0; i < n; ++i){
		const float *fp = _vnTets.nodeSpatialCoordinatePtr(i);
		*(ngp++) = fp[0];
		*(ngp++) = fp[1];
		*(ngp++) = fp[2];
		++ngp;
	}
	std::set<std::pair<int, int> > segs;
	for (int n = _vnTets.tetNumber(), i=0; i<n; ++i){  // numberOfMegaTets()
		std::pair<int, int> ll;
		const int* tetNodes = _vnTets.tetNodes(i);
		for (int j = 0; j < 3; ++j){
			for (int k = j+1; k < 4; ++k){
				ll.first = std::min(tetNodes[j], tetNodes[k]);
				ll.second = std::max(tetNodes[j], tetNodes[k]);
				segs.insert(ll);
			}
		}
	}
	std::vector<GLuint> lines;
	lines.reserve(segs.size() * 3);
	for (auto &s : segs){
		lines.push_back(s.first);
		lines.push_back(s.second);
		lines.push_back(0xffffffff);
	}
	_gl3w->getLines()->setGl3wGraphics(_gl3w);
	float white2[4] = {1.0f, 1.0f, 1.0f, 1.0f};
	_gl3w->getLines()->addLines(_nodeGraphicsPositions, lines);
	_gl3w->getLines()->getSceneNode()->setColor(white2);
}

void bccTetScene::eraseTetLattice()
{
	_nodeGraphicsPositions.clear();
	_gl3w->getLines()->clear();
	_gl3w->getLines()->getSceneNode()->visible = false;
}

void bccTetScene::drawTetLattice()
{
	if (_nodeGraphicsPositions.empty())
		return;
	GLfloat *ngp = &_nodeGraphicsPositions[0];
	for (int n = _vnTets.nodeNumber(), i = 0; i < n; ++i){
		const float *fp = _vnTets.nodeSpatialCoordinatePtr(i);
		*(ngp++) = fp[0];
		*(ngp++) = fp[1];
		*(ngp++) = fp[2];
		++ngp;
	}
	_gl3w->getLines()->updatePoints(_nodeGraphicsPositions);
}

bccTetScene::bccTetScene() : _physicsPaused(false), _forcesApplied(false), _tetsModified(false)
{
	_tetCol.setPdTetPhysics(&_ptp); // Qisi:set ptp for tetCol so things of ptp are accessible inside of tetCol
}


bccTetScene::~bccTetScene()
{
}
