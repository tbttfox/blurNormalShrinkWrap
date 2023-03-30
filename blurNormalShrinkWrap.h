#pragma once
#include <maya/MPxDeformerNode.h>
#include <maya/MTypeId.h>
#include <maya/MPlug.h>
#include <maya/MString.h>
#include <maya/MDataBlock.h>
#include <maya/MMatrix.h>
#include <maya/MDagModifier.h>
#include <maya/MPxGPUDeformer.h>
#include <maya/MGPUDeformerRegistry.h>
#include <maya/MOpenCLInfo.h>
#include <maya/MFnNumericAttribute.h>
#include <vector>

#include "xxhash.h"
#include "cpom_normal.h"

#define DEFORMER_NAME "blurNormalShrinkWrap"


class NormalShrinkWrapDeformer : public MPxDeformerNode {
public:
	NormalShrinkWrapDeformer() {};
	virtual ~NormalShrinkWrapDeformer() {};

    static void* creator();
    static MStatus initialize();
	//virtual MStatus compute(const MPlug& plug, MDataBlock& block);
    virtual MStatus deform(MDataBlock& block, MItGeometry& iter, const MMatrix& mat, unsigned int multiIndex);
    static MTypeId id;
	
	static MObject aMaxParam;
	static MObject aAngleTolerance;
	static MObject aTargetMesh;
	static MObject aTargetInvWorld;
private:

	XXH64_hash_t vertHash = 0;

	Bvh bvh;
    std::vector<Tri> tris;
    std::vector<BBox> bboxes;
    std::vector<Vec3> centers;
    std::vector<Vec3> normals;

};

