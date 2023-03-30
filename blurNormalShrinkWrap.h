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

#define DEFORMER_NAME "blurNormalShrinkWrap"


class NormalShrinkWrapDeformer : public MPxDeformerNode {
public:
	NormalShrinkWrapDeformer() {};
	virtual ~NormalShrinkWrapDeformer() {};

    static void* creator();
    static MStatus initialize();
    virtual MStatus deform(MDataBlock& block, MItGeometry& iter, const MMatrix& mat, unsigned int multiIndex);
    static MTypeId id;
	
	static MObject aMaxParam;
	static MObject aReverse;
	static MObject aBidirectional;
	static MObject aClosestIfNone;
	static MObject aProjectionType;
	static MObject aProjectionVector;
	static MObject aVectorSpace;
	static MObject aTargetMesh;
	static MObject aTargetInvWorld;

};

