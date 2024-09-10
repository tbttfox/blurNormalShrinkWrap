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

#include "cpom_types.h"
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

    virtual MStatus compute(const MPlug& plug, MDataBlock& block);

    static MTypeId id;
    
    static MObject aBvhComputed;

    static MObject aBaryIndices;
    static MObject aBaryValues;

    static MObject aAngleTolerance;

    static MObject aTargetStaticMesh;
    static MObject aTargetStaticInvWorld;

    static MObject aSourceStaticMesh;
    static MObject aSourceStaticInvWorld;

    static MObject aTargetMesh;
    static MObject aTargetInvWorld;

private:

    Bvh bvh;
    std::vector<Tri> tris;
    std::vector<BBox> bboxes;
    std::vector<Vec3> centers;
    std::vector<Vec3> normals;
    std::vector<Vec3> barys;
    std::vector<Index> baryIdxs;
    MIntArray triVerts;
};
