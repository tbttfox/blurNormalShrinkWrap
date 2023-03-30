#include "blurNormalShrinkWrap.h"

#include <vector>
#include <algorithm>
#include <maya/MItGeometry.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MGlobal.h>
#include <maya/MFnMesh.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MMeshIntersector.h>
#include <maya/MFnMeshData.h>
#include <maya/MFloatMatrix.h>
#include <maya/MBoundingBox.h>
#include <maya/MPointArray.h>

#define CHECKSTAT(stat, msg) if ( !stat ) {  MGlobal::displayError(msg); return stat; }

#define PA_POSX      0
#define PA_POSY      1
#define PA_POSZ      2
#define PA_CENTER    3
#define PA_NORMAL    4
#define PA_CLOSEST   5
#define PA_CLOSESTSM 6
#define PA_VECTOR    7
#define PA_TOWARDS   8

#define VS_OBJECT 0
#define VS_TARGET 1
#define VS_WORLD  2

MTypeId NormalShrinkWrapDeformer::id(0x001226FF);
MObject NormalShrinkWrapDeformer::aMaxParam;
MObject NormalShrinkWrapDeformer::aReverse;
MObject NormalShrinkWrapDeformer::aBidirectional;
MObject NormalShrinkWrapDeformer::aClosestIfNone;
MObject NormalShrinkWrapDeformer::aProjectionType;
MObject NormalShrinkWrapDeformer::aProjectionVector;
MObject NormalShrinkWrapDeformer::aVectorSpace;
MObject NormalShrinkWrapDeformer::aTargetMesh;
MObject NormalShrinkWrapDeformer::aTargetInvWorld;

void* NormalShrinkWrapDeformer::creator() { return new NormalShrinkWrapDeformer(); }
MStatus NormalShrinkWrapDeformer::initialize() {
    MStatus status;
    MFnNumericAttribute nAttr;
    MFnEnumAttribute eAttr;
    MFnTypedAttribute tAttr;
    MFnMatrixAttribute mAttr;

    aMaxParam = nAttr.create("maxParam", "mx", MFnNumericData::kFloat, 99999.0f, &status);
    nAttr.setKeyable(true);
    status = addAttribute(aMaxParam);
    status = attributeAffects(aMaxParam, outputGeom);

    aReverse = nAttr.create("reverse", "rv", MFnNumericData::kBoolean, false, &status);
    nAttr.setKeyable(false);
    eAttr.setChannelBox(true);
    status = addAttribute(aReverse);
    status = attributeAffects(aReverse, outputGeom);

    aBidirectional = nAttr.create("biDirectional", "bi", MFnNumericData::kBoolean, false, &status);
    nAttr.setKeyable(false);
    eAttr.setChannelBox(true);
    status = addAttribute(aBidirectional);
    status = attributeAffects(aBidirectional, outputGeom);

    aClosestIfNone = nAttr.create("closestIfNone", "cn", MFnNumericData::kBoolean, false, &status);
    nAttr.setKeyable(false);
    eAttr.setChannelBox(true);
    status = addAttribute(aClosestIfNone);
    status = attributeAffects(aClosestIfNone, outputGeom);

    aProjectionType = eAttr.create("projectionType", "pt", PA_CENTER, &status);
    eAttr.setKeyable(false);
    eAttr.setChannelBox(true);
    eAttr.addField("X", PA_POSX);
    eAttr.addField("Y", PA_POSY);
    eAttr.addField("Z", PA_POSZ);
    eAttr.addField("Center", PA_CENTER);
    eAttr.addField("Normal", PA_NORMAL);
    eAttr.addField("Closest", PA_CLOSEST);
    eAttr.addField("ClosestSmooth", PA_CLOSESTSM);
    eAttr.addField("Vector", PA_VECTOR);
    eAttr.addField("Towards", PA_TOWARDS);
    status = addAttribute(aProjectionType);
    status = attributeAffects(aProjectionType, outputGeom);

    aVectorSpace = eAttr.create("vectorSpace", "vs", VS_TARGET, &status);
    eAttr.setKeyable(false);
    eAttr.setChannelBox(true);
    eAttr.addField("Target", VS_TARGET);
    eAttr.addField("Object", VS_OBJECT);
    eAttr.addField("World", VS_WORLD);
    status = addAttribute(aVectorSpace);
    status = attributeAffects(aVectorSpace, outputGeom);

    aProjectionVector = nAttr.create("projectionVector", "pv", MFnNumericData::k3Double);
    status = addAttribute(aProjectionVector);
    status = attributeAffects(aProjectionVector, outputGeom);

    aTargetMesh = tAttr.create("target", "t", MFnData::kMesh);
    addAttribute(aTargetMesh);
    attributeAffects(aTargetMesh, outputGeom);

    aTargetInvWorld = mAttr.create("targetInvWorld", "tiw");
    addAttribute(aTargetInvWorld);
    attributeAffects(aTargetInvWorld, outputGeom);


    return MStatus::kSuccess;
}


MStatus NormalShrinkWrapDeformer::deform(
        MDataBlock& block, MItGeometry& iter,
        const MMatrix& m, unsigned int multiIndex
        ) {
    MStatus stat;

    float env = block.inputValue(envelope, &stat).asFloat();
    if (env == 0.0f) return stat;

    MObject target = block.inputValue(aTargetMesh, &stat).asMesh();
    if (target.isNull()) return MStatus::kInvalidParameter;
    MFnMesh fnTarget(target);

    float maxParam = block.inputValue(aMaxParam, &stat).asFloat();
    bool reverse = block.inputValue(aReverse, &stat).asBool();
    bool biDir = block.inputValue(aBidirectional, &stat).asBool();
    bool cin = block.inputValue(aClosestIfNone, &stat).asBool();
    short projType = block.inputValue(aProjectionType, &stat).asShort();
    short vSpace = block.inputValue(aVectorSpace, &stat).asShort();
    MMatrix tWInv = block.inputValue(aTargetInvWorld, &stat).asMatrix();
    double* projVector = block.inputValue(aProjectionVector, &stat).asDouble3();

    MMatrix tranMatInv = m * tWInv;
    MMatrix tranMat = tranMatInv.inverse();

    MVector vec;
    MPoint ctr;
    bool perVert = false;
    bool doClosest = false;
    switch (projType){
        case PA_POSX: vec = MFloatVector::xAxis; break;
        case PA_POSY: vec = MFloatVector::yAxis; break;
        case PA_POSZ: vec = MFloatVector::zAxis; break;
                      vec = (vec * tranMatInv).normal();
                      break;
        case PA_VECTOR:
                      vec.x = projVector[0];
                      vec.y = projVector[1];
                      vec.z = projVector[2];
                      if (vSpace == VS_OBJECT) vec *= tranMatInv;
                      else if (vSpace == VS_WORLD) vec *= tWInv;
                      break;

        case PA_CENTER:
                      vec = MFnDagNode(target).boundingBox().center();
                      perVert = true; break;
        case PA_TOWARDS:
                      // Because maya treats points and vectors differently
                      // Gotta make sure to use an MPoint
                      ctr.x = projVector[0];
                      ctr.y = projVector[1];
                      ctr.z = projVector[2];
                      if (vSpace == VS_OBJECT) ctr *= tranMatInv;
                      else if (vSpace == VS_WORLD) ctr *= tWInv;
                      vec = ctr;
                      perVert = true; break;
        case PA_NORMAL:
                      perVert = true; break;

        case PA_CLOSEST:
        case PA_CLOSESTSM:
                      doClosest = true; break;
    }

    MMeshIntersector octree;
    MObject smoothMeshPar, smoothMesh;
    if (doClosest || cin) {
        if (projType == PA_CLOSESTSM){
            MFnMeshData smoothMeshParFn;
            MMeshSmoothOptions smoothOpt;
            smoothMeshPar = smoothMeshParFn.create();
            smoothOpt.setDivisions(2);
            smoothOpt.setKeepBorderEdge(true);
            smoothOpt.setSubdivisionType(MMeshSmoothOptions::kCatmullClark);
            smoothMesh = fnTarget.generateSmoothMesh(smoothMeshPar, &smoothOpt);
            octree.create(smoothMesh);
        }
        else {
            octree.create(target);
        }
    }

    MMeshIsectAccelParams mmAccelParams = fnTarget.autoUniformGridParams();

    for (size_t i=0; !iter.isDone(); iter.next(), i++) {
        float w = weightValue(block, multiIndex, iter.index());
        if (w == 0.0f) continue;
        MPoint pt = iter.position();
        MPoint tpt = pt * tranMatInv; // target space point

        bool sect = false;
        MVector pvec(vec);

        if (perVert){
            if (projType == PA_NORMAL){
                pvec = (iter.normal() * -1) * tranMatInv;
            }
            else { pvec = pvec - tpt; }
            pvec.normalize();
        }
        if (reverse) pvec *= -1;

        if (!doClosest){
            MFloatPoint hit;
            sect = fnTarget.closestIntersection(
                    tpt, pvec, nullptr, nullptr, true, MSpace::kObject, maxParam, false, &mmAccelParams,
                    hit, nullptr, nullptr, nullptr, nullptr, nullptr, 1e-6f, &stat 
                    );
            if (!sect && biDir) {
                sect = fnTarget.closestIntersection(
                        tpt, pvec*-1, nullptr, nullptr, true, MSpace::kObject, maxParam, false, &mmAccelParams,
                        hit, nullptr, nullptr, nullptr, nullptr, nullptr, 1e-6f, &stat 
                        );
            }

            if (sect){
                iter.setPosition(env * w * ((MPoint(hit) * tranMat) - pt) + pt);
                continue;
            }
        }

        if (doClosest || (!sect && cin)){
            MPointOnMesh res;
            octree.getClosestPoint(tpt, res, maxParam);
            iter.setPosition(env * w * ((MPoint(res.getPoint()) * tranMat) - pt) + pt);
        }
    }
    return stat;
}
