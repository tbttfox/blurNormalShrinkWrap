#include <iostream>
#include <numbers>
#include <vector>
#include <algorithm>
#include <maya/MItGeometry.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MGlobal.h>
#include <maya/MFnMesh.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MMeshIntersector.h>
#include <maya/MFnMeshData.h>
#include <maya/MFloatMatrix.h>
#include <maya/MBoundingBox.h>
#include <maya/MPointArray.h>
#include <maya/MAngle.h>
#include <maya/MFnPointArrayData.h>
#include <maya/MFnIntArrayData.h>

#include "blurNormalShrinkWrap.h"
#include "cpom_types.h"

#define CHECKSTAT(stat, msg) if ( !stat ) {  MGlobal::displayError(msg); return stat; }

MTypeId NormalShrinkWrapDeformer::id(0x00122714);

MObject NormalShrinkWrapDeformer::aBvhComputed;

MObject NormalShrinkWrapDeformer::aBaryIndices;
MObject NormalShrinkWrapDeformer::aBaryValues;

MObject NormalShrinkWrapDeformer::aAngleTolerance;

MObject NormalShrinkWrapDeformer::aTargetStaticMesh;
MObject NormalShrinkWrapDeformer::aTargetStaticInvWorld;

MObject NormalShrinkWrapDeformer::aSourceStaticMesh;
MObject NormalShrinkWrapDeformer::aSourceStaticInvWorld;

MObject NormalShrinkWrapDeformer::aTargetMesh;
MObject NormalShrinkWrapDeformer::aTargetInvWorld;


void* NormalShrinkWrapDeformer::creator() { return new NormalShrinkWrapDeformer(); }

MStatus NormalShrinkWrapDeformer::initialize() {
    MStatus status;
    MFnNumericAttribute nAttr;
    MFnEnumAttribute eAttr;
    MFnTypedAttribute tAttr;
    MFnMatrixAttribute mAttr;
    MFnUnitAttribute uAttr;

    aBvhComputed = nAttr.create("bvhComputed", "bc", MFnNumericData::kBoolean, false);
    CHECKSTAT(status, "Error creating aBvhComputed");
    nAttr.setHidden(true);
    status = addAttribute(aBvhComputed);
    CHECKSTAT(status, "Error adding aBvhComputed");

    aBaryIndices = tAttr.create("baryIndices", "bi", MFnData::kIntArray, &status);
    CHECKSTAT(status, "Error creating aBaryIndices");
    uAttr.setKeyable(false);
    status = addAttribute(aBaryIndices);
    CHECKSTAT(status, "Error adding aBaryIndices");
    aBaryValues = tAttr.create("baryValues", "bv", MFnData::kPointArray, &status);
    CHECKSTAT(status, "Error creating aBaryValues");
    uAttr.setKeyable(false);
    status = addAttribute(aBaryValues);
    CHECKSTAT(status, "Error adding aBaryValues");

    aAngleTolerance = uAttr.create("angleTolerance", "at", MFnUnitAttribute::kAngle, std::numbers::pi / 3.0, &status);
    CHECKSTAT(status, "Error creating angleTolerance");
    uAttr.setMin(std::numbers::pi / 6000.0);
    uAttr.setMax(std::numbers::pi);
    uAttr.setKeyable(true);
    uAttr.setChannelBox(true);
    status = addAttribute(aAngleTolerance);
    CHECKSTAT(status, "Error adding angleTolerance");

    aTargetStaticMesh = tAttr.create("targetStatic", "ts", MFnData::kMesh, MObject::kNullObj, &status);
    CHECKSTAT(status, "Error creating targetStatic");
    status = addAttribute(aTargetStaticMesh);
    CHECKSTAT(status, "Error adding targetStatic");
    aTargetStaticInvWorld = mAttr.create("targetStaticInvWorld", "tsiw", MFnMatrixAttribute::kDouble, &status);
    CHECKSTAT(status, "Error creating targetStaticInvWorld");
    status = addAttribute(aTargetStaticInvWorld);
    CHECKSTAT(status, "Error adding targetStaticInvWorld");

    aSourceStaticMesh = tAttr.create("sourceStatic", "ss", MFnData::kMesh, MObject::kNullObj, &status);
    CHECKSTAT(status, "Error creating sourceStatic");
    status = addAttribute(aSourceStaticMesh);
    CHECKSTAT(status, "Error adding sourceStatic");
    aSourceStaticInvWorld = mAttr.create("sourceStaticInvWorld", "ssiw", MFnMatrixAttribute::kDouble, &status);
    CHECKSTAT(status, "Error creating sourceStaticInvWorld");
    status = addAttribute(aSourceStaticInvWorld);
    CHECKSTAT(status, "Error adding sourceStaticInvWorld");

    aTargetMesh = tAttr.create("target", "t", MFnData::kMesh, MObject::kNullObj, &status);
    CHECKSTAT(status, "Error creating target");
    status = addAttribute(aTargetMesh);
    CHECKSTAT(status, "Error adding target");
    aTargetInvWorld = mAttr.create("targetInvWorld", "tiw", MFnMatrixAttribute::kDouble, &status);
    CHECKSTAT(status, "Error creating targetInvWorld");
    status = addAttribute(aTargetInvWorld);
    CHECKSTAT(status, "Error adding targetInvWorld");
    

    std::vector<MObject*> masters, clients;

    masters.push_back(&aAngleTolerance);
    masters.push_back(&aBvhComputed);
    masters.push_back(&aSourceStaticInvWorld);
    masters.push_back(&aSourceStaticMesh);
    masters.push_back(&aTargetStaticInvWorld);
    masters.push_back(&aTargetStaticMesh);

    clients.push_back(&aBaryIndices);
    clients.push_back(&aBaryValues);
    clients.push_back(&outputGeom);

    for (auto master: masters){
        for (auto client: clients){
            attributeAffects(*master, *client);
        }
    }

    attributeAffects(aBaryIndices, outputGeom);
    attributeAffects(aBaryValues, outputGeom);
    attributeAffects(aTargetStaticMesh, aBvhComputed);
    attributeAffects(aTargetMesh, outputGeom);
    attributeAffects(aTargetInvWorld, outputGeom);

    return MStatus::kSuccess;
}


MStatus NormalShrinkWrapDeformer::compute(const MPlug& plug, MDataBlock& block) {

    MStatus stat;
    if (plug == aBvhComputed) {
        MObject targetStatic = block.inputValue(aTargetStaticMesh, &stat).asMesh();
        if (targetStatic.isNull()) return MStatus::kInvalidParameter;
        MFnMesh fnTargetStatic(targetStatic);
        const auto fptr = fnTargetStatic.getRawPoints(&stat);
        if (fptr == NULL) {
            return MStatus::kInvalidParameter;
        }

        bboxes.clear();
        centers.clear();
        normals.clear();
        tris.clear();
        barys.clear();
        baryIdxs.clear();
        triVerts.clear();

        MIntArray triCounts;
        fnTargetStatic.getTriangles(triCounts, triVerts);
        for (UINT i = 0; i < triVerts.length(); i += 3) {
            int tv0 = triVerts[i + 0];
            double v00 = fptr[(tv0 * 3) + 0];
            double v01 = fptr[(tv0 * 3) + 1];
            double v02 = fptr[(tv0 * 3) + 2];
            Vec3 v0(v00, v01, v02);

            int tv1 = triVerts[i + 1];
            double v10 = fptr[(tv1 * 3) + 0];
            double v11 = fptr[(tv1 * 3) + 1];
            double v12 = fptr[(tv1 * 3) + 2];
            Vec3 v1(v10, v11, v12);

            int tv2 = triVerts[i + 2];
            double v20 = fptr[(tv2 * 3) + 0];
            double v21 = fptr[(tv2 * 3) + 1];
            double v22 = fptr[(tv2 * 3) + 2];
            Vec3 v2(v20, v21, v22);

            // notice 0 2 1.  This reverses the direction of the normal
            // Also the order of the barycenters
            tris.emplace_back(v0, v2, v1);
        }
        bvh = build_bvh(tris, bboxes, centers, normals);

        MDataHandle compH = block.outputValue(aBvhComputed, &stat);
        compH.setBool(true);
        block.setClean(aBvhComputed);
    }
    else if (plug == aBaryIndices || plug == aBaryValues) {
        // force evaluation of the BVH
        MDataHandle compH = block.inputValue(aBvhComputed, &stat);
        bool bvhComputed = compH.asBool();
        if (!bvhComputed) return stat;

        MDataHandle sourceStaticH = block.inputValue(aSourceStaticMesh, &stat);
        MObject sourceStatic = sourceStaticH.asMesh();
        if (sourceStatic.isNull()) return MStatus::kInvalidParameter;
        MFnMesh fnSourceStatic(sourceStatic);
        const auto fptr = fnSourceStatic.getRawPoints(&stat);
        if (fptr == NULL) {
            return MStatus::kInvalidParameter;
        }

        MMatrix tWInv = block.inputValue(aTargetStaticInvWorld, &stat).asMatrix();
        MMatrix sWInv = block.inputValue(aSourceStaticInvWorld, &stat).asMatrix();
        MMatrix sWMat = sWInv.inverse();
        MMatrix tranMatInv = sWMat * tWInv;
        MMatrix tranMat = tranMatInv.inverse();

        MAngle angleTolA = block.inputValue(aAngleTolerance, &stat).asAngle();
        double angleTol = angleTolA.asRadians();


        int numVerts = fnSourceStatic.numVertices();
        MIntArray baryIdxs;
        MPointArray baryVals;
        baryIdxs.setLength(numVerts);
        baryVals.setLength(numVerts);

        MFloatVectorArray vnorms;
        MPointArray qpts;
        fnSourceStatic.getVertexNormals(false, vnorms);
        fnSourceStatic.getPoints(qpts);

        for (UINT i = 0; i < qpts.length(); ++i) {
            
            MPoint pt = qpts[i];
            MPoint tpt = pt * tranMatInv; // target space point
            MVector n = vnorms[i];

            Vec3 tv(tpt.x, tpt.y, tpt.z);
            Vec3 tn(n.x, n.y, n.z);

            auto [cpom, baryIdx, bary] = get_closest(bvh,
                tris,
                bboxes,
                centers,
                normals,
                tv,
                tn,
                angleTol
            );

            // Notice the 0 2 1.  This fixes the flipped normal thing
            // from the triangles
            baryVals[i] = MPoint(bary[0], bary[2], bary[1]);
            baryIdxs[i] = baryIdx;
        }

        MDataHandle bvDataH = block.outputValue(aBaryValues, &stat);
        MDataHandle biDataH = block.outputValue(aBaryIndices, &stat);

        bvDataH.set(MFnPointArrayData().create(baryVals));
        biDataH.set(MFnIntArrayData().create(baryIdxs));

        block.setClean(aBaryValues);
        block.setClean(aBaryIndices);
    }
    else if (plug == outputGeom) {
        return MPxDeformerNode::compute(plug, block);
    }

    return stat;
}


MStatus NormalShrinkWrapDeformer::deform(
        MDataBlock& block, MItGeometry& iter,
        const MMatrix& dMat, unsigned int multiIndex
        ) {
    MStatus stat;

    float env = block.inputValue(envelope, &stat).asFloat();
    if (env == 0.0f) return stat;

    MObject target = block.inputValue(aTargetMesh, &stat).asMesh();
    if (target.isNull()) return MStatus::kInvalidParameter;
    MFnMesh fnTarget(target);

    MMatrix tMatInv = block.inputValue(aTargetInvWorld, &stat).asMatrix();
    MMatrix tMat = tMatInv.inverse();

    MMatrix dMatInv = dMat.inverse();

    // Force the barys to compute if they haven't
    MDataHandle bvDataH = block.inputValue(aBaryValues, &stat);
    MFnPointArrayData bvDataA(bvDataH.data());
    MPointArray baryValues = bvDataA.array();

    MDataHandle biDataH = block.inputValue(aBaryIndices, &stat);
    MFnIntArrayData biDataA(biDataH.data());
    MIntArray baryIdxs = biDataA.array();

    for (; !iter.isDone(); iter.next()) {
        int i = iter.index();
        if (i >= baryIdxs.length()) continue;

        float w = weightValue(block, multiIndex, i);
        if (w == 0.0f) continue;

        int qIdx = baryIdxs[i];
        if (qIdx < 0) continue;

        MPoint bary = baryValues[i];

        MPoint A, B, C;
        fnTarget.getPoint(triVerts[3 * qIdx + 0], A);
        fnTarget.getPoint(triVerts[3 * qIdx + 1], B);
        fnTarget.getPoint(triVerts[3 * qIdx + 2], C);
        MPoint P = (MVector)A * bary[0] + (MVector)B * bary[1] + (MVector)C * bary[2];
        P *= tMat;
        P *= dMatInv;

        MPoint Po = iter.position();
        iter.setPosition(((P - Po) * (env * w)) + Po);

    }
    return stat;
}
