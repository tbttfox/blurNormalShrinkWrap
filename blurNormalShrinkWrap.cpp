#include "blurNormalShrinkWrap.h"

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


#define CHECKSTAT(stat, msg) if ( !stat ) {  MGlobal::displayError(msg); return stat; }

MTypeId NormalShrinkWrapDeformer::id(0x00122714);
MObject NormalShrinkWrapDeformer::aMaxParam;
MObject NormalShrinkWrapDeformer::aAngleTolerance;
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

    aMaxParam = nAttr.create("maxParam", "mx", MFnNumericData::kFloat, 99999.0f, &status);
    CHECKSTAT(status, "Error creating maxParam");
    nAttr.setKeyable(true);
    status = addAttribute(aMaxParam);
    CHECKSTAT(status, "Error adding maxParam");
    attributeAffects(aMaxParam, outputGeom);

    aAngleTolerance = uAttr.create("angleTolerance", "at", MFnUnitAttribute::kAngle, std::numbers::pi / 20.0, &status);
    CHECKSTAT(status, "Error creating angleTolerance");
    uAttr.setKeyable(false);
    status = addAttribute(aAngleTolerance);
    CHECKSTAT(status, "Error adding angleTolerance");
    attributeAffects(aAngleTolerance, outputGeom);

    aTargetMesh = tAttr.create("target", "t", MFnData::kMesh, MObject::kNullObj, &status);
    CHECKSTAT(status, "Error creating target");
    status = addAttribute(aTargetMesh);
    CHECKSTAT(status, "Error adding target");
    attributeAffects(aTargetMesh, outputGeom);

    aTargetInvWorld = mAttr.create("targetInvWorld", "tiw", MFnMatrixAttribute::kDouble, &status);
    CHECKSTAT(status, "Error creating targetInvWorld");
    status = addAttribute(aTargetInvWorld);
    CHECKSTAT(status, "Error adding targetInvWorld");
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

    // Hash the vertex positions so we can tell if they changed
    const auto dptr = fnTarget.getRawDoublePoints(&stat);
    XXH64_hash_t vertHashChk = XXH3_64bits(dptr, fnTarget.numVertices() * sizeof(double) * 3);

    // If the local vertex positions have changed, only then do we rebuild the BVH
    if (vertHash == 0 || vertHashChk != vertHash) {
        vertHash = vertHashChk;
        bboxes.clear();
        centers.clear();
        normals.clear();
        tris.clear();
        
        MIntArray triCounts, triVerts;
        fnTarget.getTriangles(triCounts, triVerts);
        for (UINT i = 0; i < triVerts.length(); i += 3) {
            int tv0 = triVerts[i + 0];
            double v00 = (tv0 * 3) + 0;
            double v01 = (tv0 * 3) + 1;
            double v02 = (tv0 * 3) + 2;
            Vec3 v0(v00, v01, v02);

            int tv1 = triVerts[i + 1];
            double v10 = (tv1 * 3) + 0;
            double v11 = (tv1 * 3) + 1;
            double v12 = (tv1 * 3) + 2;
            Vec3 v1(v10, v11, v12);

            int tv2 = triVerts[i + 2];
            double v20 = (tv2 * 3) + 0;
            double v21 = (tv2 * 3) + 1;
            double v22 = (tv2 * 3) + 2;
            Vec3 v2(v20, v21, v22);

            tris.emplace_back(v0, v1, v2);
        }
        bvh = build_bvh(tris, bboxes, centers, normals);
    }






    float maxParam = block.inputValue(aMaxParam, &stat).asFloat();
    float angleTol = block.inputValue(aAngleTolerance, &stat).asFloat();
    MMatrix tWInv = block.inputValue(aTargetInvWorld, &stat).asMatrix();

    MMatrix tranMatInv = m * tWInv;
    MMatrix tranMat = tranMatInv.inverse();



    //MMeshIntersector octree;
    //octree.create(target);

    for (size_t i=0; !iter.isDone(); iter.next(), i++) {

        float w = weightValue(block, multiIndex, iter.index());
        if (w == 0.0f) continue;
        MPoint pt = iter.position();
        MPoint tpt = pt * tranMatInv; // target space point

        MVector n = iter.normal();


        Vec3 tv(tpt.x, tpt.y, tpt.z);
        Vec3 tn(n.x, n.y, n.z);

        Vec3 cpom = get_closest(bvh,
            tris,
            bboxes,
            centers,
            normals,
            tv,
            tn,
            angleTol
        );

        MPoint res(cpom[0], cpom[1], cpom[2]);
        iter.setPosition(env * w * ((res * tranMat) - pt) + pt);
    }
    return stat;
}
