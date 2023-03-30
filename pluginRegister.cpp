#include "blurNormalShrinkWrap.h"
#include <maya/MGlobal.h>
#include <maya/MFnPlugin.h>

// standard initialization procedures
MStatus initializePlugin(MObject obj) {
    MStatus result;
    MFnPlugin plugin(obj, "Blur Studio", "1.0", "Any");
    result = plugin.registerNode(DEFORMER_NAME, NormalShrinkWrapDeformer::id, NormalShrinkWrapDeformer::creator,
                                  NormalShrinkWrapDeformer::initialize, MPxNode::kDeformerNode);

    MString nodeClassName(DEFORMER_NAME);
    MString registrantId("BlurPlugin");

	MGlobal::executeCommand("makePaintable -attrType \"multiFloat\" -sm \"deformer\" \"" DEFORMER_NAME "\" \"weights\";");

    //MGPUDeformerRegistry::registerGPUDeformerCreator(nodeClassName, registrantId, PushGPUDeformer::getGPUDeformerInfo());
    //MGPUDeformerRegistry::addConditionalAttribute(nodeClassName, registrantId, MPxDeformerNode::envelope);
    //PushGPUDeformer::pluginLoadPath = plugin.loadPath();

    return result;
}

MStatus uninitializePlugin(MObject obj) {
    MStatus result;
    MFnPlugin plugin(obj);
    result = plugin.deregisterNode(NormalShrinkWrapDeformer::id);

    MString nodeClassName(DEFORMER_NAME);
    MString registrantId("BlurPlugin");
    //MGPUDeformerRegistry::deregisterGPUDeformerCreator(nodeClassName, registrantId);

    return result;
}

