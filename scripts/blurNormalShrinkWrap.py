from maya import cmds


def buildNormalShrinkWrap(deformed, controlMesh):
    sourceStatic = cmds.duplicate(deformed, name=deformed + "Static")[0]
    targetStatic = cmds.duplicate(controlMesh, name=controlMesh + "Static")[0]

    sourceStaticShape = cmds.listRelatives(
        sourceStatic, shapes=True, noIntermediate=True, fullPath=True
    )[0]
    targetStaticShape = cmds.listRelatives(
        targetStatic, shapes=True, noIntermediate=True, fullPath=True
    )[0]
    targetShape = cmds.listRelatives(
        controlMesh, shapes=True, noIntermediate=True, fullPath=True
    )[0]

    dfm = cmds.deformer(deformed, type="blurNormalShrinkWrap")[0]
    cmds.connectAttr(
        f"{sourceStatic}.worldInverseMatrix[0]", f"{dfm}.sourceStaticInvWorld"
    )
    cmds.connectAttr(
        f"{targetStatic}.worldInverseMatrix[0]", f"{dfm}.targetStaticInvWorld"
    )
    cmds.connectAttr(f"{controlMesh}.worldInverseMatrix[0]", f"{dfm}.targetInvWorld")

    cmds.connectAttr(f"{sourceStaticShape}.outMesh", f"{dfm}.sourceStatic")
    cmds.connectAttr(f"{targetStaticShape}.outMesh", f"{dfm}.targetStatic")
    cmds.connectAttr(f"{targetShape}.outMesh", f"{dfm}.target")


#buildNormalShrinkWrap(*cmds.ls(selection=True)[:2])
