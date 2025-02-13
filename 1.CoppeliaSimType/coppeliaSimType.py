class SimType:
    def __init__(self):
        pass

    def acquireLock(self):
        """
        :return:
        """
        pass

    def addDrawingObject(objectType: int, size: float, duplicateTolerance: float, parentObjectHandle: int,
                         maxItemCount: int, color: float):
        """
        :param objectType : Int
        :param size : Float
        :param duplicateTolerance : Float
        :param parentObjectHandle : Int
        :param maxItemCount : Int
        :param color : Float[3]
        :return: int drawingObjectHandle
        """
        pass

    def addDrawingObjectItem(drawingObjectHandle: int, itemData: float):
        """
        :param drawingObjectHandle : Int
        :param itemData : Float[]
        :return: int result
        """
        pass

    def addForce(shapeHandle: int, position: float, force: float):
        """
        :param shapeHandle : Int
        :param position : Float[3]
        :param force : Float[3]
        :return:
        """
        pass

    def addForceAndTorque(shapeHandle: int, force: float, torque: float):
        """
        :param shapeHandle : Int
        :param force : Float[3]
        :param torque : Float[3]
        :return:
        """
        pass

    def addGraphCurve(graphHandle: int, curveName: str, dim: int,streamIds: list, defaultValues: list, unitStr: str, options: int, color: float,
                      curveWidth: int):
        """
        :param graphHandle : Int
        :param curveName : String
        :param dim : Int
        :param streamIds : Int[2..3]
        :param defaultValues : Float[2..3]
        :param unitStr : String
        :param options : Int
        :param color : Float[3]
        :param curveWidth : Int
        :return: int curveId
        """
        pass

    def addGraphStream(graphHandle: int, streamName: str, unit: str, options: int, color: float, cyclicRange: float):
        """
        :param graphHandle : Int
        :param streamName : String
        :param unit : String
        :param options : Int
        :param color : Float[3]
        :param cyclicRange : Float
        :return: int streamId
        """
        pass

    def addItemToCollection(collectionHandle: int, what: int, objectHandle: int, options: int):
        """
        :param collectionHandle : Int
        :param what : Int
        :param objectHandle : Int
        :param options : Int
        :return:
        """
        pass

    def addLog(verbosityLevel: int, logMessage: str):
        """
        :param verbosityLevel : Int
        :param logMessage : String
        :return:
        """
        pass

    def addParticleObject(objectType: int, size: float, density: float, params: float, lifeTime: float,
                          maxItemCount: int, color: float):
        """
        :param objectType : Int
        :param size : Float
        :param density : Float
        :param params : Float[]
        :param lifeTime : Float
        :param maxItemCount : Int
        :param color : Float[3]
        :return: int particleObjectHandle
        """
        pass

    def addParticleObjectItem(objectHandle: int, itemData: float):
        """
        :param objectHandle : Int
        :param itemData : Float[]
        :return:
        """
        pass

    def addReferencedHandle(objectHandle: int, referencedHandle: int, tag: str, opts: map):
        """
        :param objectHandle : Int
        :param referencedHandle : Int
        :param tag : String
        :param opts : Map
        :return:
        """
        pass

    def adjustView(viewHandleOrIndex: int, objectHandle: int, options: int, viewLabel: str):
        """
        :param viewHandleOrIndex : Int
        :param objectHandle : Int
        :param options : Int
        :param viewLabel : String
        :return: int res
        """
        pass

    def alignShapeBB(shapeHandle: int, pose: float):
        """
        :param shapeHandle : Int
        :param pose : Float[7]
        :return: int result
        """
        pass

    def alphaBetaGammaToYawPitchRoll(alphaAngle: float, betaAngle: float, gammaAngle: float):
        """
        :param alphaAngle : Float
        :param betaAngle : Float
        :param gammaAngle : Float
        :return: float yawAngle, float pitchAngle, float rollAngle
        """
        pass

    def announceSceneContentChange(self):
        """
        :return: int result
        """
        pass

    def auxiliaryConsoleClose(consoleHandle: int):
        """
        :param consoleHandle : Int
        :return: int result
        """
        pass

    def auxiliaryConsoleOpen(title: str, maxLines: int, mode: int, position: int, size: int, textColor: float,
                             backgroundColor: float):
        """
        :param title : String
        :param maxLines : Int
        :param mode : Int
        :param position : Int[2]
        :param size : Int[2]
        :param textColor : Float[3]
        :param backgroundColor : Float[3]
        :return: int consoleHandle
        """
        pass

    def auxiliaryConsolePrint(consoleHandle: int, text: str):
        """
        :param consoleHandle : Int
        :param text : String
        :return: int result
        """
        pass

    def auxiliaryConsoleShow(consoleHandle: int, showState: bool):
        """
        :param consoleHandle : Int
        :param showState : Bool
        :return: int result
        """
        pass

    def broadcastMsg(message: map, options: int):
        """
        :param message : Map
        :param options : Int
        :return:
        """
        pass

    def buildIdentityMatrix(self):
        """
        :return: float[12] matrix
        """
        pass

    def buildMatrix(position: float, eulerAngles: float):
        """
        :param position : Float[3]
        :param eulerAngles : Float[3]
        :return: float[12] matrix
        """
        pass

    def buildPose(position: float, eulerAnglesOrAxis: float, mode: int, axis2: float):
        """
        :param position : Float[3]
        :param eulerAnglesOrAxis : Float[3]
        :param mode : Int
        :param axis2 : Float[3]
        :return: float[7] pose
        """
        pass

    def callScriptFunction(functionName: str, scriptHandle: int):
        """
        :param functionName : String
        :param scriptHandle : Int
        :return: ...
        """
        pass

    def cameraFitToView(viewHandleOrIndex: int, objectHandles: int, options: int, scaling: float):
        """
        :param viewHandleOrIndex : Int
        :param objectHandles : Int[]
        :param options : Int
        :param scaling : Float
        :return: int result
        """
        pass

    def cancelScheduledExecution(id: int):
        """
        :param id : Int
        :return: bool canceled
        """
        pass

    def changeEntityColor(entityHandle: int, newColor: float, colorComponent: int):
        """
        :param entityHandle : Int
        :param newColor : Float[3]
        :param colorComponent : Int
        :return: map[] originalColorData
        """
        pass

    def checkCollision(entity1Handle: int, entity2Handle: int):
        """
        :param entity1Handle : Int
        :param entity2Handle : Int
        :return: int result, int[2] collidingObjects
        """
        pass

    def checkCollisionEx(entity1Handle: int, entity2Handle: int):
        """
        :param entity1Handle : Int
        :param entity2Handle : Int
        :return: int segmentCount, float[6..*] segmentData
        """
        pass

    def checkDistance(entity1Handle: int, entity2Handle: int, threshold: float):
        """
        :param entity1Handle : Int
        :param entity2Handle : Int
        :param threshold : Float
        :return: int result, float[7] distanceData, int[2] objectHandlePair
        """
        pass

    def checkOctreePointOccupancy(octreeHandle: int, options: int, points: float):
        """
        :param octreeHandle : Int
        :param options : Int
        :param points : Float[]
        :return: int result, int tag, int locationLow, int locationHigh
        """
        pass

    def checkProximitySensor(sensorHandle: int, entityHandle: int):
        """
        :param sensorHandle : Int
        :param entityHandle : Int
        :return: int result, float distance, float[3] detectedPoint, int detectedObjectHandle, float[3] normalVector
        """
        pass

    def checkProximitySensorEx(sensorHandle: int, entityHandle: int, mode: int, threshold: float, maxAngle: float):
        """
        :param sensorHandle : Int
        :param entityHandle : Int
        :param mode : Int
        :param threshold : Float
        :param maxAngle : Float
        :return: int result, float distance, float[3] detectedPoint, int detectedObjectHandle, float[3] normalVector
        """
        pass

    def checkProximitySensorEx2(sensorHandle: int, vertices: list, itemType: int, itemCount: int, mode: int, threshold: float,
                                maxAngle: float):
        """
        :param sensorHandle : Int
        :param vertices : Float[3..*]
        :param itemType : Int
        :param itemCount : Int
        :param mode : Int
        :param threshold : Float
        :param maxAngle : Float
        :return: int result, float distance, float[3] detectedPoint, float[3] normalVector
        """
        pass

    def checkVisionSensor(sensorHandle: int, entityHandle: int):
        """
        :param sensorHandle : Int
        :param entityHandle : Int
        :return: int result, float[] auxPacket1, float[] auxPacket2
        """
        pass

    def checkVisionSensorEx(sensorHandle: int, entityHandle: int, returnImage: bool):
        """
        :param sensorHandle : Int
        :param entityHandle : Int
        :param returnImage : Bool
        :return: float[] theBuffer
        """
        pass

    def clearBufferSignal(signalName: str):
        """
        :param signalName : String
        :return:
        """
        pass

    def clearFloatSignal(signalName: str):
        """
        :param signalName : String
        :return:
        """
        pass

    def clearInt32Signal(signalName: str):
        """
        :param signalName : String
        :return:
        """
        pass

    def clearStringSignal(signalName: str):
        """
        :param signalName : String
        :return:
        """
        pass

    def closeScene(self):
        """
        :return: int result
        """
        pass

    def combineRgbImages(img1: bytes, img1Res: int, img2: bytes, img2Res: int, operation: int):
        """
        :param img1 : Buffer
        :param img1Res : Int[2]
        :param img2 : Buffer
        :param img2Res : Int[2]
        :param operation : Int
        :return: buffer outImg
        """
        pass

    def computeMassAndInertia(shapeHandle: int, density: float):
        """
        :param shapeHandle : Int
        :param density : Float
        :return: int result
        """
        pass

    def copyPasteObjects(objectHandles: list, options: int):
        """
        :param objectHandles : Int[1..*]
        :param options : Int
        :return: int[1..*] copiedObjectHandles
        """
        pass

    def copyTable(original: any):
        """
        :param original : Any[]
        :return: any[] copy
        """
        pass

    def createCollection(options: int):
        """
        :param options : Int
        :return: int collectionHandle
        """
        pass

    def createDummy(size: float):
        """
        :param size : Float
        :return: int dummyHandle
        """
        pass

    def createForceSensor(options: int, intParams: int, floatParams: float):
        """
        :param options : Int
        :param intParams : Int[5]
        :param floatParams : Float[5]
        :return: int sensorHandle
        """
        pass

    def createHeightfieldShape(options: int, shadingAngle: float, xPointCount: int, yPointCount: int, xSize: float,
                               heights: float):
        """
        :param options : Int
        :param shadingAngle : Float
        :param xPointCount : Int
        :param yPointCount : Int
        :param xSize : Float
        :param heights : Float[]
        :return: int shapeHandle
        """
        pass

    def createJoint(jointType: int, jointMode: int, options: int, sizes: float):
        """
        :param jointType : Int
        :param jointMode : Int
        :param options : Int
        :param sizes : Float[2]
        :return: int jointHandle
        """
        pass

    def createOctree(voxelSize: float, options: int, pointSize: float):
        """
        :param voxelSize : Float
        :param options : Int
        :param pointSize : Float
        :return: int handle
        """
        pass

    def createPath(ctrlPts: float, options: int, subdiv: int, smoothness: float, orientationMode: int, upVector: float):
        """
        :param ctrlPts : Float[]
        :param options : Int
        :param subdiv : Int
        :param smoothness : Float
        :param orientationMode : Int
        :param upVector : Float[3]
        :return: int pathHandle
        """
        pass

    def createPointCloud(maxVoxelSize: float, maxPtCntPerVoxel: int, options: int, pointSize: float):
        """
        :param maxVoxelSize : Float
        :param maxPtCntPerVoxel : Int
        :param options : Int
        :param pointSize : Float
        :return: int handle
        """
        pass

    def createPrimitiveShape(primitiveType: int, sizes: float, options: int):
        """
        :param primitiveType : Int
        :param sizes : Float[3]
        :param options : Int
        :return: int shapeHandle
        """
        pass

    def createProximitySensor(sensorType: int, subType: int, options: int, intParams: int, floatParams: float):
        """
        :param sensorType : Int
        :param subType : Int
        :param options : Int
        :param intParams : Int[8]
        :param floatParams : Float[15]
        :return: int sensorHandle
        """
        pass

    def createScript(scriptType: int, scriptString: str, options: int, lang: str):
        """
        :param scriptType : Int
        :param scriptString : String
        :param options : Int
        :param lang : String
        :return: int scriptHandle
        """
        pass

    def createShape(options: int, shadingAngle: float, vertices: float, indices: int, normals: float,
                    textureCoordinates: float, texture: bytes, textureResolution: int):
        """
        :param options : Int
        :param shadingAngle : Float
        :param vertices : Float[]
        :param indices : Int[]
        :param normals : Float[]
        :param textureCoordinates : Float[]
        :param texture : Buffer
        :param textureResolution : Int[2]
        :return: int shapeHandle
        """
        pass

    def createTexture(fileName: str, options: int, planeSizes: float, scalingUV: float, xy_g: float,
                      fixedResolution: int, resolution: int):
        """
        :param fileName : String
        :param options : Int
        :param planeSizes : Float[2]
        :param scalingUV : Float[2]
        :param xy_g : Float[2]
        :param fixedResolution : Int
        :param resolution : Int[2]
        :return: int shapeHandle, int textureId, int[2] resolution
        """
        pass

    def createVisionSensor(options: int, intParams: int, floatParams: float):
        """
        :param options : Int
        :param intParams : Int[4]
        :param floatParams : Float[11]
        :return: int sensorHandle
        """
        pass

    def destroyCollection(collectionHandle: int):
        """
        :param collectionHandle : Int
        :return:
        """
        pass

    def destroyGraphCurve(graphHandle: int, curveId: int):
        """
        :param graphHandle : Int
        :param curveId : Int
        :return:
        """
        pass

    def duplicateGraphCurveToStatic(graphHandle: int, curveId: int, curveName: str):
        """
        :param graphHandle : Int
        :param curveId : Int
        :param curveName : String
        :return: int curveId
        """
        pass

    def executeScriptString(stringToExecute: str, scriptHandle: int):
        """
        :param stringToExecute : String
        :param scriptHandle : Int
        :return: int result, any value
        """
        pass

    def exportMesh(fileformat: int, pathAndFilename: str, options: int, scalingFactor: float, vertices: list, indices: list):
        """
        :param fileformat : Int
        :param pathAndFilename : String
        :param options : Int
        :param scalingFactor : Float
        :param vertices : Float[1..*]
        :param indices : Int[1..*]
        :return:
        """
        pass

    def floatingViewAdd(posX: float, posY: float, sizeX: float, sizeY: float, options: int):
        """
        :param posX : Float
        :param posY : Float
        :param sizeX : Float
        :param sizeY : Float
        :param options : Int
        :return: int floatingViewHandle
        """
        pass

    def floatingViewRemove(floatingViewHandle: int):
        """
        :param floatingViewHandle : Int
        :return: int result
        """
        pass

    def generateShapeFromPath(path: float, section: float, options: int, upVector: float):
        """
        :param path : Float[]
        :param section : Float[]
        :param options : Int
        :param upVector : Float[3]
        :return: int shapeHandle
        """
        pass

    def generateTextShape(txt: str, color: float, height: float, centered: bool, alphabetLocation: str):
        """
        :param txt : String
        :param color : Float[3]
        :param height : Float
        :param centered : Bool
        :param alphabetLocation : String
        :return: int modelHandle
        """
        pass

    def generateTimeOptimalTrajectory(path: float, pathLengths: float, minMaxVel: float, minMaxAccel: float,
                                      trajPtSamples: int, boundaryCondition: str, timeout: float):
        """
        :param path : Float[]
        :param pathLengths : Float[]
        :param minMaxVel : Float[]
        :param minMaxAccel : Float[]
        :param trajPtSamples : Int
        :param boundaryCondition : String
        :param timeout : Float
        :return: float[] path, float[] times
        """
        pass

    def getAlternateConfigs(jointHandles: int, inputConfig: float, tipHandle: int, lowLimits: float, ranges: float):
        """
        :param jointHandles : Int[]
        :param inputConfig : Float[]
        :param tipHandle : Int
        :param lowLimits : Float[]
        :param ranges : Float[]
        :return: float[] configs
        """
        pass

    def getApiFunc(scriptHandle: int, apiWord: str):
        """
        :param scriptHandle : Int
        :param apiWord : String
        :return: string[] funcsAndVars
        """
        pass

    def getApiInfo(scriptHandle: int, apiWord: str):
        """
        :param scriptHandle : Int
        :param apiWord : String
        :return: string info
        """
        pass

    def getArrayParam(parameter: int):
        """
        :param parameter : Int
        :return: float[3] arrayOfValues
        """
        pass

    def getAutoYieldDelay(self):
        """
        :return: float dt
        """
        pass

    def getBoolParam(parameter: int):
        """
        :param parameter : Int
        :return: bool boolState
        """
        pass

    def getBoolProperty(target: int, pName: str, options: map):
        """
        :param target : Int
        :param pName : String
        :param options : Map
        :return: bool pValue
        """
        pass

    def getBufferProperty(target: int, pName: str, options: map):
        """
        :param target : Int
        :param pName : String
        :param options : Map
        :return: buffer pValue
        """
        pass

    def getBufferSignal(signalName: str):
        """
        :param signalName : String
        :return: buffer signalValue
        """
        pass

    def getClosestPosOnPath(path: float, pathLengths: float, absPt: float):
        """
        :param path : Float[]
        :param pathLengths : Float[]
        :param absPt : Float[3]
        :return: float posAlongPath
        """
        pass

    def getCollectionObjects(collectionHandle: int):
        """
        :param collectionHandle : Int
        :return: int[] objectHandles
        """
        pass

    def getColorProperty(target: int, pName: str, options: map):
        """
        :param target : Int
        :param pName : String
        :param options : Map
        :return: float[3] pValue
        """
        pass

    def getConfigDistance(configA: float, configB: float, metric: float, types: int):
        """
        :param configA : Float[]
        :param configB : Float[]
        :param metric : Float[]
        :param types : Int[]
        :return: float distance
        """
        pass

    def getContactInfo(dynamicPass: int, objectHandle: int, index: int):
        """
        :param dynamicPass : Int
        :param objectHandle : Int
        :param index : Int
        :return: int[2] collidingObjects, float[3] collisionPoint, float[3] reactionForce, float[3] normalVector
        """
        pass

    def getEngineBoolParam(paramId: int, objectHandle: int):
        """
        :param paramId : Int
        :param objectHandle : Int
        :return: bool boolParam
        """
        pass

    def getEngineFloatParam(paramId: int, objectHandle: int):
        """
        :param paramId : Int
        :param objectHandle : Int
        :return: float floatParam
        """
        pass

    def getEngineInt32Param(paramId: int, objectHandle: int):
        """
        :param paramId : Int
        :param objectHandle : Int
        :return: int int32Param
        """
        pass

    def getEulerAnglesFromMatrix(matrix: float):
        """
        :param matrix : Float[12]
        :return: float[3] eulerAngles
        """
        pass

    def getExplicitHandling(objectHandle: int):
        """
        :param objectHandle : Int
        :return: int explicitHandlingFlags
        """
        pass

    def getExtensionString(objectHandle: int, index: int, key: str):
        """
        :param objectHandle : Int
        :param index : Int
        :param key : String
        :return: string theString
        """
        pass

    def getFloatArrayProperty(target: int, pName: str, options: map):
        """
        :param target : Int
        :param pName : String
        :param options : Map
        :return: float[] pValue
        """
        pass

    def getFloatParam(parameter: int):
        """
        :param parameter : Int
        :return: float floatState
        """
        pass

    def getFloatProperty(target: int, pName: str, options: map):
        """
        :param target : Int
        :param pName : String
        :param options : Map
        :return: float pValue
        """
        pass

    def getFloatSignal(signalName: str):
        """
        :param signalName : String
        :return: float signalValue
        """
        pass

    def getGenesisEvents(self):
        """
        :return: map[] events
        """
        pass

    def getGraphCurve(graphHandle: int, graphType: int, curveIndex: int):
        """
        :param graphHandle : Int
        :param graphType : Int
        :param curveIndex : Int
        :return: string label, int attributes, float[3] curveColor, float[] xData, float[] yData, float[6] minMax, int curveId, int curveWidth
        """
        pass

    def getGraphInfo(graphHandle: int):
        """
        :param graphHandle : Int
        :return: int bitCoded, float[3] bgColor, float[3] fgColor
        """
        pass

    def getInt32Param(parameter: int):
        """
        :param parameter : Int
        :return: int intState
        """
        pass

    def getInt32Signal(signalName: str):
        """
        :param signalName : String
        :return: int signalValue
        """
        pass

    def getIntArray2Property(target: int, pName: str, options: map):
        """
        :param target : Int
        :param pName : String
        :param options : Map
        :return: int[2] pValue
        """
        pass

    def getIntArrayProperty(target: int, pName: str, options: map):
        """
        :param target : Int
        :param pName : String
        :param options : Map
        :return: int[] pValue
        """
        pass

    def getIntProperty(target: int, pName: str, options: map):
        """
        :param target : Int
        :param pName : String
        :param options : Map
        :return: int pValue
        """
        pass

    def getIsRealTimeSimulation(self):
        """
        :return: int result
        """
        pass

    def getJointDependency(jointHandle: int):
        """
        :param jointHandle : Int
        :return: int masterJointHandle, float offset, float multCoeff
        """
        pass

    def getJointForce(jointHandle: int):
        """
        :param jointHandle : Int
        :return: float forceOrTorque
        """
        pass

    def getJointInterval(objectHandle: int):
        """
        :param objectHandle : Int
        :return: bool cyclic, float[2] interval
        """
        pass

    def getJointMode(jointHandle: int):
        """
        :param jointHandle : Int
        :return: int jointMode, int options
        """
        pass

    def getJointPosition(objectHandle: int):
        """
        :param objectHandle : Int
        :return: float position
        """
        pass

    def getJointTargetForce(jointHandle: int):
        """
        :param jointHandle : Int
        :return: float forceOrTorque
        """
        pass

    def getJointTargetPosition(objectHandle: int):
        """
        :param objectHandle : Int
        :return: float targetPosition
        """
        pass

    def getJointTargetVelocity(objectHandle: int):
        """
        :param objectHandle : Int
        :return: float targetVelocity
        """
        pass

    def getJointType(objectHandle: int):
        """
        :param objectHandle : Int
        :return: int jointType
        """
        pass

    def getJointVelocity(jointHandle: int):
        """
        :param jointHandle : Int
        :return: float velocity
        """
        pass

    def getLastInfo(self):
        """
        :return: string info
        """
        pass

    def getLightParameters(lightHandle: int):
        """
        :param lightHandle : Int
        :return: int state, float[3] zero, float[3] diffusePart, float[3] specular
        """
        pass

    def getLinkDummy(dummyHandle: int):
        """
        :param dummyHandle : Int
        :return: int linkDummyHandle
        """
        pass

    def getLongProperty(target: int, pName: str, options: map):
        """
        :param target : Int
        :param pName : String
        :param options : Map
        :return: int pValue
        """
        pass

    def getMatrixInverse(matrix: float):
        """
        :param matrix : Float[12]
        :return: float[12] matrix
        """
        pass

    def getModelProperty(objectHandle: int):
        """
        :param objectHandle : Int
        :return: int property
        """
        pass

    def getNamedBoolParam(name: str):
        """
        :param name : String
        :return: bool value
        """
        pass

    def getNamedFloatParam(name: str):
        """
        :param name : String
        :return: float value
        """
        pass

    def getNamedInt32Param(name: str):
        """
        :param name : String
        :return: int value
        """
        pass

    def getNamedStringParam(paramName: str):
        """
        :param paramName : String
        :return: buffer stringParam
        """
        pass

    def getNavigationMode(self):
        """
        :return: int navigationMode
        """
        pass

    def getObject(path: str, options: map):
        """
        :param path : String
        :param options : Map
        :return: int objectHandle
        """
        pass

    def getObjectAlias(objectHandle: int, options: int):
        """
        :param objectHandle : Int
        :param options : Int
        :return: string objectAlias
        """
        pass

    def getObjectAliasRelative(handle: int, baseHandle: int, options: int):
        """
        :param handle : Int
        :param baseHandle : Int
        :param options : Int
        :return: string alias
        """
        pass

    def getObjectChild(objectHandle: int, index: int):
        """
        :param objectHandle : Int
        :param index : Int
        :return: int childObjectHandle
        """
        pass

    def getObjectChildPose(objectHandle: int):
        """
        :param objectHandle : Int
        :return: float[7] pose
        """
        pass

    def getObjectColor(objectHandle: int, index: int, colorComponent: int):
        """
        :param objectHandle : Int
        :param index : Int
        :param colorComponent : Int
        :return: float[3] rgbData
        """
        pass

    def getObjectFloatArrayParam(objectHandle: int, parameterID: int):
        """
        :param objectHandle : Int
        :param parameterID : Int
        :return: float[] params
        """
        pass

    def getObjectFloatParam(objectHandle: int, parameterID: int):
        """
        :param objectHandle : Int
        :param parameterID : Int
        :return: float parameter
        """
        pass

    def getObjectFromUid(uid: int, options: map):
        """
        :param uid : Int
        :param options : Map
        :return:
        """
        pass

    def getObjectHierarchyOrder(objectHandle: int):
        """
        :param objectHandle : Int
        :return: int order, int totalSiblingsCount
        """
        pass

    def getObjectInt32Param(objectHandle: int, parameterID: int):
        """
        :param objectHandle : Int
        :param parameterID : Int
        :return: int parameter
        """
        pass

    def getObjectMatrix(objectHandle: int, relativeToObjectHandle: int):
        """
        :param objectHandle : Int
        :param relativeToObjectHandle : Int
        :return: float[12] matrix
        """
        pass

    def getObjectOrientation(objectHandle: int, relativeToObjectHandle: int):
        """
        :param objectHandle : Int
        :param relativeToObjectHandle : Int
        :return: float[3] eulerAngles
        """
        pass

    def getObjectParent(objectHandle: int):
        """
        :param objectHandle : Int
        :return: int parentObjectHandle
        """
        pass

    def getObjectPose(objectHandle: int, relativeToObjectHandle: int):
        """
        :param objectHandle : Int
        :param relativeToObjectHandle : Int
        :return: float[7] pose
        """
        pass

    def getObjectPosition(objectHandle: int, relativeToObjectHandle: int):
        """
        :param objectHandle : Int
        :param relativeToObjectHandle : Int
        :return: float[3] position
        """
        pass

    def getObjectProperty(objectHandle: int):
        """
        :param objectHandle : Int
        :return: int property
        """
        pass

    def getObjectQuaternion(objectHandle: int, relativeToObjectHandle: int):
        """
        :param objectHandle : Int
        :param relativeToObjectHandle : Int
        :return: float[4] quaternion
        """
        pass

    def getObjectSel(self):
        """
        :return: int[] objectHandles
        """
        pass

    def getObjectSizeFactor(ObjectHandle: int):
        """
        :param ObjectHandle : Int
        :return: float sizeFactor
        """
        pass

    def getObjectSpecialProperty(objectHandle: int):
        """
        :param objectHandle : Int
        :return: int property
        """
        pass

    def getObjectStringParam(objectHandle: int, parameterID: int):
        """
        :param objectHandle : Int
        :param parameterID : Int
        :return: buffer parameter
        """
        pass

    def getObjectType(objectHandle: int):
        """
        :param objectHandle : Int
        :return: int objectType
        """
        pass

    def getObjectUid(objectHandle: int):
        """
        :param objectHandle : Int
        :return: int uid
        """
        pass

    def getObjectVelocity(objectHandle: int):
        """
        :param objectHandle : Int
        :return: float[3] linearVelocity, float[3] angularVelocity
        """
        pass

    def getObjects(index: int, objectType: int):
        """
        :param index : Int
        :param objectType : Int
        :return: int objectHandle
        """
        pass

    def getObjectsInTree(treeBaseHandle: int, objectType: int, options: int):
        """
        :param treeBaseHandle : Int
        :param objectType : Int
        :param options : Int
        :return: int[] objects
        """
        pass

    def getOctreeVoxels(octreeHandle: int):
        """
        :param octreeHandle : Int
        :return: float[] voxels
        """
        pass

    def getPage(self):
        """
        :return: int pageIndex
        """
        pass

    def getPathInterpolatedConfig(path: float, pathLengths: float, t: float, method: map, types: int):
        """
        :param path : Float[]
        :param pathLengths : Float[]
        :param t : Float
        :param method : Map
        :param types : Int[]
        :return: float[] config
        """
        pass

    def getPathLengths(path: float, dof: int, distCallback: str):
        """
        :param path : Float[]
        :param dof : Int
        :param distCallback : Func
        :return: float[] pathLengths, float totalLength
        """
        pass

    def getPluginInfo(pluginName: str, infoType: int):
        """
        :param pluginName : String
        :param infoType : Int
        :return: string info
        """
        pass

    def getPluginName(index: int):
        """
        :param index : Int
        :return: string pluginName
        """
        pass

    def getPointCloudOptions(pointCloudHandle: int):
        """
        :param pointCloudHandle : Int
        :return: float maxVoxelSize, int maxPtCntPerVoxel, int options, float pointSize
        """
        pass

    def getPointCloudPoints(pointCloudHandle: int):
        """
        :param pointCloudHandle : Int
        :return: float[] points
        """
        pass

    def getPoseInverse(pose: float):
        """
        :param pose : Float[7]
        :return: float[7] pose
        """
        pass

    def getPoseProperty(target: int, pName: str, options: map):
        """
        :param target : Int
        :param pName : String
        :param options : Map
        :return: float[7] pValue
        """
        pass

    def getProperties(target: int, opts: map):
        """
        :param target : Int
        :param opts : Map
        :return: map values
        """
        pass

    def getPropertiesInfos(target: int, opts: map):
        """
        :param target : Int
        :param opts : Map
        :return: map infos
        """
        pass

    def getProperty(target: int, pName: str, options: map):
        """
        :param target : Int
        :param pName : String
        :param options : Map
        :return: any pValue
        """
        pass

    def getPropertyInfo(target: int, pName: str, options: map):
        """
        :param target : Int
        :param pName : String
        :param options : Map
        :return: int pType, int pFlags, string description
        """
        pass

    def getPropertyName(target: int, index: int, options: map):
        """
        :param target : Int
        :param index : Int
        :param options : Map
        :return: string pName, string appartenance
        """
        pass

    def getPropertyTypeString(pType: int):
        """
        :param pType : Int
        :return: string pTypeStr
        """
        pass

    def getQuaternionProperty(target: int, pName: str, options: map):
        """
        :param target : Int
        :param pName : String
        :param options : Map
        :return: float[4] pValue
        """
        pass

    def getRandom(seed: int):
        """
        :param seed : Int
        :return: float randomNumber
        """
        pass

    def getRealTimeSimulation(self):
        """
        :return: bool result
        """
        pass

    def getReferencedHandle(objectHandle: int, tag: str):
        """
        :param objectHandle : Int
        :param tag : String
        :return: int referencedHandle
        """
        pass

    def getReferencedHandles(objectHandle: int, tag: str):
        """
        :param objectHandle : Int
        :param tag : String
        :return: int[] referencedHandles
        """
        pass

    def getReferencedHandlesTags(objectHandle: int):
        """
        :param objectHandle : Int
        :return: string[] tags
        """
        pass

    def getRotationAxis(matrixStart: float, matrixGoal: float, angle: float):
        """
        :param matrixStart : Float[12]
        :param matrixGoal : Float[12]
        :return: float[3] axis, float angle
        """
        pass

    def getScaledImage(imageIn: bytes, resolutionIn: int, desiredResolutionOut: int, options: int):
        """
        :param imageIn : Buffer
        :param resolutionIn : Int[2]
        :param desiredResolutionOut : Int[2]
        :param options : Int
        :return: buffer imageOut, int[2] effectiveResolutionOut
        """
        pass

    def getScript(scriptType: int, scriptName: str):
        """
        :param scriptType : Int
        :param scriptName : String
        :return: int scriptHandle
        """
        pass

    def getScriptFunctions(scriptHandle: int):
        """
        :param scriptHandle : Int
        :return: map wrapper
        """
        pass

    def getSettingBool(key: str):
        """
        :param key : String
        :return: bool value
        """
        pass

    def getSettingFloat(key: str):
        """
        :param key : String
        :return: float value
        """
        pass

    def getSettingInt32(key: str):
        """
        :param key : String
        :return: int value
        """
        pass

    def getSettingString(key: str):
        """
        :param key : String
        :return: string value
        """
        pass

    def getShapeAppearance(handle: int, opts: map):
        """
        :param handle : Int
        :param opts : Map
        :return: map savedData
        """
        pass

    def getShapeBB(shapeHandle: int):
        """
        :param shapeHandle : Int
        :return: float[3] size, float[7] pose
        """
        pass

    def getShapeColor(shapeHandle: int, colorName: str, colorComponent: int):
        """
        :param shapeHandle : Int
        :param colorName : String
        :param colorComponent : Int
        :return: int result, float[] rgbData
        """
        pass

    def getShapeGeomInfo(shapeHandle: int):
        """
        :param shapeHandle : Int
        :return: int result, int pureType, float[4] dimensions
        """
        pass

    def getShapeInertia(shapeHandle: int):
        """
        :param shapeHandle : Int
        :return: float[9] inertiaMatrix, float[12] comMatrix
        """
        pass

    def getShapeMass(shapeHandle: int):
        """
        :param shapeHandle : Int
        :return: float mass
        """
        pass

    def getShapeMesh(shapeHandle: int):
        """
        :param shapeHandle : Int
        :return: float[] vertices, int[] indices, float[] normals
        """
        pass

    def getShapeTextureId(shapeHandle: int):
        """
        :param shapeHandle : Int
        :return: int textureId
        """
        pass

    def getShapeViz(shapeHandle: int, itemIndex: int):
        """
        :param shapeHandle : Int
        :param itemIndex : Int
        :return: map data
        """
        pass

    def getSignalName(signalIndex: int, signalType: int):
        """
        :param signalIndex : Int
        :param signalType : Int
        :return: string signalName
        """
        pass

    def getSimulationState(self):
        """
        :return: int simulationState
        """
        pass

    def getSimulationStopping(self):
        """
        :return: bool stopping
        """
        pass

    def getSimulationTime(self):
        """
        :return: float simulationTime
        """
        pass

    def getSimulationTimeStep(self):
        """
        :return: float timeStep
        """
        pass

    def getSimulatorMessage(self):
        """
        :return: int messageID, int[4] auxiliaryData, int[1..*] auxiliaryData2
        """
        pass

    def getStackTraceback(scriptHandle: int):
        """
        :param scriptHandle : Int
        :return: string stacktraceback
        """
        pass

    def getStringParam(parameter: int):
        """
        :param parameter : Int
        :return: string stringState
        """
        pass

    def getStringProperty(target: int, pName: str, options: map):
        """
        :param target : Int
        :param pName : String
        :param options : Map
        :return: string pValue
        """
        pass

    def getStringSignal(signalName: str):
        """
        :param signalName : String
        :return: string signalValue
        """
        pass

    def getSystemTime(self):
        """
        :return: float time
        """
        pass

    def getTableProperty(target: int, pName: str, options: map):
        """
        :param target : Int
        :param pName : String
        :param options : Map
        :return: map pValue
        """
        pass

    def getTextureId(textureName: str):
        """
        :param textureName : String
        :return: int textureId, int[2] resolution
        """
        pass

    def getThreadId(self):
        """
        :return: int threadId
        """
        pass

    def getUserVariables(self):
        """
        :return: string[] variables
        """
        pass

    def getVector2Property(target: int, pName: str, options: map):
        """
        :param target : Int
        :param pName : String
        :param options : Map
        :return: float[2] pValue
        """
        pass

    def getVector3Property(target: int, pName: str, options: map):
        """
        :param target : Int
        :param pName : String
        :param options : Map
        :return: float[3] pValue
        """
        pass

    def getVelocity(shapeHandle: int):
        """
        :param shapeHandle : Int
        :return: float[3] linearVelocity, float[3] angularVelocity
        """
        pass

    def getVisionSensorDepth(sensorHandle: int, options: int, pos: int, size: int):
        """
        :param sensorHandle : Int
        :param options : Int
        :param pos : Int[2]
        :param size : Int[2]
        :return: buffer depth, int[2] resolution
        """
        pass

    def getVisionSensorImg(sensorHandle: int, options: int, rgbaCutOff: float, pos: int, size: int):
        """
        :param sensorHandle : Int
        :param options : Int
        :param rgbaCutOff : Float
        :param pos : Int[2]
        :param size : Int[2]
        :return: buffer image, int[2] resolution
        """
        pass

    def getVisionSensorRes(sensorHandle: int):
        """
        :param sensorHandle : Int
        :return:
        """
        pass

    def groupShapes(shapeHandles: int, merge: bool):
        """
        :param shapeHandles : Int[]
        :param merge : Bool
        :return: int shapeHandle
        """
        pass

    def handleAddOnScripts(callType: int):
        """
        :param callType : Int
        :return: int count
        """
        pass

    def handleDynamics(deltaTime: float):
        """
        :param deltaTime : Float
        :return: int result
        """
        pass

    def handleEmbeddedScripts(callType: int):
        """
        :param callType : Int
        :return: int calledScripts
        """
        pass

    def handleExtCalls(self):
        """
        :return:
        """
        pass

    def handleGraph(objectHandle: int, simulationTime: float):
        """
        :param objectHandle : Int
        :param simulationTime : Float
        :return:
        """
        pass

    def handleJointMotion(self):
        """
        :return:
        """
        pass

    def handleProximitySensor(sensorHandle: int):
        """
        :param sensorHandle : Int
        :return: int result, float distance, float[3] detectedPoint, int detectedObjectHandle, float[3] normalVector
        """
        pass

    def handleSandboxScript(callType: int):
        """
        :param callType : Int
        :return:
        """
        pass

    def handleSensingStart(self):
        """
        :return:
        """
        pass

    def handleSimulationScripts(callType: int):
        """
        :param callType : Int
        :return: int calledScripts
        """
        pass

    def handleSimulationStart(self):
        """
        :return:
        """
        pass

    def handleVisionSensor(sensorHandle: int):
        """
        :param sensorHandle : Int
        :return: int detectionCount, float[] auxPacket1, float[] auxPacket2
        """
        pass

    def importMesh(fileformat: int, pathAndFilename: str, options: int, identicalVerticeTolerance: float,
                   scalingFactor: float):
        """
        :param fileformat : Int
        :param pathAndFilename : String
        :param options : Int
        :param identicalVerticeTolerance : Float
        :param scalingFactor : Float
        :return: float[1..*] vertices, int[1..*] indices
        """
        pass

    def importShape(fileformat: int, pathAndFilename: str, options: int, identicalVerticeTolerance: float,
                    scalingFactor: float):
        """
        :param fileformat : Int
        :param pathAndFilename : String
        :param options : Int
        :param identicalVerticeTolerance : Float
        :param scalingFactor : Float
        :return: int shapeHandle
        """
        pass

    def initScript(scriptHandle: int):
        """
        :param scriptHandle : Int
        :return:
        """
        pass

    def insertObjectIntoOctree(octreeHandle: int, objectHandle: int, options: int, color: float, tag: int):
        """
        :param octreeHandle : Int
        :param objectHandle : Int
        :param options : Int
        :param color : Float[]
        :param tag : Int
        :return: int totalVoxelCnt
        """
        pass

    def insertObjectIntoPointCloud(pointCloudHandle: int, objectHandle: int, options: int, gridSize: float,
                                   color: float, duplicateTolerance: float):
        """
        :param pointCloudHandle : Int
        :param objectHandle : Int
        :param options : Int
        :param gridSize : Float
        :param color : Float[]
        :param duplicateTolerance : Float
        :return: int totalPointCnt
        """
        pass

    def insertPointsIntoPointCloud(pointCloudHandle: int, options: int, points: float, color: float,
                                   duplicateTolerance: float):
        """
        :param pointCloudHandle : Int
        :param options : Int
        :param points : Float[]
        :param color : Float[]
        :param duplicateTolerance : Float
        :return: int totalPointCnt
        """
        pass

    def insertVoxelsIntoOctree(octreeHandle: int, options: int, points: float, color: float, tag: int):
        """
        :param octreeHandle : Int
        :param options : Int
        :param points : Float[]
        :param color : Float[]
        :param tag : Int[]
        :return: int totalVoxelCnt
        """
        pass

    def interpolateMatrices(matrixIn1: float, matrixIn2: float, interpolFactor: float):
        """
        :param matrixIn1 : Float[12]
        :param matrixIn2 : Float[12]
        :param interpolFactor : Float
        :return: float[12] resultMatrix
        """
        pass

    def interpolatePoses(poseIn1: float, poseIn2: float, interpolFactor: float):
        """
        :param poseIn1 : Float[7]
        :param poseIn2 : Float[7]
        :param interpolFactor : Float
        :return: float[7] resultPose
        """
        pass

    def intersectPointsWithPointCloud(pointCloudHandle: int, options: int, points: float, tolerance: float):
        """
        :param pointCloudHandle : Int
        :param options : Int
        :param points : Float[]
        :param tolerance : Float
        :return: int totalPointCnt
        """
        pass

    def isDeprecated(funcOrConst: str):
        """
        :param funcOrConst : String
        :return: int result
        """
        pass

    def isDynamicallyEnabled(objectHandle: int):
        """
        :param objectHandle : Int
        :return: bool enabled
        """
        pass

    def isHandle(objectHandle: int):
        """
        :param objectHandle : Int
        :return: bool result
        """
        pass

    def launchExecutable(filename: str, parameters: str, showStatus: int):
        """
        :param filename : String
        :param parameters : String
        :param showStatus : Int
        :return:
        """
        pass

    def loadImage(options: int, filename: str, resolution: int):
        """
        :param options : Int
        :param filename : String
        :return: buffer image, int[2] resolution
        """
        pass

    def loadModel(filename: str):
        """
        :param filename : String
        :return: int objectHandle
        """
        pass

    def loadScene(filename: str):
        """
        :param filename : String
        :return:
        """
        pass

    def matrixToPose(matrix: float):
        """
        :param matrix : Float[12]
        :return: float[7] pose
        """
        pass

    def moduleEntry(handle: int, label: str, state: int):
        """
        :param handle : Int
        :param label : String
        :param state : Int
        :return: int handle
        """
        pass

    def moveToConfig(params: map):
        """
        :param params : Map
        :return: map data
        """
        pass

    def moveToConfig_cleanup(motionObject: map):
        """
        :param motionObject : Map
        :return:
        """
        pass

    def moveToConfig_init(params: map):
        """
        :param params : Map
        :return: map motionObject
        """
        pass

    def moveToConfig_step(motionObject: map):
        """
        :param motionObject : Map
        :return: int res, map data
        """
        pass

    def moveToPose(params: map):
        """
        :param params : Map
        :return: map data
        """
        pass

    def moveToPose_cleanup(motionObject: map):
        """
        :param motionObject : Map
        :return:
        """
        pass

    def moveToPose_init(params: map):
        """
        :param params : Map
        :return: map motionObject
        """
        pass

    def moveToPose_step(motionObject: map):
        """
        :param motionObject : Map
        :return: int res, map data
        """
        pass

    def multiplyMatrices(matrixIn1: float, matrixIn2: float):
        """
        :param matrixIn1 : Float[12]
        :param matrixIn2 : Float[12]
        :return: float[12] resultMatrix
        """
        pass

    def multiplyPoses(poseIn1: float, poseIn2: float):
        """
        :param poseIn1 : Float[7]
        :param poseIn2 : Float[7]
        :return: float[7] resultPose
        """
        pass

    def multiplyVector(matrix: float, inVectors: float):
        """
        :param matrix : Float[12]
        :param inVectors : Float[]
        :return: float[] resultVectors
        """
        pass

    def packDoubleTable(doubleNumbers: float, startDoubleIndex: int, doubleCount: int):
        """
        :param doubleNumbers : Float[]
        :param startDoubleIndex : Int
        :param doubleCount : Int
        :return: buffer data
        """
        pass

    def packFloatTable(floatNumbers: float, startFloatIndex: int, floatCount: int):
        """
        :param floatNumbers : Float[]
        :param startFloatIndex : Int
        :param floatCount : Int
        :return: buffer data
        """
        pass

    def packInt32Table(int32Numbers: int, startInt32Index: int, int32Count: int):
        """
        :param int32Numbers : Int[]
        :param startInt32Index : Int
        :param int32Count : Int
        :return: buffer data
        """
        pass

    def packTable(aTable: any, scheme: int):
        """
        :param aTable : Any[]
        :param scheme : Int
        :return: buffer data
        """
        pass

    def packUInt16Table(uint16Numbers: int, startUint16Index: int, uint16Count: int):
        """
        :param uint16Numbers : Int[]
        :param startUint16Index : Int
        :param uint16Count : Int
        :return: buffer data
        """
        pass

    def packUInt32Table(uint32Numbers: int, startUInt32Index: int, uint32Count: int):
        """
        :param uint32Numbers : Int[]
        :param startUInt32Index : Int
        :param uint32Count : Int
        :return: buffer data
        """
        pass

    def packUInt8Table(uint8Numbers: int, startUint8Index: int, uint8count: int):
        """
        :param uint8Numbers : Int[]
        :param startUint8Index : Int
        :param uint8count : Int
        :return: buffer data
        """
        pass

    def pauseSimulation(self):
        """
        :return:
        """
        pass

    def poseToMatrix(pose: float):
        """
        :param pose : Float[7]
        :return: float[12] matrix
        """
        pass

    def pushUserEvent(event: str, handle: int, uid: int, eventData: map, options: int):
        """
        :param event : String
        :param handle : Int
        :param uid : Int
        :param eventData : Map
        :param options : Int
        :return:
        """
        pass

    def quitSimulator(self):
        """
        :return:
        """
        pass

    def readCustomBufferData(objectHandle: int, tagName: str):
        """
        :param objectHandle : Int
        :param tagName : String
        :return: buffer data
        """
        pass

    def readCustomDataTags(objectHandle: int):
        """
        :param objectHandle : Int
        :return: string[] tags
        """
        pass

    def readCustomStringData(objectHandle: int, tagName: str):
        """
        :param objectHandle : Int
        :param tagName : String
        :return: string data
        """
        pass

    def readCustomTableData(handle: int, tagName: str, options: map):
        """
        :param handle : Int
        :param tagName : String
        :param options : Map
        :return: map data
        """
        pass

    def readForceSensor(objectHandle: int):
        """
        :param objectHandle : Int
        :return: int result, float[3] forceVector, float[3] torqueVector
        """
        pass

    def readProximitySensor(sensorHandle: int):
        """
        :param sensorHandle : Int
        :return: int result, float distance, float[3] detectedPoint, int detectedObjectHandle, float[3] normalVector
        """
        pass

    def readTexture(textureId: int, options: int, posX: int, posY: int, sizeX: int, sizeY: int):
        """
        :param textureId : Int
        :param options : Int
        :param posX : Int
        :param posY : Int
        :param sizeX : Int
        :param sizeY : Int
        :return: buffer textureData
        """
        pass

    def readVisionSensor(sensorHandle: int):
        """
        :param sensorHandle : Int
        :return: int result, float[] auxPacket1, float[] auxPacket2
        """
        pass

    def refreshDialogs(refreshDegree: int):
        """
        :param refreshDegree : Int
        :return: int result
        """
        pass

    def releaseLock(self):
        """
        :return:
        """
        pass

    def relocateShapeFrame(shapeHandle: int, pose: float):
        """
        :param shapeHandle : Int
        :param pose : Float[7]
        :return: int result
        """
        pass

    def removeDrawingObject(drawingObjectHandle: int):
        """
        :param drawingObjectHandle : Int
        :return:
        """
        pass

    def removeModel(objectHandle: int, delayedRemoval: bool):
        """
        :param objectHandle : Int
        :param delayedRemoval : Bool
        :return: int objectCount
        """
        pass

    def removeObjects(objectHandles: list, delayedRemoval: bool):
        """
        :param objectHandles : Int[1..*]
        :param delayedRemoval : Bool
        :return:
        """
        pass

    def removeParticleObject(particleObjectHandle: int):
        """
        :param particleObjectHandle : Int
        :return:
        """
        pass

    def removePointsFromPointCloud(pointCloudHandle: int, options: int, points: float, tolerance: float):
        """
        :param pointCloudHandle : Int
        :param options : Int
        :param points : Float[]
        :param tolerance : Float
        :return: int totalPointCnt
        """
        pass

    def removeProperty(target: int, pName: str, options: map):
        """
        :param target : Int
        :param pName : String
        :param options : Map
        :return:
        """
        pass

    def removeReferencedObjects(objectHandle: int, tag: str):
        """
        :param objectHandle : Int
        :param tag : String
        :return:
        """
        pass

    def removeVoxelsFromOctree(octreeHandle: int, options: int, points: float):
        """
        :param octreeHandle : Int
        :param options : Int
        :param points : Float[]
        :return: int totalVoxelCnt
        """
        pass

    def resamplePath(path: float, pathLengths: float, finalConfigCnt: int, method: map, types: int):
        """
        :param path : Float[]
        :param pathLengths : Float[]
        :param finalConfigCnt : Int
        :param method : Map
        :param types : Int[]
        :return: float[] path
        """
        pass

    def resetDynamicObject(objectHandle: int):
        """
        :param objectHandle : Int
        :return:
        """
        pass

    def resetGraph(objectHandle: int):
        """
        :param objectHandle : Int
        :return:
        """
        pass

    def resetProximitySensor(objectHandle: int):
        """
        :param objectHandle : Int
        :return:
        """
        pass

    def resetVisionSensor(sensorHandle: int):
        """
        :param sensorHandle : Int
        :return:
        """
        pass

    def restoreEntityColor(originalColorData: map):
        """
        :param originalColorData : Map[]
        :return:
        """
        pass

    def rotateAroundAxis(matrixIn: float, axis: float, axisPos: float, angle: float):
        """
        :param matrixIn : Float[12]
        :param axis : Float[3]
        :param axisPos : Float[3]
        :param angle : Float
        :return: float[12] matrixOut
        """
        pass

    def ruckigPos(dofs: int, baseCycleTime: float, flags: int, currentPosVelAccel: float, maxVelAccelJerk: float,
                  selection: int, targetPosVel: float):
        """
        :param dofs : Int
        :param baseCycleTime : Float
        :param flags : Int
        :param currentPosVelAccel : Float[]
        :param maxVelAccelJerk : Float[]
        :param selection : Int[]
        :param targetPosVel : Float[]
        :return: int handle
        """
        pass

    def ruckigRemove(handle: int):
        """
        :param handle : Int
        :return:
        """
        pass

    def ruckigStep(handle: int, cycleTime: float):
        """
        :param handle : Int
        :param cycleTime : Float
        :return: int result, float[] newPosVelAccel, float synchronizationTime
        """
        pass

    def ruckigVel(dofs: int, baseCycleTime: float, flags: int, currentPosVelAccel: float, maxAccelJerk: float,
                  selection: int, targetVel: float):
        """
        :param dofs : Int
        :param baseCycleTime : Float
        :param flags : Int
        :param currentPosVelAccel : Float[]
        :param maxAccelJerk : Float[]
        :param selection : Int[]
        :param targetVel : Float[]
        :return: int handle
        """
        pass

    def saveImage(image: bytes, resolution: int, options: int, filename: str, quality: int):
        """
        :param image : Buffer
        :param resolution : Int[2]
        :param options : Int
        :param filename : String
        :param quality : Int
        :return: buffer serializedImage
        """
        pass

    def saveModel(modelBaseHandle: int, filename: str):
        """
        :param modelBaseHandle : Int
        :param filename : String
        :return:
        """
        pass

    def saveScene(filename: str):
        """
        :param filename : String
        :return:
        """
        pass

    def scaleObject(objectHandle: int, xScale: float, yScale: float, zScale: float, options: int):
        """
        :param objectHandle : Int
        :param xScale : Float
        :param yScale : Float
        :param zScale : Float
        :param options : Int
        :return:
        """
        pass

    def scaleObjects(objectHandles: list, scalingFactor: float, scalePositionsToo: bool):
        """
        :param objectHandles : Int[1..*]
        :param scalingFactor : Float
        :param scalePositionsToo : Bool
        :return:
        """
        pass

    def scheduleExecution(f: str, args: any, timePoint: float, simTime: bool):
        """
        :param f : Func
        :param args : Any[]
        :param timePoint : Float
        :param simTime : Bool
        :return: int id
        """
        pass

    def serialCheck(portHandle: int):
        """
        :param portHandle : Int
        :return: int byteCount
        """
        pass

    def serialClose(portHandle: int):
        """
        :param portHandle : Int
        :return:
        """
        pass

    def serialOpen(portString: str, baudrate: int):
        """
        :param portString : String
        :param baudrate : Int
        :return: int portHandle
        """
        pass

    def serialRead(portHandle: int, dataLengthToRead: int, blockingOperation: bool, closingString: bytes,
                   timeout: float):
        """
        :param portHandle : Int
        :param dataLengthToRead : Int
        :param blockingOperation : Bool
        :param closingString : Buffer
        :param timeout : Float
        :return: buffer data
        """
        pass

    def serialSend(portHandle: int, data: bytes):
        """
        :param portHandle : Int
        :param data : Buffer
        :return: int charsSent
        """
        pass

    def setArrayParam(parameter: int, arrayOfValues: float):
        """
        :param parameter : Int
        :param arrayOfValues : Float[3]
        :return:
        """
        pass

    def setAutoYieldDelay(dt: float):
        """
        :param dt : Float
        :return:
        """
        pass

    def setBoolParam(parameter: int, boolState: bool):
        """
        :param parameter : Int
        :param boolState : Bool
        :return:
        """
        pass

    def setBoolProperty(target: int, pName: str, pValue: bool, options: map):
        """
        :param target : Int
        :param pName : String
        :param pValue : Bool
        :param options : Map
        :return:
        """
        pass

    def setBufferProperty(target: int, pName: str, pValue: bytes, options: map):
        """
        :param target : Int
        :param pName : String
        :param pValue : Buffer
        :param options : Map
        :return:
        """
        pass

    def setBufferSignal(signalName: str, signalValue: bytes):
        """
        :param signalName : String
        :param signalValue : Buffer
        :return:
        """
        pass

    def setColorProperty(target: int, pName: str, pValue: float, options: map):
        """
        :param target : Int
        :param pName : String
        :param pValue : Float[3]
        :param options : Map
        :return:
        """
        pass

    def setEngineBoolParam(paramId: int, objectHandle: int, boolParam: bool):
        """
        :param paramId : Int
        :param objectHandle : Int
        :param boolParam : Bool
        :return:
        """
        pass

    def setEngineFloatParam(paramId: int, objectHandle: int, floatParam: float):
        """
        :param paramId : Int
        :param objectHandle : Int
        :param floatParam : Float
        :return:
        """
        pass

    def setEngineInt32Param(paramId: int, objectHandle: int, int32Param: int):
        """
        :param paramId : Int
        :param objectHandle : Int
        :param int32Param : Int
        :return:
        """
        pass

    def setEventFilters(filters: map):
        """
        :param filters : Map
        :return:
        """
        pass

    def setExplicitHandling(objectHandle: int, explicitHandlingFlags: int):
        """
        :param objectHandle : Int
        :param explicitHandlingFlags : Int
        :return:
        """
        pass

    def setFloatArrayProperty(target: int, pName: str, pValue: float, options: map):
        """
        :param target : Int
        :param pName : String
        :param pValue : Float[]
        :param options : Map
        :return:
        """
        pass

    def setFloatParam(parameter: int, floatState: float):
        """
        :param parameter : Int
        :param floatState : Float
        :return:
        """
        pass

    def setFloatProperty(target: int, pName: str, pValue: float, options: map):
        """
        :param target : Int
        :param pName : String
        :param pValue : Float
        :param options : Map
        :return:
        """
        pass

    def setFloatSignal(signalName: str, signalValue: float):
        """
        :param signalName : String
        :param signalValue : Float
        :return:
        """
        pass

    def setGraphStreamTransformation(graphHandle: int, streamId: int, trType: int, mult: float, off: float,
                                     movAvgPeriod: int):
        """
        :param graphHandle : Int
        :param streamId : Int
        :param trType : Int
        :param mult : Float
        :param off : Float
        :param movAvgPeriod : Int
        :return:
        """
        pass

    def setGraphStreamValue(graphHandle: int, streamId: int, value: float):
        """
        :param graphHandle : Int
        :param streamId : Int
        :param value : Float
        :return:
        """
        pass

    def setInt32Param(parameter: int, intState: int):
        """
        :param parameter : Int
        :param intState : Int
        :return:
        """
        pass

    def setInt32Signal(signalName: str, signalValue: int):
        """
        :param signalName : String
        :param signalValue : Int
        :return:
        """
        pass

    def setIntArray2Property(target: int, pName: str, pValue: int, options: map):
        """
        :param target : Int
        :param pName : String
        :param pValue : Int[2]
        :param options : Map
        :return:
        """
        pass

    def setIntArrayProperty(target: int, pName: str, pValue: int, options: map):
        """
        :param target : Int
        :param pName : String
        :param pValue : Int[]
        :param options : Map
        :return:
        """
        pass

    def setIntProperty(target: int, pName: str, pValue: int, options: map):
        """
        :param target : Int
        :param pName : String
        :param pValue : Int
        :param options : Map
        :return:
        """
        pass

    def setJointDependency(jointHandle: int, masterJointHandle: int, offset: float, multCoeff: float):
        """
        :param jointHandle : Int
        :param masterJointHandle : Int
        :param offset : Float
        :param multCoeff : Float
        :return:
        """
        pass

    def setJointInterval(objectHandle: int, cyclic: bool, interval: float):
        """
        :param objectHandle : Int
        :param cyclic : Bool
        :param interval : Float[2]
        :return:
        """
        pass

    def setJointMode(jointHandle: int, jointMode: int, options: int):
        """
        :param jointHandle : Int
        :param jointMode : Int
        :param options : Int
        :return:
        """
        pass

    def setJointPosition(objectHandle: int, position: float):
        """
        :param objectHandle : Int
        :param position : Float
        :return:
        """
        pass

    def setJointTargetForce(objectHandle: int, forceOrTorque: float, signedValue: bool):
        """
        :param objectHandle : Int
        :param forceOrTorque : Float
        :param signedValue : Bool
        :return:
        """
        pass

    def setJointTargetPosition(objectHandle: int, targetPosition: float, motionParams: float):
        """
        :param objectHandle : Int
        :param targetPosition : Float
        :param motionParams : Float[]
        :return:
        """
        pass

    def setJointTargetVelocity(objectHandle: int, targetVelocity: float, motionParams: float):
        """
        :param objectHandle : Int
        :param targetVelocity : Float
        :param motionParams : Float[]
        :return:
        """
        pass

    def setLightParameters(lightHandle: int, state: int, reserved: float, diffusePart: float, specularPart: float):
        """
        :param lightHandle : Int
        :param state : Int
        :param reserved : Float[3]
        :param diffusePart : Float[3]
        :param specularPart : Float[3]
        :return:
        """
        pass

    def setLinkDummy(dummyHandle: int, linkDummyHandle: int):
        """
        :param dummyHandle : Int
        :param linkDummyHandle : Int
        :return:
        """
        pass

    def setLongProperty(target: int, pName: str, pValue: int, options: map):
        """
        :param target : Int
        :param pName : String
        :param pValue : Int
        :param options : Map
        :return:
        """
        pass

    def setModelProperty(objectHandle: int, property: int):
        """
        :param objectHandle : Int
        :param property : Int
        :return:
        """
        pass

    def setNamedBoolParam(name: str, value: bool):
        """
        :param name : String
        :param value : Bool
        :return:
        """
        pass

    def setNamedFloatParam(name: str, value: float):
        """
        :param name : String
        :param value : Float
        :return:
        """
        pass

    def setNamedInt32Param(name: str, value: int):
        """
        :param name : String
        :param value : Int
        :return:
        """
        pass

    def setNamedStringParam(paramName: str, stringParam: bytes):
        """
        :param paramName : String
        :param stringParam : Buffer
        :return:
        """
        pass

    def setNavigationMode(navigationMode: int):
        """
        :param navigationMode : Int
        :return:
        """
        pass

    def setObjectAlias(objectHandle: int, objectAlias: str):
        """
        :param objectHandle : Int
        :param objectAlias : String
        :return:
        """
        pass

    def setObjectChildPose(objectHandle: int, pose: float):
        """
        :param objectHandle : Int
        :param pose : Float[7]
        :return:
        """
        pass

    def setObjectColor(objectHandle: int, index: int, colorComponent: int, rgbData: float):
        """
        :param objectHandle : Int
        :param index : Int
        :param colorComponent : Int
        :param rgbData : Float[3]
        :return: bool result
        """
        pass

    def setObjectFloatArrayParam(objectHandle: int, parameterID: int, params: float):
        """
        :param objectHandle : Int
        :param parameterID : Int
        :param params : Float[]
        :return:
        """
        pass

    def setObjectFloatParam(objectHandle: int, parameterID: int, parameter: float):
        """
        :param objectHandle : Int
        :param parameterID : Int
        :param parameter : Float
        :return:
        """
        pass

    def setObjectHierarchyOrder(objectHandle: int, order: int):
        """
        :param objectHandle : Int
        :param order : Int
        :return:
        """
        pass

    def setObjectInt32Param(objectHandle: int, parameterID: int, parameter: int):
        """
        :param objectHandle : Int
        :param parameterID : Int
        :param parameter : Int
        :return:
        """
        pass

    def setObjectMatrix(objectHandle: int, matrix: float, relativeToObjectHandle: int):
        """
        :param objectHandle : Int
        :param matrix : Float[12]
        :param relativeToObjectHandle : Int
        :return:
        """
        pass

    def setObjectOrientation(objectHandle: int, eulerAngles: float, relativeToObjectHandle: int):
        """
        :param objectHandle : Int
        :param eulerAngles : Float[3]
        :param relativeToObjectHandle : Int
        :return:
        """
        pass

    def setObjectParent(objectHandle: int, parentObjectHandle: int, keepInPlace: bool):
        """
        :param objectHandle : Int
        :param parentObjectHandle : Int
        :param keepInPlace : Bool
        :return:
        """
        pass

    def setObjectPose(objectHandle: int, pose: float, relativeToObjectHandle: int):
        """
        :param objectHandle : Int
        :param pose : Float[7]
        :param relativeToObjectHandle : Int
        :return:
        """
        pass

    def setObjectPosition(objectHandle: int, position: float, relativeToObjectHandle: int):
        """
        :param objectHandle : Int
        :param position : Float[3]
        :param relativeToObjectHandle : Int
        :return:
        """
        pass

    def setObjectProperty(objectHandle: int, property: int):
        """
        :param objectHandle : Int
        :param property : Int
        :return:
        """
        pass

    def setObjectQuaternion(objectHandle: int, quaternion: float, relativeToObjectHandle: int):
        """
        :param objectHandle : Int
        :param quaternion : Float[4]
        :param relativeToObjectHandle : Int
        :return:
        """
        pass

    def setObjectSel(objectHandles: int):
        """
        :param objectHandles : Int[]
        :return:
        """
        pass

    def setObjectSpecialProperty(objectHandle: int, property: int):
        """
        :param objectHandle : Int
        :param property : Int
        :return:
        """
        pass

    def setObjectStringParam(objectHandle: int, parameterID: int, parameter: bytes):
        """
        :param objectHandle : Int
        :param parameterID : Int
        :param parameter : Buffer
        :return:
        """
        pass

    def setPage(pageIndex: int):
        """
        :param pageIndex : Int
        :return:
        """
        pass

    def setPluginInfo(pluginName: str, infoType: int, info: str):
        """
        :param pluginName : String
        :param infoType : Int
        :param info : String
        :return:
        """
        pass

    def setPointCloudOptions(pointCloudHandle: int, maxVoxelSize: float, maxPtCntPerVoxel: int, options: int,
                             pointSize: float):
        """
        :param pointCloudHandle : Int
        :param maxVoxelSize : Float
        :param maxPtCntPerVoxel : Int
        :param options : Int
        :param pointSize : Float
        :return:
        """
        pass

    def setPoseProperty(target: int, pName: str, pValue: float, options: map):
        """
        :param target : Int
        :param pName : String
        :param pValue : Float[7]
        :param options : Map
        :return:
        """
        pass

    def setProperties(target: int, props: map):
        """
        :param target : Int
        :param props : Map
        :return:
        """
        pass

    def setProperty(target: int, pName: str, pValue: any, pType: int):
        """
        :param target : Int
        :param pName : String
        :param pValue : Any
        :param pType : Int
        :return:
        """
        pass

    def setQuaternionProperty(target: int, pName: str, pValue: float, options: map):
        """
        :param target : Int
        :param pName : String
        :param pValue : Float[4]
        :param options : Map
        :return:
        """
        pass

    def setReferencedHandles(objectHandle: int, referencedHandles: int, tag: str):
        """
        :param objectHandle : Int
        :param referencedHandles : Int[]
        :param tag : String
        :return:
        """
        pass

    def setShapeAppearance(handle: int, savedData: map, opts: map):
        """
        :param handle : Int
        :param savedData : Map
        :param opts : Map
        :return: int handle
        """
        pass

    def setShapeBB(shapeHandle: int, size: float):
        """
        :param shapeHandle : Int
        :param size : Float[3]
        :return:
        """
        pass

    def setShapeColor(shapeHandle: int, colorName: str, colorComponent: int, rgbData: float):
        """
        :param shapeHandle : Int
        :param colorName : String
        :param colorComponent : Int
        :param rgbData : Float[3]
        :return:
        """
        pass

    def setShapeInertia(shapeHandle: int, inertiaMatrix: float, comMatrix: float):
        """
        :param shapeHandle : Int
        :param inertiaMatrix : Float[9]
        :param comMatrix : Float[12]
        :return:
        """
        pass

    def setShapeMass(shapeHandle: int, mass: float):
        """
        :param shapeHandle : Int
        :param mass : Float
        :return:
        """
        pass

    def setShapeMaterial(shapeHandle: int, materialIdOrShapeHandle: int):
        """
        :param shapeHandle : Int
        :param materialIdOrShapeHandle : Int
        :return:
        """
        pass

    def setShapeTexture(shapeHandle: int, textureId: int, mappingMode: int, options: int, uvScaling: float,
                        position: float, orientation: float):
        """
        :param shapeHandle : Int
        :param textureId : Int
        :param mappingMode : Int
        :param options : Int
        :param uvScaling : Float[2]
        :param position : Float[3]
        :param orientation : Float[3]
        :return:
        """
        pass

    def setStepping(enabled: bool):
        """
        :param enabled : Bool
        :return: int prevStepLevel
        """
        pass

    def setStringParam(parameter: int, stringState: str):
        """
        :param parameter : Int
        :param stringState : String
        :return:
        """
        pass

    def setStringProperty(target: int, pName: str, pValue: str, options: map):
        """
        :param target : Int
        :param pName : String
        :param pValue : String
        :param options : Map
        :return:
        """
        pass

    def setStringSignal(signalName: str, signalValue: str):
        """
        :param signalName : String
        :param signalValue : String
        :return:
        """
        pass

    def setTableProperty(target: int, pName: str, pValue: map, options: map):
        """
        :param target : Int
        :param pName : String
        :param pValue : Map
        :param options : Map
        :return:
        """
        pass

    def setVector2Property(target: int, pName: str, pValue: float, options: map):
        """
        :param target : Int
        :param pName : String
        :param pValue : Float[2]
        :param options : Map
        :return:
        """
        pass

    def setVector3Property(target: int, pName: str, pValue: float, options: map):
        """
        :param target : Int
        :param pName : String
        :param pValue : Float[3]
        :param options : Map
        :return:
        """
        pass

    def setVisionSensorImg(sensorHandle: int, image: bytes, options: int, pos: int, size: int):
        """
        :param sensorHandle : Int
        :param image : Buffer
        :param options : Int
        :param pos : Int[2]
        :param size : Int[2]
        :return:
        """
        pass

    def startSimulation(self):
        """
        :return:
        """
        pass

    def step(self):
        """
        :return:
        """
        pass

    def stopSimulation(wait: bool):
        """
        :param wait : Bool
        :return:
        """
        pass

    def subtractObjectFromOctree(octreeHandle: int, objectHandle: int, options: int):
        """
        :param octreeHandle : Int
        :param objectHandle : Int
        :param options : Int
        :return: int totalVoxelCnt
        """
        pass

    def subtractObjectFromPointCloud(pointCloudHandle: int, objectHandle: int, options: int, tolerance: float):
        """
        :param pointCloudHandle : Int
        :param objectHandle : Int
        :param options : Int
        :param tolerance : Float
        :return: int totalPointCnt
        """
        pass

    def systemSemaphore(key: str, acquire: bool):
        """
        :param key : String
        :param acquire : Bool
        :return:
        """
        pass

    def testCB(a: int, cb: str, b: int):
        """
        :param a : Int
        :param cb : Func
        :param b : Int
        :return: int ret
        """
        pass

    def textEditorClose(handle: int):
        """
        :param handle : Int
        :return: string text, int[2] pos, int[2] size
        """
        pass

    def textEditorGetInfo(handle: int):
        """
        :param handle : Int
        :return: string text, int[2] pos, int[2] size, bool visible
        """
        pass

    def textEditorOpen(initText: str, properties: str):
        """
        :param initText : String
        :param properties : String
        :return: int handle
        """
        pass

    def textEditorShow(handle: int, showState: bool):
        """
        :param handle : Int
        :param showState : Bool
        :return:
        """
        pass

    def transformBuffer(inBuffer: bytes, inFormat: int, multiplier: float, offset: float, outFormat: int):
        """
        :param inBuffer : Buffer
        :param inFormat : Int
        :param multiplier : Float
        :param offset : Float
        :param outFormat : Int
        :return: buffer outBuffer
        """
        pass

    def transformImage(image: bytes, resolution: int, options: int):
        """
        :param image : Buffer
        :param resolution : Int[2]
        :param options : Int
        :return:
        """
        pass

    def ungroupShape(shapeHandle: int):
        """
        :param shapeHandle : Int
        :return: int[] simpleShapeHandles
        """
        pass

    def unpackDoubleTable(data: bytes, startDoubleIndex: int, doubleCount: int, additionalByteOffset: int):
        """
        :param data : Buffer
        :param startDoubleIndex : Int
        :param doubleCount : Int
        :param additionalByteOffset : Int
        :return: float[] doubleNumbers
        """
        pass

    def unpackFloatTable(data: bytes, startFloatIndex: int, floatCount: int, additionalByteOffset: int):
        """
        :param data : Buffer
        :param startFloatIndex : Int
        :param floatCount : Int
        :param additionalByteOffset : Int
        :return: float[] floatNumbers
        """
        pass

    def unpackInt32Table(data: bytes, startInt32Index: int, int32Count: int, additionalByteOffset: int):
        """
        :param data : Buffer
        :param startInt32Index : Int
        :param int32Count : Int
        :param additionalByteOffset : Int
        :return: int[] int32Numbers
        """
        pass

    def unpackTable(buffer: bytes):
        """
        :param buffer : Buffer
        :return: any aTable
        """
        pass

    def unpackUInt16Table(data: bytes, startUint16Index: int, uint16Count: int, additionalByteOffset: int):
        """
        :param data : Buffer
        :param startUint16Index : Int
        :param uint16Count : Int
        :param additionalByteOffset : Int
        :return: int[] uint16Numbers
        """
        pass

    def unpackUInt32Table(data: bytes, startUint32Index: int, uint32Count: int, additionalByteOffset: int):
        """
        :param data : Buffer
        :param startUint32Index : Int
        :param uint32Count : Int
        :param additionalByteOffset : Int
        :return: int[] uint32Numbers
        """
        pass

    def unpackUInt8Table(data: bytes, startUint8Index: int, uint8count: int):
        """
        :param data : Buffer
        :param startUint8Index : Int
        :param uint8count : Int
        :return: int[] uint8Numbers
        """
        pass

    def visitTree(rootHandle: int, visitorFunc: str, options: map):
        """
        :param rootHandle : Int
        :param visitorFunc : Func
        :param options : Map
        :return:
        """
        pass

    def wait(dt: float, simulationTime: bool):
        """
        :param dt : Float
        :param simulationTime : Bool
        :return: float timeLeft
        """
        pass

    def waitForSignal(target: int, sigName: str):
        """
        :param target : Int
        :param sigName : String
        :return: any sigVal
        """
        pass

    def writeCustomBufferData(objectHandle: int, tagName: str, data: bytes):
        """
        :param objectHandle : Int
        :param tagName : String
        :param data : Buffer
        :return:
        """
        pass

    def writeCustomStringData(objectHandle: int, tagName: str, data: str):
        """
        :param objectHandle : Int
        :param tagName : String
        :param data : String
        :return:
        """
        pass

    def writeCustomTableData(handle: int, tagName: str, theTable: map, options: map):
        """
        :param handle : Int
        :param tagName : String
        :param theTable : Map
        :param options : Map
        :return:
        """
        pass

    def writeTexture(textureId: int, options: int, textureData: bytes, posX: int, posY: int, sizeX: int, sizeY: int,
                     interpol: float):
        """
        :param textureId : Int
        :param options : Int
        :param textureData : Buffer
        :param posX : Int
        :param posY : Int
        :param sizeX : Int
        :param sizeY : Int
        :param interpol : Float
        :return:
        """
        pass

    def yawPitchRollToAlphaBetaGamma(yawAngle: float, pitchAngle: float, rollAngle: float):
        """
        :param yawAngle : Float
        :param pitchAngle : Float
        :param rollAngle : Float
        :return: float alphaAngle, float betaAngle, float gammaAngle
        """
        pass

class SimIKType:
    def __init__(self):
        pass

    def addElement(environmentHandle: int, ikGroupHandle: int, tipDummyHandle: int):
        """
        :param environmentHandle : Int
        :param ikGroupHandle : Int
        :param tipDummyHandle : Int
        :return: int elementHandle
        """
        pass

    def addElementFromScene(environmentHandle: int, ikGroup: int, baseHandle: int, tipHandle: int, targetHandle: int,
                            constraints: int):
        """
        :param environmentHandle : Int
        :param ikGroup : Int
        :param baseHandle : Int
        :param tipHandle : Int
        :param targetHandle : Int
        :param constraints : Int
        :return: int ikElement, map simToIkMap, map ikToSimMap
        """
        pass

    def computeGroupJacobian(environmentHandle: int, ikGroupHandle: int):
        """
        :param environmentHandle : Int
        :param ikGroupHandle : Int
        :return: float[] jacobian, float[] errorVector
        """
        pass

    def computeJacobian(environmentHandle: int, baseObject: int, lastJoint: int, constraints: int, tipMatrix: list, targetMatrix: list, constrBaseMatrix: list):
        """
        :param environmentHandle : Int
        :param baseObject : Int
        :param lastJoint : Int
        :param constraints : Int
        :param tipMatrix : Float[7..12]
        :param targetMatrix : Float[7..12]
        :param constrBaseMatrix : Float[7..12]
        :return: float[] jacobian, float[] errorVector
        """
        pass

    def createDebugOverlay(environmentHandle: int, tipHandle: int, baseHandle: int):
        """
        :param environmentHandle : Int
        :param tipHandle : Int
        :param baseHandle : Int
        :return: int debugObject
        """
        pass

    def createDummy(environmentHandle: int, dummyName: str):
        """
        :param environmentHandle : Int
        :param dummyName : String
        :return: int dummyHandle
        """
        pass

    def createEnvironment(flags: int):
        """
        :param flags : Int
        :return: int environmentHandle
        """
        pass

    def createGroup(environmentHandle: int, ikGroupName: str):
        """
        :param environmentHandle : Int
        :param ikGroupName : String
        :return: int ikGroupHandle
        """
        pass

    def createJoint(environmentHandle: int, jointType: int, jointName: str):
        """
        :param environmentHandle : Int
        :param jointType : Int
        :param jointName : String
        :return: int jointHandle
        """
        pass

    def doesGroupExist(environmentHandle: int, ikGroupName: str):
        """
        :param environmentHandle : Int
        :param ikGroupName : String
        :return: bool result
        """
        pass

    def doesObjectExist(environmentHandle: int, objectName: str):
        """
        :param environmentHandle : Int
        :param objectName : String
        :return: bool result
        """
        pass

    def duplicateEnvironment(environmentHandle: int):
        """
        :param environmentHandle : Int
        :return: int duplicateEnvHandle
        """
        pass

    def eraseDebugOverlay(debugObject: int):
        """
        :param debugObject : Int
        :return:
        """
        pass

    def eraseEnvironment(environmentHandle: int):
        """
        :param environmentHandle : Int
        :return:
        """
        pass

    def eraseObject(environmentHandle: int, objectHandle: int):
        """
        :param environmentHandle : Int
        :param objectHandle : Int
        :return:
        """
        pass

    def findConfigs(envHandle: int, ikGroupHandle: int, jointHandles: int, params: map, configs: any):
        """
        :param envHandle : Int
        :param ikGroupHandle : Int
        :param jointHandles : Int[]
        :param params : Map
        :param configs : Any[]
        :return: any[] configs
        """
        pass

    def generatePath(environmentHandle: int, ikGroupHandle: int, jointHandles: int, tipHandle: int, pathPointCount: int,
                     validationCallback: str, auxData: any):
        """
        :param environmentHandle : Int
        :param ikGroupHandle : Int
        :param jointHandles : Int[]
        :param tipHandle : Int
        :param pathPointCount : Int
        :param validationCallback : Func
        :param auxData : Any
        :return: float[] path
        """
        pass

    def getAlternateConfigs(environmentHandle: int, jointHandles: int, lowLimits: float, ranges: float):
        """
        :param environmentHandle : Int
        :param jointHandles : Int[]
        :param lowLimits : Float[]
        :param ranges : Float[]
        :return: float[] configs
        """
        pass

    def getElementBase(environmentHandle: int, ikGroupHandle: int, elementHandle: int):
        """
        :param environmentHandle : Int
        :param ikGroupHandle : Int
        :param elementHandle : Int
        :return: int baseHandle, int constraintsBaseHandle
        """
        pass

    def getElementConstraints(environmentHandle: int, ikGroupHandle: int, elementHandle: int):
        """
        :param environmentHandle : Int
        :param ikGroupHandle : Int
        :param elementHandle : Int
        :return: int constraints
        """
        pass

    def getElementFlags(environmentHandle: int, ikGroupHandle: int, elementHandle: int):
        """
        :param environmentHandle : Int
        :param ikGroupHandle : Int
        :param elementHandle : Int
        :return: int flags
        """
        pass

    def getElementPrecision(environmentHandle: int, ikGroupHandle: int, elementHandle: int):
        """
        :param environmentHandle : Int
        :param ikGroupHandle : Int
        :param elementHandle : Int
        :return: float[2] precision
        """
        pass

    def getElementWeights(environmentHandle: int, ikGroupHandle: int, elementHandle: int):
        """
        :param environmentHandle : Int
        :param ikGroupHandle : Int
        :param elementHandle : Int
        :return: float[2] weights
        """
        pass

    def getFailureDescription(reason: int):
        """
        :param reason : Int
        :return: string description
        """
        pass

    def getGroupCalculation(environmentHandle: int, ikGroupHandle: int):
        """
        :param environmentHandle : Int
        :param ikGroupHandle : Int
        :return: int method, float damping, int maxIterations
        """
        pass

    def getGroupFlags(environmentHandle: int, ikGroupHandle: int):
        """
        :param environmentHandle : Int
        :param ikGroupHandle : Int
        :return: int flags
        """
        pass

    def getGroupHandle(environmentHandle: int, ikGroupName: str):
        """
        :param environmentHandle : Int
        :param ikGroupName : String
        :return: int ikGroupHandle
        """
        pass

    def getGroupJointLimitHits(environmentHandle: int, ikGroupHandle: int):
        """
        :param environmentHandle : Int
        :param ikGroupHandle : Int
        :return: int[] jointHandles, float[] underOrOvershots
        """
        pass

    def getGroupJoints(environmentHandle: int, ikGroupHandle: int):
        """
        :param environmentHandle : Int
        :param ikGroupHandle : Int
        :return: int[] jointHandles
        """
        pass

    def getJointDependency(environmentHandle: int, jointHandle: int):
        """
        :param environmentHandle : Int
        :param jointHandle : Int
        :return: int depJointHandle, float offset, float mult
        """
        pass

    def getJointInterval(environmentHandle: int, jointHandle: int):
        """
        :param environmentHandle : Int
        :param jointHandle : Int
        :return: bool cyclic, float[2] interval
        """
        pass

    def getJointLimitMargin(environmentHandle: int, jointHandle: int):
        """
        :param environmentHandle : Int
        :param jointHandle : Int
        :return: float margin
        """
        pass

    def getJointMatrix(environmentHandle: int, jointHandle: int):
        """
        :param environmentHandle : Int
        :param jointHandle : Int
        :return: float[12] matrix
        """
        pass

    def getJointMaxStepSize(environmentHandle: int, jointHandle: int):
        """
        :param environmentHandle : Int
        :param jointHandle : Int
        :return: float stepSize
        """
        pass

    def getJointMode(environmentHandle: int, jointHandle: int):
        """
        :param environmentHandle : Int
        :param jointHandle : Int
        :return: int jointMode
        """
        pass

    def getJointPosition(environmentHandle: int, jointHandle: int):
        """
        :param environmentHandle : Int
        :param jointHandle : Int
        :return: float position
        """
        pass

    def getJointScrewLead(environmentHandle: int, jointHandle: int):
        """
        :param environmentHandle : Int
        :param jointHandle : Int
        :return: float lead
        """
        pass

    def getJointTransformation(environmentHandle: int, jointHandle: int):
        """
        :param environmentHandle : Int
        :param jointHandle : Int
        :return: float[3] position, float[4] quaternion, float[3] euler
        """
        pass

    def getJointType(environmentHandle: int, jointHandle: int):
        """
        :param environmentHandle : Int
        :param jointHandle : Int
        :return: int jointType
        """
        pass

    def getJointWeight(environmentHandle: int, jointHandle: int):
        """
        :param environmentHandle : Int
        :param jointHandle : Int
        :return: float weight
        """
        pass

    def getObjectHandle(environmentHandle: int, objectName: str):
        """
        :param environmentHandle : Int
        :param objectName : String
        :return: int objectHandle
        """
        pass

    def getObjectMatrix(environmentHandle: int, objectHandle: int, relativeToObjectHandle: int):
        """
        :param environmentHandle : Int
        :param objectHandle : Int
        :param relativeToObjectHandle : Int
        :return: float[12] matrix
        """
        pass

    def getObjectParent(environmentHandle: int, objectHandle: int):
        """
        :param environmentHandle : Int
        :param objectHandle : Int
        :return: int parentObjectHandle
        """
        pass

    def getObjectPose(environmentHandle: int, objectHandle: int, relativeToObjectHandle: int):
        """
        :param environmentHandle : Int
        :param objectHandle : Int
        :param relativeToObjectHandle : Int
        :return: float[7] pose
        """
        pass

    def getObjectTransformation(environmentHandle: int, objectHandle: int, relativeToObjectHandle: int):
        """
        :param environmentHandle : Int
        :param objectHandle : Int
        :param relativeToObjectHandle : Int
        :return: float[3] position, float[4] quaternion, float[3] euler
        """
        pass

    def getObjectType(environmentHandle: int, objectHandle: int):
        """
        :param environmentHandle : Int
        :param objectHandle : Int
        :return: int objectType
        """
        pass

    def getObjects(environmentHandle: int, index: int):
        """
        :param environmentHandle : Int
        :param index : Int
        :return: int objectHandle, string objectName, bool isJoint, int jointType
        """
        pass

    def getTargetDummy(environmentHandle: int, dummyHandle: int):
        """
        :param environmentHandle : Int
        :param dummyHandle : Int
        :return: int targetDummyHandle
        """
        pass

    def handleGroup(environmentHandle: int, ikGroup: int, options: map):
        """
        :param environmentHandle : Int
        :param ikGroup : Int
        :param options : Map
        :return: int success, int flags, float[2] precision
        """
        pass

    def handleGroups(environmentHandle: int, ikGroups: int, options: map):
        """
        :param environmentHandle : Int
        :param ikGroups : Int[]
        :param options : Map
        :return: int success, int flags, float[2] precision
        """
        pass

    def load(environmentHandle: int, data: str):
        """
        :param environmentHandle : Int
        :param data : String
        :return:
        """
        pass

    def save(environmentHandle: int):
        """
        :param environmentHandle : Int
        :return: string data
        """
        pass

    def setElementBase(environmentHandle: int, ikGroupHandle: int, elementHandle: int, baseHandle: int,
                       constraintsBaseHandle: int):
        """
        :param environmentHandle : Int
        :param ikGroupHandle : Int
        :param elementHandle : Int
        :param baseHandle : Int
        :param constraintsBaseHandle : Int
        :return:
        """
        pass

    def setElementConstraints(environmentHandle: int, ikGroupHandle: int, elementHandle: int, constraints: int):
        """
        :param environmentHandle : Int
        :param ikGroupHandle : Int
        :param elementHandle : Int
        :param constraints : Int
        :return:
        """
        pass

    def setElementFlags(environmentHandle: int, ikGroupHandle: int, elementHandle: int, flags: int):
        """
        :param environmentHandle : Int
        :param ikGroupHandle : Int
        :param elementHandle : Int
        :param flags : Int
        :return:
        """
        pass

    def setElementPrecision(environmentHandle: int, ikGroupHandle: int, elementHandle: int, precision: float):
        """
        :param environmentHandle : Int
        :param ikGroupHandle : Int
        :param elementHandle : Int
        :param precision : Float[2]
        :return:
        """
        pass

    def setElementWeights(environmentHandle: int, ikGroupHandle: int, elementHandle: int, weights: float):
        """
        :param environmentHandle : Int
        :param ikGroupHandle : Int
        :param elementHandle : Int
        :param weights : Float[2]
        :return:
        """
        pass

    def setGroupCalculation(environmentHandle: int, ikGroupHandle: int, method: int, damping: float,
                            maxIterations: int):
        """
        :param environmentHandle : Int
        :param ikGroupHandle : Int
        :param method : Int
        :param damping : Float
        :param maxIterations : Int
        :return:
        """
        pass

    def setGroupFlags(environmentHandle: int, ikGroupHandle: int, flags: int):
        """
        :param environmentHandle : Int
        :param ikGroupHandle : Int
        :param flags : Int
        :return:
        """
        pass

    def setJointDependency(environmentHandle: int, jointHandle: int, masterJointHandle: int, offset: float, mult: float,
                           callback: str):
        """
        :param environmentHandle : Int
        :param jointHandle : Int
        :param masterJointHandle : Int
        :param offset : Float
        :param mult : Float
        :param callback : Func
        :return:
        """
        pass

    def setJointInterval(environmentHandle: int, jointHandle: int, cyclic: bool, interval: float):
        """
        :param environmentHandle : Int
        :param jointHandle : Int
        :param cyclic : Bool
        :param interval : Float[2]
        :return:
        """
        pass

    def setJointLimitMargin(environmentHandle: int, jointHandle: int, margin: float):
        """
        :param environmentHandle : Int
        :param jointHandle : Int
        :param margin : Float
        :return:
        """
        pass

    def setJointMaxStepSize(environmentHandle: int, jointHandle: int, stepSize: float):
        """
        :param environmentHandle : Int
        :param jointHandle : Int
        :param stepSize : Float
        :return:
        """
        pass

    def setJointMode(environmentHandle: int, jointHandle: int, jointMode: int):
        """
        :param environmentHandle : Int
        :param jointHandle : Int
        :param jointMode : Int
        :return:
        """
        pass

    def setJointPosition(environmentHandle: int, jointHandle: int, position: float):
        """
        :param environmentHandle : Int
        :param jointHandle : Int
        :param position : Float
        :return:
        """
        pass

    def setJointScrewLead(environmentHandle: int, jointHandle: int, lead: float):
        """
        :param environmentHandle : Int
        :param jointHandle : Int
        :param lead : Float
        :return:
        """
        pass

    def setJointWeight(environmentHandle: int, jointHandle: int, weight: float):
        """
        :param environmentHandle : Int
        :param jointHandle : Int
        :param weight : Float
        :return:
        """
        pass

    def setObjectMatrix(environmentHandle: int, objectHandle: int, matrix: float, relativeToObjectHandle: int):
        """
        :param environmentHandle : Int
        :param objectHandle : Int
        :param matrix : Float[12]
        :param relativeToObjectHandle : Int
        :return:
        """
        pass

    def setObjectParent(environmentHandle: int, objectHandle: int, parentObjectHandle: int, keepInPlace: bool):
        """
        :param environmentHandle : Int
        :param objectHandle : Int
        :param parentObjectHandle : Int
        :param keepInPlace : Bool
        :return:
        """
        pass

    def setObjectPose(environmentHandle: int, objectHandle: int, pose: float, relativeToObjectHandle: int):
        """
        :param environmentHandle : Int
        :param objectHandle : Int
        :param pose : Float[7]
        :param relativeToObjectHandle : Int
        :return:
        """
        pass

    def setObjectTransformation(environmentHandle: int, objectHandle: int, position: float, eulerOrQuaternion: float,
                                relativeToObjectHandle: int):
        """
        :param environmentHandle : Int
        :param objectHandle : Int
        :param position : Float[3]
        :param eulerOrQuaternion : Float[]
        :param relativeToObjectHandle : Int
        :return:
        """
        pass

    def setSphericalJointMatrix(environmentHandle: int, jointHandle: int, matrix: float):
        """
        :param environmentHandle : Int
        :param jointHandle : Int
        :param matrix : Float[12]
        :return:
        """
        pass

    def setSphericalJointRotation(environmentHandle: int, jointHandle: int, eulerOrQuaternion: float):
        """
        :param environmentHandle : Int
        :param jointHandle : Int
        :param eulerOrQuaternion : Float[]
        :return:
        """
        pass

    def setTargetDummy(environmentHandle: int, dummyHandle: int, targetDummyHandle: int):
        """
        :param environmentHandle : Int
        :param dummyHandle : Int
        :param targetDummyHandle : Int
        :return:
        """
        pass

    def syncFromSim(environmentHandle: int, ikGroups: int):
        """
        :param environmentHandle : Int
        :param ikGroups : Int[]
        :return:
        """
        pass

    def syncToSim(environmentHandle: int, ikGroups: int):
        """
        :param environmentHandle : Int
        :param ikGroups : Int[]
        :return:
        """
        pass

