QT       += core gui opengl openglwidgets
QT += printsupport
QT += axcontainer
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17
# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
SOURCES += \
    component/contralwidget.cpp \
    component/datawidget.cpp \
    component/elementlistwidget.cpp \
    component/filemanagerwidget.cpp \
    component/logwidget.cpp \
    component/presetelemwidget.cpp \
    component/reportwidget.cpp \
    component/toolaction.cpp \
    component/toolwidget.cpp \
    component/vtkpresetwidget.cpp \
    component/vtkwidget.cpp \
    component/vtkwindowreportwidget.cpp \
    constructor/circleconstructor.cpp \
    constructor/coneconstructor.cpp \
    constructor/constructor.cpp \
    constructor/cylinderconstructor.cpp \
    constructor/distanceconstructor.cpp \
    constructor/lineconstructor.cpp \
    constructor/planeconstructor.cpp \
    constructor/pointconstructor.cpp \
    constructor/rectangleconstructor.cpp \
    constructor/sphereconstructor.cpp \
    geometry/centity.cpp \
    geometry/centitytypes.cpp \
    geometry/cobject.cpp \
    geometry/cpcs.cpp \
    geometry/cpcsnode.cpp \
    geometry/cshape.cpp \
    main.cpp \
    mainwindow.cpp \
    manager/centitymgr.cpp \
    manager/cobjectmgr.cpp \
    manager/cpcsmgr.cpp

HEADERS += \
    component/contralwidget.h \
    component/datawidget.h \
    component/elementlistwidget.h \
    component/filemanagerwidget.h \
    component/logwidget.h \
    component/presetelemwidget.h \
    component/reportwidget.h \
    component/toolaction.h \
    component/toolwidget.h \
    component/vtkpresetwidget.h \
    component/vtkwidget.h \
    component/vtkwindowreportwidget.h \
    constructor/circleconstructor.h \
    constructor/coneconstructor.h \
    constructor/constructor.h \
    constructor/cylinderconstructor.h \
    constructor/distanceconstructor.h \
    constructor/lineconstructor.h \
    constructor/planeconstructor.h \
    constructor/pointconstructor.h \
    constructor/rectangleconstructor.h \
    constructor/sphereconstructor.h \
    geometry/centity.h \
    geometry/centitytypes.h \
    geometry/cobject.h \
    geometry/cpcs.h \
    geometry/cpcsnode.h \
    geometry/cshape.h \
    geometry/globes.h \
    mainwindow.h \
    manager/centitymgr.h \
    manager/cobjectmgr.h \
    manager/cpcsmgr.h

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    filemanagerwidget.qrc \
    graph.qrc \
    toolwidget.qrc


PCL_ROOT        =   $$quote(C:/Program Files/PCL 1.12.1)
3rdParty_Qhull  =   $$quote(C:/Program Files/PCL 1.12.1/3rdParty/Qhull)
3rdParty_FLANN  =   $$quote(C:/Program Files/PCL 1.12.1/3rdParty/FLANN)
3rdParty_Boost  =   $$quote(C:/Program Files/PCL 1.12.1/3rdParty/Boost)
3rdParty_Eigen  =   $$quote(C:/Program Files/PCL 1.12.1/3rdParty/Eigen)
3rdParty_VTK    =   $$quote(C:/Program Files/PCL 1.12.1/3rdParty/VTK)
OpenNI_ROOT     =   $$quote(C:/Program Files/PCL 1.12.1/3rdParty/OpenNI2)


INCLUDEPATH += \
    $$PCL_ROOT/include/pcl-1.12 \
    $$3rdParty_Qhull/include \
    $$3rdParty_FLANN/include \
    $$3rdParty_Boost/include/boost-1_78 \
    $$3rdParty_Eigen/eigen3 \
    $$3rdParty_VTK/include/vtk-9.1\
    $$OpenNI_ROOT/Include

LIBS+= \
    -L$$PCL_ROOT/lib \
    -L$$3rdParty_Qhull/lib \
    -L$$3rdParty_FLANN/lib \
    -L$$3rdParty_Boost/lib \
    -L$$3rdParty_Eigen/lib \
    -L$$3rdParty_VTK/lib \
    -L$$OpenNI_ROOT/Lib

LIBS += -lOpenNI2

win32:CONFIG(release, debug|release): LIBS += \
    -lpcl_common \
    -lpcl_features \
    -lpcl_filters \
    -lpcl_io \
    -lpcl_io_ply \
    -lpcl_kdtree \
    -lpcl_keypoints \
    -lpcl_ml \
    -lpcl_octree \
    -lpcl_outofcore \
    -lpcl_people \
    -lpcl_recognition \
    -lpcl_registration \
    -lpcl_sample_consensus \
    -lpcl_search \
    -lpcl_segmentation \
    -lpcl_stereo \
    -lpcl_surface \
    -lpcl_tracking \
    -lpcl_visualization \
    -lvtkcgns-9.1 \
    -lvtkChartsCore-9.1 \
    -lvtkCommonColor-9.1 \
    -lvtkCommonComputationalGeometry-9.1 \
    -lvtkCommonCore-9.1 \
    -lvtkCommonDataModel-9.1 \
    -lvtkCommonExecutionModel-9.1 \
    -lvtkCommonMath-9.1 \
    -lvtkCommonMisc-9.1 \
    -lvtkCommonSystem-9.1 \
    -lvtkCommonTransforms-9.1 \
    -lvtkDICOMParser-9.1 \
    -lvtkDomainsChemistry-9.1 \
    -lvtkDomainsChemistryOpenGL2-9.1 \
    -lvtkdoubleconversion-9.1 \
    -lvtkexodusII-9.1 \
    -lvtkexpat-9.1 \
    -lvtkFiltersAMR-9.1 \
    -lvtkFiltersCore-9.1 \
    -lvtkFiltersExtraction-9.1 \
    -lvtkFiltersFlowPaths-9.1 \
    -lvtkFiltersGeneral-9.1 \
    -lvtkFiltersGeneric-9.1 \
    -lvtkFiltersGeometry-9.1 \
    -lvtkFiltersHybrid-9.1 \
    -lvtkFiltersHyperTree-9.1 \
    -lvtkFiltersImaging-9.1 \
    -lvtkFiltersModeling-9.1 \
    -lvtkFiltersParallel-9.1 \
    -lvtkFiltersParallelImaging-9.1 \
    -lvtkFiltersPoints-9.1 \
    -lvtkFiltersProgrammable-9.1 \
    -lvtkFiltersSelection-9.1 \
    -lvtkFiltersSMP-9.1 \
    -lvtkFiltersSources-9.1 \
    -lvtkFiltersStatistics-9.1 \
    -lvtkFiltersTexture-9.1 \
    -lvtkFiltersTopology-9.1 \
    -lvtkFiltersVerdict-9.1 \
    -lvtkfmt-9.1 \
    -lvtkfreetype-9.1 \
    -lvtkGeovisCore-9.1 \
    -lvtkgl2ps-9.1 \
    -lvtkglew-9.1 \
    -lvtkhdf5-9.1 \
    -lvtkhdf5_hl-9.1 \
    -lvtkImagingColor-9.1 \
    -lvtkImagingCore-9.1 \
    -lvtkImagingFourier-9.1 \
    -lvtkImagingGeneral-9.1 \
    -lvtkImagingHybrid-9.1 \
    -lvtkImagingMath-9.1 \
    -lvtkImagingMorphological-9.1 \
    -lvtkImagingSources-9.1 \
    -lvtkImagingStatistics-9.1 \
    -lvtkImagingStencil-9.1 \
    -lvtkInfovisCore-9.1 \
    -lvtkInfovisLayout-9.1 \
    -lvtkInteractionImage-9.1 \
    -lvtkInteractionStyle-9.1 \
    -lvtkInteractionWidgets-9.1 \
    -lvtkIOAMR-9.1 \
    -lvtkIOAsynchronous-9.1 \
  #  -lvtkIOCesium3DTiles-9.1 \
    -lvtkIOCGNSReader-9.1 \
    -lvtkIOChemistry-9.1 \
    -lvtkIOCityGML-9.1 \
    -lvtkIOCONVERGECFD-9.1 \
    -lvtkIOCore-9.1 \
    -lvtkIOEnSight-9.1 \
    -lvtkIOExodus-9.1 \
    -lvtkIOExport-9.1 \
    -lvtkIOExportGL2PS-9.1 \
    -lvtkIOExportPDF-9.1 \
    -lvtkIOGeometry-9.1 \
    -lvtkIOHDF-9.1 \
    -lvtkIOImage-9.1 \
    -lvtkIOImport-9.1 \
    -lvtkIOInfovis-9.1 \
    -lvtkIOIOSS-9.1 \
    -lvtkIOLegacy-9.1 \
    -lvtkIOLSDyna-9.1 \
    -lvtkIOMINC-9.1 \
    -lvtkIOMotionFX-9.1 \
    -lvtkIOMovie-9.1 \
    -lvtkIONetCDF-9.1 \
    -lvtkIOOggTheora-9.1 \
    -lvtkIOParallel-9.1 \
    -lvtkIOParallelXML-9.1 \
    -lvtkIOPLY-9.1 \
    -lvtkIOSegY-9.1 \
    -lvtkIOSQL-9.1 \
    -lvtkioss-9.1 \
    -lvtkIOTecplotTable-9.1 \
    -lvtkIOVeraOut-9.1 \
    -lvtkIOVideo-9.1 \
    -lvtkIOXML-9.1 \
    -lvtkIOXMLParser-9.1 \
    -lvtkjpeg-9.1 \
    -lvtkjsoncpp-9.1 \
    -lvtkkissfft-9.1 \
    -lvtklibharu-9.1 \
    -lvtklibproj-9.1 \
    -lvtklibxml2-9.1 \
    -lvtkloguru-9.1 \
    -lvtklz4-9.1 \
    -lvtklzma-9.1 \
    -lvtkmetaio-9.1 \
    -lvtknetcdf-9.1 \
    -lvtkogg-9.1 \
    -lvtkParallelCore-9.1 \
    -lvtkParallelDIY-9.1 \
    -lvtkpng-9.1 \
    -lvtkpugixml-9.1 \
    -lvtkRenderingAnnotation-9.1 \
    -lvtkRenderingContext2D-9.1 \
    -lvtkRenderingContextOpenGL2-9.1 \
    -lvtkRenderingCore-9.1 \
    -lvtkRenderingFreeType-9.1 \
    -lvtkRenderingGL2PSOpenGL2-9.1 \
  #  -lvtkRenderingHyperTreeGrid-9.1 \
    -lvtkRenderingImage-9.1 \
    -lvtkRenderingLabel-9.1 \
 #   -lvtkRenderingLICOpenGL2-9.1 \
    -lvtkRenderingLOD-9.1 \
    -lvtkRenderingOpenGL2-9.1 \
    -lvtkRenderingSceneGraph-9.1 \
    -lvtkRenderingUI-9.1 \
    -lvtkRenderingVolume-9.1 \
    -lvtkRenderingVolumeOpenGL2-9.1 \
    -lvtkRenderingVtkJS-9.1 \
    -lvtksqlite-9.1 \
    -lvtksys-9.1 \
    -lvtkTestingRendering-9.1 \
    -lvtktheora-9.1 \
    -lvtktiff-9.1 \
    -lvtkverdict-9.1 \
    -lvtkViewsContext2D-9.1 \
    -lvtkViewsCore-9.1 \
    -lvtkViewsInfovis-9.1 \
    -lvtkWrappingTools-9.1 \
    -lvtkzlib-9.1
else:win32:CONFIG(debug, debug|release): LIBS += \
    -lpcl_commond \
    -lpcl_featuresd \
    -lpcl_filtersd \
    -lpcl_iod \
    -lpcl_io_plyd \
    -lpcl_kdtreed \
    -lpcl_keypointsd \
    -lpcl_mld \
    -lpcl_octreed \
    -lpcl_outofcored \
    -lpcl_peopled \
    -lpcl_recognitiond \
    -lpcl_registrationd \
    -lpcl_sample_consensusd \
    -lpcl_searchd \
    -lpcl_segmentationd \
    -lpcl_stereod \
    -lpcl_surfaced \
    -lpcl_trackingd \
    -lpcl_visualizationd \
   -lvtkcgns-9.1d \
   -lvtkChartsCore-9.1d \
   -lvtkCommonColor-9.1d \
   -lvtkCommonComputationalGeometry-9.1d \
   -lvtkCommonCore-9.1d \
   -lvtkCommonDataModel-9.1d \
   -lvtkCommonExecutionModel-9.1d \
   -lvtkCommonMath-9.1d \
   -lvtkCommonMisc-9.1d \
    -lvtkCommonSystem-9.1d \
    -lvtkCommonTransforms-9.1d \
    -lvtkDICOMParser-9.1d \
    -lvtkDomainsChemistry-9.1d \
    -lvtkDomainsChemistryOpenGL2-9.1d \
    -lvtkdoubleconversion-9.1d \
    -lvtkexodusII-9.1d \
    -lvtkexpat-9.1d \
    -lvtkFiltersAMR-9.1d \
    -lvtkFiltersCore-9.1d \
    -lvtkFiltersExtraction-9.1d \
    -lvtkFiltersFlowPaths-9.1d \
    -lvtkFiltersGeneral-9.1d \
    -lvtkFiltersGeneric-9.1d \
    -lvtkFiltersGeometry-9.1d \
    -lvtkFiltersHybrid-9.1d \
    -lvtkFiltersHyperTree-9.1d \
    -lvtkFiltersImaging-9.1d \
    -lvtkFiltersModeling-9.1d \
    -lvtkFiltersParallel-9.1d \
    -lvtkFiltersParallelImaging-9.1d \
    -lvtkFiltersPoints-9.1d \
    -lvtkFiltersProgrammable-9.1d \
    -lvtkFiltersSelection-9.1d \
    -lvtkFiltersSMP-9.1d \
    -lvtkFiltersSources-9.1d \
    -lvtkFiltersStatistics-9.1d \
    -lvtkFiltersTexture-9.1d \
    -lvtkFiltersTopology-9.1d \
    -lvtkFiltersVerdict-9.1d \
    -lvtkfmt-9.1d \
    -lvtkfreetype-9.1d \
    -lvtkGeovisCore-9.1d \
    -lvtkgl2ps-9.1d \
    -lvtkglew-9.1d \
    -lvtkhdf5-9.1d \
    -lvtkhdf5_hl-9.1d \
    -lvtkImagingColor-9.1d \
    -lvtkImagingCore-9.1d \
    -lvtkImagingFourier-9.1d \
    -lvtkImagingGeneral-9.1d \
    -lvtkImagingHybrid-9.1d \
    -lvtkImagingMath-9.1d \
    -lvtkImagingMorphological-9.1d \
    -lvtkImagingSources-9.1d \
    -lvtkImagingStatistics-9.1d \
    -lvtkImagingStencil-9.1d \
    -lvtkInfovisCore-9.1d \
    -lvtkInfovisLayout-9.1d \
    -lvtkInteractionImage-9.1d \
    -lvtkInteractionStyle-9.1d \
    -lvtkInteractionWidgets-9.1d \
    -lvtkIOAMR-9.1d \
    -lvtkIOAsynchronous-9.1d \
    #-lvtkIOCesium3DTiles-9.1d \
    -lvtkIOCGNSReader-9.1d \
    -lvtkIOChemistry-9.1d \
    -lvtkIOCityGML-9.1d \
    -lvtkIOCONVERGECFD-9.1d \
    -lvtkIOCore-9.1d \
    -lvtkIOEnSight-9.1d \
    -lvtkIOExodus-9.1d \
    -lvtkIOExport-9.1d \
    -lvtkIOExportGL2PS-9.1d \
    -lvtkIOExportPDF-9.1d \
    -lvtkIOGeometry-9.1d \
    -lvtkIOHDF-9.1d \
    -lvtkIOImage-9.1d \
    -lvtkIOImport-9.1d \
    -lvtkIOInfovis-9.1d \
    -lvtkIOIOSS-9.1d \
    -lvtkIOLegacy-9.1d \
    -lvtkIOLSDyna-9.1d \
    -lvtkIOMINC-9.1d \
    -lvtkIOMotionFX-9.1d \
    -lvtkIOMovie-9.1d \
    -lvtkIONetCDF-9.1d \
    -lvtkIOOggTheora-9.1d \
    -lvtkIOParallel-9.1d \
    -lvtkIOParallelXML-9.1d \
    -lvtkIOPLY-9.1d \
    -lvtkIOSegY-9.1d \
    -lvtkIOSQL-9.1d \
    -lvtkioss-9.1d \
    -lvtkIOTecplotTable-9.1d \
    -lvtkIOVeraOut-9.1d \
    -lvtkIOVideo-9.1d \
    -lvtkIOXML-9.1d \
    -lvtkIOXMLParser-9.1d \
    -lvtkjpeg-9.1d \
    -lvtkjsoncpp-9.1d \
    -lvtkkissfft-9.1d \
    -lvtklibharu-9.1d \
    -lvtklibproj-9.1d \
    -lvtklibxml2-9.1d \
    -lvtkloguru-9.1d \
    -lvtklz4-9.1d \
    -lvtklzma-9.1d \
    -lvtkmetaio-9.1d \
    -lvtknetcdf-9.1d \
    -lvtkogg-9.1d \
    -lvtkParallelCore-9.1d \
    -lvtkParallelDIY-9.1d \
    -lvtkpng-9.1d \
    -lvtkpugixml-9.1d \
    -lvtkRenderingAnnotation-9.1d \
    -lvtkRenderingContext2D-9.1d \
    -lvtkRenderingContextOpenGL2-9.1d \
    -lvtkRenderingCore-9.1d \
    -lvtkRenderingFreeType-9.1d \
    -lvtkRenderingGL2PSOpenGL2-9.1d \
    #-lvtkRenderingHyperTreeGrid-9.1d \
    -lvtkRenderingImage-9.1d \
    -lvtkRenderingLabel-9.1d \
    #-lvtkRenderingLICOpenGL2-9.1d \
    -lvtkRenderingLOD-9.1d \
    -lvtkRenderingOpenGL2-9.1d \
    -lvtkRenderingSceneGraph-9.1d \
    -lvtkRenderingUI-9.1d \
    -lvtkRenderingVolume-9.1d \
    -lvtkRenderingVolumeOpenGL2-9.1d \
    -lvtkRenderingVtkJS-9.1d \
    -lvtksqlite-9.1d \
    -lvtksys-9.1d \
    -lvtkTestingRendering-9.1d \
    -lvtktheora-9.1d \
    -lvtktiff-9.1d \
    -lvtkverdict-9.1d \
    -lvtkViewsContext2D-9.1d \
    -lvtkViewsCore-9.1d \
    -lvtkViewsInfovis-9.1d \
    -lvtkWrappingTools-9.1d \
   -lvtkzlib-9.1d\

CONFIG += debug_and_release
CONFIG(debug, debug|release){
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkcgns-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkChartsCore-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkCommonColor-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkCommonComputationalGeometry-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkCommonCore-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkCommonDataModel-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkCommonExecutionModel-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkCommonMath-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkCommonMisc-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkCommonSystem-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkCommonTransforms-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkDICOMParser-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkDomainsChemistry-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkDomainsChemistryOpenGL2-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkdoubleconversion-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkexodusII-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkexpat-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersAMR-9.1d.lib"
    # LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersCellGrid-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersCore-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersExtraction-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersFlowPaths-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersGeneral-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersGeneric-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersGeometry-9.1d.lib"
    # LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersGeometryPreview-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersHybrid-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersHyperTree-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersImaging-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersModeling-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersParallel-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersParallelImaging-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersPoints-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersProgrammable-9.1d.lib"
    # LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersReduction-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersSelection-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersSMP-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersSources-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersStatistics-9.1d.lib"
    # LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersTensor-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersTexture-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersTopology-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersVerdict-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkfmt-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkfreetype-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkGeovisCore-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkgl2ps-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkglew-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkGUISupportQt-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkGUISupportQtQuick-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkGUISupportQtSQL-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkhdf5-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkhdf5_hl-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkImagingColor-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkImagingCore-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkImagingFourier-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkImagingGeneral-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkImagingHybrid-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkImagingMath-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkImagingMorphological-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkImagingSources-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkImagingStatistics-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkImagingStencil-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkInfovisCore-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkInfovisLayout-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkInteractionImage-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkInteractionStyle-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkInteractionWidgets-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOAMR-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOAsynchronous-9.1d.lib"
    # LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOCellGrid-9.1d.lib"
    # LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOCesium3DTiles-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOCGNSReader-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOChemistry-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOCityGML-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOCONVERGECFD-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOCore-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOEnSight-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOExodus-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOExport-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOExportGL2PS-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOExportPDF-9.1d.lib"
    # LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOFLUENTCFF-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOGeometry-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOHDF-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOImage-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOImport-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOInfovis-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOIOSS-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOLegacy-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOLSDyna-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOMINC-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOMotionFX-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOMovie-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIONetCDF-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOOggTheora-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOParallel-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOParallelXML-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOPLY-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOSegY-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOSQL-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkioss-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOTecplotTable-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOVeraOut-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOVideo-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOXML-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOXMLParser-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkjpeg-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkjsoncpp-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkkissfft-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtklibharu-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtklibproj-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtklibxml2-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkloguru-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtklz4-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtklzma-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkmetaio-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtknetcdf-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkogg-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkParallelCore-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkParallelDIY-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkpng-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkpugixml-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingAnnotation-9.1d.lib"
    # LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingCellGrid-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingContext2D-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingContextOpenGL2-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingCore-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingFreeType-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingGL2PSOpenGL2-9.1d.lib"
    # LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingHyperTreeGrid-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingImage-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingLabel-9.1d.lib"
    # LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingLICOpenGL2-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingLOD-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingOpenGL2-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingQt-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingSceneGraph-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingUI-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingVolume-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingVolumeOpenGL2-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingVtkJS-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtksqlite-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtksys-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkTestingRendering-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtktheora-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtktiff-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkverdict-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkViewsContext2D-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkViewsCore-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkViewsInfovis-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkViewsQt-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkWrappingTools-9.1d.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkzlib-9.1d.lib"

    contains(DEFINES, WIN64) {
    TARGET = ../_debug64/AppName
    } else {
    TARGET = ../_debug32/AppName
    }
} else {
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkcgns-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkChartsCore-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkCommonColor-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkCommonComputationalGeometry-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkCommonCore-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkCommonDataModel-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkCommonExecutionModel-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkCommonMath-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkCommonMisc-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkCommonSystem-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkCommonTransforms-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkDICOMParser-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkDomainsChemistry-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkDomainsChemistryOpenGL2-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkdoubleconversion-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkexodusII-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkexpat-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersAMR-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersCellGrid-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersCore-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersExtraction-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersFlowPaths-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersGeneral-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersGeneric-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersGeometry-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersGeometryPreview-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersHybrid-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersHyperTree-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersImaging-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersModeling-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersParallel-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersParallelImaging-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersPoints-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersProgrammable-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersReduction-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersSelection-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersSMP-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersSources-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersStatistics-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersTensor-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersTexture-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersTopology-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkFiltersVerdict-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkfmt-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkfreetype-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkGeovisCore-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkgl2ps-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkglew-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkGUISupportQt-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkGUISupportQtQuick-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkGUISupportQtSQL-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkhdf5-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkhdf5_hl-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkImagingColor-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkImagingCore-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkImagingFourier-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkImagingGeneral-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkImagingHybrid-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkImagingMath-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkImagingMorphological-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkImagingSources-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkImagingStatistics-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkImagingStencil-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkInfovisCore-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkInfovisLayout-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkInteractionImage-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkInteractionStyle-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkInteractionWidgets-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOAMR-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOAsynchronous-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOCellGrid-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOCesium3DTiles-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOCGNSReader-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOChemistry-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOCityGML-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOCONVERGECFD-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOCore-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOEnSight-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOExodus-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOExport-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOExportGL2PS-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOExportPDF-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOFLUENTCFF-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOGeometry-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOHDF-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOImage-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOImport-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOInfovis-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOIOSS-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOLegacy-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOLSDyna-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOMINC-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOMotionFX-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOMovie-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIONetCDF-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOOggTheora-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOParallel-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOParallelXML-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOPLY-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOSegY-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOSQL-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkioss-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOTecplotTable-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOVeraOut-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOVideo-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOXML-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkIOXMLParser-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkjpeg-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkjsoncpp-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkkissfft-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtklibharu-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtklibproj-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtklibxml2-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkloguru-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtklz4-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtklzma-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkmetaio-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtknetcdf-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkogg-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkParallelCore-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkParallelDIY-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkpng-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkpugixml-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingAnnotation-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingCellGrid-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingContext2D-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingContextOpenGL2-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingCore-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingFreeType-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingGL2PSOpenGL2-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingHyperTreeGrid-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingImage-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingLabel-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingLICOpenGL2-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingLOD-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingOpenGL2-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingQt-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingSceneGraph-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingUI-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingVolume-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingVolumeOpenGL2-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkRenderingVtkJS-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtksqlite-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtksys-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkTestingRendering-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtktheora-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtktiff-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkverdict-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkViewsContext2D-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkViewsCore-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkViewsInfovis-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkViewsQt-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkWrappingTools-9.1.lib"
    LIBS += "C:\Program Files\PCL 1.12.1\3rdParty\VTK\lib\vtkzlib-9.1.lib"

    contains(DEFINES, WIN64) {
    TARGET = ../_release64/AppName
    } else {
    TARGET = ../_release32/AppName
    }
}


# INCLUDEPATH += $$PWD/include
# LIBS += -L$$PWD/lib -lmyDll
