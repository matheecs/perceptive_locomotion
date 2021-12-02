//
// Created by zxm on 18-8-15.
//

#ifndef STAIR_MODELING_VIEWER_ADD_TEXT_H
#define STAIR_MODELING_VIEWER_ADD_TEXT_H


#include <pcl/visualization/common/common.h>
#include <pcl/conversions.h>
#include <vtkVersion.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkCellData.h>
#include <vtkWorldPointPicker.h>
#include <vtkPropPicker.h>
#include <vtkPlatonicSolidSource.h>
#include <vtkLoopSubdivisionFilter.h>
#include <vtkTriangle.h>
#include <vtkTransform.h>
#include <vtkPolyDataNormals.h>
#include <vtkMapper.h>
#include <vtkDataSetMapper.h>

#if VTK_MAJOR_VERSION >= 6 || (VTK_MAJOR_VERSION == 5 && VTK_MINOR_VERSION > 4)

#include <vtkHardwareSelector.h>
#include <vtkSelectionNode.h>

#else
#include <vtkVisibleCellSelector.h>
#endif

#include <vtkSelection.h>
#include <vtkPointPicker.h>

#include <pcl/visualization/boost.h>
#include <pcl/visualization/vtk/vtkRenderWindowInteractorFix.h>

#if VTK_RENDERING_BACKEND_OPENGL_VERSION < 2

#include <pcl/visualization/vtk/vtkVertexBufferObjectMapper.h>

#endif

#include <vtkPolyLine.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>
#include <vtkAppendPolyData.h>
#include <vtkPointData.h>
#include <vtkTransformFilter.h>
#include <vtkProperty.h>
#include <vtkPLYReader.h>
#include <vtkAxes.h>
#include <vtkTubeFilter.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkAreaPicker.h>
#include <vtkXYPlotActor.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkJPEGReader.h>
#include <vtkBMPReader.h>
#include <vtkPNMReader.h>
#include <vtkPNGReader.h>
#include <vtkTIFFReader.h>
#include <vtkLookupTable.h>
#include <vtkTextureUnitManager.h>

#if VTK_MAJOR_VERSION > 7
#include <vtkTexture.h>
#endif

#include <pcl/visualization/common/shapes.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/common/time.h>

#if (BOOST_VERSION >= 106600)
#include <boost/uuid/detail/sha1.hpp>
#else

#include <boost/uuid/sha1.hpp>

#endif

#include <boost/filesystem.hpp>
#include <pcl/console/parse.h>

// Support for VTK 7.1 upwards
#ifdef vtkGenericDataArray_h
#define SetTupleValue SetTypedTuple
#define InsertNextTupleValue InsertNextTypedTuple
#define GetTupleValue GetTypedTuple
#endif

#include <string>

enum MyJustification
{
    up_left = 0,
    up_right,
    down_left,
    down_right
};

enum PolygonShowMode
{
    polygon_line_mode = 0,
    polygon_surface_mode
};

bool myaddText(pcl::visualization::PCLVisualizer &viewer, const std::string &text,MyJustification justification,
          int xpos, int ypos, int fontsize, double r, double g, double b, const std::string &id, int viewport = 0);

bool myaddPolygon(pcl::visualization::PCLVisualizer &viewer,
                                            const typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,PolygonShowMode polygonShowMode1,
                                            double r, double g, double b, const std::string &id, int viewport = 0);

#endif //STAIR_MODELING_VIEWER_ADD_TEXT_H
