//
// Created by zxm on 18-8-15.
//

#ifndef STAIR_MODELING_VIEWER_ADD_TEXT_H
#define STAIR_MODELING_VIEWER_ADD_TEXT_H

#include <pcl/conversions.h>
#include <pcl/visualization/common/common.h>
#include <vtkCellData.h>
#include <vtkDataSetMapper.h>
#include <vtkLoopSubdivisionFilter.h>
#include <vtkMapper.h>
#include <vtkPlatonicSolidSource.h>
#include <vtkPolyDataNormals.h>
#include <vtkPropPicker.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkTransform.h>
#include <vtkTriangle.h>
#include <vtkVersion.h>
#include <vtkWorldPointPicker.h>

#if VTK_MAJOR_VERSION >= 6 || (VTK_MAJOR_VERSION == 5 && VTK_MINOR_VERSION > 4)

#include <vtkHardwareSelector.h>
#include <vtkSelectionNode.h>

#else
#include <vtkVisibleCellSelector.h>
#endif

#include <pcl/visualization/boost.h>
#include <pcl/visualization/vtk/vtkRenderWindowInteractorFix.h>
#include <vtkPointPicker.h>
#include <vtkSelection.h>

#if VTK_RENDERING_BACKEND_OPENGL_VERSION < 2

#include <pcl/visualization/vtk/vtkVertexBufferObjectMapper.h>

#endif

#include <vtkAppendPolyData.h>
#include <vtkAreaPicker.h>
#include <vtkAxes.h>
#include <vtkAxesActor.h>
#include <vtkBMPReader.h>
#include <vtkCamera.h>
#include <vtkJPEGReader.h>
#include <vtkLookupTable.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkPLYReader.h>
#include <vtkPNGReader.h>
#include <vtkPNMReader.h>
#include <vtkPointData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyLine.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkTIFFReader.h>
#include <vtkTextureUnitManager.h>
#include <vtkTransformFilter.h>
#include <vtkTubeFilter.h>
#include <vtkXYPlotActor.h>

#if VTK_MAJOR_VERSION > 7
#include <vtkTexture.h>
#endif

#include <pcl/common/time.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/common/shapes.h>
#include <pcl/visualization/pcl_visualizer.h>

#if (BOOST_VERSION >= 106600)
#include <boost/uuid/detail/sha1.hpp>
#else

#include <boost/uuid/sha1.hpp>

#endif

#include <pcl/console/parse.h>

#include <boost/filesystem.hpp>

// Support for VTK 7.1 upwards
#ifdef vtkGenericDataArray_h
#define SetTupleValue SetTypedTuple
#define InsertNextTupleValue InsertNextTypedTuple
#define GetTupleValue GetTypedTuple
#endif

#include <string>

enum MyJustification { up_left = 0, up_right, down_left, down_right };

enum PolygonShowMode { polygon_line_mode = 0, polygon_surface_mode };

bool myaddText(pcl::visualization::PCLVisualizer &viewer,
               const std::string &text, MyJustification justification, int xpos,
               int ypos, int fontsize, double r, double g, double b,
               const std::string &id, int viewport = 0);

bool myaddPolygon(
    pcl::visualization::PCLVisualizer &viewer,
    const typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
    PolygonShowMode polygonShowMode1, double r, double g, double b,
    const std::string &id, int viewport = 0);

#endif  // STAIR_MODELING_VIEWER_ADD_TEXT_H
