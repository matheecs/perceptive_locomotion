//
// Created by zxm on 18-8-15.
//

#include <stair_modeling/viewer_custom_func.h>

using namespace pcl::visualization;


void addActorToRenderer(pcl::visualization::PCLVisualizer &viewer, const vtkSmartPointer<vtkProp> &actor, int viewport)
{
    vtkSmartPointer<vtkRendererCollection> rens_ = viewer.getRendererCollection();

    // Add it to all renderers
    rens_->InitTraversal();
    vtkRenderer *renderer = NULL;
    int i = 0;
    while ((renderer = rens_->GetNextItem()) != NULL)
    {
        // Should we add the actor to all renderers?
        if (viewport == 0)
        {
            renderer->AddActor(actor);
        } else if (viewport == i)               // add the actor only to the specified viewport
        {
            renderer->AddActor(actor);
        }
        ++i;
    }
}

bool myaddText(pcl::visualization::PCLVisualizer &viewer, const std::string &text, MyJustification justification,
               int xpos, int ypos, int fontsize, double r, double g, double b, const std::string &id, int viewport)
{
    ShapeActorMapPtr shape_actor_map_ = viewer.getShapeActorMap();

    std::string tid;
    if (id.empty())
        tid = text;
    else
        tid = id;

    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    ShapeActorMap::iterator am_it = shape_actor_map_->find(tid);
    if (am_it != shape_actor_map_->end())
    {
        pcl::console::print_warn(stderr,
                                 "[addText] A text with id <%s> already exists! Please choose a different id and retry.\n",
                                 tid.c_str());
        return (false);
    }

    // Create an Actor
    vtkSmartPointer<vtkTextActor> actor = vtkSmartPointer<vtkTextActor>::New();
    actor->SetPosition(xpos, ypos);
    actor->SetInput(text.c_str());

    vtkSmartPointer<vtkTextProperty> tprop = actor->GetTextProperty();
    tprop->SetFontSize(fontsize);
    tprop->SetFontFamilyToArial();
    tprop->BoldOn();
    tprop->SetColor(r, g, b);

    switch (justification)
    {
        case up_left:
            tprop->SetVerticalJustificationToTop();
            tprop->SetJustificationToLeft();
            break;
        case up_right:
            tprop->SetVerticalJustificationToTop();
            tprop->SetJustificationToRight();
            break;
        case down_left:
            tprop->SetVerticalJustificationToBottom();
            tprop->SetJustificationToLeft();
            break;
        case down_right:
            tprop->SetVerticalJustificationToBottom();
            tprop->SetJustificationToRight();
            break;
    };

    addActorToRenderer(viewer, actor, viewport);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[tid] = actor;
    return (true);
}

int getDefaultScalarInterpolationForDataSet(vtkDataSet *data)
{
    vtkPolyData *polyData = vtkPolyData::SafeDownCast(data); // Check that polyData != NULL in case of segfault
    return (polyData && polyData->GetNumberOfCells() != polyData->GetNumberOfVerts());
}

void createActorFromVTKDataSet(pcl::visualization::PCLVisualizer &viewer, const vtkSmartPointer<vtkDataSet> &data,
                               vtkSmartPointer<vtkActor> &actor, bool use_scalars = true)
{
    // If actor is not initialized, initialize it here
    if (!actor)
        actor = vtkSmartPointer<vtkActor>::New();

    bool use_vbos_ = false;

#if VTK_RENDERING_BACKEND_OPENGL_VERSION < 2
    if (use_vbos_)
    {
        vtkSmartPointer<vtkVertexBufferObjectMapper> mapper = vtkSmartPointer<vtkVertexBufferObjectMapper>::New();

        mapper->SetInput(data);

        if (use_scalars)
        {
            vtkSmartPointer<vtkDataArray> scalars = data->GetPointData()->GetScalars();
            double minmax[2];
            if (scalars)
            {
                scalars->GetRange(minmax);
                mapper->SetScalarRange(minmax);

                mapper->SetScalarModeToUsePointData();
                mapper->SetInterpolateScalarsBeforeMapping(getDefaultScalarInterpolationForDataSet(data));
                mapper->ScalarVisibilityOn();
            }
        }

        //actor->SetNumberOfCloudPoints (int (std::max<vtkIdType> (1, data->GetNumberOfPoints () / 10)));
        actor->GetProperty()->SetInterpolationToFlat();

        /// FIXME disabling backface culling due to known VTK bug: vtkTextActors are not
        /// shown when there is a vtkActor with backface culling on present in the scene
        /// Please see VTK bug tracker for more details: http://www.vtk.org/Bug/view.php?id=12588
        // actor->GetProperty ()->BackfaceCullingOn ();

        actor->SetMapper(mapper);
    } else
#endif
    {
        vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New();
#if VTK_MAJOR_VERSION < 6
        mapper->SetInput (data);
#else
        mapper->SetInputData(data);
#endif

        if (use_scalars)
        {
            vtkSmartPointer<vtkDataArray> scalars = data->GetPointData()->GetScalars();
            double minmax[2];
            if (scalars)
            {
                scalars->GetRange(minmax);
                mapper->SetScalarRange(minmax);

                mapper->SetScalarModeToUsePointData();
                mapper->SetInterpolateScalarsBeforeMapping(getDefaultScalarInterpolationForDataSet(data));
                mapper->ScalarVisibilityOn();
            }
        }
#if VTK_RENDERING_BACKEND_OPENGL_VERSION < 2
        mapper->ImmediateModeRenderingOff();
#endif

        //actor->SetNumberOfCloudPoints (int (std::max<vtkIdType> (1, data->GetNumberOfPoints () / 10)));
        actor->GetProperty()->SetInterpolationToFlat();

        /// FIXME disabling backface culling due to known VTK bug: vtkTextActors are not
        /// shown when there is a vtkActor with backface culling on present in the scene
        /// Please see VTK bug tracker for more details: http://www.vtk.org/Bug/view.php?id=12588
        // actor->GetProperty ()->BackfaceCullingOn ();

        actor->SetMapper(mapper);
    }

    //actor->SetNumberOfCloudPoints (std::max<vtkIdType> (1, data->GetNumberOfPoints () / 10));
    actor->GetProperty()->SetInterpolationToFlat();
}

//////////////////////////////////////////////////////////////////////////////////////////
bool
removeActorFromRenderer(pcl::visualization::PCLVisualizer &viewer, const vtkSmartPointer<vtkProp> &actor, int viewport)
{
    vtkSmartPointer<vtkRendererCollection> rens_ = viewer.getRendererCollection();

    vtkProp *actor_to_remove = vtkProp::SafeDownCast(actor);

    // Initialize traversal
    rens_->InitTraversal();
    vtkRenderer *renderer = NULL;
    int i = 0;
    while ((renderer = rens_->GetNextItem()) != NULL)
    {
        // Should we remove the actor from all renderers?
        if (viewport == 0)
        {
            renderer->RemoveActor(actor);
        } else if (viewport == i)               // add the actor only to the specified viewport
        {
            // Iterate over all actors in this renderer
            vtkPropCollection *actors = renderer->GetViewProps();
            actors->InitTraversal();
            vtkProp *current_actor = NULL;
            while ((current_actor = actors->GetNextProp()) != NULL)
            {
                if (current_actor != actor_to_remove)
                    continue;
                renderer->RemoveActor(actor);
                // Found the correct viewport and removed the actor
                return (true);
            }
        }
        ++i;
    }
    return viewport == 0;
}

bool myaddPolygon(pcl::visualization::PCLVisualizer &viewer,
                  const typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, PolygonShowMode polygonShowMode1,
                  double r, double g, double b, const std::string &id, int viewport)
{
    ShapeActorMapPtr shape_actor_map_ = viewer.getShapeActorMap();

    vtkSmartPointer<vtkDataSet> data = createPolygon<pcl::PointXYZ>(cloud);
    if (!data)
        return (false);

    // Check to see if this ID entry already exists (has it been already added to the visualizer?)
    ShapeActorMap::iterator am_it = shape_actor_map_->find(id);
    if (am_it != shape_actor_map_->end())
    {
        vtkSmartPointer<vtkAppendPolyData> all_data = vtkSmartPointer<vtkAppendPolyData>::New();

        // Add old data
#if VTK_MAJOR_VERSION < 6
        all_data->AddInput (reinterpret_cast<vtkPolyDataMapper*> ((vtkActor::SafeDownCast (am_it->second))->GetMapper ())->GetInput ());
#else
        all_data->AddInputData(reinterpret_cast<vtkPolyDataMapper *> ((vtkActor::SafeDownCast(
                am_it->second))->GetMapper())->GetInput());
#endif

        // Add new data
        vtkSmartPointer<vtkDataSetSurfaceFilter> surface_filter = vtkSmartPointer<vtkDataSetSurfaceFilter>::New();
#if VTK_MAJOR_VERSION < 6
        surface_filter->AddInput (vtkUnstructuredGrid::SafeDownCast (data));
#else
        surface_filter->AddInputData(vtkUnstructuredGrid::SafeDownCast(data));
#endif
        vtkSmartPointer<vtkPolyData> poly_data = surface_filter->GetOutput();
#if VTK_MAJOR_VERSION < 6
        all_data->AddInput (poly_data);
#else
        all_data->AddInputData(poly_data);
#endif

        // Create an Actor
        vtkSmartPointer<vtkActor> actor;
        createActorFromVTKDataSet(viewer, all_data->GetOutput(), actor);
        if (polygonShowMode1 == polygon_line_mode)
            actor->GetProperty()->SetRepresentationToWireframe();
        else if (polygonShowMode1 == polygon_surface_mode)
            actor->GetProperty()->SetRepresentationToSurface();
        actor->GetProperty()->SetColor(r, g, b);
        actor->GetMapper()->ScalarVisibilityOff();
        removeActorFromRenderer(viewer, am_it->second, viewport);
        addActorToRenderer(viewer, actor, viewport);

        // Save the pointer/ID pair to the global actor map
        (*shape_actor_map_)[id] = actor;
    } else
    {
        // Create an Actor
        vtkSmartPointer<vtkActor> actor;
        createActorFromVTKDataSet(viewer, data, actor);
        if (polygonShowMode1 == polygon_line_mode)
            actor->GetProperty()->SetRepresentationToWireframe();
        else if (polygonShowMode1 == polygon_surface_mode)
            actor->GetProperty()->SetRepresentationToSurface();
        actor->GetProperty()->SetColor(r, g, b);
        actor->GetMapper()->ScalarVisibilityOff();
        addActorToRenderer(viewer, actor, viewport);

        // Save the pointer/ID pair to the global actor map
        (*shape_actor_map_)[id] = actor;
    }

    return (true);
}