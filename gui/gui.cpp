#include "gui.h"

using namespace imr::gui;

Gui::Gui(RawPoints initial, RawPoints tentative, Point pos)
{
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    RawPoints2vtkPoints(initial,points);
    map = vtkSmartPointer<vtkPolyData>::New();
    map->SetPoints(points);

    points = vtkSmartPointer<vtkPoints>::New();
    RawPoints2vtkPoints(tentative, points);
    tentativeMeasurement = vtkSmartPointer<vtkPolyData>::New();
    tentativeMeasurement->SetPoints(points);

    points = vtkSmartPointer<vtkPoints>::New();
    RawPoints2vtkPoints(pos,points);
    position = vtkSmartPointer<vtkPolyData>::New();
    position->SetPoints(points);
    
    points = vtkSmartPointer<vtkPoints>::New();
    RawPoints2vtkPoints(pos,points);
    icpPosition = vtkSmartPointer<vtkPolyData>::New();
    icpPosition->SetPoints(points);
    mapFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
        mapFilter->AddInput(map);
    #else
        mapFilter->AddInputData(map);
    #endif
    mapFilter->Update();

    mapMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapMapper->SetInputConnection(mapFilter->GetOutputPort());

    mapActor = vtkSmartPointer<vtkActor>::New();
    mapActor->SetMapper(mapMapper);
    mapActor->GetProperty()->SetColor(0,0,0);
    mapActor->GetProperty()->SetPointSize(5);

    tentativeFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
        tentativeFilter->AddInput(tentativeMeasurement);
    #else
        tentativeFilter->AddInputData(tentativeMeasurement);
    #endif
    tentativeFilter->Update();

    tentativeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    tentativeMapper->SetInputConnection(tentativeFilter->GetOutputPort());

    tentativeActor = vtkSmartPointer<vtkActor>::New();
    tentativeActor->SetMapper(tentativeMapper);
    tentativeActor->GetProperty()->SetColor(1,0,0);
    tentativeActor->GetProperty()->SetPointSize(3);
    
    positionFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
        positionFilter->AddInput(position);
    #else
        positionFilter->AddInputData(position);
    #endif
    positionFilter->Update();

    positionMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    positionMapper->SetInputConnection(positionFilter->GetOutputPort());

    positionActor = vtkSmartPointer<vtkActor>::New();
    positionActor->SetMapper(positionMapper);
    positionActor->GetProperty()->SetColor(0,1,0);
    positionActor->GetProperty()->SetPointSize(5);

    icpPositionFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
        icpPositionFilter->AddInput(icpPosition);
    #else
        icpPositionFilter->AddInputData(icpPosition);
    #endif
    icpPositionFilter->Update();

    icpPositionMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    icpPositionMapper->SetInputConnection(icpPositionFilter->GetOutputPort());

    icpPositionActor = vtkSmartPointer<vtkActor>::New();
    icpPositionActor->SetMapper(icpPositionMapper);
    icpPositionActor->GetProperty()->SetColor(0,0,1);
    icpPositionActor->GetProperty()->SetPointSize(5);


    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);

    renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);
    
    interactorStyle = vtkSmartPointer<vtkInteractorStyleImage>::New();
    renderWindowInteractor->SetInteractorStyle(interactorStyle);

    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);

    renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);
    
    interactorStyle = vtkSmartPointer<vtkInteractorStyleImage>::New();
    renderWindowInteractor->SetInteractorStyle(interactorStyle);

    renderer->AddActor(mapActor);
    renderer->AddActor(tentativeActor);
    renderer->AddActor(positionActor);
    renderer->AddActor(icpPositionActor);
    renderer->SetBackground(1,1,1);
    renderWindow->Render();
}

void Gui::RawPoints2vtkPoints(const RawPoints &pts, vtkSmartPointer<vtkPoints> vtkPts)
{
    for(int i=0; i<pts.size(); i++) {
        vtkPts->InsertNextPoint(pts[i].x, pts[i].y, .0);
    }
}
void Gui::RawPoints2vtkPoints(const Point &pts, vtkSmartPointer<vtkPoints> vtkPts)
{
        vtkPts->InsertNextPoint(pts.x, pts.y, .0);
}
void Gui::startInteractor()
{
    renderWindowInteractor->Start();
}

void Gui::setTentativePoints(RawPoints pts)
{
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    RawPoints2vtkPoints(pts,points);
    
    tentativeMeasurement->SetPoints(points);
    tentativeMeasurement->Modified();
    renderWindow->Render();
}

void Gui::clearTentativePoints()
{
    tentativeMeasurement->SetPoints(vtkSmartPointer<vtkPoints>::New());
    tentativeMeasurement->Modified();
    renderWindow->Render();
}

void Gui::clearMapPoints()
{
    map->SetPoints(vtkSmartPointer<vtkPoints>::New());
    position->SetPoints(vtkSmartPointer<vtkPoints>::New());
    icpPosition->SetPoints(vtkSmartPointer<vtkPoints>::New());
    map->Modified();
    position->Modified();
    icpPosition->Modified();
    renderWindow->Render();
}


void Gui::setPointsToMap(RawPoints pts, Point pos, Point icpPos)
{
    vtkSmartPointer<vtkPoints> p = position->GetPoints();
    RawPoints2vtkPoints(pos,p);

    position->SetPoints(p);
    position->Modified();

    vtkSmartPointer<vtkPoints> ip = icpPosition->GetPoints();
    RawPoints2vtkPoints(icpPos,ip);

    icpPosition->SetPoints(ip);
    icpPosition->Modified();



    vtkSmartPointer<vtkPoints> points = map->GetPoints();
    RawPoints2vtkPoints(pts,points);

    map->SetPoints(points);
    map->Modified();
    renderWindow->Render();
}
