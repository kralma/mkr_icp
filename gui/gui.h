#ifndef __GUI__
#define __GUI__

#include<vector>
#include<iostream>

#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>
#include <vtkIterativeClosestPointTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkLandmarkTransform.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkProperty.h>
#include <vtkInteractorStyleImage.h>

namespace imr {
    namespace gui {

        struct Point {
            double x;
            double y;

            Point() : x(0.0), y(0.0) {}
            Point(const double &_x, const double &_y) : x(_x), y(_y) {}
        };

        typedef std::vector<Point> RawPoints;

        class Gui
        {
            private:
                vtkSmartPointer<vtkPolyData> map;
                vtkSmartPointer<vtkVertexGlyphFilter> mapFilter;
                vtkSmartPointer<vtkPolyDataMapper> mapMapper; 
                vtkSmartPointer<vtkActor> mapActor;

                vtkSmartPointer<vtkPolyData> tentativeMeasurement;
                vtkSmartPointer<vtkVertexGlyphFilter> tentativeFilter;
                vtkSmartPointer<vtkPolyDataMapper> tentativeMapper; 
                vtkSmartPointer<vtkActor> tentativeActor;

                vtkSmartPointer<vtkPolyData> position;
                vtkSmartPointer<vtkVertexGlyphFilter> positionFilter;
                vtkSmartPointer<vtkPolyDataMapper> positionMapper; 
                vtkSmartPointer<vtkActor> positionActor;

		vtkSmartPointer<vtkPolyData> icpPosition;
                vtkSmartPointer<vtkVertexGlyphFilter> icpPositionFilter;
                vtkSmartPointer<vtkPolyDataMapper> icpPositionMapper; 
                vtkSmartPointer<vtkActor> icpPositionActor;


                vtkSmartPointer<vtkRenderer> renderer; 
                vtkSmartPointer<vtkRenderWindow> renderWindow;
                vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;
                vtkSmartPointer<vtkInteractorStyleImage> interactorStyle;
                
                void RawPoints2vtkPoints(const RawPoints &pts, vtkSmartPointer<vtkPoints> vtkPts); 
                void RawPoints2vtkPoints(const Point &pts,
                        vtkSmartPointer<vtkPoints> vtkPts); 

            public:
                Gui(RawPoints intial, RawPoints tentative, Point pos);
                void demo(RawPoints pts);
                void startInteractor();

                void setPointsToMap(RawPoints pts, Point pos, Point icpPos);
		void clearMapPoints();
                void setTentativePoints(RawPoints pts);
                void clearTentativePoints();
        };
    } // namespace gui
} // namespace imr

#endif
