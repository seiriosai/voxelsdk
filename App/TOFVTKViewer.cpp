// VTK includes
#include "vtkPoints.h"
#include "vtkPolyData.h"
#include "vtkActor.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkPolyDataMapper.h"
#include "vtkInteractorStyle.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkProperty.h"
#include "vtkCellArray.h"
#include "vtkInteractorStyleTrackballCamera.h"
#include "vtkInteractorStyleUnicam.h"

int tofVtkViewer(vtkPoints* ps,vtkCellArray* cs)
{
	vtkPoints *points = ps;// vtkPoints::New();
	vtkCellArray *cells = cs;// vtkCellArray::New();

	/*
	//  ifstream fs(argv[1]);
	ifstream fs("simplePoints.txt");
	vtkIdType idtype;
	double x, y, z;
	while (fs >> x >> y >> z){
		//插入点坐标，此处可改为其它的xyz
		idtype = points->InsertNextPoint(x, y, z);
		cells->InsertNextCell(1, &idtype);
	}
	*/

	// 渲染机制未知，需要同时设置点坐标与点坐标对应的verts
	// verts中的id必须与点坐标对应
	vtkPolyData *polyData = vtkPolyData::New();
	polyData->SetPoints(points);
	polyData->SetVerts(cells);

	//下面为正常的可视化流程，可设置的点云颜色、大小等已注释
	vtkPolyDataMapper *mapper = vtkPolyDataMapper::New();
	mapper->SetInputData(polyData);

	vtkActor *actor = vtkActor::New();
	actor->SetMapper(mapper);
	//设置颜色与点大小
	actor->GetProperty()->SetColor(0.3, 0.5, 0.6);  
	actor->GetProperty()->SetPointSize(2);


	vtkRenderer *renderer = vtkRenderer::New();
	renderer->AddActor(actor);
	// 设置背景颜色
	// renderer->SetBackground(1, 1, 1);

	vtkRenderWindow *renderWindow = vtkRenderWindow::New();
	renderWindow->AddRenderer(renderer);

	vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
	iren->SetRenderWindow(renderWindow);

	vtkInteractorStyleUnicam *style = vtkInteractorStyleUnicam::New();
	iren->SetInteractorStyle(style);

	iren->Initialize();
	iren->Start();

	points->Delete();
	polyData->Delete();
	mapper->Delete();
	actor->Delete();
	renderer->Delete();
	renderWindow->Delete();
	iren->Delete();
	style->Delete();

	return 0;
}