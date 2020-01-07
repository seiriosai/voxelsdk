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
#include "Configuration.h"

int tofVtkViewer(vtkPoints* ps,vtkCellArray* cs)
{
	
	
	vtkPoints *points = ps;// vtkPoints::New();
	vtkCellArray *cells = cs;// vtkCellArray::New();

	
	vtkPolyData *polyData = vtkPolyData::New();
	polyData->SetPoints(points);
	polyData->SetVerts(cells);

	
	vtkPolyDataMapper *mapper = vtkPolyDataMapper::New();
	mapper->SetInputData(polyData);

	vtkActor *actor = vtkActor::New();
	actor->SetMapper(mapper);
	
	actor->GetProperty()->SetColor(0.3, 0.5, 0.6);  
	actor->GetProperty()->SetPointSize(2);


	vtkRenderer *renderer = vtkRenderer::New();
	renderer->AddActor(actor);
	

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