#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "linear_motion.h"
#include <chrono>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    occ = new OcctQtViewer();
    ui->gridLayout->addWidget(occ);
    occ->set_view_top();
    occ->set_orthographic();

    calculate();
}

void MainWindow::calculate(){

    occ->shapevec.clear();
    occ->remove_all();

    double vo=ui->doubleSpinBox_vo->value();    // Velocity begin.
    double ve=ui->doubleSpinBox_ve->value();    // Velocity end.
    double vm=ui->doubleSpinBox_vm->value();    // Velocity max.
    double a=ui->doubleSpinBox_a->value();      // Acceleration max.
    double s=ui->doubleSpinBox_s->value();      // Displacement.
    double interval=0.2;                        // Interval time.
    bool debug=1;                               // Set debug output.
    bool debug_time=1;                          // Set debug output.

    linear_motion *lm = new linear_motion();
    lm->set_debug(debug, debug_time);
    lm->set_curve_values(vo,ve,vm,a,s);

    lm->get_curve_ve();             // Is "ve" velocity end changed to fit "s" displacement?

    double sr=0,vr=0,ar=0;          // Results for "s" displacement, "v" velocity , "a" acceleration.
    double sp=0,vp=0,ap=0,tp=0;     // Results for previous s,v,a,t.
    double scale_s=0.1;             // Scale the "s" displacement graph down.

    for(float t=0; t<lm->get_curve_total_time(); t+=interval){

        lm->get_curve_at_time(t,sr,vr,ar);

        if(t==0){ // Startup graph without vertical lines.
            sp=sr;
            vp=vr;
            ap=ar;
            tp=t;
        }

        // Show graph.
        occ->add_shapevec( draw_primitives().colorize( draw_primitives().draw_3d_line({tp,sp*scale_s,0},{t,sr*scale_s,0}) ,Quantity_NOC_GREEN,0) );
        occ->add_shapevec( draw_primitives().colorize( draw_primitives().draw_3d_line({tp,vp,0},{t,vr,0}) ,Quantity_NOC_BLUE,0) );
        occ->add_shapevec( draw_primitives().colorize( draw_primitives().draw_3d_line({tp,ap,0},{t,ar,0}) ,Quantity_NOC_YELLOW,0) );

        sp=sr;
        vp=vr;
        ap=ar;
        tp=t;
    }

    occ->redraw();

    // Calculate performance for the linear_motion class.
    // Start time point
    auto startTime = std::chrono::high_resolution_clock::now();

    // Call the function to measure
    lm->set_curve_values(vo,ve,vm,a,s);

    // End time point
    auto endTime = std::chrono::high_resolution_clock::now();

    // Calculate duration in microseconds
    auto durationMicro = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();

    // Calculate duration in milliseconds
    auto durationMilli = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();

    // Print the durations
    if(debug){
        std::cout << "Duration in microseconds: " << durationMicro << " us" << std::endl;   // Duration in microseconds: ~67 us on my old pc.
        std::cout << "Duration in milliseconds: " << durationMilli << " ms" << std::endl;   // Duration in milliseconds: ~0 ms
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_doubleSpinBox_s_valueChanged(double arg1)
{
    calculate();
}

void MainWindow::on_doubleSpinBox_ve_valueChanged(double arg1)
{
    calculate();
}

void MainWindow::on_doubleSpinBox_vo_valueChanged(double arg1)
{
    calculate();
}

void MainWindow::on_doubleSpinBox_vm_valueChanged(double arg1)
{
    calculate();
}

void MainWindow::on_doubleSpinBox_a_valueChanged(double arg1)
{
    calculate();
}
