#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QTimer>
#include <QKeyEvent>
#include <QColorDialog>
#include <QDebug>
#include <QMessageBox>
#include <iostream>

#ifdef Success
#undef Success
#endif

//! Draw lib.
#include "draw_primitives.h"

//! Opencascade.
#include "OcctQtViewer.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void calculate();

private slots:

    void on_doubleSpinBox_s_valueChanged(double arg1);

    void on_doubleSpinBox_ve_valueChanged(double arg1);

    void on_doubleSpinBox_vo_valueChanged(double arg1);

    void on_doubleSpinBox_vm_valueChanged(double arg1);

    void on_doubleSpinBox_a_valueChanged(double arg1);

private:
    Ui::MainWindow *ui;
    OcctQtViewer *occ;
};
#endif // MAINWINDOW_H
