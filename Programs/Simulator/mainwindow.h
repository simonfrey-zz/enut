#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "IPM_Plot/plot3d_interface.h"

#include "ModulesChain/ModuleController.h"
#include "IPM_SCPI/Remote/IPM_SCPI_Remote.h"
#include <IPM_SCPI++/ipm_scpipp_server2.h>
#include "IPM_Parameter/IPM_SectionedParmFile.h"

namespace Ui {
class MainWindow;
}

class Enut;
class IPM_TCPSocket;
struct IPM_SCPI_Client;
class Enut_Simulator;
class Enut_Controller;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_sbShoulders_valueChanged(int arg1);

    void on_sbLegs_valueChanged(int arg1);

    void on_sbFoots_valueChanged(int arg1);

    void on_sbHead_valueChanged(int arg1);

    void on_cbAttitude_currentTextChanged(const QString &arg1);

    void on_dspHeight_valueChanged(double arg1);

    void on_dspGroundRoll_valueChanged(double arg1);

    void on_dspGroundPitch_valueChanged(double arg1);

    void on_dspBodyRoll_valueChanged(double arg1);

    void on_dspBodyPitch_valueChanged(double arg1);

    void on_dspBodyYaw_valueChanged(double arg1);

private:
    Ui::MainWindow *ui;

    Plot3D_Interface::shared_t m_p3d;
    p3t::IPM_SectionedParmFile m_db;

    ipm::modules::controller * m_modules;
    Enut_Simulator * m_simul;
    Enut_Controller * m_cont;

    std::vector<IPM_TCPSocket *> m_sock;
    std::vector<IPM_SCPI_Client *> m_scpi;
    SCPIClassAdaptorCollection all_scpis;

    void send_cmd( std::string cmd);
};

#endif // MAINWINDOW_H
