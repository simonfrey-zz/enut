#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "Shared/enut.h"

#include "PTS_Stream/IPM_TCPSocket.h"
#include "IPM_SCPI/Remote/IPM_SCPI_Remote.h"

#include "Shared/enut_simulator.h"
#include "Shared/enut_controller.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_sock(nullptr),
    m_scpi(nullptr)
{
    ui->setupUi(this);

    m_p3d = ui->widget->getPlot3dInterface("enut" );

    m_p3d->setCameraTranslation({1.5,0.5,0.3} );
    m_p3d->setCameraRotation( {-1,0,-0.1}, {0,0,1});
    m_p3d->setPointSize( 6 );

    m_modules = new ipm::modules::controller();
    m_simul = new Enut_Simulator( m_p3d );
    m_modules->register_module( "l", m_simul );

    m_cont = new Enut_Controller( m_simul, m_simul );
    m_modules->register_module( "l", m_cont );

    m_modules->start_module();

    ui->dspHeight->setValue( m_cont->get_height() * 1000 );

    //m_sock = new IPM_TCPSocket( "192.168.0.110", 50010, 3 );
    //m_scpi = new IPM_SCPI_Client( *m_sock );
}

MainWindow::~MainWindow()
{
    delete m_modules;
    delete m_scpi;
    delete m_sock;
    delete ui;
}

void MainWindow::on_sbShoulders_valueChanged(int)
{
    const double s = (double)ui->sbShoulders->value() * M_PI/180.;
    const double l = (double)ui->sbLegs->value() * M_PI/180.;
    const double f = (double)ui->sbFoots->value() * M_PI/180.;
    const double h = (double)ui->sbHead->value() * M_PI/180.;
    m_simul->dbg_set_angles( s, s, s, s, l, l, l, l, f, f, f, f, h );
    //enut->draw();
    send_values();
}

void MainWindow::on_sbLegs_valueChanged(int )
{
    on_sbShoulders_valueChanged(0);
}

void MainWindow::on_sbFoots_valueChanged(int )
{
    on_sbShoulders_valueChanged(0);
}

void MainWindow::on_sbHead_valueChanged(int )
{
    on_sbShoulders_valueChanged(0);
}


void MainWindow::on_cbAttitude_currentTextChanged(const QString &arg1)
{
    if( arg1 == "relax" ){
        m_cont->set_attitude( enut::relax );
    }
    else if( arg1 == "standing" ){
        m_cont->set_attitude( enut::standing );
    }
}

void MainWindow::on_dspHeight_valueChanged(double arg1)
{
    m_cont->set_height( arg1*0.001 );
}

void MainWindow::send_values()
{
    /*
    if( m_sock && m_sock->get_state() != 0 ){
        int err(0);
        std::string cmd = "JOINT:ANGLES \"[ ";
        auto angles = enut->get_angles();
        for( unsigned i = 0; i < angles.size(); i++ ){
            cmd += p3t::to_string( angles[i]*180.0 / M_PI ) + " ";
        }
        cmd += "]\"";
        PT_INFO("send " << cmd);
        std::string ret = m_scpi->command( cmd, &err );
        if( err ){
            PT_SEVERE( ret );
        }
    }
    */
}


void MainWindow::on_dspGroundRoll_valueChanged(double )
{
    m_simul->set_ground_angles( ui->dspGroundRoll->value() * M_PI/180.,
                                ui->dspGroundPitch->value() * M_PI/180.);
}

void MainWindow::on_dspGroundPitch_valueChanged(double )
{
    m_simul->set_ground_angles( ui->dspGroundRoll->value() * M_PI/180.,
                                ui->dspGroundPitch->value() * M_PI/180.);
}
