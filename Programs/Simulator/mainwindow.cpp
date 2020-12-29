#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "Shared/enut.h"

#include "PTS_Stream/IPM_TCPSocket.h"

#include "Shared/enut_simulator.h"
#include "Shared/enut_controller.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    all_scpis("SCPI_COLLECTION")
{

    m_db.scan("./config.cnf");

    ui->setupUi(this);

    m_p3d = ui->widget->getPlot3dInterface("enut" );

    m_p3d->setCameraTranslation({1.5,0.5,0.3} );
    m_p3d->setCameraRotation( {-1,0,-0.1}, {0,0,1});

    //m_p3d->setCameraTranslation({0.5,-1.5,0.3} );
    //m_p3d->setCameraRotation( {0,1,-0.1}, {0,0,1});

    m_p3d->setPointSize( 6 );


    m_modules = new ipm::modules::controller();
    m_simul = new Enut_Simulator( m_p3d );
    m_modules->register_module( "l", m_simul );

    m_cont = new Enut_Controller( m_db, m_simul, m_simul );
    m_modules->register_module( "l", m_cont );

    // collect all SCPIs
    all_scpis.addAdaptor( m_cont );

    // scpi server
    IPM_SCPIpp_Server2 * scpi_server = new IPM_SCPIpp_Server2("scpi_server", all_scpis, 50010, true, false );
    m_modules->register_module( "l", scpi_server );


    m_modules->start_module();


    ui->dspHeight->setValue( m_cont->get_height().second * 1000 );

    m_sock.push_back( new IPM_TCPSocket( "127.0.0.1", 50010, 3 ) );
    m_scpi.push_back( new IPM_SCPI_Client( *m_sock[0] ) );

    /*
    m_sock.push_back( new IPM_TCPSocket( "192.168.0.110", 50010, 3 ) );
    m_scpi.push_back( new IPM_SCPI_Client( *m_sock[1] ) );
    */
}

MainWindow::~MainWindow()
{
    delete m_modules;
    for( auto &s : m_scpi )
        delete s;
    for( auto &s : m_sock )
        delete s;
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
    send_cmd( "CTRL:ATTITUDE " + arg1.toStdString() );
}

void MainWindow::on_dspHeight_valueChanged(double arg1)
{
    send_cmd( "CTRL:HEIGHT " + p3t::to_string(arg1*0.001) );
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

void MainWindow::send_cmd(std::string cmd)
{
    PT_INFO("send " << cmd);
    for(unsigned i = 0; i < m_sock.size(); i++ ){
        if( m_sock[i] && m_sock[i]->get_state() != 0 ){
            int err(0);
            std::string ret = m_scpi[i]->command( cmd, &err );
            if( err ){
                PT_SEVERE( ret );
            }
        }
    }
}

void MainWindow::on_dspBodyRoll_valueChanged(double)
{
    send_cmd( "CTRL:RPY " + p3t::to_string(ui->dspBodyRoll->value()*M_PI/180.) + " "
               + p3t::to_string(ui->dspBodyPitch->value()*M_PI/180.) + " "
               + p3t::to_string(ui->dspBodyYaw->value()*M_PI/180.) );
}

void MainWindow::on_dspBodyPitch_valueChanged(double)
{
    send_cmd( "CTRL:RPY " + p3t::to_string(ui->dspBodyRoll->value()*M_PI/180.) + " "
               + p3t::to_string(ui->dspBodyPitch->value()*M_PI/180.) + " "
               + p3t::to_string(ui->dspBodyYaw->value()*M_PI/180.) );
}

void MainWindow::on_dspBodyYaw_valueChanged(double)
{
    send_cmd( "CTRL:RPY " + p3t::to_string(ui->dspBodyRoll->value()*M_PI/180.) + " "
               + p3t::to_string(ui->dspBodyPitch->value()*M_PI/180.) + " "
               + p3t::to_string(ui->dspBodyYaw->value()*M_PI/180.) );
}
