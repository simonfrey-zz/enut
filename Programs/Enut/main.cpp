
#include "PTDebug.h"
#include "PT_Misc/PT_MiscString.h"
#include "PTS_System/PTS_Clocks.h"
#include "Shared/enut_servos.h"
#include "Shared/enut_imu.h"
#include "Shared/enut_sonar.h"
#include "Shared/enut_controller.h"

#include "ModulesChain/ModuleController.h"
#include <IPM_SCPI++/ipm_scpipp_server2.h>

#include "PTS_Factory/PTS_FactoryInclude3.h"
#include "PTS_System/PTC_Signal.h"
#include "IPM_Parameter/IPM_SectionedParmFile.h"

p3t::PTS_FactoryFromTag<IPM_Serialize3> * get_object_factory() {

    static p3t::PTS_FactoryFromTag<IPM_Serialize3> * our_factory;
    if(our_factory == 0) {
        our_factory = new p3t::PTS_FactoryFromTag<IPM_Serialize3>;
    }
    return our_factory;
}

int main(int argc , char *argv[])
{
    p3t::IPM_SectionedParmFile config;
    config.scan("./config.cnf");

    ipm::modules::controller * controller = new ipm::modules::controller();

    Enut_Servos * servo = new Enut_Servos( config );
    if( servo->opened() == false ){
        delete servo;
        return -1;
    }

    Enut_imu * imu = new Enut_imu();
    if( imu->opened() == false ){
        delete imu;
        return -1;
    }

    Enut_Sonar * sonar = new Enut_Sonar( config );


    // collect all SCPIs
    SCPIClassAdaptorCollection all_scpis("SCPI_COLLECTION");


    // scpi server
    IPM_SCPIpp_Server2 * scpi_server = new IPM_SCPIpp_Server2("scpi_server", all_scpis, 50010, true, false );

    auto enut_controller = new Enut_Controller( config, imu, servo, sonar );

    all_scpis.addAdaptor( enut_controller );

    controller->register_module( "l", scpi_server );
    controller->register_module( "l", servo );
    controller->register_module( "l", imu );
    controller->register_module( "l", sonar );
    controller->register_module( "l", enut_controller );

    controller->start_module();

    p3t::PTC_Signal<SIGTERM> sig_term;
    p3t::PTC_Signal<SIGINT> sig_int;
    while( controller->all_modules_are_running() && sig_term.get_signal_count() == 0 && sig_int.get_signal_count() == 0 ){
        p3t::sleep(0.5);
    }

    if( enut_controller->get_attitude().second != "relax" ){
        enut_controller->set_attitude("relax");
        p3t::sleep(3);
    }

    PT_INFO("exit");
    delete controller;

    return 0;
}
