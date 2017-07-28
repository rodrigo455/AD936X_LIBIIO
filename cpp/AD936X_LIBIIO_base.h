#ifndef AD936X_LIBIIO_BASE_IMPL_BASE_H
#define AD936X_LIBIIO_BASE_IMPL_BASE_H

#include <boost/thread.hpp>
#include <frontend/frontend.h>
#include <ossie/ThreadedComponent.h>

#include <frontend/frontend.h>
#include <bulkio/bulkio.h>
#include "struct_props.h"

#define BOOL_VALUE_HERE 0

class AD936X_LIBIIO_base : public frontend::FrontendTunerDevice<frontend_tuner_status_struct_struct>, public virtual frontend::digital_tuner_delegation, public virtual frontend::rfinfo_delegation, protected ThreadedComponent
{
    public:
        AD936X_LIBIIO_base(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl);
        AD936X_LIBIIO_base(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, char *compDev);
        AD936X_LIBIIO_base(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities);
        AD936X_LIBIIO_base(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities, char *compDev);
        ~AD936X_LIBIIO_base();

        void start() throw (CF::Resource::StartError, CORBA::SystemException);

        void stop() throw (CF::Resource::StopError, CORBA::SystemException);

        void releaseObject() throw (CF::LifeCycle::ReleaseError, CORBA::SystemException);

        void loadProperties();
        void matchAllocationIdToStreamId(const std::string allocation_id, const std::string stream_id, const std::string port_name="");
        void removeAllocationIdRouting(const size_t tuner_id);
        void removeStreamIdRouting(const std::string stream_id, const std::string allocation_id="");

        virtual CF::Properties* getTunerStatus(const std::string& allocation_id);
        virtual void assignListener(const std::string& listen_alloc_id, const std::string& allocation_id);
        virtual void removeListener(const std::string& listen_alloc_id);
        void frontendTunerStatusChanged(const std::vector<frontend_tuner_status_struct_struct>* oldValue, const std::vector<frontend_tuner_status_struct_struct>* newValue);

    protected:
        void connectionTableChanged(const std::vector<connection_descriptor_struct>* oldValue, const std::vector<connection_descriptor_struct>* newValue);

        // Member variables exposed as properties
        /// Property: global_group_id
        std::string global_group_id;
        /// Property: global_settings
        global_settings_struct global_settings;
        /// Property: receive_chain
        receive_chain_struct receive_chain;
        /// Property: transmit_chain
        transmit_chain_struct transmit_chain;
        /// Property: target_device
        target_device_struct target_device;
        /// Property: connectionTable
        std::vector<connection_descriptor_struct> connectionTable;

        // Ports
        /// Port: RX1A_0
        frontend::InRFInfoPort *RX1A_0;
        /// Port: RX1B_1
        frontend::InRFInfoPort *RX1B_1;
        /// Port: RX2A_2
        frontend::InRFInfoPort *RX2A_2;
        /// Port: RX2B_3
        frontend::InRFInfoPort *RX2B_3;
        /// Port: DigitalTuner_in
        frontend::InDigitalTunerPort *DigitalTuner_in;
        /// Port: dataShortTX_in
        bulkio::InShortPort *dataShortTX_in;
        /// Port: TX1A_0
        frontend::OutRFInfoPort *TX1A_0;
        /// Port: TX1B_1
        frontend::OutRFInfoPort *TX1B_1;
        /// Port: TX2A_2
        frontend::OutRFInfoPort *TX2A_2;
        /// Port: TX2B_3
        frontend::OutRFInfoPort *TX2B_3;
        /// Port: dataShortRX_out
        bulkio::OutShortPort *dataShortRX_out;

        std::map<std::string, std::string> listeners;

        virtual void setNumChannels(size_t num);
        virtual void setNumChannels(size_t num, std::string tuner_type);

    private:
        void construct();
};
#endif // AD936X_LIBIIO_BASE_IMPL_BASE_H
