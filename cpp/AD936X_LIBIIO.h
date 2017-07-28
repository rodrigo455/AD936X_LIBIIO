/*
 * Author: Rodrigo Rolim Mendes de Alencar <alencar.fmce@imbel.gov.br>
 *
 * This file is protected by Copyright. Please refer to the COPYRIGHT file
 * distributed with this source distribution.
 *
 * This file is part of AD936X_LIBIIO.
 *
 * AD936X_LIBIIO is based on REDHAWK USRP_UHD, thus it is released under the same
 * Copyright and the same License.
 *
 * AD936X_LIBIIO is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * AD936X_LIBIIO is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see http://www.gnu.org/licenses/.
 */
#ifndef AD936X_LIBIIO_I_IMPL_H
#define AD936X_LIBIIO_I_IMPL_H

#include <iio.h>
#include "AD936X_LIBIIO_base.h"

/*********************************************************************************************/
/**************************        Multi Process Thread Class       **************************/
/*********************************************************************************************/
/** Note:: This class is based off of the process thread class in the USRP_base.h file.      */
/**             Changed to accept serviceFunction as argument, rather than hard coded        */
/**             Added interrupt() member function to interrupt underlying boost::thread      */
/*********************************************************************************************/
template <typename TargetClass>
class MultiProcessThread{
public:
    MultiProcessThread(TargetClass *_target, int (TargetClass::*_func)(),float _delay){
        service_function = boost::bind(_func, _target);
        _mythread = 0;
        _thread_running = false;
        _udelay = (__useconds_t)(_delay * 1000000);
    };

    // kick off the thread
    void start() {
        if (_mythread == 0) {
            _thread_running = true;
            _mythread = new boost::thread(&MultiProcessThread::run, this);
        }
    };

    // manage calls to target's service function
    void run() {
        int state = NORMAL;
        while (_thread_running and (state != FINISH)) {
            state = service_function();
            if (state == NOOP) usleep(_udelay);
        }
    };

    // stop thread and wait for termination
    bool release(unsigned long secs = 0, unsigned long usecs = 0) {
        _thread_running = false;
        if (_mythread)  {
            if ((secs == 0) and (usecs == 0)){
                _mythread->join();
            } else {
                boost::system_time waitime= boost::get_system_time() + boost::posix_time::seconds(secs) +  boost::posix_time::microseconds(usecs) ;
                if (!_mythread->timed_join(waitime)) {
                    return 0;
                }
            }
            delete _mythread;
            _mythread = 0;
        }

        return 1;
    };

    virtual ~MultiProcessThread(){
        if (_mythread != 0) {
            release(0);
            _mythread = 0;
        }
    };

    void updateDelay(float _delay) { _udelay = (__useconds_t)(_delay * 1000000); };

private:
    boost::thread *_mythread;
    bool _thread_running;
    boost::function<int ()> service_function;
    __useconds_t _udelay;
    boost::condition_variable _end_of_run;
    boost::mutex _eor_mutex;
};

typedef struct ticket_lock {
    ticket_lock(){
        cond=NULL;
        mutex=NULL;
        queue_head=queue_tail=0;
    }
    boost::condition_variable* cond;
    boost::mutex* mutex;
    size_t queue_head, queue_tail;
} ticket_lock_t;

class enqueued_scoped_lock{
    public:
	enqueued_scoped_lock(ticket_lock_t& _ticket){
            ticket = &_ticket;

            boost::mutex::scoped_lock lock(*ticket->mutex);
            queue_me = ticket->queue_tail++;
            while (queue_me != ticket->queue_head)
            {
                ticket->cond->wait(lock);
            }
        }
        ~enqueued_scoped_lock(){
            boost::mutex::scoped_lock lock(*ticket->mutex);
            ticket->queue_head++;
            ticket->cond->notify_all();
        }
    private:
        ticket_lock_t* ticket;
        size_t queue_me;
};

struct extendedRFInfoPkt {
    size_t tuner_id;
    std::string port;
    std::string rf_port_select;
    frontend::RFInfoPkt rfinfo_pkt;
};

struct ad936xTuner{
	bool update_sri;
	struct iio_channel *inphase;
	struct iio_channel *quadrature;
	struct iio_channel *config;
	std::vector<short> buffer;
	BULKIO::PrecisionUTCTime time;
	ticket_lock_t lock;
};

class AD936X_LIBIIO_i : public AD936X_LIBIIO_base
{
    ENABLE_LOGGING
    public:
        AD936X_LIBIIO_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl);
        AD936X_LIBIIO_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, char *compDev);
        AD936X_LIBIIO_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities);
        AD936X_LIBIIO_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities, char *compDev);
        ~AD936X_LIBIIO_i();

        void constructor();

        int serviceFunction(){return FINISH;}
		int serviceFunctionReceive();
		int serviceFunctionTransmit();
		void start() throw (CF::Resource::StartError, CORBA::SystemException);
		void stop() throw (CF::Resource::StopError, CORBA::SystemException);

    protected:
        std::string getTunerType(const std::string& allocation_id);
        bool getTunerDeviceControl(const std::string& allocation_id);
        std::string getTunerGroupId(const std::string& allocation_id);
        std::string getTunerRfFlowId(const std::string& allocation_id);
        double getTunerCenterFrequency(const std::string& allocation_id);
        void setTunerCenterFrequency(const std::string& allocation_id, double freq);
        double getTunerBandwidth(const std::string& allocation_id);
        void setTunerBandwidth(const std::string& allocation_id, double bw);
        bool getTunerAgcEnable(const std::string& allocation_id);
        void setTunerAgcEnable(const std::string& allocation_id, bool enable);
        float getTunerGain(const std::string& allocation_id);
        void setTunerGain(const std::string& allocation_id, float gain);
        long getTunerReferenceSource(const std::string& allocation_id);
        void setTunerReferenceSource(const std::string& allocation_id, long source);
        bool getTunerEnable(const std::string& allocation_id);
        void setTunerEnable(const std::string& allocation_id, bool enable);
        double getTunerOutputSampleRate(const std::string& allocation_id);
        void setTunerOutputSampleRate(const std::string& allocation_id, double sr);
        std::string get_rf_flow_id(const std::string& port_name);
        void set_rf_flow_id(const std::string& port_name, const std::string& id);
        frontend::RFInfoPkt get_rfinfo_pkt(const std::string& port_name);
        void set_rfinfo_pkt(const std::string& port_name, const frontend::RFInfoPkt& pkt);

    private:
        ////////////////////////////////////////
        // Required device specific functions // -- to be implemented by device developer
        ////////////////////////////////////////

        // these are pure virtual, must be implemented here
        void deviceEnable(frontend_tuner_status_struct_struct &fts, size_t tuner_id);
        void deviceDisable(frontend_tuner_status_struct_struct &fts, size_t tuner_id);
        bool deviceSetTuning(const frontend::frontend_tuner_allocation_struct &request, frontend_tuner_status_struct_struct &fts, size_t tuner_id);
        bool deviceDeleteTuning(frontend_tuner_status_struct_struct &fts, size_t tuner_id);

        /////////////////////////
		// Developer additions //
		/////////////////////////

        std::vector<ad936xTuner> ad936x_tuners;
        struct iio_context *context;
        struct iio_device *rx_device, *tx_device, *phy;
        struct iio_buffer *rx_buffer, *tx_buffer;
        struct iio_channel *rx_LO, *tx_LO;
        size_t buffer_size;
        bool isAD9364;

        // Service Functions
		MultiProcessThread<AD936X_LIBIIO_i> *receive_service_thread;
		MultiProcessThread<AD936X_LIBIIO_i> *transmit_service_thread;
		boost::mutex receive_service_thread_lock;
		boost::mutex transmit_service_thread_lock;

		// Ensure some thread safety
		ticket_lock_t rx_buffer_lock;
		ticket_lock_t tx_buffer_lock;
		boost::mutex prop_lock;

		// keep track of RFInfoPkt from each RF port
        std::vector<extendedRFInfoPkt> RX_info_port;
        std::vector<extendedRFInfoPkt> TX_info_port;
        void updateRfFlowId(extendedRFInfoPkt *rf_info);
        void updateGroupId(const std::string &group);

        std::string getStreamId(size_t tuner_id);

        // configure callbacks
        void targetDeviceChanged(const target_device_struct& old_value, const target_device_struct& new_value);
        void globalSettingsChanged(const global_settings_struct& old_value, const global_settings_struct& new_value);
        void receiveChainChanged(const receive_chain_struct& old_value, const receive_chain_struct& new_value);
        void transmitChainChanged(const transmit_chain_struct& old_value, const transmit_chain_struct& new_value);
        void deviceGroupIdChanged(std::string old_value, std::string new_value);

        // interface with ad936x device
        void initAD936x() throw (CF::PropertySet::InvalidConfiguration);
        struct iio_context * getContext(const std::string &uri);
        void updateReceiveFrequency(double frequency);
        void updateReceiveBandwidth(double bandwidth);
        void updateReceiveSampleRate(double sampleRate);
        void updateReceivePort(const std::string& rf_port);
        void updateTransmitFrequency(double frequency);
        void updateTransmitBandwidth(double bandwidth);
        void updateTransmitSampleRate(double sampleRate);
        void updateTransmitPort(const std::string& rf_port);
        void tunerReceive(size_t tuner_id, short *output);
        void tunerTransmit(size_t tuner_id, bulkio::ShortDataBlock block);
        bool createReceiveBuffer();
        bool destroyReceiveBuffer();
        bool createTransmitBuffer();
        bool destroyTransmitBuffer();
        void loadCurrentConfig();
        bool loadFIRFilter(const std::string &filter);

    protected:
		void construct();

};

#endif // AD936X_LIBIIO_I_IMPL_H
