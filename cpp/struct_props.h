#ifndef STRUCTPROPS_H
#define STRUCTPROPS_H

/*******************************************************************************************

    AUTO-GENERATED CODE. DO NOT MODIFY

*******************************************************************************************/

#include <ossie/CorbaUtils.h>
#include <CF/cf.h>
#include <ossie/PropertyMap.h>
#include <bulkio/bulkio.h>
typedef bulkio::connection_descriptor_struct connection_descriptor_struct;

#include <frontend/fe_tuner_struct_props.h>

struct global_settings_struct {
    global_settings_struct ()
    {
    }

    static std::string getId() {
        return std::string("global_settings");
    }

    static const char* getFormat() {
        return "siiib";
    }

    std::string filter_fir_config;
    CORBA::Long dcxo_tune_coarse;
    CORBA::Long dcxo_tune_fine;
    CORBA::Long xo_correction;
    bool filter_fir_en;
};

inline bool operator>>= (const CORBA::Any& a, global_settings_struct& s) {
    CF::Properties* temp;
    if (!(a >>= temp)) return false;
    const redhawk::PropertyMap& props = redhawk::PropertyMap::cast(*temp);
    if (props.contains("filter_fir_config")) {
        if (!(props["filter_fir_config"] >>= s.filter_fir_config)) return false;
    }
    if (props.contains("dcxo_tune_coarse")) {
        if (!(props["dcxo_tune_coarse"] >>= s.dcxo_tune_coarse)) return false;
    }
    if (props.contains("dcxo_tune_fine")) {
        if (!(props["dcxo_tune_fine"] >>= s.dcxo_tune_fine)) return false;
    }
    if (props.contains("xo_correction")) {
        if (!(props["xo_correction"] >>= s.xo_correction)) return false;
    }
    if (props.contains("filter_fir_en")) {
        if (!(props["filter_fir_en"] >>= s.filter_fir_en)) return false;
    }
    return true;
}

inline void operator<<= (CORBA::Any& a, const global_settings_struct& s) {
    redhawk::PropertyMap props;
 
    props["filter_fir_config"] = s.filter_fir_config;
 
    props["dcxo_tune_coarse"] = s.dcxo_tune_coarse;
 
    props["dcxo_tune_fine"] = s.dcxo_tune_fine;
 
    props["xo_correction"] = s.xo_correction;
 
    props["filter_fir_en"] = s.filter_fir_en;
    a <<= props;
}

inline bool operator== (const global_settings_struct& s1, const global_settings_struct& s2) {
    if (s1.filter_fir_config!=s2.filter_fir_config)
        return false;
    if (s1.dcxo_tune_coarse!=s2.dcxo_tune_coarse)
        return false;
    if (s1.dcxo_tune_fine!=s2.dcxo_tune_fine)
        return false;
    if (s1.xo_correction!=s2.xo_correction)
        return false;
    if (s1.filter_fir_en!=s2.filter_fir_en)
        return false;
    return true;
}

inline bool operator!= (const global_settings_struct& s1, const global_settings_struct& s2) {
    return !(s1==s2);
}

struct receive_chain_struct {
    receive_chain_struct ()
    {
        rf_bandwidth = 200000;
        sampling_frequency = 2500000;
        frequency = 70000000;
        rx_lo_external = false;
        rf_port_select = "A_BALANCED";
        quadrature_tracking_en = true;
        rf_dc_offset_tracking_en = true;
        bb_dc_offset_tracking_en = true;
        rx1_hardwaregain = 73;
        rx1_gain_control_mode = "slow_attack";
        rx2_hardwaregain = 73;
        rx2_gain_control_mode = "slow_attack";
    }

    static std::string getId() {
        return std::string("receive_chain");
    }

    static const char* getFormat() {
        return "dddbsbbbdsds";
    }

    double rf_bandwidth;
    double sampling_frequency;
    double frequency;
    bool rx_lo_external;
    std::string rf_port_select;
    bool quadrature_tracking_en;
    bool rf_dc_offset_tracking_en;
    bool bb_dc_offset_tracking_en;
    double rx1_hardwaregain;
    std::string rx1_gain_control_mode;
    double rx2_hardwaregain;
    std::string rx2_gain_control_mode;
};

inline bool operator>>= (const CORBA::Any& a, receive_chain_struct& s) {
    CF::Properties* temp;
    if (!(a >>= temp)) return false;
    const redhawk::PropertyMap& props = redhawk::PropertyMap::cast(*temp);
    if (props.contains("rx_rf_bandwidth")) {
        if (!(props["rx_rf_bandwidth"] >>= s.rf_bandwidth)) return false;
    }
    if (props.contains("rx_sampling_frequency")) {
        if (!(props["rx_sampling_frequency"] >>= s.sampling_frequency)) return false;
    }
    if (props.contains("rx_frequency")) {
        if (!(props["rx_frequency"] >>= s.frequency)) return false;
    }
    if (props.contains("rx_lo_external")) {
        if (!(props["rx_lo_external"] >>= s.rx_lo_external)) return false;
    }
    if (props.contains("rx_rf_port_select")) {
        if (!(props["rx_rf_port_select"] >>= s.rf_port_select)) return false;
    }
    if (props.contains("quadrature_tracking_en")) {
        if (!(props["quadrature_tracking_en"] >>= s.quadrature_tracking_en)) return false;
    }
    if (props.contains("rf_dc_offset_tracking_en")) {
        if (!(props["rf_dc_offset_tracking_en"] >>= s.rf_dc_offset_tracking_en)) return false;
    }
    if (props.contains("bb_dc_offset_tracking_en")) {
        if (!(props["bb_dc_offset_tracking_en"] >>= s.bb_dc_offset_tracking_en)) return false;
    }
    if (props.contains("rx1_hardwaregain")) {
        if (!(props["rx1_hardwaregain"] >>= s.rx1_hardwaregain)) return false;
    }
    if (props.contains("rx1_gain_control_mode")) {
        if (!(props["rx1_gain_control_mode"] >>= s.rx1_gain_control_mode)) return false;
    }
    if (props.contains("rx2_hardwaregain")) {
        if (!(props["rx2_hardwaregain"] >>= s.rx2_hardwaregain)) return false;
    }
    if (props.contains("rx2_gain_control_mode")) {
        if (!(props["rx2_gain_control_mode"] >>= s.rx2_gain_control_mode)) return false;
    }
    return true;
}

inline void operator<<= (CORBA::Any& a, const receive_chain_struct& s) {
    redhawk::PropertyMap props;
 
    props["rx_rf_bandwidth"] = s.rf_bandwidth;
 
    props["rx_sampling_frequency"] = s.sampling_frequency;
 
    props["rx_frequency"] = s.frequency;
 
    props["rx_lo_external"] = s.rx_lo_external;
 
    props["rx_rf_port_select"] = s.rf_port_select;
 
    props["quadrature_tracking_en"] = s.quadrature_tracking_en;
 
    props["rf_dc_offset_tracking_en"] = s.rf_dc_offset_tracking_en;
 
    props["bb_dc_offset_tracking_en"] = s.bb_dc_offset_tracking_en;
 
    props["rx1_hardwaregain"] = s.rx1_hardwaregain;
 
    props["rx1_gain_control_mode"] = s.rx1_gain_control_mode;
 
    props["rx2_hardwaregain"] = s.rx2_hardwaregain;
 
    props["rx2_gain_control_mode"] = s.rx2_gain_control_mode;
    a <<= props;
}

inline bool operator== (const receive_chain_struct& s1, const receive_chain_struct& s2) {
    if (s1.rf_bandwidth!=s2.rf_bandwidth)
        return false;
    if (s1.sampling_frequency!=s2.sampling_frequency)
        return false;
    if (s1.frequency!=s2.frequency)
        return false;
    if (s1.rx_lo_external!=s2.rx_lo_external)
        return false;
    if (s1.rf_port_select!=s2.rf_port_select)
        return false;
    if (s1.quadrature_tracking_en!=s2.quadrature_tracking_en)
        return false;
    if (s1.rf_dc_offset_tracking_en!=s2.rf_dc_offset_tracking_en)
        return false;
    if (s1.bb_dc_offset_tracking_en!=s2.bb_dc_offset_tracking_en)
        return false;
    if (s1.rx1_hardwaregain!=s2.rx1_hardwaregain)
        return false;
    if (s1.rx1_gain_control_mode!=s2.rx1_gain_control_mode)
        return false;
    if (s1.rx2_hardwaregain!=s2.rx2_hardwaregain)
        return false;
    if (s1.rx2_gain_control_mode!=s2.rx2_gain_control_mode)
        return false;
    return true;
}

inline bool operator!= (const receive_chain_struct& s1, const receive_chain_struct& s2) {
    return !(s1==s2);
}

struct transmit_chain_struct {
    transmit_chain_struct ()
    {
        rf_bandwidth = 200000;
        sampling_frequency = 2500000;
        frequency = 70000000;
        tx_lo_external = false;
        rf_port_select = "A";
        tx1_hardwaregain = -10;
        tx2_hardwaregain = -10;
    }

    static std::string getId() {
        return std::string("transmit_chain");
    }

    static const char* getFormat() {
        return "dddbsdd";
    }

    double rf_bandwidth;
    double sampling_frequency;
    double frequency;
    bool tx_lo_external;
    std::string rf_port_select;
    double tx1_hardwaregain;
    double tx2_hardwaregain;
};

inline bool operator>>= (const CORBA::Any& a, transmit_chain_struct& s) {
    CF::Properties* temp;
    if (!(a >>= temp)) return false;
    const redhawk::PropertyMap& props = redhawk::PropertyMap::cast(*temp);
    if (props.contains("tx_rf_bandwidth")) {
        if (!(props["tx_rf_bandwidth"] >>= s.rf_bandwidth)) return false;
    }
    if (props.contains("tx_sampling_frequency")) {
        if (!(props["tx_sampling_frequency"] >>= s.sampling_frequency)) return false;
    }
    if (props.contains("tx_frequency")) {
        if (!(props["tx_frequency"] >>= s.frequency)) return false;
    }
    if (props.contains("tx_lo_external")) {
        if (!(props["tx_lo_external"] >>= s.tx_lo_external)) return false;
    }
    if (props.contains("tx_rf_port_select")) {
        if (!(props["tx_rf_port_select"] >>= s.rf_port_select)) return false;
    }
    if (props.contains("tx1_hardwaregain")) {
        if (!(props["tx1_hardwaregain"] >>= s.tx1_hardwaregain)) return false;
    }
    if (props.contains("tx2_hardwaregain")) {
        if (!(props["tx2_hardwaregain"] >>= s.tx2_hardwaregain)) return false;
    }
    return true;
}

inline void operator<<= (CORBA::Any& a, const transmit_chain_struct& s) {
    redhawk::PropertyMap props;
 
    props["tx_rf_bandwidth"] = s.rf_bandwidth;
 
    props["tx_sampling_frequency"] = s.sampling_frequency;
 
    props["tx_frequency"] = s.frequency;
 
    props["tx_lo_external"] = s.tx_lo_external;
 
    props["tx_rf_port_select"] = s.rf_port_select;
 
    props["tx1_hardwaregain"] = s.tx1_hardwaregain;
 
    props["tx2_hardwaregain"] = s.tx2_hardwaregain;
    a <<= props;
}

inline bool operator== (const transmit_chain_struct& s1, const transmit_chain_struct& s2) {
    if (s1.rf_bandwidth!=s2.rf_bandwidth)
        return false;
    if (s1.sampling_frequency!=s2.sampling_frequency)
        return false;
    if (s1.frequency!=s2.frequency)
        return false;
    if (s1.tx_lo_external!=s2.tx_lo_external)
        return false;
    if (s1.rf_port_select!=s2.rf_port_select)
        return false;
    if (s1.tx1_hardwaregain!=s2.tx1_hardwaregain)
        return false;
    if (s1.tx2_hardwaregain!=s2.tx2_hardwaregain)
        return false;
    return true;
}

inline bool operator!= (const transmit_chain_struct& s1, const transmit_chain_struct& s2) {
    return !(s1==s2);
}

struct target_device_struct {
    target_device_struct ()
    {
        context_uri = "";
        type = "ad9361";
        name = "";
        serial = "";
    }

    static std::string getId() {
        return std::string("target_device");
    }

    static const char* getFormat() {
        return "ssss";
    }

    std::string context_uri;
    std::string type;
    std::string name;
    std::string serial;
};

inline bool operator>>= (const CORBA::Any& a, target_device_struct& s) {
    CF::Properties* temp;
    if (!(a >>= temp)) return false;
    const redhawk::PropertyMap& props = redhawk::PropertyMap::cast(*temp);
    if (props.contains("context_uri")) {
        if (!(props["context_uri"] >>= s.context_uri)) return false;
    }
    if (props.contains("type")) {
        if (!(props["type"] >>= s.type)) return false;
    }
    if (props.contains("name")) {
        if (!(props["name"] >>= s.name)) return false;
    }
    if (props.contains("serial")) {
        if (!(props["serial"] >>= s.serial)) return false;
    }
    return true;
}

inline void operator<<= (CORBA::Any& a, const target_device_struct& s) {
    redhawk::PropertyMap props;
 
    props["context_uri"] = s.context_uri;
 
    props["type"] = s.type;
 
    props["name"] = s.name;
 
    props["serial"] = s.serial;
    a <<= props;
}

inline bool operator== (const target_device_struct& s1, const target_device_struct& s2) {
    if (s1.context_uri!=s2.context_uri)
        return false;
    if (s1.type!=s2.type)
        return false;
    if (s1.name!=s2.name)
        return false;
    if (s1.serial!=s2.serial)
        return false;
    return true;
}

inline bool operator!= (const target_device_struct& s1, const target_device_struct& s2) {
    return !(s1==s2);
}

struct frontend_tuner_status_struct_struct : public frontend::default_frontend_tuner_status_struct_struct {
    frontend_tuner_status_struct_struct () : frontend::default_frontend_tuner_status_struct_struct()
    {
    }

    static std::string getId() {
        return std::string("FRONTEND::tuner_status_struct");
    }

    static const char* getFormat() {
        return "sddbssdsssdd";
    }

    std::string rf_port_select;
    std::string stream_id;
    double bandwidth_tolerance;
    double sample_rate_tolerance;
};

inline bool operator>>= (const CORBA::Any& a, frontend_tuner_status_struct_struct& s) {
    CF::Properties* temp;
    if (!(a >>= temp)) return false;
    const redhawk::PropertyMap& props = redhawk::PropertyMap::cast(*temp);
    if (props.contains("FRONTEND::tuner_status::allocation_id_csv")) {
        if (!(props["FRONTEND::tuner_status::allocation_id_csv"] >>= s.allocation_id_csv)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::bandwidth")) {
        if (!(props["FRONTEND::tuner_status::bandwidth"] >>= s.bandwidth)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::center_frequency")) {
        if (!(props["FRONTEND::tuner_status::center_frequency"] >>= s.center_frequency)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::enabled")) {
        if (!(props["FRONTEND::tuner_status::enabled"] >>= s.enabled)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::group_id")) {
        if (!(props["FRONTEND::tuner_status::group_id"] >>= s.group_id)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::rf_flow_id")) {
        if (!(props["FRONTEND::tuner_status::rf_flow_id"] >>= s.rf_flow_id)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::sample_rate")) {
        if (!(props["FRONTEND::tuner_status::sample_rate"] >>= s.sample_rate)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::tuner_type")) {
        if (!(props["FRONTEND::tuner_status::tuner_type"] >>= s.tuner_type)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::rf_port_select")) {
        if (!(props["FRONTEND::tuner_status::rf_port_select"] >>= s.rf_port_select)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::stream_id")) {
        if (!(props["FRONTEND::tuner_status::stream_id"] >>= s.stream_id)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::bandwidth_tolerance")) {
        if (!(props["FRONTEND::tuner_status::bandwidth_tolerance"] >>= s.bandwidth_tolerance)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::sample_rate_tolerance")) {
        if (!(props["FRONTEND::tuner_status::sample_rate_tolerance"] >>= s.sample_rate_tolerance)) return false;
    }
    return true;
}

inline void operator<<= (CORBA::Any& a, const frontend_tuner_status_struct_struct& s) {
    redhawk::PropertyMap props;
 
    props["FRONTEND::tuner_status::allocation_id_csv"] = s.allocation_id_csv;
 
    props["FRONTEND::tuner_status::bandwidth"] = s.bandwidth;
 
    props["FRONTEND::tuner_status::center_frequency"] = s.center_frequency;
 
    props["FRONTEND::tuner_status::enabled"] = s.enabled;
 
    props["FRONTEND::tuner_status::group_id"] = s.group_id;
 
    props["FRONTEND::tuner_status::rf_flow_id"] = s.rf_flow_id;
 
    props["FRONTEND::tuner_status::sample_rate"] = s.sample_rate;
 
    props["FRONTEND::tuner_status::tuner_type"] = s.tuner_type;
 
    props["FRONTEND::tuner_status::rf_port_select"] = s.rf_port_select;
 
    props["FRONTEND::tuner_status::stream_id"] = s.stream_id;
 
    props["FRONTEND::tuner_status::bandwidth_tolerance"] = s.bandwidth_tolerance;
 
    props["FRONTEND::tuner_status::sample_rate_tolerance"] = s.sample_rate_tolerance;
    a <<= props;
}

inline bool operator== (const frontend_tuner_status_struct_struct& s1, const frontend_tuner_status_struct_struct& s2) {
    if (s1.allocation_id_csv!=s2.allocation_id_csv)
        return false;
    if (s1.bandwidth!=s2.bandwidth)
        return false;
    if (s1.center_frequency!=s2.center_frequency)
        return false;
    if (s1.enabled!=s2.enabled)
        return false;
    if (s1.group_id!=s2.group_id)
        return false;
    if (s1.rf_flow_id!=s2.rf_flow_id)
        return false;
    if (s1.sample_rate!=s2.sample_rate)
        return false;
    if (s1.tuner_type!=s2.tuner_type)
        return false;
    if (s1.rf_port_select!=s2.rf_port_select)
        return false;
    if (s1.stream_id!=s2.stream_id)
        return false;
    if (s1.bandwidth_tolerance!=s2.bandwidth_tolerance)
        return false;
    if (s1.sample_rate_tolerance!=s2.sample_rate_tolerance)
        return false;
    return true;
}

inline bool operator!= (const frontend_tuner_status_struct_struct& s1, const frontend_tuner_status_struct_struct& s2) {
    return !(s1==s2);
}

#endif // STRUCTPROPS_H
