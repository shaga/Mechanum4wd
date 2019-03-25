#include "BtGamepad.hpp"

static const uint32_t MaxAttributeSize = 300;
static const uint8_t ReportDefaultLength = 17;
static const uint8_t IdxReportButton = 15;
static const uint8_t IdxReportHat = 14;
static const uint8_t IdxReportAnalogLH = 3;
static const uint8_t IdxReportAnalogLV = 5;
static const uint8_t IdxReportAnalogRH = 7;
static const uint8_t IdxReportAnalogRV = 9;
static const uint8_t IdxReportButtonL2 = 11;
static const uint8_t IdxReportButtonR2 = 13;
static const uint8_t ValueL2R2On = 0x03;
static const uint16_t ReportValueHatNone = 0x0080;
static const uint16_t ReportValueHatUp = 0x0000;
static const uint16_t ReportValueHatDown = 0xFFFF;
static const uint16_t ReportValueHatLeft = 0x0000;
static const uint16_t ReportValueHatRight = 0xFFFF;

static const char* BtGamepadMacAddr = "E4:17:D8:2C:FB:8C";

static void initBtHidHost();
static void handleBtPacket(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void handleSdpClientQueryResult(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void notifyConnectionState(EBtGamepadConnection_t connection);
static void notifyUpdateState(EGamepadHat_t hat, EGamepadButtons_t buttons, int8_t*);
static void parseReport(const uint8_t *report, uint16_t report_len);

static BtGamepad* bt_gamepad = nullptr;

static uint8_t hid_descriptor[MaxAttributeSize];
static uint16_t hid_descriptor_length;

static uint16_t hid_control_psm;
static uint16_t hid_interrupt_psm;

static uint8_t attribute_value[MaxAttributeSize];

static uint16_t l2cap_hid_control_cid;
static uint16_t l2cap_hid_interrupt_cid;

static bd_addr_t remote_addr;

static btstack_packet_callback_registration_t hci_event_callback;

static GamepadConnectionCallback_t connection_call = nullptr;
static GamepadUpdateStateCallback_t update_state_call = nullptr;

static EBtGamepadConnection_t connection_state = PadDisconnected;

static int8_t analog_value[EGamepadStick_t::AnalogCount];

static void initBtHidHost() {
    // Initialize L2CAP 
    l2cap_init();

    // register for HCI events
    hci_event_callback.callback = &handleBtPacket;
    hci_add_event_handler(&hci_event_callback);

    // Disable stdout buffering
    // setbuf(stdout, NULL);
}

static void handleSdpClientQueryResult(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(packet_type);
    UNUSED(channel);
    UNUSED(size);

    des_iterator_t attribute_list_it;
    des_iterator_t additional_des_it;
    des_iterator_t prot_it;

    uint8_t *des_element;
    uint8_t *element;
    uint32_t uuid;
    uint8_t status;

    switch (hci_event_packet_get_type(packet)) {
        case SDP_EVENT_QUERY_ATTRIBUTE_VALUE:
            if (sdp_event_query_attribute_byte_get_attribute_length(packet) <= MaxAttributeSize) {
                attribute_value[sdp_event_query_attribute_byte_get_data_offset(packet)] = sdp_event_query_attribute_byte_get_data(packet);
                if ((uint16_t)(sdp_event_query_attribute_byte_get_data_offset(packet)+1) == sdp_event_query_attribute_byte_get_attribute_length(packet)) {
                    switch (sdp_event_query_attribute_byte_get_attribute_id(packet)) {
                        case BLUETOOTH_ATTRIBUTE_PROTOCOL_DESCRIPTOR_LIST:
                            for (des_iterator_init(&attribute_list_it, attribute_value); des_iterator_has_more(&attribute_list_it); des_iterator_next(&attribute_list_it)) {                                    
                                if (des_iterator_get_type(&attribute_list_it) != DE_DES) continue;
                                des_element = des_iterator_get_element(&attribute_list_it);
                                des_iterator_init(&prot_it, des_element);
                                element = des_iterator_get_element(&prot_it);
                                if (!element) continue;
                                if (de_get_element_type(element) != DE_UUID) continue;
                                uuid = de_get_uuid32(element);
                                des_iterator_next(&prot_it);
                                switch (uuid){
                                    case BLUETOOTH_PROTOCOL_L2CAP:
                                        if (!des_iterator_has_more(&prot_it)) continue;
                                        de_element_get_uint16(des_iterator_get_element(&prot_it), &hid_control_psm);
                                        printf("HID Control PSM: 0x%04x\n", (int) hid_control_psm);
                                        break;
                                    default:
                                        break;
                                }
                            }
                            break;
                        case BLUETOOTH_ATTRIBUTE_ADDITIONAL_PROTOCOL_DESCRIPTOR_LISTS:
                            for (des_iterator_init(&attribute_list_it, attribute_value); des_iterator_has_more(&attribute_list_it); des_iterator_next(&attribute_list_it)) {                                    
                                if (des_iterator_get_type(&attribute_list_it) != DE_DES) continue;
                                des_element = des_iterator_get_element(&attribute_list_it);
                                for (des_iterator_init(&additional_des_it, des_element); des_iterator_has_more(&additional_des_it); des_iterator_next(&additional_des_it)) {                                    
                                    if (des_iterator_get_type(&additional_des_it) != DE_DES) continue;
                                    des_element = des_iterator_get_element(&additional_des_it);
                                    des_iterator_init(&prot_it, des_element);
                                    element = des_iterator_get_element(&prot_it);
                                    if (!element) continue;
                                    if (de_get_element_type(element) != DE_UUID) continue;
                                    uuid = de_get_uuid32(element);
                                    des_iterator_next(&prot_it);
                                    switch (uuid){
                                        case BLUETOOTH_PROTOCOL_L2CAP:
                                            if (!des_iterator_has_more(&prot_it)) continue;
                                            de_element_get_uint16(des_iterator_get_element(&prot_it), &hid_interrupt_psm);
                                            printf("HID Interrupt PSM: 0x%04x\n", (int) hid_interrupt_psm);
                                            break;
                                        default:
                                            break;
                                    }
                                }
                            }
                            break;
                        case BLUETOOTH_ATTRIBUTE_HID_DESCRIPTOR_LIST:
                            for (des_iterator_init(&attribute_list_it, attribute_value); des_iterator_has_more(&attribute_list_it); des_iterator_next(&attribute_list_it)) {
                                if (des_iterator_get_type(&attribute_list_it) != DE_DES) continue;
                                des_element = des_iterator_get_element(&attribute_list_it);
                                for (des_iterator_init(&additional_des_it, des_element); des_iterator_has_more(&additional_des_it); des_iterator_next(&additional_des_it)) {                                    
                                    if (des_iterator_get_type(&additional_des_it) != DE_STRING) continue;
                                    element = des_iterator_get_element(&additional_des_it);
                                    const uint8_t * descriptor = de_get_string(element);
                                    hid_descriptor_length = de_get_data_size(element);
                                    memcpy(hid_descriptor, descriptor, hid_descriptor_length);
                                    printf("HID Descriptor:\n");
                                    printf_hexdump(hid_descriptor, hid_descriptor_length);
                                }
                            }                        
                            break;
                        default:
                            break;
                    }
                }
            }
            break;
        case SDP_EVENT_QUERY_COMPLETE:
            if (!hid_control_psm) {
                break;
            }
            if (!hid_interrupt_psm) {
                break;
            }
            status = l2cap_create_channel(handleBtPacket, remote_addr, hid_control_psm, 48, &l2cap_hid_control_cid);

            if (status) {
                // connection failed
                notifyConnectionState(PadConnectionFailed);
            }
            break;
    }
}

static void handleBtPacket(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    uint8_t event;
    bd_addr_t event_addr;
    uint8_t status;
    uint16_t l2cap_cid;

    switch (packet_type) {
        case HCI_EVENT_PACKET:
            event = hci_event_packet_get_type(packet);
            switch (event) {
                case BTSTACK_EVENT_STATE:
                    if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                        sdp_client_query_uuid16(&handleSdpClientQueryResult, remote_addr, BLUETOOTH_SERVICE_CLASS_HUMAN_INTERFACE_DEVICE_SERVICE);
                    }
                    break;
                case HCI_EVENT_PIN_CODE_REQUEST:
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    gap_pin_code_response(event_addr, "0000");
                    break;
                case HCI_EVENT_USER_CONFIRMATION_REQUEST:
                    break;
                case L2CAP_EVENT_CHANNEL_OPENED:
                    printf("!!!!!!!!!!!!!!!!!channel opened\r\n");
                    status = packet[2];
                    if (status) {
                        // connection failed
                        notifyConnectionState(PadConnectionFailed);
                        break;
                    }

                    l2cap_cid = little_endian_read_16(packet, 13);
                    if (!l2cap_cid) break;
                    if (l2cap_cid == l2cap_hid_control_cid) {
                        status = l2cap_create_channel(handleBtPacket, remote_addr, hid_interrupt_psm, 48, &l2cap_hid_interrupt_cid);

                        if (status) {
                            // connection failed
                            notifyConnectionState(PadConnectionFailed);
                            break;
                        }
                    }
                    if (l2cap_cid == l2cap_hid_interrupt_cid) {
                        // connection succeeded
                        notifyConnectionState(PadConnected);
                    }
                    break;
                case L2CAP_EVENT_CHANNEL_CLOSED:
                    notifyConnectionState(PadDisconnected);
                    break;
            }
            break;
        case L2CAP_DATA_PACKET:
            // received controller data packet
            parseReport(packet, size);
            if (channel == l2cap_hid_interrupt_cid) {
                parseReport(packet, size);
            }
            break;
        default:
            break;
    }
}

static void parseReport(const uint8_t *report, uint16_t report_len) {
    if (report_len != ReportDefaultLength) return;
    if (report[0] != 0xa1 || report[1] != 0x01) return;

    EGamepadHat_t hat = (EGamepadHat_t)report[IdxReportHat];


    //EGamepadButtons_t buttons = (EGamepadButtons_t)report[IdxReportButton];
    EGamepadButtons_t buttons = (EGamepadButtons_t)((report[IdxReportButton] << 8) | report[IdxReportButton+1]);
    if (report[IdxReportButtonL2] == ValueL2R2On) buttons = (EGamepadButtons_t)(buttons | EGamepadButtons_t::ButtonL2);
    if (report[IdxReportButtonR2] == ValueL2R2On) buttons = (EGamepadButtons_t)(buttons | EGamepadButtons_t::ButtonR2);

    if (report[IdxReportAnalogLH] > 0)  
        analog_value[EGamepadStick_t::AnalogLH] = (int8_t)((int)report[IdxReportAnalogLH] - 128);
    else
        analog_value[EGamepadStick_t::AnalogLH] = -127;
    if (report[IdxReportAnalogLV] > 0)
        analog_value[EGamepadStick_t::AnalogLV] = (int8_t)((int)report[IdxReportAnalogLV] - 128);
    else
        analog_value[EGamepadStick_t::AnalogLV] = -127;
    if (report[IdxReportAnalogRH] > 0)
        analog_value[EGamepadStick_t::AnalogRH] = (int8_t)((int)report[IdxReportAnalogRH] - 128);
    else
        analog_value[EGamepadStick_t::AnalogRH] = -127;
    if (report[IdxReportAnalogRV] > 0)
        analog_value[EGamepadStick_t::AnalogRV] = (int8_t)((int)report[IdxReportAnalogRV] - 128);
    else
        analog_value[EGamepadStick_t::AnalogRV] = -127;

    notifyUpdateState(hat, buttons, (int8_t *)analog_value);
}

static void notifyConnectionState(EBtGamepadConnection_t connection) {
    connection_state = connection;

    if (connection_call != nullptr) {
        connection_call(connection);
    }
}

static void notifyUpdateState(EGamepadHat_t hat, EGamepadButtons_t buttons, int8_t* sticks) {
    if (update_state_call != nullptr) {
        update_state_call(hat, buttons, sticks);
    }
}


BtGamepad::BtGamepad() {
}

BtGamepad* BtGamepad::getInstance() {
    if (bt_gamepad == nullptr) {
        bt_gamepad = new BtGamepad();
    }

    return bt_gamepad;
}

void BtGamepad::initialize() {
    initBtHidHost();
    sscanf_bd_addr(BtGamepadMacAddr, remote_addr);
    hci_power_control(HCI_POWER_ON);
}

void BtGamepad::registerCallbacks(GamepadConnectionCallback_t connection_callback, GamepadUpdateStateCallback_t update_state_callback) {
    connection_call = connection_callback;
    update_state_call = update_state_callback;
}