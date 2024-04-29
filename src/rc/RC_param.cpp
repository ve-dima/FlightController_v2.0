#include "RC.hpp"
#include "param/param.hpp"
namespace RC
{
    extern int32_t _channelsAssign[static_cast<int>(ChannelFunction::__end)];
    extern float _minChannelValue[maxChannelCount],
        _maxChannelValue[maxChannelCount],
        _channelDeadZone[maxChannelCount],
        _channelIsReverse[maxChannelCount];
    extern ProtocolDetector _selectedProtocol;
    static_assert(sizeof(int32_t) == sizeof(_selectedProtocol), "ban");

    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC1_MIN, &RC::_minChannelValue[0], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC2_MIN, &RC::_minChannelValue[1], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC3_MIN, &RC::_minChannelValue[2], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC4_MIN, &RC::_minChannelValue[3], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC5_MIN, &RC::_minChannelValue[4], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC6_MIN, &RC::_minChannelValue[5], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC7_MIN, &RC::_minChannelValue[6], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC8_MIN, &RC::_minChannelValue[7], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC9_MIN, &RC::_minChannelValue[8], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC10_MIN, &RC::_minChannelValue[9], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC11_MIN, &RC::_minChannelValue[10], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC12_MIN, &RC::_minChannelValue[11], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC13_MIN, &RC::_minChannelValue[12], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC14_MIN, &RC::_minChannelValue[13], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC15_MIN, &RC::_minChannelValue[14], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC16_MIN, &RC::_minChannelValue[15], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC17_MIN, &RC::_minChannelValue[16], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC18_MIN, &RC::_minChannelValue[17], RC::checkValues);

    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC1_MAX, &RC::_maxChannelValue[0], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC2_MAX, &RC::_maxChannelValue[1], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC3_MAX, &RC::_maxChannelValue[2], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC4_MAX, &RC::_maxChannelValue[3], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC5_MAX, &RC::_maxChannelValue[4], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC6_MAX, &RC::_maxChannelValue[5], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC7_MAX, &RC::_maxChannelValue[6], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC8_MAX, &RC::_maxChannelValue[7], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC9_MAX, &RC::_maxChannelValue[8], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC10_MAX, &RC::_maxChannelValue[9], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC11_MAX, &RC::_maxChannelValue[10], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC12_MAX, &RC::_maxChannelValue[11], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC13_MAX, &RC::_maxChannelValue[12], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC14_MAX, &RC::_maxChannelValue[13], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC15_MAX, &RC::_maxChannelValue[14], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC16_MAX, &RC::_maxChannelValue[15], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC17_MAX, &RC::_maxChannelValue[16], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC18_MAX, &RC::_maxChannelValue[17], RC::checkValues);

    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC1_DZ, &RC::_channelDeadZone[0], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC2_DZ, &RC::_channelDeadZone[1], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC3_DZ, &RC::_channelDeadZone[2], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC4_DZ, &RC::_channelDeadZone[3], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC5_DZ, &RC::_channelDeadZone[4], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC6_DZ, &RC::_channelDeadZone[5], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC7_DZ, &RC::_channelDeadZone[6], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC8_DZ, &RC::_channelDeadZone[7], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC9_DZ, &RC::_channelDeadZone[8], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC10_DZ, &RC::_channelDeadZone[9], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC11_DZ, &RC::_channelDeadZone[10], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC12_DZ, &RC::_channelDeadZone[11], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC13_DZ, &RC::_channelDeadZone[12], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC14_DZ, &RC::_channelDeadZone[13], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC15_DZ, &RC::_channelDeadZone[14], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC16_DZ, &RC::_channelDeadZone[15], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC17_DZ, &RC::_channelDeadZone[16], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC18_DZ, &RC::_channelDeadZone[17], RC::checkValues);

    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC1_REV, &RC::_channelIsReverse[0], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC2_REV, &RC::_channelIsReverse[1], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC3_REV, &RC::_channelIsReverse[2], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC4_REV, &RC::_channelIsReverse[3], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC5_REV, &RC::_channelIsReverse[4], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC6_REV, &RC::_channelIsReverse[5], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC7_REV, &RC::_channelIsReverse[6], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC8_REV, &RC::_channelIsReverse[7], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC9_REV, &RC::_channelIsReverse[8], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC10_REV, &RC::_channelIsReverse[9], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC11_REV, &RC::_channelIsReverse[10], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC12_REV, &RC::_channelIsReverse[11], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC13_REV, &RC::_channelIsReverse[12], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC14_REV, &RC::_channelIsReverse[13], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC15_REV, &RC::_channelIsReverse[14], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC16_REV, &RC::_channelIsReverse[15], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC17_REV, &RC::_channelIsReverse[16], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::FLOAT, RC18_REV, &RC::_channelIsReverse[17], RC::checkValues);

    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_THROTTLE, &RC::_channelsAssign[0], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_ROLL, &RC::_channelsAssign[1], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_PITCH, &RC::_channelsAssign[2], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_YAW, &RC::_channelsAssign[3], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_RETURN, &RC::_channelsAssign[4], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_LOITER, &RC::_channelsAssign[5], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_OFFBOARD, &RC::_channelsAssign[6], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_FLAPS, &RC::_channelsAssign[7], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_AUX_1, &RC::_channelsAssign[8], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_AUX_2, &RC::_channelsAssign[9], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_AUX_3, &RC::_channelsAssign[10], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_AUX_4, &RC::_channelsAssign[11], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_AUX_5, &RC::_channelsAssign[12], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_AUX_6, &RC::_channelsAssign[13], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_PARAM_1, &RC::_channelsAssign[14], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_PARAM_2, &RC::_channelsAssign[15], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_PARAM_3_5, &RC::_channelsAssign[16], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_KILLSWITCH, &RC::_channelsAssign[17], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_TRANSITION, &RC::_channelsAssign[18], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_GEAR, &RC::_channelsAssign[19], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_ARMSWITCH, &RC::_channelsAssign[20], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_FLTBTN_SLOT_1, &RC::_channelsAssign[21], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_FLTBTN_SLOT_2, &RC::_channelsAssign[22], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_FLTBTN_SLOT_3, &RC::_channelsAssign[23], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_FLTBTN_SLOT_4, &RC::_channelsAssign[24], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_FLTBTN_SLOT_5, &RC::_channelsAssign[25], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_FLTBTN_SLOT_6, &RC::_channelsAssign[26], RC::checkValues);
    PARAM_ADD_WITH_CALLBACK(param::INT32, RC_MAP_ENGAGE_MAIN_MOTOR, &RC::_channelsAssign[27], RC::checkValues);
}