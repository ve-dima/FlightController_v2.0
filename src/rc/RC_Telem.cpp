#include "RC.hpp"
#include "Board.hpp"
#include "I-Bus.hpp"
#include "SRT/SRT.hpp"
#include "Common.hpp"
#include <algorithm>

namespace RCTelem
{
    IBus iBusTelem;

    void idleCallbackHandler()
    {
        while (rcTelemUart.available())
        {
            uint8_t buff[64];
            size_t count = rcTelemUart.readBytes(buff, std::min<size_t>(sizeof(buff), rcTelemUart.available()));

            int16_t ch[RC::maxChannelCount];
            unsigned channelCount;
            uint8_t rssi;
            bool signalAvailable;

            iBusTelem.parseData(buff, count, rcTelemUart.getParityErrorFlag(),
                                ch, channelCount, rssi, signalAvailable);
            rcTelemUart.clearParityErrorFlag();

            if (iBusTelem.sendTelemetry(buff, count))
            {
                rcTelemUart.setRXEnable(false);
                rcTelemUart.setTXEnable(true);
                rcTelemUart.write(buff, count);
            }
        }
    }

    void transferCompleteCallback()
    {
        rcTelemUart.setTXEnable(false);
        rcTelemUart.setRXEnable(true);
    }

    void enable()
    {
        rcTelemUart.setHalfDuplex(true);

        rcTelemUart.begin(115'200);
        rcTelemUart.setTXEnable(false);
        rcTelemUart.setRXEnable(true);

        rcTelemUart.idleCallback = idleCallbackHandler;
        rcTelemUart.transferCompleteCallback = transferCompleteCallback;
    }

    void init() {}
    void handler()
    {
        iBusTelem.sensorHandler();
    }

    REGISTER_SRT_MODULE(RCTelem, init, enable, handler);
}