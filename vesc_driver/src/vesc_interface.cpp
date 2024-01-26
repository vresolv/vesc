// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_driver/vesc_interface.hpp"

#include <algorithm>
#include <cassert>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "vesc_driver/vesc_packet_factory.hpp"
#include "serial_driver/serial_driver.hpp"

namespace vesc_driver
{

class VescInterface::Impl
{
public:
  Impl()
  : owned_ctx{new IoContext(2)},
    serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx)}
  {}
  void packet_creation_thread();
  void on_configure();
  void connect(const std::string & port);

  bool packet_thread_run_;
  std::unique_ptr<std::thread> packet_thread_;
  PacketHandlerFunction packet_handler_;
  ErrorHandlerFunction error_handler_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::string device_name_;
  std::unique_ptr<IoContext> owned_ctx{};
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  ~Impl()
  {
    if (owned_ctx) {
      owned_ctx->waitForExit();
    }
  }

private:
  std::vector<uint8_t> buffer_;
};

void VescInterface::Impl::packet_creation_thread()
{
  static auto temp_buffer = Buffer(2048, 0);
  while (packet_thread_run_) {
    const auto bytes_read = serial_driver_->port()->receive(temp_buffer);
    buffer_.reserve(buffer_.size() + temp_buffer.size());
    buffer_.insert(buffer_.end(), temp_buffer.begin(), temp_buffer.begin() + bytes_read);
    int bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;
    if (!buffer_.empty()) {
      // search buffer for valid packet(s)
      auto iter = buffer_.begin();
      auto iter_begin = buffer_.begin();
      while (iter != buffer_.end()) {
        // check if valid start-of-frame character
        if (VescFrame::VESC_SOF_VAL_SMALL_FRAME == *iter ||
          VescFrame::VESC_SOF_VAL_LARGE_FRAME == *iter)
        {
          // good start, now attempt to create packet
          std::string error;
          VescPacketConstPtr packet =
            VescPacketFactory::createPacket(iter, buffer_.end(), &bytes_needed, &error);
          if (packet) {
            // good packet, check if we skipped any data
            if (std::distance(iter_begin, iter) > 0) {
              std::ostringstream ss;
              ss << "Out-of-sync with VESC, unknown data leading valid frame. Discarding " <<
                std::distance(iter_begin, iter) << " bytes.";
              error_handler_(ss.str());
            }
            // call packet handler
            packet_handler_(packet);
            // update state
            iter = iter + packet->frame().size();
            iter_begin = iter;
            // continue to look for another frame in buffer
            continue;
          } else if (bytes_needed > 0) {
            // need more data, break out of while loop
            break;  // for (iter_sof...
          } else {
            // else, this was not a packet, move on to next byte
            error_handler_(error);
          }
        }

        iter++;
      }

      // if iter is at the end of the buffer, more bytes are needed
      if (iter == buffer_.end()) {
        bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;
      }

      // erase "used" buffer
      if (std::distance(iter_begin, iter) > 0) {
        std::ostringstream ss;
        ss << "Out-of-sync with VESC, discarding " << std::distance(iter_begin, iter) << " bytes.";
        error_handler_(ss.str());
      }
      buffer_.erase(buffer_.begin(), iter);
    }
    // Only attempt to read every 5 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void VescInterface::Impl::connect(const std::string & port)
{
  uint32_t baud_rate = 115200;
  auto fc = drivers::serial_driver::FlowControl::HARDWARE;
  auto pt = drivers::serial_driver::Parity::NONE;
  auto sb = drivers::serial_driver::StopBits::ONE;
  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
  serial_driver_->init_port(port, *device_config_);
  if (!serial_driver_->port()->is_open()) {
    serial_driver_->port()->open();
  }
}

VescInterface::VescInterface(
  const std::string & port,
  const PacketHandlerFunction & packet_handler,
  const ErrorHandlerFunction & error_handler)
: impl_(new Impl())
{
  setPacketHandler(packet_handler);
  setErrorHandler(error_handler);
  // attempt to conect if the port is specified
  if (!port.empty()) {
    connect(port);
  }

  mRxTimer = 0;
  mByteTimeout = 50;
  mMaxPacketLen = 10000;
  mRxReadPtr = 0;
  mRxWritePtr = 0;
  mBytesLeft = 0;
  mBufferLen = mMaxPacketLen + 8;
  mRxBuffer = new unsigned char[mBufferLen];

  mTimer = new QTimer(this);
  mTimer->setInterval(10);
  mTimer->start();

  connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));

  mLastConnType = static_cast<conn_t>(mSettings.value("connection_type", CONN_NONE).toInt());
  mLastTcpServer = mSettings.value("tcp_server", "127.0.0.1").toString();
  mLastTcpPort = mSettings.value("tcp_port", 65102).toInt();
  mLastTcpHubServer = mSettings.value("tcp_hub_server", "veschub.vedder.se").toString();
  mLastTcpHubPort = mSettings.value("tcp_hub_port", 65101).toInt();
  mLastTcpHubVescID = mSettings.value("tcp_hub_vesc_id", "").toString();
  mLastTcpHubVescPass = mSettings.value("tcp_hub_vesc_pass", "").toString();

  // TCP
  mTcpSocket = new QTcpSocket(this);
  mTcpConnected = false;

  connect(mTcpSocket, SIGNAL(readyRead()), this, SLOT(tcpInputDataAvailable()));
  connect(mTcpSocket, SIGNAL(connected()), this, SLOT(tcpInputConnected()));
  connect(mTcpSocket, SIGNAL(disconnected()), this, SLOT(tcpInputDisconnected()));
  connect(mTcpSocket, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(tcpInputError(QAbstractSocket::SocketError)));
}

void VescInterface::tcpInputDataAvailable()
{
    while (mTcpSocket->bytesAvailable() > 0) 
    {
      processData(mTcpSocket->readAll());// TODO Will Update this line
    }
}

void VescInterface::tcpInputConnected()
{
    mTcpSocket->setSocketOption(QAbstractSocket::LowDelayOption, true);

    if (!mLastTcpHubVescID.isEmpty()) {
        QString login = QString("VESCTOOL:%1:%2\n\0").arg(mLastTcpHubVescID).arg(mLastTcpHubVescPass);
        mTcpSocket->write(login.toLocal8Bit());

        mSettings.setValue("tcp_hub_server", mLastTcpHubServer);
        mSettings.setValue("tcp_hub_port", mLastTcpHubPort);
        mSettings.setValue("tcp_hub_vesc_id", mLastTcpHubVescID);
        mSettings.setValue("tcp_hub_vesc_pass", mLastTcpHubVescPass);
        setLastConnectionType(CONN_TCP_HUB);

        TCP_HUB_DEVICE devNow;
        devNow.server = mLastTcpHubServer;
        devNow.port = mLastTcpHubPort;
        devNow.id = mLastTcpHubVescID;
        devNow.password = mLastTcpHubVescPass;

        bool found = false;
        for (const auto &i: qAsConst(mTcpHubDevs)) {
            auto dev = i.value<TCP_HUB_DEVICE>();
            if (dev.uuid() == devNow.uuid()) {
                found = true;
                updateTcpHubPassword(dev.uuid(), mLastTcpHubVescPass);
                break;
            }
        }

        if (!found) {
            mTcpHubDevs.append(QVariant::fromValue(devNow));
            storeSettings();
        }
    } else {
        mSettings.setValue("tcp_server", mLastTcpServer);
        mSettings.setValue("tcp_port", mLastTcpPort);
        setLastConnectionType(CONN_TCP);
    }

    mTcpConnected = true;
    //updateFwRx(false); //TODO check if not needed then remove.
}

void VescInterface::tcpInputDisconnected()
{
    mTcpConnected = false;
    //updateFwRx(false); //TODO check and remove if not needed.
}

void VescInterface::tcpInputError(QAbstractSocket::SocketError socketError)
{
    (void)socketError;

    QString errorStr = mTcpSocket->errorString();
    //emit statusMessage(tr("TCP Error") + errorStr, false); TODO remove this line.
    mTcpSocket->close();

    std::stringstream ss;
    ss << "TCP Error " << errorStr  << e.what();
    throw SerialException(ss.str().c_str());
    ///updateFwRx(false); //TODO check and remove if not needed
}

void VescInterface::setLastConnectionType(conn_t type)
{
    mLastConnType = type;
    mSettings.setValue("connection_type", type);
}

bool VescInterface::updateTcpHubPassword(QString uuid, QString newPass)
{
    for (int i = 0;i < mTcpHubDevs.size();i++) {
        auto dev = mTcpHubDevs.at(i).value<TCP_HUB_DEVICE>();
        if (dev.uuid() == uuid) {
            dev.password = newPass;
            mTcpHubDevs[i] = QVariant::fromValue(dev);
            return true;
        }
    }

    return false;
}

void VescInterface::storeSettings()
{
    //TODO will remove if not needed
    // mSettings.remove("bleNames");
    // {
    //     mSettings.beginWriteArray("bleNames");
    //     QHashIterator<QString, QString> i(mBleNames);
    //     int ind = 0;
    //     while (i.hasNext()) {
    //         i.next();
    //         mSettings.setArrayIndex(ind);
    //         mSettings.setValue("address", i.key());
    //         mSettings.setValue("name", i.value());
    //         ind++;
    //     }
    //     mSettings.endArray();
    // }

    //TODO will remove if not needed
    // mSettings.remove("blePreferred");
    // {
    //     mSettings.beginWriteArray("blePreferred");
    //     QHashIterator<QString, bool> i(mBlePreferred);
    //     int ind = 0;
    //     while (i.hasNext()) {
    //         i.next();
    //         mSettings.setArrayIndex(ind);
    //         mSettings.setValue("address", i.key());
    //         mSettings.setValue("preferred", i.value());
    //         ind++;
    //     }
    //     mSettings.endArray();
    // }
    //TODO will remove if not needed
    // mSettings.remove("profiles");
    // mSettings.beginWriteArray("profiles");
    // for (int i = 0; i < mProfiles.size(); ++i) {
    //     auto cfg = mProfiles.value(i).value<MCCONF_TEMP>();
    //     mSettings.setArrayIndex(i);
    //     mSettings.setValue("current_min_scale", cfg.current_min_scale);
    //     mSettings.setValue("current_max_scale", cfg.current_max_scale);
    //     mSettings.setValue("erpm_or_speed_min", cfg.erpm_or_speed_min);
    //     mSettings.setValue("erpm_or_speed_max", cfg.erpm_or_speed_max);
    //     mSettings.setValue("duty_min", cfg.duty_min);
    //     mSettings.setValue("duty_max", cfg.duty_max);
    //     mSettings.setValue("watt_min", cfg.watt_min);
    //     mSettings.setValue("watt_max", cfg.watt_max);
    //     mSettings.setValue("name", cfg.name);
    // }
    // mSettings.endArray();

    mSettings.remove("tcpHubDevices");
    mSettings.beginWriteArray("tcpHubDevices");
    for (int i = 0; i < mTcpHubDevs.size(); ++i) {
        auto dev = mTcpHubDevs.value(i).value<TCP_HUB_DEVICE>();
        mSettings.setArrayIndex(i);
        mSettings.setValue("server", dev.server);
        mSettings.setValue("port", dev.port);
        mSettings.setValue("id", dev.id);
        mSettings.setValue("pw", dev.password);
    }
    mSettings.endArray();

    //TODO will remove if not needed
    // mSettings.beginWriteArray("pairedUuids");
    // for (int i = 0;i < mPairedUuids.size();i++) {
    //     mSettings.setArrayIndex(i);
    //     mSettings.setValue("uuid", mPairedUuids.at(i));
    // }
    // mSettings.endArray();

    // mSettings.remove("configurationBackups");
    // {
    //     mSettings.beginWriteArray("configurationBackups");
    //     QHashIterator<QString, CONFIG_BACKUP> i(mConfigurationBackups);
    //     int ind = 0;
    //     while (i.hasNext()) {
    //         i.next();
    //         mSettings.setArrayIndex(ind);
    //         mSettings.setValue("uuid", i.key());
    //         mSettings.setValue("mcconf", i.value().mcconf_xml_compressed);
    //         mSettings.setValue("appconf", i.value().appconf_xml_compressed);
    //         mSettings.setValue("customconf", i.value().customconf_xml_compressed);
    //         mSettings.setValue("name", i.value().name);
    //         ind++;
    //     }
    //     mSettings.endArray();
    // }

    // mSettings.setValue("useImperialUnits", mUseImperialUnits);
    // mSettings.setValue("keepScreenOn", mKeepScreenOn);
    // mSettings.setValue("useWakeLock", mUseWakeLock);
    // mSettings.setValue("loadQmlUiOnConnect", mLoadQmlUiOnConnect);
    // mSettings.setValue("darkMode", Utility::isDarkMode());
    // mSettings.setValue("allowScreenRotation", mAllowScreenRotation);
    // mSettings.setValue("speedGaugeUseNegativeValues", mSpeedGaugeUseNegativeValues);
    // mSettings.setValue("askQmlLoad", mAskQmlLoad);
    mSettings.sync();
}

void VescInterface::connectTcp(QString server, int port)
{
    mLastTcpServer = server;
    mLastTcpPort = port;
    mLastTcpHubVescID = "";
    mLastTcpHubVescPass = "";

    QHostAddress host;
    host.setAddress(server);

    // Try DNS lookup
    if (host.isNull()) {
        QList<QHostAddress> addresses = QHostInfo::fromName(server).addresses();

        if (!addresses.isEmpty()) {
            host.setAddress(addresses.first().toString());
        }
    }

    mTcpSocket->abort();
    mTcpSocket->connectToHost(host,port);
}

void VescInterface::connectTcpHub(QString server, int port, QString id, QString pass)
{
    mLastTcpHubServer = server;
    mLastTcpHubPort = port;
    mLastTcpHubVescID = id;
    mLastTcpHubVescPass = pass;

    QHostAddress host;
    host.setAddress(server);

    // Try DNS lookup
    if (host.isNull()) {
        QList<QHostAddress> addresses = QHostInfo::fromName(server).addresses();

        if (!addresses.isEmpty()) {
            host.setAddress(addresses.first().toString());
        }
    }

    mTcpSocket->abort();
    mTcpSocket->connectToHost(host,port);
}

void VescInterface::disconnectPort()
{
  if (mTcpConnected) {
      mTcpSocket->flush();
      mTcpSocket->close();
      updateFwRx(false);
  }

}

void VescInterface::processData(QByteArray data)
{
    QVector<QByteArray> decodedPackets;

    for(unsigned char rx_data: data) {
        mRxTimer = mByteTimeout;

        unsigned int data_len = mRxWritePtr - mRxReadPtr;

        // Out of space (should not happen)
        if (data_len >= mBufferLen) {
            mRxWritePtr = 0;
            mRxReadPtr = 0;
            mBytesLeft = 0;
            mRxBuffer[mRxWritePtr++] = rx_data;
            continue;
        }

        // Everything has to be aligned, so shift buffer if we are out of space.
        // (as opposed to using a circular buffer)
        if (mRxWritePtr >= mBufferLen) {
            memmove(mRxBuffer, mRxBuffer + mRxReadPtr, data_len);
            mRxReadPtr = 0;
            mRxWritePtr = data_len;
        }

        mRxBuffer[mRxWritePtr++] = rx_data;
        data_len++;

        if (mBytesLeft > 1) {
            mBytesLeft--;
            continue;
        }

        // Try decoding the packet at various offsets until it succeeds, or
        // until we run out of data.
        for (;;) {
            int res = try_decode_packet(mRxBuffer + mRxReadPtr, data_len,
                                        &mBytesLeft, decodedPackets);

            // More data is needed
            if (res == -2) {
                break;
            }

            if (res > 0) {
                data_len -= res;
                mRxReadPtr += res;
            } else if (res == -1) {
                // Something went wrong. Move pointer forward and try again.
                mRxReadPtr++;
                data_len--;
            }
        }

        // Nothing left, move pointers to avoid memmove
        if (data_len == 0) {
            mRxReadPtr = 0;
            mRxWritePtr = 0;
        }
    }

    for (QByteArray b: decodedPackets) {
        emit packetReceived(b);
    }
}

void VescInterface::timerSlot()
{
  if (mRxTimer) 
  {
    mRxTimer--;
  } else 
  {
    mRxReadPtr = 0;
    mRxWritePtr = 0;
    mBytesLeft = 0;
  }
}

int VescInterface::try_decode_packet(unsigned char *buffer, unsigned int in_len, int *bytes_left, QVector<QByteArray> &decodedPackets)
{
    *bytes_left = 0;

    if (in_len == 0) {
        *bytes_left = 1;
        return -2;
    }

    unsigned int data_start = buffer[0];
    bool is_len_8b = buffer[0] == 2;
    bool is_len_16b = buffer[0] == 3;
    bool is_len_24b = buffer[0] == 4;

    // No valid start byte
    if (!is_len_8b && !is_len_16b && !is_len_24b) {
        return -1;
    }

    // Not enough data to determine length
    if (in_len < data_start) {
        *bytes_left = data_start - in_len;
        return -2;
    }

    unsigned int len = 0;

    if (is_len_8b) {
        len = (unsigned int)buffer[1];

        // No support for zero length packets
        if (len < 1) {
            return -1;
        }
    } else if (is_len_16b) {
        len = (unsigned int)buffer[1] << 8 | (unsigned int)buffer[2];

        // A shorter packet should use less length bytes
        if (len < 255) {
            return -1;
        }
    } else if (is_len_24b) {
        len = (unsigned int)buffer[1] << 16 |
              (unsigned int)buffer[2] << 8 |
              (unsigned int)buffer[3];

        // A shorter packet should use less length bytes
        if (len < 65535) {
            return -1;
        }
    }

    // Too long packet
    if (len > mMaxPacketLen) {
        return -1;
    }

    // Need more data to determine rest of packet
    if (in_len < (len + data_start + 3)) {
        *bytes_left = (len + data_start + 3) - in_len;
        return -2;
    }

    // Invalid stop byte
    if (buffer[data_start + len + 2] != 3) {
        return -1;
    }

    unsigned short crc_calc = crc16(buffer + data_start, len);
    unsigned short crc_rx = (unsigned short)buffer[data_start + len] << 8
                          | (unsigned short)buffer[data_start + len + 1];

    if (crc_calc == crc_rx) {
        QByteArray res((const char*)(buffer + data_start), (int)len);
        decodedPackets.append(res);
        return len + data_start + 3;
    } else {
        return -1;
    }
}

void Commands::processPacket(QByteArray data)
{
    VByteArray vb(data);
    COMM_PACKET_ID id = COMM_PACKET_ID(vb.vbPopFrontUint8());

    switch (id) {
    case COMM_FW_VERSION:break;

    case COMM_ERASE_NEW_APP:break;

    case COMM_WRITE_NEW_APP_DATA:break;

    case COMM_ERASE_BOOTLOADER:break;

    case COMM_GET_VALUES:
    case COMM_GET_VALUES_SELECTIVE:break;

    case COMM_PRINT:break;

    case COMM_SAMPLE_PRINT:break;

    case COMM_ROTOR_POSITION:
       double motor_rotor_position = vb.vbPopFrontDouble32(1e5)
       std::stringstream ss;
       ss << "Received Motor Rotor Position: " << motor_rotor_position;
       throw SerialException(ss.str().c_str());
      break;

    default:
        break;
    }
}

VescInterface::~VescInterface()
{
  disconnect();
}

void VescInterface::setPacketHandler(const PacketHandlerFunction & handler)
{
  // todo - definately need mutex
  impl_->packet_handler_ = handler;
}

void VescInterface::setErrorHandler(const ErrorHandlerFunction & handler)
{
  // todo - definately need mutex
  impl_->error_handler_ = handler;
}

void VescInterface::connect(const std::string & port)
{
  // todo - mutex?

  if (isConnected()) {
    throw SerialException("Already connected to serial port.");
  }

  // connect to serial port
  try {
    impl_->connect(port);
  } catch (const std::exception & e) {
    std::stringstream ss;
    ss << "Failed to open the serial port " << port << " to the VESC. " << e.what();
    throw SerialException(ss.str().c_str());
  }

  // start up a monitoring thread
  impl_->packet_thread_run_ = true;
  impl_->packet_thread_ = std::unique_ptr<std::thread>(
    new std::thread(
      &VescInterface::Impl::packet_creation_thread, impl_.get()));
}

void VescInterface::disconnect()
{
  // todo - mutex?

  if (isConnected()) {
    // bring down read thread
    impl_->packet_thread_run_ = false;
    requestFWVersion();
    impl_->packet_thread_->join();
    impl_->serial_driver_->port()->close();
  }
}

bool VescInterface::isConnected() const
{
  auto port = impl_->serial_driver_->port();
  if (port) {
    return port->is_open();
  } else {
    return false;
  }
}

void VescInterface::send(const VescPacket & packet)
{
  impl_->serial_driver_->port()->async_send(packet.frame());
}

void VescInterface::requestFWVersion()
{
  send(VescPacketRequestFWVersion());
}

void VescInterface::requestState()
{
  send(VescPacketRequestValues());
}

void VescInterface::setDutyCycle(double duty_cycle)
{
  send(VescPacketSetDuty(duty_cycle));
}

void VescInterface::setCurrent(double current)
{
  send(VescPacketSetCurrent(current));
}

void VescInterface::setBrake(double brake)
{
  send(VescPacketSetCurrentBrake(brake));
}

void VescInterface::setSpeed(double speed)
{
  send(VescPacketSetRPM(speed));
}

void VescInterface::setPosition(double position)
{
  send(VescPacketSetPos(position));
}

void VescInterface::setServo(double servo)
{
  send(VescPacketSetServoPos(servo));
}

void VescInterface::requestImuData()
{
  send(VescPacketRequestImu());
}

}  // namespace vesc_driver
