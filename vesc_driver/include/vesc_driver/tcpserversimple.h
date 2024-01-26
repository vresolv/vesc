/*
    Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

    This file is part of VESC Tool.

    VESC Tool is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    VESC Tool is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef TCPSERVERSIMPLE_H
#define TCPSERVERSIMPLE_H

#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
//#include "packet.h" //TODO remove this line

class TcpServerSimple : public QObject
{
    Q_OBJECT
public:
    explicit TcpServerSimple(QObject *parent = nullptr);
    Q_INVOKABLE bool startServer(int port, QHostAddress addr = QHostAddress::Any);
    Q_INVOKABLE bool connectToHub(QString server, int port, QString id, QString pass);
    Q_INVOKABLE void stopServer();
    Q_INVOKABLE bool sendData(const QByteArray &data);
    Q_INVOKABLE QString errorString();
    //Packet *packet(); //TODO Remove this line 
    bool usePacket() const; //TODO check if not required remove it
    void setUsePacket(bool usePacket); //TODO check if not required remove it.
    Q_INVOKABLE bool isClientConnected();
    Q_INVOKABLE QString getConnectedClientIp();
    Q_INVOKABLE bool isServerRunning();
    int lastPort() const;

signals:
    void dataRx(const QByteArray &data);
    void connectionChanged(bool connected, QString address);

public slots:
    void newTcpConnection();
    void tcpInputDisconnected();
    void tcpInputDataAvailable();
    void tcpInputError(QAbstractSocket::SocketError socketError);
    void dataToSend(QByteArray &data);

private:
    QTcpServer *mTcpServer;
    QTcpSocket *mTcpSocket;
    //Packet *mPacket; //TODO remove it
    bool mUsePacket;
    int mLastPort;

};

#endif // TCPSERVERSIMPLE_H
