const dgram = require('dgram');

msgToEsp = Buffer.from('0');
const udpServer = dgram.createSocket('udp4');

//from Peter

const { SerialPort } = require('serialport');
var http = require('http');
const fs = require('fs');
const express = require('express');
const socketIo = require('socket.io');
const app = express();
const server = http.createServer(app);
const io = socketIo(server);

let latestData = {}
let flag = false;

//end

udpServer.on('error', (err) => {
  console.error(`UDP server error:\n${err.stack}`);
  udpServer.close();
});

udpServer.on('message', (msg, rinfo) => {
  console.log(`Received UDP message from ${rinfo.address}:${rinfo.port}: ${msg}`);
  if (flag == 0) {
    msgToEsp = Buffer.from('0');
  }
  else {
    msgToEsp = Buffer.from('1');
  }
  udpServer.send(msgToEsp, rinfo.port, rinfo.address);
  const [seconds, temp, roll, pitch, voltage] = msg.toString().split(',');
    const data = {
        seconds: parseInt(seconds, 10),
        temp: parseFloat(temp),
        roll: parseFloat(roll),
        pitch: parseFloat(pitch),
        voltage: parseFloat(voltage)
    };
    latestData = data;
    console.log(latestData);
    io.emit('data', latestData);
    
});

udpServer.on('listening', () => {
  const address = udpServer.address();
  console.log(`UDP server listening on ${address.address}:${address.port}`);
});

udpServer.bind(3333); // UDP port to listen on

app.get('/', (req, res) => {
  res.sendFile(__dirname + '/graph.html');
});

io.on('connection', (socket) => {
  console.log('A user connected');
  socket.emit('data', latestData);
  socket.on('toggleFlag', () => {
    flag = !flag; // Toggle the flag
    console.log('Flag toggled:', flag);
  });
});


server.listen(3000, () => {
  console.log('Server is running on http://localhost:3000');
});
