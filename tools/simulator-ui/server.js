import { Server } from "socket.io";
import dgram from "dgram";
import express from "express";
import http from "http";
import path from "path";
import compression from "compression";
import { fileURLToPath } from "url";

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

const HTTP_PORT = 3001;
const UDP_LISTEN_PORT = 9000;
const UDP_SEND_PORT = 9001;
const C_PLUS_PLUS_IP = "127.0.0.1";

// 1. Setup Express and HTTP Server for static UI and Socket.IO
const app = express();
const httpServer = http.createServer(app);

// Serve the static files
app.use(express.static(path.join(__dirname, "public")));

const io = new Server(httpServer, {
  cors: { origin: "*" }
});
const udpServer = dgram.createSocket("udp4");
const udpClient = dgram.createSocket("udp4");

udpServer.on("error", (err) => {
  console.error(`UDP server error:\n${err.stack}`);
  udpServer.close();
});

// Forward Telemetry (C++ -> Web)
let latestTelemetry = {};
let packetCount = 0;
udpServer.on("message", (msg, rinfo) => {
  try {
    latestTelemetry = JSON.parse(msg.toString());
    packetCount++;
    if (packetCount % 60 === 0) {
       console.log("Telemetry packet received: " + packetCount);
    }
    // Broadcast to all connected web clients via Socket.IO
    io.emit("telemetry", latestTelemetry);
  } catch (e) {
    console.error("Failed to parse UDP telemetry:", e);
  }
});

// HTTP fallback for strict proxies (like VS Code server)
app.get('/telemetry', (req, res) => {
    res.json(latestTelemetry);
});

// Command fallback
app.use(express.json());
app.post('/command', (req, res) => {
    const msg = Buffer.from(JSON.stringify({ type: "command", data: req.body }));
    udpClient.send(msg, UDP_SEND_PORT, C_PLUS_PLUS_IP);
    res.sendStatus(200);
});

app.post('/config', (req, res) => {
    const msg = Buffer.from(JSON.stringify({ type: "config", data: req.body }));
    udpClient.send(msg, UDP_SEND_PORT, C_PLUS_PLUS_IP);
    res.sendStatus(200);
});

udpServer.bind(UDP_LISTEN_PORT, () => {
  console.log(`UDP Server listening on ${UDP_LISTEN_PORT}`);
});

// 3. Forward Commands/Config (Web -> C++)
io.on("connection", (socket) => {
  console.log("Web client connected:", socket.id);

  socket.on("command", (data) => {
    const msg = Buffer.from(JSON.stringify({ type: "command", data }));
    udpClient.send(msg, UDP_SEND_PORT, C_PLUS_PLUS_IP);
  });

  socket.on("config", (data) => {
    const msg = Buffer.from(JSON.stringify({ type: "config", data }));
    udpClient.send(msg, UDP_SEND_PORT, C_PLUS_PLUS_IP);
  });

  socket.on("disconnect", () => {
    console.log("Web client disconnected:", socket.id);
  });
});

httpServer.listen(HTTP_PORT, () => {
  console.log(`Web UI and Socket.IO server running on port ${HTTP_PORT}`);
});
