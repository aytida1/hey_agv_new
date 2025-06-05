#!/usr/bin/env node

const express = require('express');
const http = require('http');
const { createProxyMiddleware } = require('http-proxy-middleware');
const path = require('path');

const app = express();
const server = http.createServer(app);

// Serve static files from web_interface directory
app.use(express.static(path.join(__dirname, '../web_interface')));

// Proxy API requests to Pharmacy Dock Server
app.use('/api/dock', createProxyMiddleware({
    target: 'http://localhost:5000',
    changeOrigin: true,
    pathRewrite: {
        '^/api/dock': '',
    },
}));

// Proxy WebSocket connections to ROSBridge
app.use('/rosbridge', createProxyMiddleware({
    target: 'ws://localhost:9091',
    ws: true,
    changeOrigin: true,
    pathRewrite: {
        '^/rosbridge': '',
    },
}));

const PORT = 3000;

server.listen(PORT, () => {
    console.log(`AGV Control Center running on http://localhost:${PORT}`);
    console.log('WebSocket proxy available at ws://localhost:3000/rosbridge');
});

// Handle WebSocket upgrade
server.on('upgrade', (request, socket, head) => {
    if (request.url === '/rosbridge') {
        // Proxy WebSocket connections to ROSBridge
        const WebSocket = require('ws');
        const rosbridge = new WebSocket('ws://localhost:9091');
        
        rosbridge.on('open', () => {
            socket.write('HTTP/1.1 101 Switching Protocols\r\n' +
                        'Upgrade: websocket\r\n' +
                        'Connection: Upgrade\r\n' +
                        '\r\n');
        });
        
        rosbridge.on('message', (data) => {
            socket.write(data);
        });
        
        socket.on('data', (data) => {
            rosbridge.send(data);
        });
        
        socket.on('close', () => {
            rosbridge.close();
        });
        
        rosbridge.on('close', () => {
            socket.end();
        });
    }
});