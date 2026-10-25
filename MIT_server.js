const express = require('express');
const cors = require('cors');
const { SerialPort } = require('serialport');

const app = express();
app.use(cors());
app.use(express.json());
app.use(express.static(__dirname));

// TODO: update this if arduino shows up different port
const portPath = process.env.ROBOT_PORT || '/dev/cu.usbserial-0001';

const arduino = new SerialPort({ 
  path: portPath, 
  baudRate: 9600, 
  autoOpen: true 
});

arduino.on('open', () => {
  console.log('Serial port open:', portPath);
});

arduino.on('error', (err) => {
  console.error('Serial error:', err.message);
});

arduino.on('data', (data) => {
  console.log('Arduino:', data.toString().trim());
});

const speedMap = { 1: 40, 2: 80, 3: 120, 4: 170, 5: 220 };

app.get('/', (req, res) => {
  res.redirect('/jointjogger.html');
});

app.post('/send', (req, res) => {
  const { angles, speedLevel } = req.body;

  if (!angles || !Array.isArray(angles) || angles.length !== 6) {
    return res.status(400).send('Need an array of 6 joint angles');
  }

  const speed = speedMap[speedLevel] || 120;
  const payload = angles.map(v => Number(v)).join(',') + ',' + speed + '\n';

  if (!arduino.isOpen) {
    return res.status(503).send('Serial port not open');
  }

  arduino.write(payload, (err) => {
    if (err) {
      console.error('Write failed:', err);
      return res.status(500).send('Failed to write to serial');
    }
    console.log('Sent:', payload.trim());
    res.send('OK');
  });
});

app.get('/status', (req, res) => {
  res.json({ 
    online: true, 
    port: portPath, 
    isOpen: arduino.isOpen 
  });
});

app.listen(3000, () => {
  console.log('Server running at http://localhost:3000');
});
