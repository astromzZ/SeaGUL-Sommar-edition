#include <pgmspace.h>

const char page_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>SEAGUL</title>
  <style>
    body {
      margin: 0;
      padding: 0;
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      background-color: #1e1e1e;
      color: #e2e2e2;
      transition: background-color 0.3s, color 0.3s;
    }
    .header {
      background-color: #333;
      padding: 20px;
      text-align: center;
      position: relative;
    }
    .header h1 {
      margin: 0;
      font-size: 2.5em;
      color: #ffd700;  /* Yellow text for "SEAGUL" */
    }
    .mode-toggle {
      position: absolute;
      top: 10px;
      left: 10px;
      padding: 10px 20px;
      background-color: #ffd700;
      color: #1e1e1e;
      border: none;
      border-radius: 5px;
      cursor: pointer;
      font-size: 1em;
    }
    .container {
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 20px;
    }
    .status-card, .control-card {
      background: #2e2e2e;
      border-radius: 10px;
      box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
      margin: 20px;
      padding: 20px;
      width: 100%;
      max-width: 800px;  /* Wider boxes */
      text-align: center;
      transition: background 0.3s, color 0.3s;
    }
    .status-card h2, .control-card h2 {
      margin-top: 0;
      color: #cccccc;
    }
    .button-container {
      display: flex;
      justify-content: center;
      flex-wrap: wrap;
      margin-top: 20px;
    }
    .button {
      border: none;
      color: #e2e2e2;
      padding: 15px 30px;
      margin: 10px;
      text-align: center;
      text-decoration: none;
      display: inline-block;
      font-size: 1em;
      border-radius: 5px;
      transition: background-color 0.3s;
      cursor: pointer;
      flex: 1 0 30%;  /* Ensure equal width */
      max-width: 150px; /* Set a maximum width */
    }
    .button-container > .button {
      margin: 10px; /* Margin between buttons */
    }
    .button:hover {
      opacity: 0.9;
    }
    .initiate-dive {
      background-color: #1e90ff;  /* Blue button */
      color: white;
    }
    .initiate-dive:hover {
      background-color: #1c86ee;
    }
    .calibrate {
      background-color: #008000;  /* Green button */
      color: white;
    }
    .calibrate:hover {
      background-color: #006400;
    }
    .idle-button {
      background-color: #ff4500;  /* Orange button */
      color: white;
    }
    .idle-button:hover {
      background-color: #e03b00;
    }
    .test-button {
      background-color: #444;  /* Light gray for better visibility */
      color: #e2e2e2;
    }
    .test-button:hover {
      background-color: #555;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      margin: 20px 0;
      background: #333;
      color: #e2e2e2;
      table-layout: fixed;
      transition: background 0.3s, color 0.3s;
    }
    th, td {
      border: 1px solid #444;
      text-align: center;
      padding: 12px;
      font-size: 1em;
    }
    th {
      background-color: #444;
      color: #e2e2e2;
    }
    .fixed-table th, .fixed-table td {
      width: calc(100% / 3);
    }
    .light-mode {
      background-color: #ffffff;
      color: #000000;  /* High contrast text */
    }
    .light-mode .header {
      background-color: #333;  /* Keep the header background */
    }
    .light-mode .mode-toggle {
      background-color: #1e1e1e;
      color: #ffd700;
    }
    .light-mode .status-card, .light-mode .control-card {
      background: #f9f9f9;
      box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
      color: #333; /* Dark gray for better visibility */
    }
    .light-mode .status-card h2, .light-mode .control-card h2, .light-mode th, .light-mode td {
      color: #333; /* Dark gray for better visibility */
    }
    .light-mode .button {
      color: #1e1e1e;
    }
    .light-mode .initiate-dive {
      background-color: #1e90ff;
      color: white;
    }
    .light-mode .calibrate {
      background-color: #008000;
      color: white;
    }
    .light-mode .idle-button {
      background-color: #ff4500;  /* Orange button */
      color: white;
    }
    .light-mode .test-button {
      background-color: #ddd;  /* Light gray for better visibility */
    }
    .light-mode table {
      background: #f1f1f1;
      color: #333;  /* High contrast text */
    }
    .light-mode th {
      background-color: #e1e1e1;
    }
  </style>
</head>
<body>
  <div class="header">
    <h1>SEAGUL</h1>
    <button class="mode-toggle" onclick="toggleMode()">Light Mode</button>
  </div>
  <div class="container">
    <div class="status-card">
      <h2>Current State</h2>
      <p id="state">Idle</p>
    </div>
    <div class="status-card">
      <h2>Environmental Data</h2>
      <table class="fixed-table">
        <tr>
          <th>Water Temperature</th>
          <th>Pressure</th>
          <th>Salinity</th>
        </tr>
        <tr>
          <td id="waterTemperature"></td>
          <td id="pressure"></td>
          <td id="salinity"></td>
        </tr>
      </table>
    </div>
    <div class="status-card">
      <h2>Orientation Data</h2>
      <table class="fixed-table">
        <tr>
          <th>Pitch</th>
          <th>Roll</th>
          <th>Yaw</th>
        </tr>
        <tr>
          <td id="pitch"></td>
          <td id="roll"></td>
          <td id="yaw"></td>
        </tr>
      </table>
    </div>
    <div class="status-card">
      <h2>Navigation</h2>      
      <table class="fixed-table">
        <tr>
          <th>Compass Course</th>      
          <th>GNSS Coordinates</th>
        </tr>
        <tr>
          <td id="compassCourse"></td>
          <td id="gnssCoordinates"></td>
        </tr>
      </table>
    </div>
    <div class="control-card">
      <h2>Controls</h2>
      <div class="button-container">
        <button class="button idle-button" onclick="sendMessage('Idle')">Idle</button>
        <button class="button initiate-dive" onclick="sendMessage('Initiate Dive')">Initiate Dive</button>
        <button class="button calibrate" onclick="sendMessage('Calibrate')">Calibrate</button>
      </div>
      <div class="button-container">
        <button class="button test-button" onclick="sendMessage('Test 1')">Test 1</button>
        <button class="button test-button" onclick="sendMessage('Test 2')">Test 2</button>
        <button class="button test-button" onclick="sendMessage('Test 3')">Test 3</button>
        <button class="button test-button" onclick="sendMessage('Test 4')">Test 4</button>
      </div>
      <div class="button-container">
        <button class="button test-button" onclick="sendMessage('Test 5')">Test 5</button>
        <button class="button test-button" onclick="sendMessage('Test 6')">Test 6</button>
        <button class="button test-button" onclick="sendMessage('Test 7')">Test 7</button>
        <button class="button test-button" onclick="sendMessage('Test 8')">Test 8</button>
      </div>
    </div>
    <div class="status-card">
      <h2>Battery Voltage</h2>
      <p id="batteryVoltage"></p>
    </div>
    <div class="status-card">
      <h2>Internal Conditions</h2>
      <table class="fixed-table">
        <tr>
          <th>Internal Temperature</th>
          <th>Internal Pressure</th>
          <th>Internal Humidity</th>
        </tr>
        <tr>
          <td id="internalTemperature"></td>
          <td id="internalPressure"></td>
          <td id="internalHumidity"></td>
        </tr>
      </table>
    </div>
    <div class="status-card">
      <h2>Device Signal Strengths</h2>
      <div id="deviceStrengths"></div>
    </div>
  </div>
  <script>
    function fetchData() {
      fetch('/data')
      .then(response => response.json())
      .then(data => {
        document.getElementById('state').innerText = data.currentState;

        // Update device signal strengths
        const deviceContainer = document.getElementById('deviceStrengths');
        deviceContainer.innerHTML = ''; // Clear previous entries
        data.devices.forEach((device, index) => {
          const deviceElem = document.createElement('p');
          deviceElem.innerText = `Device ${index + 1} (${device.mac}): ${device.rssi} dBm`;
          deviceContainer.appendChild(deviceElem);
        });

        document.getElementById('waterTemperature').innerText = data.waterTemperature + " °C";
        document.getElementById('pressure').innerText = data.pressure + " bar";
        document.getElementById('salinity').innerText = data.salinity + " ppt";
        document.getElementById('pitch').innerText = data.pitch + "°";
        document.getElementById('roll').innerText = data.roll + "°";
        document.getElementById('yaw').innerText = data.yaw + "°";
        document.getElementById('batteryVoltage').innerText = data.batteryVoltage + " V";
        document.getElementById('compassCourse').innerText = data.compassCourse + "°";
        document.getElementById('gnssCoordinates').innerText = data.gnssCoordinates;
        document.getElementById('internalTemperature').innerText = data.internalTemperature + " °C";
        document.getElementById('internalPressure').innerText = data.internalPressure + " bar";
        document.getElementById('internalHumidity').innerText = data.internalHumidity + " %";
      });
    }

    function sendMessage(msg) {
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/update?state=" + msg, true);
      xhr.send();
    }

    function toggleMode() {
      document.body.classList.toggle('light-mode');
      const modeButton = document.querySelector('.mode-toggle');
      if (document.body.classList.contains('light-mode')) {
        modeButton.textContent = 'Dark Mode';
      } else {
        modeButton.textContent = 'Light Mode';
      }
    }

    setInterval(fetchData, 1000); // Fetch data every second
  </script>
</body>
</html>
)rawliteral";