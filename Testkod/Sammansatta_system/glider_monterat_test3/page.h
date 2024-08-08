#include <pgmspace.h>

const char page_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>SEAGUL</title>
  <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.1.1/css/all.min.css">
  <style>
    body {
      margin: 0;
      padding: 0;
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      background-color: #1e1e1e;
      color: #e2e2e2;
      transition: background-color 0.3s, color 0.3s;
      justify-content: center;
      align-items: center;
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
    .control-button {
      background-color: #333;  /* Dark gray for better visibility */
      color: #e2e2e2;
    }
    .control-button:hover {
      background-color: #444;
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
    .loading-bar-container {
      width: 100%;
      background-color: #ddd;
      border-radius: 5px;
      margin: 20px 0;
      overflow: hidden;
    }
    .loading-bar {
      height: 30px;
      background-color: #4caf50;
      width: 0; /* Initially 0 width, will be adjusted by JavaScript */
      border-radius: 5px;
      text-align: center;
      color: white;
      line-height: 30px; /* Match the height of the bar */
      transition: width 0.3s ease; /* Smooth transition on width change */
    }
    .loading-bar-text {
      position: relative;
      z-index: 1;
      color: white;
      text-align: center;
      line-height: 30px; /* Match the height of the bar */
      font-weight: bold;
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
      <h2>Last error message</h2>
      <p id="errorMessage">No errors</p>
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

    <div class="control-card">
      <h2>Arrow Controls</h2>
      <div class="button-container">
        <button class="button control-button" ontouchstart = "startRotateLeft()" ontouchend = "stopRotateLeft()"><i class="fa-solid fa-arrow-rotate-left"></i></button>
        <button class="button control-button" ontouchstart = "startMoveForward()" ontouchend = "stopMoveForward()"><i class="fa-solid fa-arrow-left"></i></button>
        <button class="button control-button" ontouchstart = "startMoveBackward()" ontouchend = "stopMoveBackward()"><i class="fa-solid fa-arrow-right"></i></button>
        <button class="button control-button" ontouchstart = "startRotateRight()" ontouchend = "stopRotateRight()"><i class="fa-solid fa-arrow-rotate-right"></i></button>
      </div>
      <table class="fixed-table">
        <tr>
          <th>Rotation of mass (steps)</th>      
          <th>Displacement of mass (steps)</th>
        </tr>
        <tr>
          <td id="rotationAngle"></td>
          <td id="translationPosition"></td>
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

    <div class="status-card">
      <h2>Soft Potentiometer ADC Value</h2>
      <div class="loading-bar-container">
        <div class="loading-bar" id="loadingBar"></div>
        <div class="loading-bar-text" id="loadingBarText">0%</div>
      </div>
      <div class="button-container">
        <button class="button pump-toggle" onclick="togglePump()">Toggle Pump</button>
        <button class="button vent-toggle" onclick="toggleVent()">Open/Close Vent</button>
      </div>
      <form id="adcForm" onsubmit="sendADCValue(); return false;">
        <label for="adcInput">Enter desired percentage (0-100):</label>
        <input type="number" id="adcInput" name="adcInput" min="0" max="100" required>
        <button type="submit">Set value</button>
      </form>
      <button onclick="setTargetPotentiometerValue()">Set Target Potentiometer Value</button>
      <span id="potentiometerValueDisplay">Value: 0</span>
    </div>

    <div class="status-card">
      <h2>Battery Voltage</h2>
      <p id="batteryVoltage"></p>
      <div class = "button-container">
        <button class="button test-button" onclick="checkBat()">Check battery</button>
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
  <!-- </div> Close the container div here -->

    <div class="control-card">
      <h2>Controls</h2>
      <div class="button-container">
        <button class="button idle-button" onclick="enterIdle()">Idle</button>
        <button class="button initiate-dive" onclick="enterDive()">Initiate Dive</button>
        <button class="button calibrate" onclick="enterCalibration()">Calibrate</button>
        <button class="button test-button" onclick="enterSetup()">Setup</button>
        <button class="button test-button" onclick="dropweight()">Dropweight</button>
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
        document.getElementById('pressure').innerText = data.pressure + " m";
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
        document.getElementById('rotationAngle').innerText = data.rotationAngle + " steps";
        document.getElementById('translationPosition').innerText = data.translationPosition + " steps";

        const adcValue = data.adcValue;
        const percentage = (adcValue / 4095) * 100; // Calculate percentage
        const loadingBar = document.getElementById('loadingBar');
        const loadingBarText = document.getElementById('loadingBarText');
        loadingBar.style.width = percentage + '%'; // Set the width of the loading bar
        loadingBarText.textContent = Math.round(percentage) + '%'; // Update the text percentage
      });
    }

    window.addEventListener('load', function() {
        fetchErrorMessage();
        setInterval(fetchErrorMessage, 5000); // Fetch every 5 seconds (adjust as needed)
    });

    function fetchErrorMessage() {
        var xhr = new XMLHttpRequest();
        xhr.onreadystatechange = function() {
            if (xhr.readyState == XMLHttpRequest.DONE) {
                if (xhr.status == 200) {
                    var errorMessage = xhr.responseText.trim();
                    if (errorMessage.length > 0) {
                        document.getElementById('errorMessage').textContent = errorMessage;
                    }
                } else {
                    console.error('Error fetching error message:', xhr.status);
                }
            }
        };
        xhr.open('GET', '/errorMessage', true); // Endpoint to fetch error message
        xhr.send();
    }

    let pumpState = false;
    let ventState = false;

    function togglePump() {
      pumpState = !pumpState;
      const pumpButton = document.querySelector('.pump-toggle');
      const pumpStateString = pumpState ? 'on' : 'off';

      // Update button class based on state
      if (pumpState) {
        pumpButton.classList.remove('toggle-off');
        pumpButton.classList.add('toggle-on');
      } else {
        pumpButton.classList.remove('toggle-on');
        pumpButton.classList.add('toggle-off');
      }

      sendMessage('IDLE: Pump ' + pumpStateString);
      fetch('/togglePump?state=' + pumpStateString, {
        method: 'GET',
      })
        .then((response) => {
          if (!response.ok) {
            throw new Error('Network response was not ok');
          }
          return response.text();
        })
        .then((data) => {
          console.log('Pump state changed:', data);
        })
        .catch((error) => {
          console.error('Error:', error);
        });
    }

    function toggleVent() {
      ventState = !ventState;
      const ventButton = document.querySelector('.vent-toggle');
      const ventStateString = ventState ? 'open' : 'closed';

      // Update button class based on state
      if (ventState) {
        ventButton.classList.remove('toggle-off');
        ventButton.classList.add('toggle-on');
      } else {
        ventButton.classList.remove('toggle-on');
        ventButton.classList.add('toggle-off');
      }

      sendMessage('IDLE: Vent ' + ventStateString);
      fetch('/toggleVent?state=' + ventStateString, {
        method: 'GET',
      })
        .then((response) => {
          if (!response.ok) {
            throw new Error('Network response was not ok');
          }
          return response.text();
        })
        .then((data) => {
          console.log('Vent state changed:', data);
        })
        .catch((error) => {
          console.error('Error:', error);
        });
    }


    // function sendMessage(msg) {
    //   var xhr = new XMLHttpRequest();
    //   xhr.open("GET", "/update?state=" + msg, true);
    //   xhr.send();
    // }
  
    // function initiateDive() {
    //   sendMessage('Initiate Dive');
    //   setTimeout(() => {
    //     sendMessage('Diving');
    //   }, 1000); // 1 second delay
    // }

    function toggleMode() {
      document.body.classList.toggle('light-mode');
      const modeButton = document.querySelector('.mode-toggle');
      if (document.body.classList.contains('light-mode')) {
        modeButton.textContent = 'Dark Mode';
      } else {
        modeButton.textContent = 'Light Mode';
      }
    }

    function checkBat() {
      sendMessage('Check battery');
      fetch('/checkBattery', { method: 'GET' })
        .then(response => {
          if (!response.ok) {
            throw new Error('Network response was not ok');
          }
          console.log('Battery checked');
        })
        .catch(error => console.error('Error:', error));
    }

    function enterIdle() {
      sendMessage('Idle');
      fetch('/idle', { method: 'GET' })
        .then(response => {
          if (!response.ok) {
            throw new Error('Network response was not ok');
          }
          console.log('Idle started');
        })
        .catch(error => console.error('Error:', error));
    }

    function enterSetup() {
      sendMessage('Setup');
      fetch('/setup', { method: 'GET' })
        .then(response => {
          if (!response.ok) {
            throw new Error('Network response was not ok');
          }
          console.log('Setup started');
        })
        .catch(error => console.error('Error:', error));
    }

    function dropweight() {
      sendMessage('Dropweight');
      fetch('/dropweight', { method: 'GET' })
        .then(response => {
          if (!response.ok) {
            throw new Error('Network response was not ok');
          }
          console.log('Dropweight released');
        })
        .catch(error => console.error('Error:', error));
    
    }

    function enterDive() {
      sendMessage('Dive');
      fetch('/initiateDive', { method: 'GET' })
        .then(response => {
          if (!response.ok) {
            throw new Error('Network response was not ok');
          }
          console.log('Glide started');
        })
        .catch(error => console.error('Error:', error));
    }

    function enterCalibration() {
      sendMessage('Calibrate');
      fetch('/calibrate', { method: 'GET' })
        .then(response => {
          if (!response.ok) {
            throw new Error('Network response was not ok');
          }
          console.log('Calibration started');
        })
        .catch(error => console.error('Error:', error));
    }

    function startRotateLeft() {
      sendMessage('Rotate Left started');
      fetch('/rotateLeft', { method: 'GET' })
        .then(response => {
          if (!response.ok) {
            throw new Error('Network response was not ok');
          }
          console.log('Rotate Left started');
        })
        .catch(error => console.error('Error:', error));
    }

    function stopRotateLeft() {
      sendMessage('Rotate Left stopped');
      fetch('/stopRotateLeft', { method: 'GET' })
        .then(response => {
          if (!response.ok) {
            throw new Error('Network response was not ok');
          }
          console.log('Rotate Left stopped');
        })
        .catch(error => console.error('Error:', error));
    }

    function startRotateRight() {
      sendMessage('Rotate Right started');
      fetch('/rotateRight', { method: 'GET' })
        .then(response => {
          if (!response.ok) {
            throw new Error('Network response was not ok');
          }
          console.log('Rotate Right started');
        })
        .catch(error => console.error('Error:', error));
    }

    function stopRotateRight() {
      sendMessage('Rotate Right stopped');
      fetch('/stopRotateRight', { method: 'GET' })
        .then(response => {
          if (!response.ok) {
            throw new Error('Network response was not ok');
          }
          console.log('Rotate Right stopped');
        })
        .catch(error => console.error('Error:', error));
    }

    function startMoveForward() {
      sendMessage('Move Forward started');
      fetch('/moveForward', { method: 'GET' })
        .then(response => {
          if (!response.ok) {
            throw new Error('Network response was not ok');
          }
          console.log('Move Forward started');
        })
        .catch(error => console.error('Error:', error));
    }

    function stopMoveForward() {
      sendMessage('Move Forward stopped');
      fetch('/stopMoveForward', { method: 'GET' })
        .then(response => {
          if (!response.ok) {
            throw new Error('Network response was not ok');
          }
          console.log('Move Forward stopped');
        })
        .catch(error => console.error('Error:', error));
    }

    function startMoveBackward() {
      sendMessage('Move Backward started');
      fetch('/moveBackward', { method: 'GET' })
        .then(response => {
          if (!response.ok) {
            throw new Error('Network response was not ok');
          }
          console.log('Move Backward started');
        })
        .catch(error => console.error('Error:', error));
    }

    function stopMoveBackward() {
      sendMessage('Move Backward stopped');
      fetch('/stopMoveBackward', { method: 'GET' })
        .then(response => {
          if (!response.ok) {
            throw new Error('Network response was not ok');
          }
          console.log('Move Backward stopped');
        })
        .catch(error => console.error('Error:', error));
    }

    function sendMessage(message) {
      fetch('/update?state=' + encodeURIComponent(message))
      .then(response => {
          if (!response.ok) {
              throw new Error('Network response was not ok');
          }
          return response.text();
      })
      .then(text => {
          console.log('Server response: ' + text);
      })
      .catch(error => {
          console.error('There has been a problem with your fetch operation:', error);
      });
    }
    
    function setTargetPotentiometerValue() {
      fetch('/setTargetPotentiometerValue')
        .then(response => response.text())
        .then(data => {
          console.log(data);
          return fetch('/data'); // Chain the fetch call to get the updated data
        })
        .then(response => response.json())
        .then(data => {
          const setPotValue = data.setPotValue;
          const percentage = ((setPotValue / 4095) * 100).toFixed(1);
          document.getElementById('potentiometerValueDisplay').textContent = 'Target percentage: ' + percentage;
          console.log('setPotValue:', setPotValue);
        })
        .catch(error => {
          console.error('Error:', error);
        });
    }

    function setTargetPotentiometerValue() {
      fetch('/setTargetPotentiometerValue')
        .then(response => response.text())
        .then(data => {
          console.log(data);
          return fetch('/data'); // Chain the fetch call to get the updated data
        })
        .then(response => response.json())
        .then(data => {
          const setPotValue = data.setPotValue;
          const percentage = ((setPotValue / 4095) * 100).toFixed(1);
          document.getElementById('potentiometerValueDisplay').textContent = 'Target percentage: ' + percentage;
          console.log('setPotValue:', setPotValue);
        })
        .catch(error => {
          console.error('Error:', error);
        });
    }


    setInterval(fetchData, 1000); // Fetch data every second
  </script>
</body>
</html>
)rawliteral";
