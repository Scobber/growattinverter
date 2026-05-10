const char MAIN_page[] PROGMEM = R"=====(
<!DOCTYPE HTML><html>
<!-- Rui Santos - Complete project details at https://RandomNerdTutorials.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files.
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software. -->
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script src="https://code.highcharts.com/highcharts.js"></script>
  <style>
    body {
      min-width: 310px;
      max-width: 800px;
      height: 400px;
      margin: 0 auto;
    }
    h2 {
      font-family: Arial;
      font-size: 2.5rem;
      text-align: center;
    }
  </style>
</head>
<body>
  <h2>Growatt Inverter</h2>
  <div id="chart-power" class="container"></div>

  <div id="DataCointainer"> </div>

  <a href="./firmware">Firmware update</a> -
  <a href="./status">Json</a> -
  <a href="./debug">Log</a> -
  <a href="./StartAp">Setup</a> -
  <a href="./postCommunicationModbus">RW Modbus</a> -
  <a href="./solar_api/v1/GetInverterRealtimeData.cgi">Fronius Inverter Data</a> -
  <a href="./solar_api/v1/GetInverterInfo.cgi">Fronius Inverter Info</a> -
  <a href="./solar_api/v1/GetDeviceInfo.cgi">Fronius Device Info</a> -
  <a href="./solar_api/v1/GetLoggerInfo.cgi">Fronius Logger Info</a> -
  <a href="./solar_api/v1/GetActiveDeviceInfo.cgi">Fronius Active Device</a> -
  <a href="./solar_api/v1/GetPowerFlowRealtimeData.fcgi">Fronius Power Flow</a>

  <h3>Runtime MQTT Settings</h3>
  <form id="mqttSettingsForm" action="./settings" method="POST">
    <label>Server <input type="text" id="mqttserver" name="mqttserver" maxlength="40"></label><br>
    <label>Port <input type="number" id="mqttport" name="mqttport" min="1" max="65535"></label><br>
    <label>Topic <input type="text" id="mqtttopic" name="mqtttopic" maxlength="64"></label><br>
    <label>User <input type="text" id="mqttuser" name="mqttuser" maxlength="40"></label><br>
    <label>Password <input type="password" id="mqttpwd" name="mqttpwd" maxlength="40"></label><br>
    <button type="submit">Save MQTT settings</button>
    <span id="settingsStatus"></span>
  </form>

</body>
<script>

const d = new Date();
let diff = d.getTimezoneOffset();
let initialised = false;
let nameToId = {};

Highcharts.setOptions({
time: {
timezoneOffset: diff
}
});

var chartT = new Highcharts.Chart({
  chart:{ renderTo : 'chart-power' },
  title: { text: 'Inverter Data' },
  series: [],
  plotOptions: {
    line: { animation: false,
      dataLabels: { enabled: true }
    },
  },
  xAxis: { type: 'datetime',
    dateTimeLabelFormats: { second: '%H:%M:%S' }
  },
  credits: {
    enabled: false
  },
  legend: {
    align: 'left',
    verticalAlign: 'top',
    borderWidth: 0,
  },
  tooltip: {
    shared: true,
    crosshairs: true,
  }
});

function updateSettingsStatus(message, isError) {
  const status = document.getElementById("settingsStatus");
  status.textContent = message;
  status.style.color = isError ? "red" : "green";
}

function loadSettings() {
  fetch("./settings")
    .then((response) => {
      if (!response.ok) {
        throw new Error("Failed to load settings");
      }
      return response.json();
    })
    .then((data) => {
      document.getElementById("mqttserver").value = data.mqttserver || "";
      document.getElementById("mqttport").value = data.mqttport || "1883";
      document.getElementById("mqtttopic").value = data.mqtttopic || "";
      document.getElementById("mqttuser").value = data.mqttuser || "";
      document.getElementById("mqttpwd").value = data.mqttpwd || "";
    })
    .catch(() => {
      updateSettingsStatus("Settings unavailable", true);
    });
}

document.getElementById("mqttSettingsForm").addEventListener("submit", function(event) {
  event.preventDefault();

  const formData = new URLSearchParams(new FormData(event.target));
  fetch("./settings", {
    method: "POST",
    headers: {
      "Content-Type": "application/x-www-form-urlencoded"
    },
    body: formData.toString()
  })
  .then((response) => response.text().then((text) => ({ ok: response.ok, text })))
  .then((result) => {
    updateSettingsStatus(result.text, !result.ok);
  })
  .catch(() => {
    updateSettingsStatus("Failed to save settings", true);
  });
});

loadSettings();

setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {

      // add data fields to the main page
      var obj = JSON.parse(this.responseText);

      // init the UI if not already done
      if (initialised == false) {
        var i = 0;

        // clear data container just in case
        container = document.getElementById("DataCointainer");
        container.innerHTML = "";

        for (var key in obj) {
          // init chart
          if (obj[key][2] == true) {
            chartT.addSeries({
              name: key + " [" + obj[key][1] + "]",
              data: []
            });
            nameToId[key] = i;
            i++;
          }
          // init data container
          var element = document.createElement("p");
          element.innerHTML = key + ": " + obj[key][0] + " " + obj[key][1];
          element.setAttribute("id", key);
          container.appendChild(element);
        }
        initialised = true;
      } else {
        let x = (new Date()).getTime();
        for (var key in obj) {
          // update site data
          var element = document.getElementById(key);
          element.innerHTML = key + ": " + obj[key][0] + " " + obj[key][1];
          // update chart data
          if (obj[key][2] == true) {
            if (chartT.series[nameToId[key]].data.length <= 50) {
              chartT.series[nameToId[key]].addPoint([x, obj[key][0]], true, false, true);
            } else {
              chartT.series[nameToId[key]].addPoint([x, obj[key][0]], true, true, true);
            }
          }
        }
      }
    };
  }
  xhttp.open("GET", "./uistatus", true);
  xhttp.send();
}, 5000 ) ;

</script>



</html>


)=====";
