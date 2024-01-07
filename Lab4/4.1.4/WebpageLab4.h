const char body[] PROGMEM = R"===(
<!DOCTYPE html>
<html>
  <body>
    <h1>Motor Control Lab 4.1.4 </h1>
    <p>Motor Speed Slider</p>
    <input type="range" min="0" max="100" value="50" id="motorSpeed">
    <span id="speedValue">50%</span> Speed <br>
    <p>Direction Buttons </p>
    <button id="forwardButton" onclick="setMotorState('forward')">Forward</button>
    <button id="backwardButton" onclick="setMotorState('backward')">Backward</button>
    <button id="stopButton" onclick="setMotorState('stop')">Stop</button>

    <script>
      var motorSpeed = document.getElementById("motorSpeed");
      var speedValue = document.getElementById("speedValue");

      motorSpeed.oninput = function() {
        var xhr1 = new XMLHttpRequest();
        xhr1.onreadystatechange = function() {
          if (xhr1.readyState === 4 && xhr1.status === 200) {
            speedValue.innerHTML = xhr1.responseText;
          }
        };
        xhr1.open("GET", "/motor?speed=" + motorSpeed.value, true);
        xhr1.send();
      };

      function setMotorState(state) {
        var xhr2 = new XMLHttpRequest();
        xhr2.onreadystatechange = function() {
          if (xhr2.readyState === 4 && xhr2.status === 200) {
            // Optional: handle the response if needed
          }
        };
        xhr2.open("GET", "/motor?action=" + state, true);
        xhr2.send();
      };
    </script>
  </body>
</html>
)===";