const char body[] PROGMEM = R"===(
<!DOCTYPE html>
<html>
  <body>
    <h1>MEAM510 Lab 4 Q1.3b</h1>
    <p>Duty Cycle Slider</p>
    <input type="range" min="0" max="100" value="50" id="dutyCycle">
    <span id="Duty_Cycle">50</span> Duty cycle <br>
    <p>Frequency Slider</p>
    <input type="range" min="3" max="30" value="15" id="slider_Freq">
    <span id="Frequency">15</span> Frequency <br>

    <script>
      var dutyCycle = document.getElementById("dutyCycle");
      var slider_Freq = document.getElementById("slider_Freq");
      var dutyCycleValue = document.getElementById("Duty_Cycle");
      var frequencyValue = document.getElementById("Frequency");

      dutyCycle.oninput = function() {
        var xhr1 = new XMLHttpRequest();
        xhr1.onreadystatechange = function() {
          if (xhr1.readyState === 4 && xhr1.status === 200) {
            dutyCycleValue.innerHTML = xhr1.responseText;
          }
        };
        xhr1.open("GET", "/setDutyCycle?value=" + dutyCycle.value, true);
        xhr1.send();
      };

      slider_Freq.oninput = function() {
        var xhr2 = new XMLHttpRequest();
        xhr2.onreadystatechange = function() {
          if (xhr2.readyState === 4 && xhr2.status === 200) {
            frequencyValue.innerHTML = xhr2.responseText;
          }
        };
        xhr2.open("GET", "/setFrequency?value=" + slider_Freq.value, true);
        xhr2.send();
      };
    </script>
  </body>
</html>
)===";