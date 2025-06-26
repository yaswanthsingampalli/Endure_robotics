//app.js
function sendCommand(cmd) {
  fetch(`/${cmd}`)
    .then(response => response.json())
    .then(data => {
      document.getElementById('status').innerText = `Status: ${data.status}`;
    });
}
