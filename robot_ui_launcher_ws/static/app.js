//app.js
function sendCommand(cmd) {
  fetch(`/${cmd}`)
    .then(response => response.json())
    .then(data => {
      const statusText = document.querySelector('.status-text');
      statusText.innerText = data.status;

      if (data.status.includes('launched')) {
        statusText.style.color = '#28a745';
      } else if (data.status.includes('stopped')) {
        statusText.style.color = '#dc3545';
      } else if (data.status.includes('running')) {
        statusText.style.color = '#ffc107';
      } else {
        statusText.style.color = '#007bff';
      }
    });
}
