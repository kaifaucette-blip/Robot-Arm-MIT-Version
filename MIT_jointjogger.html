<div class="panel">
  <h3>Joint Jogger</h3>
  
  <div class="sliders">
    <div class="slider-row" id="jointSliders"></div>
  </div>

  <div class="controls">
    <label>Speed:
      <select id="speedLevel">
        <option value="1">1 - Slow (40°/s)</option>
        <option value="2">2 - Medium-Slow (80°/s)</option>
        <option value="3" selected>3 - Normal (120°/s)</option>
        <option value="4">4 - Fast (170°/s)</option>
        <option value="5">5 - Max (220°/s)</option>
      </select>
    </label>
    
    <div class="btn-group">
      <button id="stopBtn" class="danger">Stop / Home</button>
    </div>
  </div>
</div>

<style>
  .panel { max-width: 450px; background: #fff; padding: 16px; border-radius: 8px; }
  .slider-row { display: flex; flex-direction: column; gap: 12px; margin-bottom: 20px; }
  .slider-row label { display: flex; justify-content: space-between; font-weight: 600; font-size: 14px; }
  .slider-row input[type="range"] { width: 65%; }
  .controls { display: flex; flex-direction: column; gap: 12px; }
  .controls select { padding: 6px; border-radius: 4px; border: 1px solid #ccc; width: 100%; margin-top: 4px; }
  button.danger { background: #dc3545; color: white; border: none; padding: 8px 16px; border-radius: 4px; cursor: pointer; font-weight: bold; }
  button.danger:hover { background: #b02a37; }
</style>

<script>
  const container = document.getElementById('jointSliders');
  
  // Dynamically generate sliders so we don't repeat boilerplate HTML
  for (let i = 1; i <= 6; i++) {
    const div = document.createElement('div');
    div.innerHTML = `
      <label>
        Joint ${i}: <span id="j${i}Val">0°</span>
      </label>
      <input type="range" id="j${i}" min="-180" max="180" value="0" step="0.1">
    `;
    container.appendChild(div);

    const slider = div.querySelector('input');
    const label = div.querySelector('span');

    slider.addEventListener('input', (e) => {
      label.textContent = `${e.target.value}°`;
    });
    
    slider.addEventListener('change', sendAngles);
  }

  function sendAngles() {
    const angles = [];
    for (let i = 1; i <= 6; i++) {
      angles.push(parseFloat(document.getElementById(`j${i}`).value));
    }
    const speedLevel = Number(document.getElementById('speedLevel').value);

    fetch('http://localhost:3000/send', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ angles, speedLevel })
    })
    .then(res => res.text())
    .catch(err => console.error('Send error:', err));
  }

  document.getElementById('stopBtn').addEventListener('click', () => {
    for (let i = 1; i <= 6; i++) {
      document.getElementById(`j${i}`).value = 0;
      document.getElementById(`j${i}Val`).textContent = '0°';
    }
    fetch('http://localhost:3000/send', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ angles: [0,0,0,0,0,0], speedLevel: 1 })
    });
  });

  document.getElementById('speedLevel').addEventListener('change', sendAngles);
</script>
