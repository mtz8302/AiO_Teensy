// Firmware_Teensy_AiO-New-Dawn is copyright 2025 by the AOG Group
// Firmware_Teensy_AiO-New-Dawn is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// Firmware_Teensy_AiO-New-Dawn is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Firmware_Teensy_AiO-New-Dawn. If not, see <https://www.gnu.org/licenses/>.

// VWASConfigPage.h
// Virtual WAS configuration: vehicle geometry (manual entry or AgOpenGPS
// vehicle.xml import), plus centre/endstop calibration buttons.

#ifndef VWAS_CONFIG_PAGE_H
#define VWAS_CONFIG_PAGE_H

#include <Arduino.h>

const char VWAS_CONFIG_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>Virtual WAS - AiO v26</title>
<link rel="stylesheet" href="/touch.css">
<style>
  .section { background:#fff; border-radius:10px; padding:16px; margin:12px 0; box-shadow:0 1px 3px rgba(0,0,0,0.15); }
  .section h3 { margin-top:0; }
  .field { display:flex; align-items:center; justify-content:space-between; margin:8px 0; }
  .field label { flex:1; }
  .field input, .field select { width:120px; padding:6px; font-size:16px; }
  .hint { color:#666; font-size:13px; margin:2px 0 10px 0; }
  .btn { padding:12px 18px; font-size:16px; border-radius:8px; border:none; margin:6px 6px 6px 0; cursor:pointer; }
  .btn-primary { background:#2e7d32; color:#fff; }
  .btn-danger { background:#c62828; color:#fff; }
  .btn-secondary { background:#546e7a; color:#fff; }
  .status-box { font-family:monospace; font-size:14px; background:#f5f5f5; border-radius:8px; padding:10px; white-space:pre-wrap; }
  .warn { background:#fff3cd; border:1px solid #ffe08a; border-radius:8px; padding:10px; margin:10px 0; }
</style>
</head>
<body>
<div style="max-width:520px;margin:0 auto;padding:10px;">
  <h2>Virtual WAS (VWAS)</h2>

  <div class="section">
    <h3>Live-Status</h3>
    <div id="liveStatus" class="status-box">lade...</div>
  </div>

  <div class="section">
    <h3>Fahrzeugdaten aus AgOpenGPS importieren</h3>
    <div class="hint">Wählt die vehicle.xml aus AgOpenGPS (Documents\AgOpenGPS\Vehicles\...). Werte werden nur ins Formular unten übernommen - erst "Speichern" schreibt sie aufs Board.</div>
    <input type="file" id="vehicleXmlFile" accept=".xml">
    <div id="importResult" class="hint"></div>
  </div>

  <div class="section">
    <h3>Fahrzeug-Geometrie</h3>
    <div class="field"><label>Radstand (m)</label><input type="number" step="0.01" id="wheelbase"></div>
    <div class="field"><label>Max. Lenkwinkel (°)</label><input type="number" step="0.1" id="maxSteeringAngle"></div>
    <div class="field"><label>Fahrzeugtyp</label>
      <select id="vehicleType">
        <option value="0">Standard (Ackermann)</option>
        <option value="2">Knicklenker</option>
      </select>
    </div>
    <div class="field"><label>Dual-Heading bevorzugen</label>
      <input type="checkbox" id="preferDualHeading" style="width:24px;height:24px;">
    </div>

    <h3 style="margin-top:22px;">Encoder-Kalibrierung</h3>
    <div class="hint">Achtung: <b>countsPerDegree</b> hier ist die Encoder-Kalibrierung (Keya-Zählimpulse pro Grad) - komplett getrennt von AgOpenGPS' eigenem "counts per degree" für einen echten analogen WAS-Sensor. Nicht verwechseln/nicht aus vehicle.xml übernehmen.</div>
    <div class="field"><label>Encoder counts/Grad</label><input type="number" step="0.1" id="countsPerDegree"></div>
    <div class="field"><label>Encoder invertieren</label>
      <input type="checkbox" id="invertEncoder" style="width:24px;height:24px;">
    </div>

    <h3 style="margin-top:22px;">Drift-Korrektur (fortgeschritten)</h3>
    <div class="field"><label>Min. Distanz-Schritt (m)</label><input type="number" step="0.01" id="minDistanceForCurvature"></div>
    <div class="field"><label>Glättung Tau (s)</label><input type="number" step="0.1" id="gpsAngleSmoothingTau"></div>
    <div class="field"><label>Max. Korrektur (°/s)</label><input type="number" step="0.01" id="maxDriftCorrectionRate"></div>
    <div class="field"><label>Ausreißer-Schwelle (°)</label><input type="number" step="0.5" id="driftInnovationRejectThreshold"></div>
    <div class="field"><label>Max. plausible v (m/s)</label><input type="number" step="0.1" id="maxPlausibleSpeed"></div>

    <button class="btn btn-primary" onclick="saveConfig()">Speichern</button>
    <div id="saveResult" class="hint"></div>
  </div>

  <div class="section">
    <h3>Geradeausstellung</h3>
    <div class="hint">Räder von Hand exakt gerade stellen, dann drücken. Bewegt den Motor NICHT.</div>
    <button class="btn btn-secondary" onclick="setCenterNow()">Geradeausstellung jetzt setzen</button>
    <div id="centerResult" class="hint"></div>
  </div>

  <div class="section">
    <h3>Anschlag-Kalibrierung</h3>
    <div class="warn">Bewegt die Lenkung automatisch bis zu beiden Anschlägen! Nur auslösen, wenn niemand im Gefahrenbereich steht und AgOpenGPS-Lenkung nicht aktiv ist.</div>
    <button class="btn btn-danger" onclick="startCalibration()">Anschlag-Kalibrierung starten</button>
    <button class="btn btn-secondary" onclick="abortCalibration()">Abbrechen</button>
    <div id="calibStatus" class="status-box" style="margin-top:10px;">-</div>
  </div>
</div>

<script>
function loadConfig() {
  fetch('/api/vwas/config').then(r => r.json()).then(cfg => {
    document.getElementById('wheelbase').value = cfg.wheelbase;
    document.getElementById('maxSteeringAngle').value = cfg.maxSteeringAngle;
    document.getElementById('vehicleType').value = cfg.vehicleType;
    document.getElementById('preferDualHeading').checked = cfg.preferDualHeading;
    document.getElementById('countsPerDegree').value = cfg.countsPerDegree;
    document.getElementById('invertEncoder').checked = cfg.invertEncoder;
    document.getElementById('minDistanceForCurvature').value = cfg.minDistanceForCurvature;
    document.getElementById('gpsAngleSmoothingTau').value = cfg.gpsAngleSmoothingTau;
    document.getElementById('maxDriftCorrectionRate').value = cfg.maxDriftCorrectionRate;
    document.getElementById('driftInnovationRejectThreshold').value = cfg.driftInnovationRejectThreshold;
    document.getElementById('maxPlausibleSpeed').value = cfg.maxPlausibleSpeed;
  }).catch(() => {});
}

function saveConfig() {
  const body = {
    wheelbase: parseFloat(document.getElementById('wheelbase').value),
    maxSteeringAngle: parseFloat(document.getElementById('maxSteeringAngle').value),
    vehicleType: parseInt(document.getElementById('vehicleType').value),
    preferDualHeading: document.getElementById('preferDualHeading').checked,
    countsPerDegree: parseFloat(document.getElementById('countsPerDegree').value),
    invertEncoder: document.getElementById('invertEncoder').checked,
    minDistanceForCurvature: parseFloat(document.getElementById('minDistanceForCurvature').value),
    gpsAngleSmoothingTau: parseFloat(document.getElementById('gpsAngleSmoothingTau').value),
    maxDriftCorrectionRate: parseFloat(document.getElementById('maxDriftCorrectionRate').value),
    driftInnovationRejectThreshold: parseFloat(document.getElementById('driftInnovationRejectThreshold').value),
    maxPlausibleSpeed: parseFloat(document.getElementById('maxPlausibleSpeed').value)
  };
  fetch('/api/vwas/config', { method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify(body) })
    .then(r => r.json())
    .then(res => { document.getElementById('saveResult').innerText = res.status === 'saved' ? 'Gespeichert.' : ('Fehler: ' + res.message); })
    .catch(() => { document.getElementById('saveResult').innerText = 'Fehler beim Speichern.'; });
}

// --- vehicle.xml import (parsed entirely in the browser - the board never
// sees the raw file, only the extracted numbers after you click Speichern) ---
document.getElementById('vehicleXmlFile').addEventListener('change', function(e) {
  const file = e.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = function(evt) {
    try {
      const doc = new DOMParser().parseFromString(evt.target.result, 'text/xml');
      const settings = {};
      doc.querySelectorAll('setting').forEach(node => {
        const name = node.getAttribute('name');
        const valueNode = node.querySelector('value');
        if (name && valueNode) settings[name] = valueNode.textContent;
      });

      let applied = [];
      if (settings['setVehicle_wheelbase'] !== undefined) {
        document.getElementById('wheelbase').value = settings['setVehicle_wheelbase'];
        applied.push('Radstand');
      }
      if (settings['setVehicle_maxSteerAngle'] !== undefined) {
        document.getElementById('maxSteeringAngle').value = settings['setVehicle_maxSteerAngle'];
        applied.push('Max. Lenkwinkel');
      }
      if (settings['setVehicle_vehicleType'] !== undefined) {
        document.getElementById('vehicleType').value = settings['setVehicle_vehicleType'];
        applied.push('Fahrzeugtyp');
      }
      if (settings['setGPS_headingFromWhichSource'] !== undefined) {
        document.getElementById('preferDualHeading').checked =
          (settings['setGPS_headingFromWhichSource'] === 'Dual');
        applied.push('Heading-Quelle');
      }
      // NOTE: setAS_countsPerDegree is intentionally NOT imported here - it
      // calibrates AgOpenGPS' analog-WAS ADC reading, not our encoder. See
      // the hint text above the "Encoder counts/Grad" field.

      document.getElementById('importResult').innerText =
        applied.length ? ('Übernommen: ' + applied.join(', ') + ' - bitte prüfen und "Speichern" klicken.')
                       : 'Keine bekannten Felder in dieser Datei gefunden.';
    } catch (err) {
      document.getElementById('importResult').innerText = 'Fehler beim Lesen der XML-Datei.';
    }
  };
  reader.readAsText(file);
});

function setCenterNow() {
  if (!confirm('Räder sind gerade gestellt?')) return;
  fetch('/api/vwas/center', { method: 'POST' })
    .then(r => r.json())
    .then(res => { document.getElementById('centerResult').innerText = res.status === 'ok' ? 'Geradeausstellung gesetzt.' : ('Fehler: ' + res.message); });
}

function startCalibration() {
  if (!confirm('Achtung: Die Lenkung bewegt sich jetzt automatisch bis zu beiden Anschlägen. Fortfahren?')) return;
  fetch('/api/vwas/calibrate/start', { method: 'POST' })
    .then(r => r.json())
    .then(res => { if (res.status !== 'ok') alert('Fehler: ' + res.message); });
}

function abortCalibration() {
  fetch('/api/vwas/calibrate/abort', { method: 'POST' });
}

function pollStatus() {
  fetch('/api/vwas/status').then(r => r.json()).then(s => {
    document.getElementById('liveStatus').innerText =
      'Encoder-Winkel: ' + s.encoderAngle.toFixed(2) + '°\n' +
      'Fusions-Winkel (an AOG gemeldet): ' + s.fusedAngle.toFixed(2) + '°\n' +
      'GPS-Winkel (Drift-Referenz): ' + (s.gpsAngleValid ? s.gpsAngle.toFixed(2) + '°' : 'nicht verfügbar') + '\n' +
      'Encoder-Offset: ' + s.encoderOffset.toFixed(2) + '°';
  }).catch(() => {});

  fetch('/api/vwas/calibrate/status').then(r => r.json()).then(s => {
    document.getElementById('calibStatus').innerText =
      'Status: ' + s.state +
      (s.state !== 'idle' ? ('\nLinks: ' + s.leftAngle.toFixed(2) + '°  Rechts: ' + s.rightAngle.toFixed(2) + '°') : '') +
      (s.state === 'done' ? ('\nErmittelte counts/Grad: ' + s.countsPerDegree.toFixed(2)) : '');
  }).catch(() => {});
}

loadConfig();
pollStatus();
setInterval(pollStatus, 1000);
</script>
</body>
</html>
)rawliteral";

#endif // VWAS_CONFIG_PAGE_H
