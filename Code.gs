/**
 * mmWave Lab Notebook — Apps Script backend
 * ============================================
 * One Google Sheet per experiment, stored in a fixed Drive folder.
 * The phone is the only client that talks to this endpoint; the ESP32
 * never talks to Google directly (it only talks BLE to the phone).
 *
 * DEPLOYMENT
 * ----------
 * 1. Create (or pick) a Drive folder to hold all experiment Sheets.
 *    Open it in the browser and copy the ID from the URL:
 *    https://drive.google.com/drive/folders/<THIS_PART_IS_THE_ID>
 * 2. Paste that ID into FOLDER_ID below.
 * 3. Extensions > Apps Script in a blank Google Sheet (or script.google.com),
 *    paste this file in as Code.gs.
 * 4. Deploy > New deployment > type "Web app".
 *      - Execute as: Me
 *      - Who has access: Anyone
 *    (Same pattern as your existing VESC Apps Script endpoint — no auth
 *    token, relies on the URL being unguessable. Fine for now; revisit if
 *    this ever needs to be locked down.)
 * 5. Copy the resulting /exec URL into the phone webpage's config and
 *    into your notes — this is APPS_SCRIPT_URL.
 *
 * API CONTRACT
 * ------------
 * GET  ?exp=<experiment_id>&action=list[&callback=<fnName>]
 *      -> { exp_id, rows: [ {index, timestamp, distance, presence}, ... ] }
 *      Returns rows: [] if the experiment's Sheet doesn't exist yet
 *      (i.e. nothing has been synced for it yet — NOT an error).
 *      Apps Script's CORS support for fetch() is unreliable, so if a
 *      `callback` param is present the response is wrapped as JSONP
 *      (`callback({...})`) instead of raw JSON — load it via a <script>
 *      tag from the browser rather than fetch() to sidestep CORS entirely.
 *
 * POST body (JSON), either a single row:
 *      { "exp_id": "...", "name": "...", "index": 7, "timestamp": "...",
 *        "distance": 812, "presence": true }
 *   or a batch:
 *      { "exp_id": "...", "name": "...", "rows": [ {index, timestamp,
 *        distance, presence}, ... ] }
 *      -> { status: "ok", exp_id, written: <n> }
 *      Creates the experiment's Sheet on first write. Each row is an
 *      UPSERT by index: if that index already has a row, it's
 *      overwritten in place; otherwise a new row is appended. This means
 *      the phone never has to distinguish "new" vs "mismatched" before
 *      posting — just POST whenever local data doesn't match the Sheet.
 */

const FOLDER_ID = '';

function doGet(e) {
  const expId = e.parameter.exp;
  if (!expId) {
    return jsonResponse({ error: 'missing exp param' });
  }

  const sheet = getExperimentSheet(expId, false, '');
  if (!sheet) {
    return jsonResponse({ exp_id: expId, rows: [] });
  }

  const data = sheet.getDataRange().getValues();
  const rows = [];
  for (let i = 1; i < data.length; i++) { // skip header row
    const [index, timestamp, distance, presence] = data[i];
    if (index === '' || index === null || index === undefined) continue;
    rows.push({ index: index, timestamp: timestamp, distance: distance, presence: presence });
  }
  return jsonResponse({ exp_id: expId, rows: rows }, e.parameter.callback);
}

function doPost(e) {
  let body;
  try {
    body = JSON.parse(e.postData.contents);
  } catch (err) {
    return jsonResponse({ status: 'error', message: 'bad JSON: ' + err.message });
  }

  const expId = body.exp_id;
  if (!expId) {
    return jsonResponse({ status: 'error', message: 'missing exp_id' });
  }
  const name = body.name || '';
  const rows = body.rows ? body.rows : [body];

  const lock = LockService.getScriptLock();
  lock.waitLock(30000);
  try {
    const sheet = getExperimentSheet(expId, true, name);
    const data = sheet.getDataRange().getValues();

    // Map index -> 1-based sheet row number, so repeats overwrite in place.
    const indexToRow = {};
    for (let i = 1; i < data.length; i++) {
      const idx = data[i][0];
      if (idx !== '' && idx !== null && idx !== undefined) indexToRow[idx] = i + 1;
    }

    rows.forEach(function (r) {
      const rowVals = [r.index, r.timestamp, r.distance, r.presence];
      if (indexToRow[r.index]) {
        sheet.getRange(indexToRow[r.index], 1, 1, 4).setValues([rowVals]);
      } else {
        sheet.appendRow(rowVals);
        indexToRow[r.index] = sheet.getLastRow();
      }
    });

    return jsonResponse({ status: 'ok', exp_id: expId, written: rows.length });
  } finally {
    lock.releaseLock();
  }
}

/**
 * Finds (or creates) the Sheet for a given experiment inside FOLDER_ID.
 * File name == experiment_id, so it's directly discoverable in Drive.
 */
function getExperimentSheet(expId, createIfMissing, name) {
  const folder = DriveApp.getFolderById(FOLDER_ID);
  const files = folder.getFilesByName(expId);

  if (files.hasNext()) {
    const ss = SpreadsheetApp.open(files.next());
    return ss.getSheets()[0];
  }

  if (!createIfMissing) return null;

  const displayName = name ? (expId + ' - ' + name) : expId;
  const ss = SpreadsheetApp.create(expId);
  const file = DriveApp.getFileById(ss.getId());
  file.setName(displayName === expId ? expId : expId); // keep filename = expId for lookup
  file.moveTo(folder); // moves out of root My Drive into the fixed folder

  const sheet = ss.getSheets()[0];
  sheet.setName('data');
  sheet.appendRow(['index', 'timestamp', 'distance', 'presence']);
  sheet.setFrozenRows(1);
  if (name) {
    sheet.getRange('F1').setValue('experiment_name');
    sheet.getRange('F2').setValue(name);
  }
  return sheet;
}

function jsonResponse(obj, callback) {
  const jsonStr = JSON.stringify(obj);
  if (callback) {
    // JSONP: wrap as a function call and serve as JS, so a <script> tag
    // can load it client-side without hitting CORS restrictions at all.
    return ContentService.createTextOutput(callback + '(' + jsonStr + ');')
      .setMimeType(ContentService.MimeType.JAVASCRIPT);
  }
  return ContentService.createTextOutput(jsonStr)
    .setMimeType(ContentService.MimeType.JSON);
}