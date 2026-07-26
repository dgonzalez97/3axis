"use strict";

let timer = null;
let samples = [];

const CONTROLLER_INTERVAL_MS = 10;
const HEALTH_INTERVAL_MS = 5000;
const initialRate = document.getElementById("initial-rate");
const dampingGain = document.getElementById("damping-gain");
const torqueLimit = document.getElementById("torque-limit");
const graph = document.getElementById("graph");
const values = document.getElementById("values");
const resetButton = document.getElementById("reset");
const stepButton = document.getElementById("step");
const startButton = document.getElementById("start");
const stopButton = document.getElementById("stop");
const batteryVoltage = document.getElementById("battery-voltage");
const batteryCurrent = document.getElementById("battery-current");
const gnssVoltage = document.getElementById("gnss-voltage");
const gnssSatellites = document.getElementById("gnss-satellites");
const gnssFix = document.getElementById("gnss-fix");
const healthMessages = document.getElementById("health-messages");

const monitorStatusNames = [
    "OK", "NULL POINTER", "INVALID CONFIG", "INVALID INPUT",
    "TIMESTAMP ERROR", "NUMERIC ERROR"
];
const severityNames = ["OK", "WARNING", "ERROR", "CRITICAL"];

// These labels decode the same 32-bit words returned by the C monitor.
const faultNames = [
    [0x00000001, "Body rate limit"],
    [0x00000002, "Body rate change"],
    [0x00000004, "Controller error"],
    [0x00000008, "Wheel saturated"],
    [0x00000010, "Timestamp error"],
    [0x00000020, "Battery low"],
    [0x00000040, "Battery critical"],
    [0x00000080, "Battery overvoltage"],
    [0x00000100, "Battery drop rate"],
    [0x00000200, "GNSS voltage"],
    [0x00000400, "GNSS voltage rate"],
    [0x00000800, "Too few GNSS satellites"],
    [0x00001000, "GNSS fix lost"],
    [0x00002000, "Battery overcurrent"]
];
const actionNames = [
    [0x00000001, "Reject wheel command"],
    [0x00000002, "Request AOCS off"],
    [0x00000004, "Use backup navigation"]
];

function updateLabels() {
    document.getElementById("initial-rate-value").value =
        `${Number(initialRate.value).toFixed(2)} rad/s`;
    document.getElementById("damping-gain-value").value =
        `${Number(dampingGain.value).toFixed(2)} Nms`;
    document.getElementById("torque-limit-value").value =
        `${Number(torqueLimit.value).toFixed(2)} Nm`;
}

function drawGraph() {
    const context = graph.getContext("2d");
    const width = graph.width;
    const height = graph.height;
    let scale = 1;

    for (const sample of samples) {
        scale = Math.max(scale, Math.abs(sample));
    }

    context.clearRect(0, 0, width, height);
    context.strokeStyle = "#aaa";
    context.beginPath();
    context.moveTo(0, height / 2);
    context.lineTo(width, height / 2);
    context.stroke();

    if (samples.length < 2) {
        return;
    }

    context.strokeStyle = "blue";
    context.beginPath();

    samples.forEach((sample, index) => {
        const x = (index / (samples.length - 1)) * width;
        const y = (height / 2) - ((sample / scale) * (height * 0.45));

        if (index === 0) {
            context.moveTo(x, y);
        } else {
            context.lineTo(x, y);
        }
    });

    context.stroke();
}

function showValues() {
    const bodyRate = Module._web_get_body_rate();
    const wheelTorque = Module._web_get_wheel_torque();
    const saturated = Module._web_get_wheel_saturated() !== 0;

    values.textContent =
        `Body rate: ${bodyRate.toFixed(5)} rad/s | ` +
        `Wheel torque: ${wheelTorque.toFixed(5)} Nm | ` +
        `Wheel: ${saturated ? "SATURATED" : "OK"}`;
    values.classList.toggle("bad-value", saturated);

    samples.push(bodyRate);
    if (samples.length > 500) {
        samples.shift();
    }

    drawGraph();
}

function stopSimulation() {
    if (timer !== null) {
        clearInterval(timer);
        timer = null;
    }
}

function resetSimulation() {
    stopSimulation();
    samples = [];
    Module._web_reset(Number(initialRate.value));
    Module._web_health_reset();
    healthMessages.replaceChildren();
    showValues();
}

function runOneStep() {
    const status = Module._web_step(
        Number(dampingGain.value),
        Number(torqueLimit.value)
    );

    if (status !== 0) {
        document.getElementById("wasm-status").textContent =
            `Controller returned error status ${status}.`;
        stopSimulation();
        return;
    }

    showValues();
}

function startSimulation() {
    if (timer === null) {
        timer = setInterval(runOneStep, CONTROLLER_INTERVAL_MS);
    }
}

function namesFromWord(word, names, emptyText) {
    const activeNames = names
        .filter(([flag]) => (word & flag) !== 0)
        .map(([, name]) => name);

    return activeNames.length === 0 ? emptyText : activeNames.join(", ");
}

function hexWord(word) {
    return `0x${(word >>> 0).toString(16).padStart(8, "0").toUpperCase()}`;
}

function numberFromText(text) {
    return text === "" ? Number.NaN : Number(text);
}

function readHealthResult(channel, message, status, timeTag) {
    const faults = Module._web_health_get_faults() >>> 0;
    const actions = Module._web_health_get_actions() >>> 0;
    const cSeverity = Module._web_health_get_severity();
    const monitorStatus =
        monitorStatusNames[status] || `UNKNOWN STATUS ${status}`;
    const severity =
        status === 0
            ? (severityNames[cSeverity] || `UNKNOWN ${cSeverity}`)
            : "INPUT ERROR";

    return {
        timeTag,
        channel,
        message,
        severity,
        errors:
            status === 0
                ? namesFromWord(faults, faultNames, "No faults")
                : monitorStatus,
        actions: namesFromWord(actions, actionNames, "No action"),
        faultWord: hexWord(faults),
        actionWord: hexWord(actions)
    };
}

function makeHealthRow(result) {
    const row = document.createElement("tr");
    const cells = [
        result.timeTag,
        result.channel,
        result.message,
        result.severity
    ];

    row.className = `health-${result.severity.toLowerCase().replace(" ", "-")}`;
    cells.forEach((text) => {
        const cell = document.createElement("td");
        cell.textContent = text;
        row.appendChild(cell);
    });

    [
        [result.errors, result.faultWord],
        [result.actions, result.actionWord]
    ].forEach(([text, word]) => {
        const cell = document.createElement("td");
        const code = document.createElement("code");

        cell.textContent = text;
        code.textContent = word;
        cell.appendChild(document.createElement("br"));
        cell.appendChild(code);
        row.appendChild(cell);
    });

    return row;
}

function publishHealthMessages() {
    const timestampMs = Math.floor(performance.now()) >>> 0;
    const timeTag = new Date().toLocaleTimeString();
    const bodyRate = Module._web_get_body_rate();
    const wheelTorque = Module._web_get_wheel_torque();
    const wheelSaturated = Module._web_get_wheel_saturated() !== 0;
    const batteryVoltageText = batteryVoltage.value.trim();
    const batteryCurrentText = batteryCurrent.value.trim();
    const gnssVoltageText = gnssVoltage.value.trim();
    const gnssSatellitesText = gnssSatellites.value.trim();
    const results = [];

    // Each call below is a real stateful C health-monitor update.
    let status = Module._web_health_update_aocs(timestampMs);
    results.push(readHealthResult(
        "AOCS",
        `rate ${bodyRate.toFixed(3)} rad/s, torque ` +
            `${wheelTorque.toFixed(3)} Nm, wheel ` +
            `${wheelSaturated ? "saturated" : "normal"}`,
        status,
        timeTag
    ));

    status = Module._web_health_update_battery(
        numberFromText(batteryVoltageText),
        numberFromText(batteryCurrentText),
        timestampMs
    );
    results.push(readHealthResult(
        "Battery",
        `${batteryVoltageText || "empty"} V, ` +
            `${batteryCurrentText || "empty"} A`,
        status,
        timeTag
    ));

    status = Module._web_health_update_gnss(
        numberFromText(gnssVoltageText),
        numberFromText(gnssSatellitesText),
        gnssFix.checked ? 1 : 0,
        timestampMs
    );
    results.push(readHealthResult(
        "GNSS",
        `${gnssVoltageText || "empty"} V, ` +
            `${gnssSatellitesText || "empty"} satellites, ` +
            `fix ${gnssFix.checked ? "valid" : "lost"}`,
        status,
        timeTag
    ));

    const fragment = document.createDocumentFragment();
    results.forEach((result) => fragment.appendChild(makeHealthRow(result)));
    healthMessages.prepend(fragment);

    while (healthMessages.children.length > 15) {
        healthMessages.removeChild(healthMessages.lastElementChild);
    }
}

function startHealthMessages() {
    publishHealthMessages();
    setInterval(publishHealthMessages, HEALTH_INTERVAL_MS);
}

function enableButtons() {
    resetButton.disabled = false;
    stepButton.disabled = false;
    startButton.disabled = false;
    stopButton.disabled = false;
}

initialRate.addEventListener("input", updateLabels);
dampingGain.addEventListener("input", updateLabels);
torqueLimit.addEventListener("input", updateLabels);
resetButton.addEventListener("click", resetSimulation);
stepButton.addEventListener("click", runOneStep);
startButton.addEventListener("click", startSimulation);
stopButton.addEventListener("click", stopSimulation);

updateLabels();

var Module = {
    onRuntimeInitialized: function () {
        document.getElementById("wasm-status").textContent =
            "WebAssembly ready.";
        enableButtons();
        resetSimulation();
        startHealthMessages();
    }
};
