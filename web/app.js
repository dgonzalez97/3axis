"use strict";

let timer = null;
let samples = [];

const initialRate = document.getElementById("initial-rate");
const dampingGain = document.getElementById("damping-gain");
const torqueLimit = document.getElementById("torque-limit");
const graph = document.getElementById("graph");
const values = document.getElementById("values");
const resetButton = document.getElementById("reset");
const stepButton = document.getElementById("step");
const startButton = document.getElementById("start");
const stopButton = document.getElementById("stop");

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
        `Saturated: ${saturated}`;

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
        timer = setInterval(runOneStep, 20);
    }
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
    }
};
