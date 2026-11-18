import React, { useState, useEffect, useRef, useCallback } from 'react';
import {
  Power,
  Cpu,
  Usb,
  Zap,
  RotateCcw,
  Activity,
  Gauge,
  Sliders,
  Info,
  Sprout,
  Car,
  Microscope,
  Radio,
  CheckCircle2,
  AlertTriangle,
  LayoutDashboard,
  GitCompare,
  Code2,
  BookOpen,
  Play,
  Copy,
  Check,
  Download,
  Sparkles,
  Wrench
} from 'lucide-react';

// Box-Muller transform for accurate Gaussian noise matching Python's random.gauss(0, sigma)
function randomGaussian(mean = 0, stdDev = 1) {
  let u = 0, v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  const z = Math.sqrt(-2.0 * Math.log(u)) * Math.cos(2.0 * Math.PI * v);
  return z * stdDev + mean;
}

const BUFFER_LEN = 140;

const ENV_META = {
  plant: {
    id: 'plant',
    label: 'GREENHOUSE PLANTS',
    emoji: '🌿',
    subHeader: 'Active Context: 🌿 Smart Agriculture / Botanical Canopy Distance Keeping',
    animTitle: 'ENVIRONMENT: 🌿 AUTOMATED GREENHOUSE - ULTRASONIC PLANT CANOPY DISTANCE CONTROL',
    animStatus: 'APPLICATION: AUTOMATED SPRAY NOZZLE / LIGHTING BOOM HEIGHT REGULATION',
    accent: 'emerald'
  },
  vehicle: {
    id: 'vehicle',
    label: 'ROAD / VEHICLE ACC',
    emoji: '🚗',
    subHeader: 'Active Context: 🚗 Real-World Autonomous Driving & Headway Control',
    animTitle: 'ENVIRONMENT: 🚗 AUTONOMOUS ELECTRIC VEHICLE - ADAPTIVE CRUISE CONTROL (ACC)',
    animStatus: 'APPLICATION: HIGHWAY INTER-VEHICLE HEADWAY SPACING & COLLISION MITIGATION',
    accent: 'blue'
  },
  bench: {
    id: 'bench',
    label: 'LAB BENCH APPARATUS',
    emoji: '🔬',
    subHeader: 'Active Context: 🔬 Laboratory Embedded Hardware Prototype (Arduino)',
    animTitle: 'ENVIRONMENT: 🔬 EMBEDDED LABORATORY BENCH - HC-SR04 & MICRO SERVO TRACK',
    animStatus: 'APPLICATION: 1-DOF PHYSICAL EXPERIMENTAL TESTBED (ARDUINO ATMEGA328P)',
    accent: 'violet'
  }
};

const TUNING_PRESETS = [
  {
    id: 'optimal',
    name: 'Optimal Filtered PID',
    badge: 'Balanced',
    kp: 2.4,
    ki: 0.05,
    kd: 0.85,
    desc: 'Critically damped response (Mp ~7.8%, ts ~1.75s) with zero steady-state offset.'
  },
  {
    id: 'underdamped',
    name: 'Aggressive / Underdamped',
    badge: 'High Kp',
    kp: 6.2,
    ki: 0.12,
    kd: 0.15,
    desc: 'Fast initial rise time but pronounced oscillatory ringing due to low derivative braking.'
  },
  {
    id: 'overdamped',
    name: 'Heavy Viscous Damping',
    badge: 'High Kd',
    kp: 1.8,
    ki: 0.02,
    kd: 2.4,
    desc: 'Zero overshoot shock-absorber profile; sluggish approach to setpoint.'
  },
  {
    id: 'ponly',
    name: 'Proportional Only (P)',
    badge: 'Offset Demo',
    kp: 2.4,
    ki: 0.0,
    kd: 0.0,
    desc: 'Demonstrates classic steady-state error offset when Ki = 0.'
  }
];

const ARDUINO_SKETCH_CODE = `/*
 * CONTROL-X // ATmega328P Closed-Loop Ultrasonic PID Firmware
 * Hardware: Arduino Uno/Nano + HC-SR04 Ultrasonic + Micro Servo (SG90/MG90S)
 * Baud Rate: 115200 bps (Compatible with CONTROL-X Web Serial Interface)
 */
#include <Servo.h>

const int TRIG_PIN  = 9;
const int ECHO_PIN  = 10;
const int SERVO_PIN = 6;

Servo balanceServo;

// Nominal Tuned PID Parameters
double Kp = 2.40;
double Ki = 0.05;
double Kd = 0.85;
double targetDist = 20.0;

// Signal Conditioning & State Variables
const double ALPHA_EMA   = 0.30;
const double ALPHA_DERIV = 0.22;
double smoothedDist = 20.0;
double integralAcc  = 0.0;
double prevError    = 0.0;
double filteredDeriv = 0.0;
unsigned long lastMicros = 0;

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  balanceServo.attach(SERVO_PIN);
  balanceServo.write(90); // Neutral equilibrium
  lastMicros = micros();
}

double readUltrasonicCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  if (duration == 0) return smoothedDist;
  double cm = (duration * 0.0343) / 2.0;
  return constrain(cm, 2.0, 50.0);
}

void parseSerialCommands() {
  while (Serial.available() > 0) {
    char cmd = Serial.read();
    double val = Serial.parseFloat();
    if (cmd == 'P') Kp = val;
    else if (cmd == 'I') Ki = val;
    else if (cmd == 'D') Kd = val;
    else if (cmd == 'T') targetDist = constrain(val, 5.0, 45.0);
  }
}

void loop() {
  parseSerialCommands();

  unsigned long now = micros();
  double dt = (now - lastMicros) / 1000000.0;
  if (dt < 0.02) return; // 50 Hz deterministic control loop
  lastMicros = now;

  // 1. EMA Low-Pass Input Filter
  double rawDist = readUltrasonicCm();
  smoothedDist = (ALPHA_EMA * rawDist) + ((1.0 - ALPHA_EMA) * smoothedDist);

  // 2. Deadband Error Evaluation
  double error = targetDist - smoothedDist;
  double ctrlError = (abs(error) < 0.6) ? 0.0 : error;

  // 3. PID Terms with Clamped Anti-Windup & Filtered Derivative
  double P = Kp * ctrlError;
  integralAcc = constrain(integralAcc + ctrlError * dt, -40.0, 40.0);
  double I = Ki * integralAcc;

  double rawDeriv = (ctrlError - prevError) / dt;
  filteredDeriv = (ALPHA_DERIV * rawDeriv) + ((1.0 - ALPHA_DERIV) * filteredDeriv);
  double D = Kd * filteredDeriv;
  prevError = ctrlError;

  double output = P + I + D;
  double servoAngle = constrain(90.0 + output, 20.0, 160.0);
  balanceServo.write((int)servoAngle);

  // 4. Stream Tab-Delimited Telemetry to CONTROL-X Frontend
  Serial.print("Target:");   Serial.print(targetDist, 2);
  Serial.print("\\tDistance:"); Serial.print(smoothedDist, 2);
  Serial.print("\\tServo:");    Serial.print(servoAngle, 1);
  Serial.print("\\tOutput:");   Serial.print(output, 2);
  Serial.print("\\tP:");        Serial.print(P, 2);
  Serial.print("\\tI:");        Serial.print(I, 2);
  Serial.print("\\tD:");        Serial.println(D, 2);
}`;

export default function App() {
  // Multi-Page Navigation State
  const [activePage, setActivePage] = useState('bench'); // 'bench' | 'compare' | 'hardware' | 'theory'

  // React UI State (for controls, badges, insights, and connection status)
  const [systemEnabled, setSystemEnabled] = useState(true);
  const [controllerType, setControllerType] = useState('PID'); // 'PID' | 'ON_OFF'
  const [envView, setEnvView] = useState('plant'); // 'plant' | 'vehicle' | 'bench'

  // Tuning parameters state
  const [kp, setKp] = useState(2.4);
  const [ki, setKi] = useState(0.05);
  const [kd, setKd] = useState(0.85);
  const [target, setTarget] = useState(20.0);
  const [noiseAmp, setNoiseAmp] = useState(0.35);

  // Educational Insight Ticker
  const [insight, setInsight] = useState({
    text: 'System Ready. Adjust sliders or click step buttons to observe dynamic PID compensation.',
    tone: 'sky'
  });

  // Web Serial Hardware State
  const [isHwConnected, setIsHwConnected] = useState(false);
  const [hwStatusMsg, setHwStatusMsg] = useState('');
  const portRef = useRef(null);
  const writerRef = useRef(null);
  const readerRef = useRef(null);
  const hwConnectedRef = useRef(false);

  // Hardware page copy state
  const [copiedCode, setCopiedCode] = useState(false);

  // Comparative Benchmark Lab State (runs head-to-head PID vs Bang-Bang step trial)
  const [benchStepFrom, setBenchStepFrom] = useState(15);
  const [benchStepTo, setBenchStepTo] = useState(35);
  const [benchmarkData, setBenchmarkData] = useState(null);
  const benchCanvasRef = useRef(null);

  // Telemetry snapshot updated at ~20Hz
  const [telemetry, setTelemetry] = useState({
    target: 20.0,
    position: 20.0,
    error: 0.0,
    pTerm: 0.0,
    iTerm: 0.0,
    dTerm: 0.0,
    controlOutput: 0.0,
    servoAngle: 90.0,
    riseTime: 0.0,
    settlingTime: 1.75,
    peakOvershoot: 7.8,
    steadyStateError: 0.0
  });

  // Mutable high-frequency physics & simulation engine state (60fps loop)
  const simRef = useRef({
    systemEnabled: true,
    controllerType: 'PID',
    envView: 'plant',
    mode: 'sim',

    // Plant parameters
    position: 20.0,
    velocity: 0.0,
    target: 20.0,
    plantMass: 1.0,
    plantDamping: 1.8,
    noiseAmplitude: 0.35,

    // PID state
    kp: 2.4,
    ki: 0.05,
    kd: 0.85,
    integral: 0.0,
    prevError: 0.0,
    filteredDerivative: 0.0,
    alphaDeriv: 0.22,
    alphaEma: 0.30,
    smoothedDist: 20.0,
    rawDist: 20.0,
    pTerm: 0.0,
    iTerm: 0.0,
    dTerm: 0.0,
    controlOutput: 0.0,
    servoAngle: 90.0,

    // Step response analytics
    stepStartTime: performance.now() / 1000,
    stepInitialPos: 20.0,
    riseTime: 0.0,
    settlingTime: 0.0,
    peakOvershoot: 0.0,
    steadyStateError: 0.0,
    stepMonitoring: true,

    // Oscilloscope rolling buffers
    targetBuffer: new Array(BUFFER_LEN).fill(20.0),
    posBuffer: new Array(BUFFER_LEN).fill(20.0),
    ctrlBuffer: new Array(BUFFER_LEN).fill(0.0),

    // Animation phases
    lastSimTime: performance.now() / 1000,
    lastSampleTime: 0,
    lastUiPushTime: 0,
    wavePhase: 0.0,
    roadScroll: 0.0,
    sprayPhase: 0.0
  });

  const animCanvasRef = useRef(null);
  const scopeCanvasRef = useRef(null);

  // Send serial command to Arduino if connected
  const sendSerialCommand = useCallback(async (cmdString) => {
    if (!hwConnectedRef.current || !writerRef.current) return;
    try {
      const encoder = new TextEncoder();
      await writerRef.current.write(encoder.encode(cmdString));
    } catch (err) {
      console.warn('Serial write failed:', err);
    }
  }, []);

  // Trigger Step Response
  const triggerStep = useCallback((newTarget) => {
    const s = simRef.current;
    s.target = newTarget;
    s.stepStartTime = performance.now() / 1000;
    s.stepInitialPos = s.position;
    s.stepMonitoring = true;
    s.riseTime = 0.0;
    s.settlingTime = 0.0;
    s.peakOvershoot = 0.0;
    setTarget(newTarget);
    if (s.mode === 'hw') {
      sendSerialCommand(`T${newTarget}\n`);
    }
  }, [sendSerialCommand]);

  // Apply Tuning Preset
  const applyPreset = (preset) => {
    const s = simRef.current;
    s.kp = preset.kp;
    s.ki = preset.ki;
    s.kd = preset.kd;
    s.integral = 0.0;
    s.filteredDerivative = 0.0;
    s.controllerType = 'PID';
    setKp(preset.kp);
    setKi(preset.ki);
    setKd(preset.kd);
    setControllerType('PID');
    setInsight({
      text: `Preset Loaded [${preset.name}]: ${preset.desc}`,
      tone: 'emerald'
    });
    if (s.mode === 'hw') {
      sendSerialCommand(`P${preset.kp}\nI${preset.ki}\nD${preset.kd}\n`);
    }
    triggerStep(s.target === 20 ? 32 : 20);
  };

  // Slider Change Handlers
  const handleSliderChange = (tag, val) => {
    const s = simRef.current;
    if (tag === 'P') {
      s.kp = val;
      setKp(val);
      if (val > 4.5) {
        setInsight({
          text: `Kp set to ${val.toFixed(1)} (HIGH): Aggressive stiffness! Speeds up rise time, but increases oscillations.`,
          tone: 'rose'
        });
      } else {
        setInsight({
          text: `Kp set to ${val.toFixed(1)}: Restoring spring force adjusted proportionally to error.`,
          tone: 'sky'
        });
      }
      s.velocity += 1.5 * (Math.random() > 0.5 ? 1.0 : -1.0);
    } else if (tag === 'I') {
      s.ki = val;
      setKi(val);
      if (val > 0.2) {
        setInsight({
          text: `Ki set to ${val.toFixed(2)} (STRONG): High integral accumulation! Destroys steady-state error, may cause slow hunting.`,
          tone: 'blue'
        });
      } else {
        setInsight({
          text: `Ki set to ${val.toFixed(2)}: Integrator summing past errors to eliminate steady-state offset.`,
          tone: 'blue'
        });
      }
    } else if (tag === 'D') {
      s.kd = val;
      setKd(val);
      if (val > 1.5) {
        setInsight({
          text: `Kd set to ${val.toFixed(2)} (HEAVY DAMPING): Acts like high-viscosity shock absorber. Quenches overshoot!`,
          tone: 'amber'
        });
      } else {
        setInsight({
          text: `Kd set to ${val.toFixed(2)}: Derivative damper resisting rapid changes in velocity (de/dt).`,
          tone: 'amber'
        });
      }
    } else if (tag === 'T') {
      triggerStep(val);
      setInsight({
        text: `Target Setpoint adjusted to ${val.toFixed(1)} cm. System responding with transient step maneuver.`,
        tone: 'violet'
      });
    } else if (tag === 'N') {
      s.noiseAmplitude = val;
      setNoiseAmp(val);
      setInsight({
        text: `Acoustic Noise Jitter set to ${val.toFixed(2)} cm. Exponential Moving Average filter actively smoothing echoes.`,
        tone: 'rose'
      });
    }

    if (s.mode === 'hw' && hwConnectedRef.current) {
      sendSerialCommand(`P${s.kp}\nI${s.ki}\nD${s.kd}\nT${s.target}\n`);
    }
  };

  // Toggle System Power
  const toggleSystemPower = () => {
    const s = simRef.current;
    const next = !s.systemEnabled;
    s.systemEnabled = next;
    setSystemEnabled(next);
    if (next) {
      setInsight({
        text: 'System Energized: Actuator driving closed-loop feedback loop.',
        tone: 'emerald'
      });
    } else {
      s.controlOutput = 0.0;
      setInsight({
        text: 'System De-energized: Actuator output forced to 0. Mechanical plant coasting to rest.',
        tone: 'rose'
      });
    }
  };

  // Toggle Controller Algorithm (PID vs Bang-Bang)
  const toggleControllerAlgo = () => {
    const s = simRef.current;
    const next = s.controllerType === 'PID' ? 'ON_OFF' : 'PID';
    s.controllerType = next;
    setControllerType(next);
    if (next === 'ON_OFF') {
      setInsight({
        text: 'ON/OFF Bang-Bang Active: Actuator slams between full saturation (+/- 35). Produces sustained limit cycles!',
        tone: 'rose'
      });
    } else {
      setInsight({
        text: 'PID Controller Active: Smooth proportional moderation with integral elimination of offset and derivative damping.',
        tone: 'emerald'
      });
    }
    triggerStep(s.target);
  };

  // Cycle 3-Way Environment View
  const cycleEnvView = () => {
    const order = ['plant', 'vehicle', 'bench'];
    const next = order[(order.indexOf(envView) + 1) % order.length];
    simRef.current.envView = next;
    setEnvView(next);
  };

  // Inject Physical Disturbance (+8 cm Nudge)
  const injectNudge = () => {
    const s = simRef.current;
    s.position = Math.min(48.0, s.position + 8.0);
    s.velocity += 18.0;
    s.stepStartTime = performance.now() / 1000;
    s.stepMonitoring = true;
    setInsight({
      text: 'Physical Impulse Injected (+8 cm): Derivative term immediately reacting to velocity jump (de/dt) to arrest displacement!',
      tone: 'rose'
    });
  };

  // Reset Integrator
  const resetIntegrator = () => {
    simRef.current.integral = 0.0;
    setInsight({
      text: 'Integrator Reset: Cumulative past error cleared to zero.',
      tone: 'slate'
    });
  };

  // Run Deterministic Side-by-Side Head-to-Head Simulation (PID vs Bang-Bang vs P-Only)
  const runHeadToHeadBenchmark = useCallback(() => {
    const dt = 0.02;
    const totalTime = 5.0;
    const steps = Math.floor(totalTime / dt);
    const stepMag = Math.abs(benchStepTo - benchStepFrom);

    const simulateController = (mode) => {
      let pos = benchStepFrom;
      let vel = 0.0;
      let integral = 0.0;
      let prevErr = 0.0;
      let filtDeriv = 0.0;
      let smoothed = benchStepFrom;

      const trace = [];
      let riseTime = null;
      let settlingTime = null;
      let peakOvershoot = 0.0;
      let iae = 0.0; // Integral of Absolute Error

      for (let i = 0; i <= steps; i++) {
        const t = i * dt;
        // Deterministic pseudo-noise so comparison is fair across algorithms
        const pseudoNoise = Math.sin(i * 1.7) * 0.15 + Math.cos(i * 3.1) * 0.1;
        const raw = Math.max(2.0, Math.min(50.0, pos + pseudoNoise));
        smoothed = 0.3 * raw + 0.7 * smoothed;
        const err = benchStepTo - smoothed;
        iae += Math.abs(benchStepTo - pos) * dt;

        let out = 0.0;
        if (mode === 'ON_OFF') {
          if (err > 0.8) out = 35.0;
          else if (err < -0.8) out = -35.0;
          else out = 0.0;
        } else if (mode === 'P_ONLY') {
          const cErr = Math.abs(err) < 0.6 ? 0.0 : err;
          out = kp * cErr;
        } else {
          // Filtered PID
          const cErr = Math.abs(err) < 0.6 ? 0.0 : err;
          integral = Math.max(-40.0, Math.min(40.0, integral + cErr * dt));
          const rawD = (cErr - prevErr) / dt;
          filtDeriv = 0.22 * rawD + 0.78 * filtDeriv;
          prevErr = cErr;
          out = kp * cErr + ki * integral + kd * filtDeriv;
        }

        const servo = Math.max(20.0, Math.min(160.0, 90.0 + out));
        const tiltRad = ((servo - 90.0) * Math.PI) / 180.0;
        const acc = (125.0 * Math.sin(tiltRad) - 1.8 * vel) / 1.0;
        vel += acc * dt;
        pos = Math.max(2.0, Math.min(50.0, pos + vel * dt));

        trace.push({ t, pos });

        if (stepMag > 0.5) {
          const frac = (pos - benchStepFrom) / (benchStepTo - benchStepFrom);
          if (riseTime === null && frac >= 0.9) riseTime = t;

          if (
            (benchStepTo > benchStepFrom && pos > benchStepTo) ||
            (benchStepTo < benchStepFrom && pos < benchStepTo)
          ) {
            const pct = (Math.abs(pos - benchStepTo) / stepMag) * 100.0;
            if (pct > peakOvershoot) peakOvershoot = pct;
          }

          if (Math.abs(benchStepTo - pos) <= 0.05 * stepMag && t > 0.4) {
            if (settlingTime === null) settlingTime = t;
          } else if (settlingTime !== null && Math.abs(benchStepTo - pos) > 0.05 * stepMag) {
            settlingTime = null;
          }
        }
      }

      const finalErr = Math.abs(benchStepTo - pos);
      return {
        trace,
        riseTime,
        settlingTime,
        peakOvershoot,
        steadyStateError: finalErr,
        iae
      };
    };

    const pidRes = simulateController('PID');
    const bbRes = simulateController('ON_OFF');
    const pRes = simulateController('P_ONLY');

    setBenchmarkData({
      from: benchStepFrom,
      to: benchStepTo,
      duration: totalTime,
      pid: pidRes,
      bangBang: bbRes,
      pOnly: pRes
    });
  }, [benchStepFrom, benchStepTo, kp, ki, kd]);

  // Trigger initial benchmark calculation when entering Comparative Lab page
  useEffect(() => {
    if (activePage === 'compare' && !benchmarkData) {
      runHeadToHeadBenchmark();
    }
  }, [activePage, benchmarkData, runHeadToHeadBenchmark]);

  // Render Comparative Benchmark Chart Canvas
  useEffect(() => {
    if (activePage !== 'compare' || !benchmarkData || !benchCanvasRef.current) return;
    const canvas = benchCanvasRef.current;
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    if (rect.width === 0 || rect.height === 0) return;
    canvas.width = Math.round(rect.width * dpr);
    canvas.height = Math.round(rect.height * dpr);
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    const w = rect.width;
    const h = rect.height;
    const padL = 46;
    const padR = 20;
    const padT = 20;
    const padB = 30;
    const plotW = w - padL - padR;
    const plotH = h - padT - padB;
    const yMax = 50.0;

    ctx.fillStyle = '#ffffff';
    ctx.fillRect(0, 0, w, h);

    // Grid lines
    ctx.font = '600 10px Inter, system-ui, sans-serif';
    [0, 10, 20, 30, 40, 50].forEach((cm) => {
      const y = padT + plotH - (cm / yMax) * plotH;
      ctx.strokeStyle = '#e2e8f0';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(padL, y);
      ctx.lineTo(w - padR, y);
      ctx.stroke();

      ctx.fillStyle = '#64748b';
      ctx.textAlign = 'right';
      ctx.textBaseline = 'middle';
      ctx.fillText(`${cm} cm`, padL - 8, y);
    });

    [0, 1, 2, 3, 4, 5].forEach((sec) => {
      const x = padL + (sec / benchmarkData.duration) * plotW;
      ctx.strokeStyle = '#f1f5f9';
      ctx.beginPath();
      ctx.moveTo(x, padT);
      ctx.lineTo(x, padT + plotH);
      ctx.stroke();

      ctx.fillStyle = '#64748b';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      ctx.fillText(`${sec}s`, x, padT + plotH + 8);
    });

    // Target Step Line (Dashed Sky)
    const yTarget = padT + plotH - (benchmarkData.to / yMax) * plotH;
    ctx.save();
    ctx.setLineDash([6, 4]);
    ctx.strokeStyle = '#0284c7';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(padL, yTarget);
    ctx.lineTo(w - padR, yTarget);
    ctx.stroke();
    ctx.restore();

    const drawCurve = (trace, color, width, dashed = false) => {
      ctx.save();
      if (dashed) ctx.setLineDash([4, 3]);
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.beginPath();
      trace.forEach((pt, idx) => {
        const x = padL + (pt.t / benchmarkData.duration) * plotW;
        const y = padT + plotH - (pt.pos / yMax) * plotH;
        if (idx === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();
      ctx.restore();
    };

    // Bang-Bang (Rose), P-Only (Amber dashed), Filtered PID (Blue)
    drawCurve(benchmarkData.bangBang.trace, '#e11d48', 2);
    drawCurve(benchmarkData.pOnly.trace, '#d97706', 2, true);
    drawCurve(benchmarkData.pid.trace, '#2563eb', 3);
  }, [activePage, benchmarkData]);

  // Export CSV Telemetry Log
  const handleExportCsv = () => {
    const s = simRef.current;
    let csv = 'Sample_Index,Setpoint_cm,Measured_Position_cm,Control_Effort_u\n';
    for (let i = 0; i < BUFFER_LEN; i++) {
      csv += `${i},${s.targetBuffer[i].toFixed(2)},${s.posBuffer[i].toFixed(2)},${s.ctrlBuffer[i].toFixed(2)}\n`;
    }
    const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.setAttribute('download', `control_x_telemetry_${controllerType.toLowerCase()}.csv`);
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
  };

  // Copy Arduino Code
  const handleCopyCode = () => {
    navigator.clipboard?.writeText(ARDUINO_SKETCH_CODE);
    setCopiedCode(true);
    setTimeout(() => setCopiedCode(false), 2500);
  };

  // Web Serial API Connection Handler
  const toggleHwConnection = async () => {
    if (!('serial' in navigator)) {
      setHwStatusMsg('Web Serial API unavailable in this browser. Running Digital Twin Simulation.');
      setTimeout(() => setHwStatusMsg(''), 4500);
      return;
    }

    if (!hwConnectedRef.current) {
      try {
        const port = await navigator.serial.requestPort();
        await port.open({ baudRate: 115200 });
        portRef.current = port;
        writerRef.current = port.writable.getWriter();
        hwConnectedRef.current = true;
        simRef.current.mode = 'hw';
        setIsHwConnected(true);
        setHwStatusMsg('Arduino USB Connected (115200 baud)');
        setInsight({
          text: 'Hardware-in-the-Loop Active: Streaming live telemetry from Arduino ATmega328P over USB Serial.',
          tone: 'emerald'
        });

        const textDecoder = new TextDecoderStream();
        const readableStreamClosed = port.readable.pipeTo(textDecoder.writable);
        const reader = textDecoder.readable.getReader();
        readerRef.current = { reader, readableStreamClosed };

        let buffer = '';
        (async () => {
          try {
            while (hwConnectedRef.current) {
              const { value, done } = await reader.read();
              if (done) break;
              if (value) {
                buffer += value;
                const lines = buffer.split('\n');
                buffer = lines.pop() || '';
                for (const rawLine of lines) {
                  const line = rawLine.trim();
                  if (line.includes('Target:') && line.includes('Distance:')) {
                    const parts = line.split('\t');
                    const parsed = {};
                    for (const p of parts) {
                      const idx = p.indexOf(':');
                      if (idx !== -1) {
                        const k = p.slice(0, idx).trim();
                        const v = parseFloat(p.slice(idx + 1).trim());
                        if (!Number.isNaN(v)) parsed[k] = v;
                      }
                    }
                    const s = simRef.current;
                    if (parsed.Target !== undefined) {
                      s.target = parsed.Target;
                      setTarget(parsed.Target);
                    }
                    if (parsed.Distance !== undefined) {
                      s.position = parsed.Distance;
                      s.smoothedDist = parsed.Distance;
                    }
                    if (parsed.Servo !== undefined) s.servoAngle = parsed.Servo;
                    if (parsed.Output !== undefined) s.controlOutput = parsed.Output;
                    if (parsed.P !== undefined) s.pTerm = parsed.P;
                    if (parsed.I !== undefined) s.iTerm = parsed.I;
                    if (parsed.D !== undefined) s.dTerm = parsed.D;
                  }
                }
              }
            }
          } catch (readErr) {
            console.warn('Serial read loop closed:', readErr);
          }
        })();
      } catch (err) {
        if (err.name !== 'NotFoundError') {
          setHwStatusMsg(`USB Connection Error: ${err.message}`);
          setTimeout(() => setHwStatusMsg(''), 4000);
        }
      }
    } else {
      hwConnectedRef.current = false;
      simRef.current.mode = 'sim';
      setIsHwConnected(false);
      setHwStatusMsg('');
      try {
        if (readerRef.current?.reader) {
          await readerRef.current.reader.cancel();
          await readerRef.current.readableStreamClosed.catch(() => {});
        }
        if (writerRef.current) {
          writerRef.current.releaseLock();
          writerRef.current = null;
        }
        if (portRef.current) {
          await portRef.current.close();
          portRef.current = null;
        }
      } catch (closeErr) {
        console.warn('Serial close error:', closeErr);
      }
      setInsight({
        text: 'Disconnected from USB Hardware. Returned to internal 2nd-Order Physics Simulator.',
        tone: 'sky'
      });
    }
  };

  useEffect(() => {
    return () => {
      hwConnectedRef.current = false;
      if (portRef.current) {
        portRef.current.close().catch(() => {});
      }
    };
  }, []);

  // Main Physics + Canvas Rendering Loop (Runs continuously across all pages)
  useEffect(() => {
    let animationFrameId;

    const physicsStep = (dt, nowSec) => {
      const s = simRef.current;
      if (!s.systemEnabled) {
        s.controlOutput = 0.0;
        s.pTerm = 0.0;
        s.iTerm = 0.0;
        s.dTerm = 0.0;
        s.servoAngle = 90.0;
        s.velocity *= 0.94;
        s.position += s.velocity * dt;
        return;
      }

      const jitter = s.noiseAmplitude > 0 ? randomGaussian(0, s.noiseAmplitude) : 0.0;
      s.rawDist = Math.max(2.0, Math.min(50.0, s.position + jitter));

      s.smoothedDist = s.alphaEma * s.rawDist + (1.0 - s.alphaEma) * s.smoothedDist;
      const error = s.target - s.smoothedDist;

      if (s.controllerType === 'ON_OFF') {
        const deadband = 0.8;
        if (error > deadband) {
          s.controlOutput = 35.0;
        } else if (error < -deadband) {
          s.controlOutput = -35.0;
        } else {
          s.controlOutput = 0.0;
        }
        s.pTerm = s.controlOutput;
        s.iTerm = 0.0;
        s.dTerm = 0.0;
        s.servoAngle = 90.0 + s.controlOutput;
      } else {
        const ctrlError = Math.abs(error) < 0.6 ? 0.0 : error;
        s.pTerm = s.kp * ctrlError;

        s.integral += ctrlError * dt;
        s.integral = Math.max(-40.0, Math.min(40.0, s.integral));
        s.iTerm = s.ki * s.integral;

        const rawDeriv = dt > 0 ? (ctrlError - s.prevError) / dt : 0.0;
        s.filteredDerivative =
          s.alphaDeriv * rawDeriv + (1.0 - s.alphaDeriv) * s.filteredDerivative;
        s.dTerm = s.kd * s.filteredDerivative;
        s.prevError = ctrlError;

        s.controlOutput = s.pTerm + s.iTerm + s.dTerm;
        s.servoAngle = Math.max(20.0, Math.min(160.0, 90.0 + s.controlOutput));
      }

      const tiltRad = ((s.servoAngle - 90.0) * Math.PI) / 180.0;
      const actuatorForce = 125.0 * Math.sin(tiltRad);
      const dampingForce = -s.plantDamping * s.velocity;

      const netForce = actuatorForce + dampingForce;
      const acceleration = netForce / s.plantMass;

      s.velocity += acceleration * dt;
      s.position += s.velocity * dt;

      if (s.position <= 2.0) {
        s.position = 2.0;
        s.velocity = 0.0;
      } else if (s.position >= 50.0) {
        s.position = 50.0;
        s.velocity = 0.0;
      }

      if (s.stepMonitoring) {
        const elapsed = nowSec - s.stepStartTime;
        const stepMag = Math.abs(s.target - s.stepInitialPos);
        if (stepMag > 1.0) {
          if (s.riseTime === 0.0) {
            const fraction = (s.position - s.stepInitialPos) / (s.target - s.stepInitialPos);
            if (fraction >= 0.9) {
              s.riseTime = elapsed;
            }
          }

          if (
            (s.target > s.stepInitialPos && s.position > s.target) ||
            (s.target < s.stepInitialPos && s.position < s.target)
          ) {
            const overshootCm = Math.abs(s.position - s.target);
            const pct = (overshootCm / stepMag) * 100.0;
            if (pct > s.peakOvershoot) {
              s.peakOvershoot = pct;
            }
          }

          if (Math.abs(error) <= 0.05 * stepMag && elapsed > 0.6) {
            if (s.settlingTime === 0.0) {
              s.settlingTime = elapsed;
            }
          } else {
            if (elapsed > s.settlingTime && s.settlingTime > 0.0 && Math.abs(error) > 0.05 * stepMag) {
              s.settlingTime = 0.0;
            }
          }
        }
      }

      s.steadyStateError = Math.abs(error);
    };

    const prepCanvas = (canvas) => {
      if (!canvas) return null;
      const dpr = window.devicePixelRatio || 1;
      const rect = canvas.getBoundingClientRect();
      if (rect.width === 0 || rect.height === 0) return null;
      const targetW = Math.round(rect.width * dpr);
      const targetH = Math.round(rect.height * dpr);
      if (canvas.width !== targetW || canvas.height !== targetH) {
        canvas.width = targetW;
        canvas.height = targetH;
      }
      const ctx = canvas.getContext('2d');
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      return { ctx, w: rect.width, h: rect.height };
    };

    const drawCanvasBadge = (ctx, text, x, y, options = {}) => {
      const {
        bg = 'rgba(255, 255, 255, 0.94)',
        border = '#cbd5e1',
        textColor = '#0f172a',
        font = 'bold 10px Inter, system-ui, sans-serif',
        padX = 8,
        height = 20,
        canvasW = 1000
      } = options;

      ctx.save();
      ctx.font = font;
      const textWidth = ctx.measureText(text).width;
      const boxW = textWidth + padX * 2;
      const clampedX = Math.max(boxW / 2 + 6, Math.min(canvasW - boxW / 2 - 6, x));
      const boxX = clampedX - boxW / 2;
      const boxY = y - height / 2;

      ctx.fillStyle = bg;
      ctx.strokeStyle = border;
      ctx.lineWidth = 1.25;
      ctx.beginPath();
      ctx.roundRect(boxX, boxY, boxW, height, 6);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = textColor;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(text, clampedX, y + 0.5);
      ctx.restore();
      return clampedX;
    };

    const drawGreenhouseEnvironment = (ctx, w, h, s, nowSec) => {
      const groundY = h - 30;
      const skyGrad = ctx.createLinearGradient(0, 0, 0, groundY);
      skyGrad.addColorStop(0, '#ecfdf5');
      skyGrad.addColorStop(1, '#d1fae5');
      ctx.fillStyle = skyGrad;
      ctx.fillRect(0, 0, w, groundY);

      ctx.strokeStyle = '#a7f3d0';
      ctx.lineWidth = 1.5;
      for (let gx = 0; gx < w; gx += 120) {
        ctx.beginPath();
        ctx.moveTo(gx, 0);
        ctx.lineTo(gx + 60, groundY);
        ctx.moveTo(gx + 60, 0);
        ctx.lineTo(gx, groundY);
        ctx.stroke();
      }

      ctx.fillStyle = '#78350f';
      ctx.fillRect(0, groundY, w, h - groundY);
      ctx.fillStyle = '#92400e';
      ctx.fillRect(0, groundY, w, 4);

      const trackStartX = 140;
      const trackEndX = w - 120;
      const trackLen = Math.max(100, trackEndX - trackStartX);

      const gantryX = trackStartX - 55;
      const gantryY = groundY - 46;

      ctx.strokeStyle = '#64748b';
      ctx.lineWidth = 3.5;
      ctx.beginPath();
      ctx.moveTo(gantryX - 45, gantryY - 26);
      ctx.lineTo(w - 20, gantryY - 26);
      ctx.stroke();

      ctx.fillStyle = '#ffffff';
      ctx.strokeStyle = '#0284c7';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.roundRect(gantryX - 24, gantryY - 34, 48, 44, 6);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = '#0284c7';
      ctx.fillRect(gantryX + 24, gantryY - 16, 9, 20);
      ctx.fillStyle = '#f8fafc';
      ctx.strokeStyle = '#0f172a';
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.arc(gantryX + 36, gantryY - 11, 4.5, 0, Math.PI * 2);
      ctx.arc(gantryX + 36, gantryY - 1, 4.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = '#d97706';
      ctx.beginPath();
      ctx.moveTo(gantryX + 18, gantryY + 10);
      ctx.lineTo(gantryX + 28, gantryY + 20);
      ctx.lineTo(gantryX + 8, gantryY + 20);
      ctx.closePath();
      ctx.fill();

      drawCanvasBadge(ctx, 'GANTRY BOOM', gantryX + 2, 16, {
        bg: 'rgba(240, 249, 255, 0.95)',
        border: '#7dd3fc',
        textColor: '#0369a1',
        canvasW: w
      });

      if (s.systemEnabled) {
        s.sprayPhase = (s.sprayPhase + 0.14) % 1.0;
        ctx.fillStyle = 'rgba(2, 132, 199, 0.65)';
        for (let i = 0; i < 4; i++) {
          const prog = (s.sprayPhase + i * 0.25) % 1.0;
          const spX = gantryX + 20 + prog * 30;
          const spY = gantryY + 22 + prog * 18;
          ctx.beginPath();
          ctx.arc(spX, spY, 2.2, 0, Math.PI * 2);
          ctx.fill();
        }
      }

      const posClamped = Math.max(0.0, Math.min(50.0, s.position));
      const plantX = trackStartX + (posClamped / 50.0) * trackLen;
      const plantY = groundY;
      const sway = Math.sin(nowSec * 2.5) * 3.0;

      const potW = 26;
      ctx.fillStyle = '#ea580c';
      ctx.strokeStyle = '#9a3412';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(plantX - potW, plantY);
      ctx.lineTo(plantX + potW, plantY);
      ctx.lineTo(plantX + potW - 6, plantY + 25);
      ctx.lineTo(plantX - potW + 6, plantY + 25);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = '#c2410c';
      ctx.fillRect(plantX - potW - 3, plantY - 5, (potW + 3) * 2, 6);

      const stemTopX = plantX + sway;
      const stemTopY = plantY - 62;
      ctx.strokeStyle = '#15803d';
      ctx.lineWidth = 4.5;
      ctx.lineCap = 'round';
      ctx.beginPath();
      ctx.moveTo(plantX, plantY - 4);
      ctx.lineTo(stemTopX, stemTopY);
      ctx.stroke();

      [-24, 0, 24].forEach((offset) => {
        const lx = stemTopX + offset;
        const ly = stemTopY + Math.abs(offset) * 0.35;
        ctx.fillStyle = '#10b981';
        ctx.strokeStyle = '#047857';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.ellipse(lx, ly, 19, 13, offset * 0.015, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();

        ctx.strokeStyle = '#a7f3d0';
        ctx.lineWidth = 1.2;
        ctx.beginPath();
        ctx.moveTo(lx - 11, ly);
        ctx.lineTo(lx + 11, ly);
        ctx.stroke();
      });

      const spClamped = Math.max(0.0, Math.min(50.0, s.target));
      const spX = trackStartX + (spClamped / 50.0) * trackLen;
      ctx.save();
      ctx.setLineDash([4, 4]);
      ctx.strokeStyle = '#0284c7';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(spX, 26);
      ctx.lineTo(spX, groundY);
      ctx.stroke();
      ctx.restore();

      ctx.fillStyle = '#0284c7';
      ctx.beginPath();
      ctx.moveTo(spX, 28);
      ctx.lineTo(spX + 14, 35);
      ctx.lineTo(spX, 42);
      ctx.closePath();
      ctx.fill();

      drawCanvasBadge(ctx, `DESIRED GAP: ${s.target.toFixed(1)} cm`, spX, 16, {
        bg: 'rgba(240, 249, 255, 0.96)',
        border: '#38bdf8',
        textColor: '#0369a1',
        canvasW: w
      });

      drawCanvasBadge(ctx, `PLANT CANOPY: ${s.position.toFixed(1)} cm`, plantX, groundY + 13, {
        bg: 'rgba(236, 253, 245, 0.96)',
        border: '#34d399',
        textColor: '#065f46',
        canvasW: w
      });

      if (s.systemEnabled) {
        s.wavePhase = (s.wavePhase + 0.025) % 1.0;
        const gapSpan = plantX - 22 - (gantryX + 42);
        if (gapSpan > 20) {
          ctx.strokeStyle = '#0284c7';
          ctx.lineWidth = 2;
          for (let i = 0; i < 3; i++) {
            const prog = (s.wavePhase + i * 0.33) % 1.0;
            const ax = gantryX + 42 + gapSpan * prog;
            const ay = gantryY - 6;
            const rSpan = 12 + 16 * prog;
            ctx.globalAlpha = 1 - prog * 0.6;
            ctx.beginPath();
            ctx.arc(ax, ay, rSpan, -Math.PI / 4, Math.PI / 4);
            ctx.stroke();
          }
          ctx.globalAlpha = 1.0;
        }
      }

      if (Math.abs(s.controlOutput) > 1.0) {
        const forceLen = Math.max(-55, Math.min(55, s.controlOutput * 1.6));
        const fCol = forceLen > 0 ? '#0284c7' : '#e11d48';
        const endX = plantX + forceLen;
        ctx.strokeStyle = fCol;
        ctx.fillStyle = fCol;
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(plantX, stemTopY);
        ctx.lineTo(endX, stemTopY);
        ctx.stroke();

        const dir = forceLen > 0 ? 1 : -1;
        ctx.beginPath();
        ctx.moveTo(endX, stemTopY);
        ctx.lineTo(endX - 7 * dir, stemTopY - 5);
        ctx.lineTo(endX - 7 * dir, stemTopY + 5);
        ctx.closePath();
        ctx.fill();

        drawCanvasBadge(ctx, 'F_PID', plantX + forceLen / 2, stemTopY - 22, {
          bg: 'rgba(255, 255, 255, 0.92)',
          border: fCol,
          textColor: fCol,
          font: 'bold 9px Inter, system-ui, sans-serif',
          padX: 5,
          height: 16,
          canvasW: w
        });
      }
    };

    const drawVehicleEnvironment = (ctx, w, h, s) => {
      const horizonY = Math.round(h * 0.38);

      const skyGrad = ctx.createLinearGradient(0, 0, 0, horizonY);
      skyGrad.addColorStop(0, '#bae6fd');
      skyGrad.addColorStop(1, '#e0f2fe');
      ctx.fillStyle = skyGrad;
      ctx.fillRect(0, 0, w, horizonY);

      ctx.fillStyle = '#86efac';
      ctx.beginPath();
      ctx.moveTo(0, horizonY);
      ctx.lineTo(110, horizonY - 24);
      ctx.lineTo(250, horizonY);
      ctx.lineTo(420, horizonY - 30);
      ctx.lineTo(620, horizonY);
      ctx.lineTo(820, horizonY - 26);
      ctx.lineTo(w, horizonY);
      ctx.closePath();
      ctx.fill();

      ctx.fillStyle = '#bbf7d0';
      ctx.fillRect(0, horizonY, w, h - horizonY);

      const roadTopW = Math.round(w * 0.82);
      const roadLeftTop = (w - roadTopW) / 2;
      const roadRightTop = roadLeftTop + roadTopW;

      ctx.fillStyle = '#334155';
      ctx.strokeStyle = '#64748b';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(roadLeftTop, horizonY);
      ctx.lineTo(roadRightTop, horizonY);
      ctx.lineTo(w - 16, h);
      ctx.lineTo(16, h);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();

      s.roadScroll = (s.roadScroll + (s.systemEnabled ? 1.4 : 0.0)) % 50.0;
      const centerY = Math.round(horizonY + (h - horizonY) * 0.62);
      ctx.strokeStyle = '#fbbf24';
      ctx.lineWidth = 3;
      for (let xStep = -50; xStep < w + 60; xStep += 50) {
        const dashX = xStep - s.roadScroll;
        ctx.beginPath();
        ctx.moveTo(dashX, centerY);
        ctx.lineTo(dashX + 24, centerY);
        ctx.stroke();
      }

      const trackStartX = 150;
      const trackEndX = w - 120;
      const trackLen = Math.max(100, trackEndX - trackStartX);

      const carX = trackStartX - 55;
      const carY = centerY - 8;

      if (s.systemEnabled) {
        const coneGrad = ctx.createLinearGradient(carX + 38, carY, carX + 195, carY);
        coneGrad.addColorStop(0, 'rgba(14, 165, 233, 0.35)');
        coneGrad.addColorStop(1, 'rgba(14, 165, 233, 0.0)');
        ctx.fillStyle = coneGrad;
        ctx.beginPath();
        ctx.moveTo(carX + 38, carY - 5);
        ctx.lineTo(carX + 195, carY - 26);
        ctx.lineTo(carX + 195, carY + 26);
        ctx.lineTo(carX + 38, carY + 6);
        ctx.closePath();
        ctx.fill();
      }

      ctx.fillStyle = '#2563eb';
      ctx.strokeStyle = '#1d4ed8';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.roundRect(carX - 42, carY - 16, 78, 30, 6);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = '#eff6ff';
      ctx.beginPath();
      ctx.moveTo(carX - 22, carY - 16);
      ctx.lineTo(carX - 8, carY - 28);
      ctx.lineTo(carX + 18, carY - 28);
      ctx.lineTo(carX + 28, carY - 16);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = '#38bdf8';
      ctx.fillRect(carX + 33, carY - 12, 4, 7);
      ctx.fillRect(carX + 33, carY + 4, 4, 7);

      [carX - 25, carX + 20].forEach((wx) => {
        const wy = carY + 14;
        ctx.fillStyle = '#0f172a';
        ctx.beginPath();
        ctx.arc(wx, wy, 9, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = '#38bdf8';
        ctx.beginPath();
        ctx.arc(wx, wy, 3.5, 0, Math.PI * 2);
        ctx.fill();
      });

      drawCanvasBadge(ctx, 'AUTONOMOUS AGV', carX - 4, carY - 42, {
        bg: 'rgba(239, 246, 255, 0.95)',
        border: '#93c5fd',
        textColor: '#1e3a8a',
        canvasW: w
      });

      const spClamped = Math.max(0.0, Math.min(50.0, s.target));
      const spX = trackStartX + (spClamped / 50.0) * trackLen;
      ctx.save();
      ctx.setLineDash([4, 4]);
      ctx.strokeStyle = '#0284c7';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(spX, 26);
      ctx.lineTo(spX, h - 6);
      ctx.stroke();
      ctx.restore();

      drawCanvasBadge(ctx, `DESIRED GAP: ${s.target.toFixed(1)} m`, spX, 16, {
        bg: 'rgba(240, 249, 255, 0.96)',
        border: '#38bdf8',
        textColor: '#0369a1',
        canvasW: w
      });

      const posClamped = Math.max(0.0, Math.min(50.0, s.position));
      const targetX = trackStartX + (posClamped / 50.0) * trackLen;
      const targetY = centerY - 8;

      ctx.fillStyle = '#e11d48';
      ctx.strokeStyle = '#9f1239';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.roundRect(targetX - 20, targetY - 15, 62, 29, 6);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = '#ffe4e6';
      ctx.beginPath();
      ctx.moveTo(targetX - 6, targetY - 15);
      ctx.lineTo(targetX + 6, targetY - 26);
      ctx.lineTo(targetX + 26, targetY - 26);
      ctx.lineTo(targetX + 34, targetY - 15);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = s.velocity < -0.2 ? '#ff1e56' : '#fda4af';
      ctx.fillRect(targetX - 23, targetY - 11, 4, 21);

      [targetX - 6, targetX + 28].forEach((wx) => {
        const wy = targetY + 14;
        ctx.fillStyle = '#0f172a';
        ctx.beginPath();
        ctx.arc(wx, wy, 9, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = '#cbd5e1';
        ctx.beginPath();
        ctx.arc(wx, wy, 3.5, 0, Math.PI * 2);
        ctx.fill();
      });

      drawCanvasBadge(ctx, `LEAD VEHICLE: ${s.position.toFixed(1)} m`, targetX + 11, targetY - 40, {
        bg: 'rgba(255, 241, 242, 0.96)',
        border: '#fda4af',
        textColor: '#9f1239',
        canvasW: w
      });

      if (s.systemEnabled) {
        s.wavePhase = (s.wavePhase + 0.025) % 1.0;
        const gapSpan = targetX - 24 - (carX + 42);
        if (gapSpan > 20) {
          ctx.strokeStyle = '#38bdf8';
          ctx.lineWidth = 2;
          for (let i = 0; i < 3; i++) {
            const prog = (s.wavePhase + i * 0.33) % 1.0;
            const ax = carX + 42 + gapSpan * prog;
            const ay = carY;
            const rSpan = 14 + 18 * prog;
            ctx.globalAlpha = 1 - prog * 0.6;
            ctx.beginPath();
            ctx.arc(ax, ay, rSpan, -Math.PI / 4, Math.PI / 4);
            ctx.stroke();
          }
          ctx.globalAlpha = 1.0;
        }
      }
    };

    const drawBenchApparatus = (ctx, w, h, s) => {
      const trackY = h - 46;
      const trackStartX = 115;
      const trackEndX = w - 70;
      const trackLen = Math.max(100, trackEndX - trackStartX);

      ctx.fillStyle = '#f8fafc';
      ctx.fillRect(0, 0, w, h);

      ctx.strokeStyle = '#e2e8f0';
      ctx.lineWidth = 1;
      for (let gx = 20; gx < w; gx += 30) {
        ctx.beginPath();
        ctx.moveTo(gx, 0);
        ctx.lineTo(gx, h);
        ctx.stroke();
      }
      for (let gy = 20; gy < h; gy += 30) {
        ctx.beginPath();
        ctx.moveTo(0, gy);
        ctx.lineTo(w, gy);
        ctx.stroke();
      }

      ctx.fillStyle = '#cbd5e1';
      ctx.strokeStyle = '#64748b';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.roundRect(trackStartX - 20, trackY - 8, trackLen + 40, 16, 4);
      ctx.fill();
      ctx.stroke();

      ctx.textAlign = 'center';
      ctx.font = 'bold 10px Inter, system-ui, sans-serif';
      for (let tickCm = 0; tickCm <= 50; tickCm += 5) {
        const tickX = trackStartX + (tickCm / 50.0) * trackLen;
        const isMajor = tickCm % 10 === 0;
        const tLen = isMajor ? 10 : 5;
        ctx.strokeStyle = isMajor ? '#0284c7' : '#94a3b8';
        ctx.lineWidth = isMajor ? 1.5 : 1;
        ctx.beginPath();
        ctx.moveTo(tickX, trackY + 8);
        ctx.lineTo(tickX, trackY + 8 + tLen);
        ctx.stroke();

        if (isMajor) {
          ctx.fillStyle = '#475569';
          ctx.fillText(`${tickCm} cm`, tickX, trackY + 28);
        }
      }

      const sensorX = trackStartX - 58;
      const sensorY = trackY - 34;
      ctx.fillStyle = '#1d4ed8';
      ctx.strokeStyle = '#1e3a8a';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.roundRect(sensorX - 20, sensorY - 28, 40, 56, 5);
      ctx.fill();
      ctx.stroke();

      [sensorY - 13, sensorY + 11].forEach((ty) => {
        ctx.fillStyle = '#e2e8f0';
        ctx.strokeStyle = '#475569';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(sensorX, ty, 9, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
      });

      drawCanvasBadge(ctx, 'HC-SR04 SENSOR', sensorX, 16, {
        bg: 'rgba(239, 246, 255, 0.95)',
        border: '#93c5fd',
        textColor: '#1d4ed8',
        canvasW: w
      });

      const targetClamped = Math.max(0.0, Math.min(50.0, s.target));
      const targetX = trackStartX + (targetClamped / 50.0) * trackLen;
      ctx.save();
      ctx.setLineDash([4, 4]);
      ctx.strokeStyle = '#0284c7';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(targetX, 26);
      ctx.lineTo(targetX, trackY);
      ctx.stroke();
      ctx.restore();

      ctx.fillStyle = '#0284c7';
      ctx.beginPath();
      ctx.moveTo(targetX, 28);
      ctx.lineTo(targetX + 14, 35);
      ctx.lineTo(targetX, 42);
      ctx.closePath();
      ctx.fill();

      drawCanvasBadge(ctx, `SETPOINT: ${s.target.toFixed(1)} cm`, targetX, 16, {
        bg: 'rgba(240, 249, 255, 0.96)',
        border: '#38bdf8',
        textColor: '#0369a1',
        canvasW: w
      });

      const posClamped = Math.max(0.0, Math.min(50.0, s.position));
      const cartX = trackStartX + (posClamped / 50.0) * trackLen;
      const cartY = trackY - 16;

      ctx.fillStyle = '#0284c7';
      ctx.strokeStyle = '#0369a1';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.roundRect(cartX - 18, cartY - 14, 36, 16, 4);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = '#f8fafc';
      ctx.strokeStyle = '#475569';
      ctx.beginPath();
      ctx.arc(cartX, cartY - 16, 10, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      drawCanvasBadge(ctx, `CART: ${s.position.toFixed(1)} cm`, cartX, cartY - 40, {
        bg: 'rgba(255, 255, 255, 0.96)',
        border: '#94a3b8',
        textColor: '#0f172a',
        canvasW: w
      });

      if (s.systemEnabled) {
        s.wavePhase = (s.wavePhase + 0.025) % 1.0;
        const spanDist = cartX - 18 - (sensorX + 20);
        if (spanDist > 20) {
          ctx.strokeStyle = '#0284c7';
          ctx.lineWidth = 2;
          for (let i = 0; i < 3; i++) {
            const prog = (s.wavePhase + i * 0.33) % 1.0;
            const ax = sensorX + 20 + spanDist * prog;
            const ay = sensorY;
            const rSpan = 14 + 14 * prog;
            ctx.globalAlpha = 1 - prog * 0.6;
            ctx.beginPath();
            ctx.arc(ax, ay, rSpan, -Math.PI / 3.6, Math.PI / 3.6);
            ctx.stroke();
          }
          ctx.globalAlpha = 1.0;
        }
      }

      const servoX = w - 110;
      const servoY = 48;
      ctx.fillStyle = '#0369a1';
      ctx.strokeStyle = '#0c4a6e';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.roundRect(servoX - 28, servoY - 18, 56, 36, 5);
      ctx.fill();
      ctx.stroke();

      const angRad = ((s.servoAngle - 180) * Math.PI) / 180.0;
      const hornLen = 30;
      const hornEndX = servoX + hornLen * Math.cos(angRad);
      const hornEndY = servoY + hornLen * Math.sin(angRad);

      ctx.strokeStyle = '#f59e0b';
      ctx.lineWidth = 5;
      ctx.lineCap = 'round';
      ctx.beginPath();
      ctx.moveTo(servoX, servoY);
      ctx.lineTo(hornEndX, hornEndY);
      ctx.stroke();

      ctx.fillStyle = '#ffffff';
      ctx.beginPath();
      ctx.arc(servoX, servoY, 4, 0, Math.PI * 2);
      ctx.fill();

      drawCanvasBadge(ctx, `SERVO: ${s.servoAngle.toFixed(1)}°`, servoX, servoY + 30, {
        bg: 'rgba(255, 251, 235, 0.96)',
        border: '#fcd34d',
        textColor: '#b45309',
        canvasW: w
      });
    };

    const drawOscilloscope = (ctx, w, h, s) => {
      const yMax = 50.0;

      ctx.fillStyle = '#ffffff';
      ctx.fillRect(0, 0, w, h);

      ctx.font = '600 10px Inter, system-ui, sans-serif';
      ctx.textAlign = 'left';
      [10, 20, 30, 40].forEach((cmVal) => {
        const yPx = h - (cmVal / yMax) * h;
        ctx.save();
        ctx.setLineDash([3, 4]);
        ctx.strokeStyle = '#e2e8f0';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(0, yPx);
        ctx.lineTo(w, yPx);
        ctx.stroke();
        ctx.restore();

        ctx.fillStyle = '#64748b';
        ctx.fillText(`${cmVal} cm`, 8, yPx - 4);
      });

      const dx = w / (BUFFER_LEN - 1);

      ctx.beginPath();
      for (let i = 0; i < BUFFER_LEN; i++) {
        const x = i * dx;
        const yPos = Math.max(0, Math.min(h, h - (s.posBuffer[i] / yMax) * h));
        if (i === 0) ctx.moveTo(x, yPos);
        else ctx.lineTo(x, yPos);
      }
      for (let i = BUFFER_LEN - 1; i >= 0; i--) {
        const x = i * dx;
        const yTgt = Math.max(0, Math.min(h, h - (s.targetBuffer[i] / yMax) * h));
        ctx.lineTo(x, yTgt);
      }
      ctx.closePath();
      ctx.fillStyle =
        s.controllerType === 'ON_OFF'
          ? 'rgba(225, 29, 72, 0.10)'
          : 'rgba(37, 99, 235, 0.08)';
      ctx.fill();

      ctx.save();
      ctx.setLineDash([4, 3]);
      ctx.strokeStyle = '#d97706';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      for (let i = 0; i < BUFFER_LEN; i++) {
        const x = i * dx;
        const normVal = 25.0 + s.ctrlBuffer[i] * 0.38;
        const y = Math.max(0, Math.min(h, h - (normVal / yMax) * h));
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
      ctx.restore();

      ctx.strokeStyle = '#0284c7';
      ctx.lineWidth = 2;
      ctx.beginPath();
      for (let i = 0; i < BUFFER_LEN; i++) {
        const x = i * dx;
        const y = Math.max(0, Math.min(h, h - (s.targetBuffer[i] / yMax) * h));
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();

      const posColor = s.controllerType === 'PID' ? '#2563eb' : '#e11d48';
      ctx.strokeStyle = posColor;
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      for (let i = 0; i < BUFFER_LEN; i++) {
        const x = i * dx;
        const y = Math.max(0, Math.min(h, h - (s.posBuffer[i] / yMax) * h));
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
    };

    const loop = (nowMs) => {
      const nowSec = nowMs / 1000;
      const s = simRef.current;
      let dt = nowSec - s.lastSimTime;
      s.lastSimTime = nowSec;
      if (dt > 0.1) dt = 0.03;

      if (s.mode === 'sim') {
        physicsStep(dt, nowSec);
      }

      if (nowSec - s.lastSampleTime >= 0.028) {
        s.lastSampleTime = nowSec;
        s.targetBuffer.shift();
        s.targetBuffer.push(s.target);

        s.posBuffer.shift();
        s.posBuffer.push(s.position);

        s.ctrlBuffer.shift();
        s.ctrlBuffer.push(s.controlOutput);
      }

      if (nowSec - s.lastUiPushTime >= 0.05) {
        s.lastUiPushTime = nowSec;
        setTelemetry({
          target: s.target,
          position: s.position,
          error: s.target - s.position,
          pTerm: s.pTerm,
          iTerm: s.iTerm,
          dTerm: s.dTerm,
          controlOutput: s.controlOutput,
          servoAngle: s.servoAngle,
          riseTime: s.riseTime,
          settlingTime: s.settlingTime,
          peakOvershoot: s.peakOvershoot,
          steadyStateError: s.steadyStateError
        });
      }

      const animPrep = prepCanvas(animCanvasRef.current);
      if (animPrep) {
        const { ctx, w, h } = animPrep;
        ctx.clearRect(0, 0, w, h);
        if (s.envView === 'plant') {
          drawGreenhouseEnvironment(ctx, w, h, s, nowSec);
        } else if (s.envView === 'vehicle') {
          drawVehicleEnvironment(ctx, w, h, s);
        } else {
          drawBenchApparatus(ctx, w, h, s);
        }
      }

      const scopePrep = prepCanvas(scopeCanvasRef.current);
      if (scopePrep) {
        const { ctx, w, h } = scopePrep;
        drawOscilloscope(ctx, w, h, s);
      }

      animationFrameId = requestAnimationFrame(loop);
    };

    animationFrameId = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(animationFrameId);
  }, []);

  const activeEnv = ENV_META[envView];

  const getBarMetrics = (val, maxScale) => {
    const clamped = Math.max(-1, Math.min(1, val / maxScale));
    const widthPct = Math.abs(clamped) * 50;
    const leftPct = clamped >= 0 ? 50 : 50 - widthPct;
    return { widthPct, leftPct };
  };

  const pBar = getBarMetrics(telemetry.pTerm, 30.0);
  const iBar = getBarMetrics(telemetry.iTerm, 20.0);
  const dBar = getBarMetrics(telemetry.dTerm, 20.0);

  const insightStyles = {
    sky: 'bg-sky-50 border-sky-200 text-sky-900',
    blue: 'bg-blue-50 border-blue-200 text-blue-900',
    emerald: 'bg-emerald-50 border-emerald-200 text-emerald-900',
    amber: 'bg-amber-50 border-amber-200 text-amber-900',
    rose: 'bg-rose-50 border-rose-200 text-rose-900',
    violet: 'bg-violet-50 border-violet-200 text-violet-900',
    slate: 'bg-slate-100 border-slate-300 text-slate-800'
  };

  const errSign = telemetry.error > 0 ? '+' : '';
  const algoStateText = !systemEnabled
    ? 'PAUSED / IDLE'
    : controllerType === 'ON_OFF'
    ? 'BANG-BANG ACTIVE'
    : Math.abs(telemetry.error) < 0.8
    ? 'PID CONVERGED'
    : 'PID TRACKING';

  const settlingDisplay =
    controllerType === 'ON_OFF'
      ? 'LIMIT CYCLE'
      : telemetry.settlingTime > 0
      ? `${telemetry.settlingTime.toFixed(2)} s`
      : 'SETTLING...';

  const overshootDisplay =
    controllerType === 'ON_OFF' ? 'SUSTAINED' : `${telemetry.peakOvershoot.toFixed(1)} %`;

  const navPages = [
    { id: 'bench', label: 'Live Control Bench', icon: LayoutDashboard },
    { id: 'compare', label: 'Comparative Lab', icon: GitCompare },
    { id: 'hardware', label: 'Arduino & Wiring', icon: Code2 },
    { id: 'theory', label: 'Control Theory & Notes', icon: BookOpen }
  ];

  return (
    <div className="min-h-screen bg-slate-50 text-slate-900 flex flex-col font-sans select-none">
      {/* 1. TOP COCKPIT HEADER, NAVIGATION TABS & SWITCHBOARD */}
      <header className="bg-white border-b border-slate-200 px-4 lg:px-6 pt-3 pb-0 shadow-xs">
        <div className="max-w-[1440px] mx-auto flex flex-col gap-3">
          {/* Top Row: Brand + Global Hardware Switchboard */}
          <div className="flex flex-col lg:flex-row lg:items-center lg:justify-between gap-3">
            <div>
              <div className="flex items-center gap-2 flex-wrap">
                <span className="inline-flex items-center gap-1.5 px-2.5 py-0.5 rounded-md bg-sky-600 text-white text-xs font-extrabold tracking-wider">
                  <Radio className="w-3.5 h-3.5" />
                  CONTROL-X
                </span>
                <h1 className="text-sm sm:text-base font-extrabold tracking-tight text-slate-900">
                  DUAL DOMAIN PID &amp; PHYSICAL PLANT BENCH
                </h1>
                <span className="hidden md:inline-block text-xs font-medium px-2 py-0.5 rounded-full bg-slate-100 text-slate-600 border border-slate-200">
                  Closed-Loop Control Laboratory
                </span>
              </div>
              <p
                className={`text-xs font-semibold mt-1 ${
                  envView === 'plant'
                    ? 'text-emerald-700'
                    : envView === 'vehicle'
                    ? 'text-blue-700'
                    : 'text-violet-700'
                }`}
              >
                {activeEnv.subHeader}
              </p>
            </div>

            {/* Global Switchboard Controls */}
            <div className="flex flex-wrap items-center gap-2">
              <button
                onClick={toggleSystemPower}
                className={`inline-flex items-center gap-1.5 px-3 py-1.5 rounded-lg text-xs font-bold transition-all cursor-pointer shadow-xs ${
                  systemEnabled
                    ? 'bg-emerald-600 hover:bg-emerald-700 text-white'
                    : 'bg-rose-600 hover:bg-rose-700 text-white'
                }`}
              >
                <Power className="w-3.5 h-3.5" />
                {systemEnabled ? '● SYSTEM ACTIVE' : '● SYSTEM IDLE (OFF)'}
              </button>

              <button
                onClick={toggleControllerAlgo}
                className={`inline-flex items-center gap-1.5 px-3 py-1.5 rounded-lg text-xs font-bold border transition-all cursor-pointer shadow-xs ${
                  controllerType === 'PID'
                    ? 'bg-sky-50 hover:bg-sky-100 text-sky-800 border-sky-300'
                    : 'bg-rose-50 hover:bg-rose-100 text-rose-700 border-rose-300'
                }`}
              >
                <Cpu className="w-3.5 h-3.5" />
                {controllerType === 'PID' ? 'MODE: PID CONTROLLER' : 'MODE: ON/OFF BANG-BANG'}
              </button>

              <button
                onClick={cycleEnvView}
                className={`inline-flex items-center gap-1.5 px-3 py-1.5 rounded-lg text-xs font-bold text-white transition-all cursor-pointer shadow-xs ${
                  envView === 'plant'
                    ? 'bg-emerald-700 hover:bg-emerald-800'
                    : envView === 'vehicle'
                    ? 'bg-blue-600 hover:bg-blue-700'
                    : 'bg-violet-600 hover:bg-violet-700'
                }`}
              >
                {envView === 'plant' && <Sprout className="w-3.5 h-3.5" />}
                {envView === 'vehicle' && <Car className="w-3.5 h-3.5" />}
                {envView === 'bench' && <Microscope className="w-3.5 h-3.5" />}
                ENV: {activeEnv.emoji} {activeEnv.label}
              </button>

              <button
                onClick={toggleHwConnection}
                title="Connect Arduino via Web Serial API (115200 baud)"
                className={`inline-flex items-center gap-1.5 px-3 py-1.5 rounded-lg text-xs font-bold border transition-all cursor-pointer shadow-xs ${
                  isHwConnected
                    ? 'bg-rose-600 hover:bg-rose-700 text-white border-rose-700'
                    : 'bg-white hover:bg-slate-100 text-slate-700 border-slate-300'
                }`}
              >
                <Usb className="w-3.5 h-3.5" />
                {isHwConnected ? 'DISCONNECT USB' : 'CONNECT USB'}
              </button>
            </div>
          </div>

          {hwStatusMsg && (
            <div className="px-3 py-1.5 rounded-md bg-slate-100 border border-slate-200 text-xs font-medium text-slate-700 flex items-center gap-2">
              {isHwConnected ? (
                <CheckCircle2 className="w-3.5 h-3.5 text-emerald-600 shrink-0" />
              ) : (
                <AlertTriangle className="w-3.5 h-3.5 text-amber-600 shrink-0" />
              )}
              <span>{hwStatusMsg}</span>
            </div>
          )}

          {/* Multi-Page Navigation Tab Bar */}
          <nav className="flex items-center justify-between gap-2 pt-1 border-t border-slate-100 overflow-x-auto">
            <div className="flex items-center gap-1">
              {navPages.map((tab) => {
                const Icon = tab.icon;
                const isActive = activePage === tab.id;
                return (
                  <button
                    key={tab.id}
                    onClick={() => setActivePage(tab.id)}
                    className={`inline-flex items-center gap-1.5 px-3.5 py-2 text-xs font-bold border-b-2 transition-all cursor-pointer whitespace-nowrap ${
                      isActive
                        ? 'border-sky-600 text-sky-700 bg-sky-50/60 rounded-t-lg'
                        : 'border-transparent text-slate-500 hover:text-slate-800 hover:bg-slate-50 rounded-t-lg'
                    }`}
                  >
                    <Icon className="w-3.5 h-3.5" />
                    {tab.label}
                  </button>
                );
              })}
            </div>

            <button
              onClick={handleExportCsv}
              className="hidden sm:inline-flex items-center gap-1.5 px-2.5 py-1 mb-1 rounded-md text-[11px] font-bold bg-slate-100 hover:bg-slate-200 text-slate-700 border border-slate-200 transition-colors cursor-pointer"
              title="Download current 140-sample oscilloscope buffer as CSV"
            >
              <Download className="w-3 h-3" />
              Export CSV Log
            </button>
          </nav>
        </div>
      </header>

      {/* =====================================================================
          PAGE 1: LIVE CONTROL BENCH (PRIMARY SIMULATOR WORKSPACE)
         ===================================================================== */}
      {activePage === 'bench' && (
        <main className="flex-1 max-w-[1440px] w-full mx-auto px-4 lg:px-6 py-3 flex flex-col gap-3">
          {/* 6 Live Telemetry Cards */}
          <section className="grid grid-cols-2 sm:grid-cols-3 lg:grid-cols-6 gap-2.5">
            <TelemetryCard
              label="TARGET SETPOINT (r)"
              value={`${telemetry.target.toFixed(1)} cm`}
              accentClass="bg-sky-500"
              valueClass="text-sky-700"
            />
            <TelemetryCard
              label={envView === 'vehicle' ? 'HEADWAY GAP (y)' : 'PLANT DISTANCE (y)'}
              value={`${telemetry.position.toFixed(1)} ${envView === 'vehicle' ? 'm' : 'cm'}`}
              accentClass="bg-blue-600"
              valueClass="text-blue-700"
            />
            <TelemetryCard
              label="TRACKING ERROR (e)"
              value={`${errSign}${telemetry.error.toFixed(1)} cm`}
              accentClass="bg-rose-500"
              valueClass="text-rose-600"
            />
            <TelemetryCard
              label="ALGORITHM STATE"
              value={algoStateText}
              accentClass={!systemEnabled || controllerType === 'ON_OFF' ? 'bg-rose-500' : 'bg-emerald-500'}
              valueClass={!systemEnabled || controllerType === 'ON_OFF' ? 'text-rose-600' : 'text-emerald-700'}
            />
            <TelemetryCard
              label="SETTLING TIME (ts)"
              value={settlingDisplay}
              accentClass={controllerType === 'ON_OFF' ? 'bg-rose-500' : 'bg-amber-500'}
              valueClass={controllerType === 'ON_OFF' ? 'text-rose-600' : 'text-amber-700'}
            />
            <TelemetryCard
              label="PEAK OVERSHOOT (Mp)"
              value={overshootDisplay}
              accentClass={controllerType === 'ON_OFF' ? 'bg-rose-500' : 'bg-violet-500'}
              valueClass={controllerType === 'ON_OFF' ? 'text-rose-600' : 'text-violet-700'}
            />
          </section>

          {/* Live PID Decomposition HUD & Real-Time Insight Ticker */}
          <section className="bg-white border border-slate-200 rounded-xl p-3 shadow-xs grid grid-cols-1 lg:grid-cols-12 gap-3 items-center">
            <div className="lg:col-span-8 grid grid-cols-1 sm:grid-cols-3 gap-3">
              <ActionMeter
                title="P-Action (Spring Restoring)"
                value={telemetry.pTerm}
                bar={pBar}
                barColor="bg-sky-500"
                textColor="text-sky-700"
              />
              <ActionMeter
                title="I-Action (Steady-State Int.)"
                value={telemetry.iTerm}
                bar={iBar}
                barColor="bg-blue-600"
                textColor="text-blue-700"
              />
              <ActionMeter
                title="D-Action (Velocity Damper)"
                value={telemetry.dTerm}
                bar={dBar}
                barColor="bg-amber-500"
                textColor="text-amber-700"
              />
            </div>

            <div
              className={`lg:col-span-4 rounded-lg border px-3 py-2 flex items-start gap-2.5 transition-colors ${
                insightStyles[insight.tone] || insightStyles.sky
              }`}
            >
              <Info className="w-4 h-4 mt-0.5 shrink-0 opacity-80" />
              <div className="min-w-0">
                <div className="text-[10px] font-extrabold uppercase tracking-wider opacity-70">
                  Real-Time Control Insight
                </div>
                <p className="text-xs font-semibold leading-snug mt-0.5">{insight.text}</p>
              </div>
            </div>
          </section>

          {/* Center Visualizers: Multi-Environment Canvas + Digital Oscilloscope */}
          <section className="flex-1 flex flex-col gap-3 min-h-[370px]">
            <div className="bg-white border border-slate-200 rounded-xl overflow-hidden shadow-xs flex flex-col">
              <div className="px-3.5 py-2 border-b border-slate-100 bg-slate-50/70 flex flex-wrap items-center justify-between gap-2">
                <span
                  className={`text-xs font-extrabold tracking-tight ${
                    envView === 'plant'
                      ? 'text-emerald-700'
                      : envView === 'vehicle'
                      ? 'text-blue-700'
                      : 'text-violet-700'
                  }`}
                >
                  {activeEnv.animTitle}
                </span>
                <span className="text-[11px] font-bold text-slate-500">{activeEnv.animStatus}</span>
              </div>
              <div className="w-full h-[180px] relative">
                <canvas ref={animCanvasRef} className="w-full h-full block" />
              </div>
            </div>

            <div className="flex-1 bg-white border border-slate-200 rounded-xl overflow-hidden shadow-xs flex flex-col min-h-[195px]">
              <div className="px-3.5 py-2 border-b border-slate-100 bg-slate-50/70 flex flex-wrap items-center justify-between gap-2">
                <div className="flex flex-wrap items-center gap-3">
                  <span className="inline-flex items-center gap-1.5 text-xs font-extrabold text-slate-800">
                    <Activity className="w-3.5 h-3.5 text-blue-600" />
                    DYNAMIC STEP RESPONSE (DIGITAL OSCILLOSCOPE)
                  </span>
                  <div className="flex items-center gap-3 text-[11px] font-bold">
                    <span className="inline-flex items-center gap-1 text-sky-700">
                      <span className="w-2.5 h-2.5 rounded-xs bg-sky-500 inline-block" />
                      Setpoint r(t)
                    </span>
                    <span
                      className={`inline-flex items-center gap-1 ${
                        controllerType === 'PID' ? 'text-blue-700' : 'text-rose-600'
                      }`}
                    >
                      <span
                        className={`w-2.5 h-2.5 rounded-xs inline-block ${
                          controllerType === 'PID' ? 'bg-blue-600' : 'bg-rose-600'
                        }`}
                      />
                      Output y(t)
                    </span>
                    <span className="inline-flex items-center gap-1 text-amber-700">
                      <span className="w-2.5 h-2.5 rounded-xs bg-amber-500 inline-block" />
                      Effort u(t)
                    </span>
                  </div>
                </div>

                <div className="text-xs font-mono font-bold text-emerald-700 bg-emerald-50 border border-emerald-200 px-2.5 py-0.5 rounded-md">
                  [{controllerType === 'PID' ? 'PID' : 'Bang-Bang'}] tr:{' '}
                  {telemetry.riseTime > 0 ? `${telemetry.riseTime.toFixed(2)}s` : '--'} | ts:{' '}
                  {telemetry.settlingTime > 0 ? `${telemetry.settlingTime.toFixed(2)}s` : '--'} | Mp:{' '}
                  {telemetry.peakOvershoot.toFixed(1)}% | e_ss:{' '}
                  {telemetry.steadyStateError.toFixed(2)}cm
                </div>
              </div>
              <div className="flex-1 w-full relative min-h-[155px]">
                <canvas ref={scopeCanvasRef} className="w-full h-full block" />
              </div>
            </div>
          </section>

          {/* Bottom Cockpit Bench Controls + Tuning Presets */}
          <section className="bg-white border border-slate-200 rounded-xl p-3.5 shadow-xs flex flex-col gap-3">
            {/* Quick Tuning Presets Strip */}
            <div className="flex flex-wrap items-center justify-between gap-2 pb-2 border-b border-slate-100">
              <span className="inline-flex items-center gap-1.5 text-[11px] font-extrabold uppercase tracking-wider text-slate-500">
                <Sparkles className="w-3.5 h-3.5 text-amber-500" />
                One-Click Damping Profiles:
              </span>
              <div className="flex flex-wrap items-center gap-1.5">
                {TUNING_PRESETS.map((preset) => (
                  <button
                    key={preset.id}
                    onClick={() => applyPreset(preset)}
                    className="inline-flex items-center gap-1.5 px-2.5 py-1 rounded-md text-xs font-bold bg-slate-50 hover:bg-sky-50 text-slate-700 hover:text-sky-800 border border-slate-200 hover:border-sky-300 transition-colors cursor-pointer"
                  >
                    <span>{preset.name}</span>
                    <span className="text-[10px] px-1.5 py-0.2 rounded bg-white border border-slate-200 text-slate-500">
                      {preset.badge}
                    </span>
                  </button>
                ))}
              </div>
            </div>

            {/* Tuning Sliders Grid */}
            <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-5 gap-3">
              <TuningSlider
                label="PROPORTIONAL (Kp)"
                value={kp}
                min={0}
                max={10}
                step={0.1}
                accent="accent-sky-600"
                badgeClass="bg-sky-50 text-sky-700 border-sky-200"
                onChange={(v) => handleSliderChange('P', v)}
              />
              <TuningSlider
                label="INTEGRAL (Ki)"
                value={ki}
                min={0}
                max={0.5}
                step={0.01}
                accent="accent-blue-600"
                badgeClass="bg-blue-50 text-blue-700 border-blue-200"
                onChange={(v) => handleSliderChange('I', v)}
              />
              <TuningSlider
                label="DERIVATIVE (Kd)"
                value={kd}
                min={0}
                max={3.0}
                step={0.05}
                accent="accent-amber-600"
                badgeClass="bg-amber-50 text-amber-700 border-amber-200"
                onChange={(v) => handleSliderChange('D', v)}
              />
              <TuningSlider
                label="TARGET SETPOINT (cm)"
                value={target}
                min={5}
                max={45}
                step={1}
                accent="accent-violet-600"
                badgeClass="bg-violet-50 text-violet-700 border-violet-200"
                onChange={(v) => handleSliderChange('T', v)}
              />
              <TuningSlider
                label="SENSOR NOISE JITTER"
                value={noiseAmp}
                min={0}
                max={2.0}
                step={0.1}
                accent="accent-rose-600"
                badgeClass="bg-rose-50 text-rose-700 border-rose-200"
                onChange={(v) => handleSliderChange('N', v)}
              />
            </div>

            {/* Step Probes & Disturbance Impulse Bar */}
            <div className="pt-2 border-t border-slate-100 flex flex-wrap items-center justify-between gap-2">
              <div className="flex flex-wrap items-center gap-2">
                <span className="inline-flex items-center gap-1 text-[11px] font-extrabold uppercase tracking-wider text-slate-500 mr-1">
                  <Gauge className="w-3.5 h-3.5 text-slate-400" />
                  Dynamic Test Probes:
                </span>
                {[15.0, 25.0, 35.0].map((val) => (
                  <button
                    key={val}
                    onClick={() => triggerStep(val)}
                    className="px-3 py-1.5 rounded-lg text-xs font-bold bg-sky-50 hover:bg-sky-100 text-sky-800 border border-sky-200 transition-colors cursor-pointer"
                  >
                    Step {val} cm
                  </button>
                ))}
              </div>

              <div className="flex flex-wrap items-center gap-2">
                <button
                  onClick={injectNudge}
                  className="inline-flex items-center gap-1.5 px-3.5 py-1.5 rounded-lg text-xs font-bold bg-rose-600 hover:bg-rose-700 text-white shadow-xs transition-colors cursor-pointer"
                >
                  <Zap className="w-3.5 h-3.5" />
                  INJECT PHYSICAL DISTURBANCE (+8 cm NUDGE)
                </button>
                <button
                  onClick={resetIntegrator}
                  className="inline-flex items-center gap-1.5 px-3 py-1.5 rounded-lg text-xs font-bold bg-slate-100 hover:bg-slate-200 text-slate-700 border border-slate-300 transition-colors cursor-pointer"
                >
                  <RotateCcw className="w-3.5 h-3.5" />
                  Reset Integrator
                </button>
              </div>
            </div>
          </section>
        </main>
      )}

      {/* =====================================================================
          PAGE 2: COMPARATIVE BENCHMARK LAB (PID vs BANG-BANG vs P-ONLY)
         ===================================================================== */}
      {activePage === 'compare' && (
        <main className="flex-1 max-w-[1440px] w-full mx-auto px-4 lg:px-6 py-4 flex flex-col gap-4 select-text">
          <div className="bg-white border border-slate-200 rounded-xl p-4 shadow-xs flex flex-col md:flex-row md:items-center md:justify-between gap-4">
            <div>
              <h2 className="text-base font-extrabold text-slate-900 flex items-center gap-2">
                <GitCompare className="w-4 h-4 text-sky-600" />
                Head-to-Head Step Response Benchmark Lab
              </h2>
              <p className="text-xs text-slate-600 mt-1">
                Simultaneously simulates your current PID gains (Kp={kp.toFixed(2)}, Ki={ki.toFixed(2)}, Kd={kd.toFixed(2)}) against P-Only and ON/OFF Bang-Bang controllers under identical plant inertia and noise.
              </p>
            </div>

            <div className="flex flex-wrap items-center gap-3">
              <div className="flex items-center gap-2 bg-slate-50 border border-slate-200 rounded-lg px-3 py-1.5 text-xs font-bold">
                <span className="text-slate-500">Initial (cm):</span>
                <input
                  type="number"
                  min={5}
                  max={45}
                  value={benchStepFrom}
                  onChange={(e) => setBenchStepFrom(Number(e.target.value))}
                  className="w-14 bg-white border border-slate-300 rounded px-1.5 py-0.5 text-slate-800 font-mono"
                />
                <span className="text-slate-500">Target (cm):</span>
                <input
                  type="number"
                  min={5}
                  max={45}
                  value={benchStepTo}
                  onChange={(e) => setBenchStepTo(Number(e.target.value))}
                  className="w-14 bg-white border border-slate-300 rounded px-1.5 py-0.5 text-slate-800 font-mono"
                />
              </div>

              <button
                onClick={runHeadToHeadBenchmark}
                className="inline-flex items-center gap-1.5 px-4 py-2 rounded-lg text-xs font-bold bg-sky-600 hover:bg-sky-700 text-white shadow-xs cursor-pointer"
              >
                <Play className="w-3.5 h-3.5" />
                Run 5.0s Benchmark Trial
              </button>
            </div>
          </div>

          {/* Superimposed Overlay Plot */}
          <div className="bg-white border border-slate-200 rounded-xl overflow-hidden shadow-xs">
            <div className="px-4 py-2.5 border-b border-slate-100 bg-slate-50/70 flex flex-wrap items-center justify-between gap-2">
              <span className="text-xs font-extrabold text-slate-800">
                SUPERIMPOSED TRANSIENT RESPONSE OVERLAY (5.0 SECOND WINDOW)
              </span>
              <div className="flex flex-wrap items-center gap-4 text-xs font-bold">
                <span className="inline-flex items-center gap-1.5 text-sky-700">
                  <span className="w-3 h-0.5 bg-sky-500 inline-block" />
                  Target Setpoint ({benchStepTo} cm)
                </span>
                <span className="inline-flex items-center gap-1.5 text-blue-700">
                  <span className="w-3 h-1 bg-blue-600 inline-block rounded-full" />
                  Filtered PID (Proposed)
                </span>
                <span className="inline-flex items-center gap-1.5 text-amber-700">
                  <span className="w-3 h-0.5 bg-amber-600 inline-block" />
                  Proportional-Only (P)
                </span>
                <span className="inline-flex items-center gap-1.5 text-rose-600">
                  <span className="w-3 h-0.5 bg-rose-600 inline-block" />
                  ON/OFF Bang-Bang
                </span>
              </div>
            </div>
            <div className="w-full h-[260px] p-2">
              <canvas ref={benchCanvasRef} className="w-full h-full block" />
            </div>
          </div>

          {/* Quantitative Evaluation Metrics Table */}
          {benchmarkData && (
            <div className="bg-white border border-slate-200 rounded-xl overflow-hidden shadow-xs">
              <div className="px-4 py-3 border-b border-slate-100 bg-slate-50/70">
                <h3 className="text-xs font-extrabold uppercase tracking-wider text-slate-700">
                  Quantitative Time-Domain Performance Matrix ({benchmarkData.from} cm → {benchmarkData.to} cm Step)
                </h3>
              </div>
              <div className="overflow-x-auto">
                <table className="w-full text-left border-collapse text-xs">
                  <thead>
                    <tr className="bg-slate-50 border-b border-slate-200 text-slate-600 uppercase text-[11px]">
                      <th className="py-2.5 px-4 font-extrabold">Controller Architecture</th>
                      <th className="py-2.5 px-4 font-extrabold">Rise Time (tr)</th>
                      <th className="py-2.5 px-4 font-extrabold">5% Settling (ts)</th>
                      <th className="py-2.5 px-4 font-extrabold">Overshoot (Mp)</th>
                      <th className="py-2.5 px-4 font-extrabold">Final Error (ess)</th>
                      <th className="py-2.5 px-4 font-extrabold">Integral Abs. Error (IAE)</th>
                      <th className="py-2.5 px-4 font-extrabold">Stability Verdict</th>
                    </tr>
                  </thead>
                  <tbody className="divide-y divide-slate-100">
                    <tr className="bg-blue-50/40 font-semibold">
                      <td className="py-3 px-4 font-extrabold text-blue-800">
                        Filtered PID (Kp={kp.toFixed(1)}, Ki={ki.toFixed(2)}, Kd={kd.toFixed(2)})
                      </td>
                      <td className="py-3 px-4 font-mono">
                        {benchmarkData.pid.riseTime ? `${benchmarkData.pid.riseTime.toFixed(2)} s` : '--'}
                      </td>
                      <td className="py-3 px-4 font-mono text-emerald-700 font-bold">
                        {benchmarkData.pid.settlingTime
                          ? `${benchmarkData.pid.settlingTime.toFixed(2)} s`
                          : 'Unsettled'}
                      </td>
                      <td className="py-3 px-4 font-mono">
                        {benchmarkData.pid.peakOvershoot.toFixed(1)} %
                      </td>
                      <td className="py-3 px-4 font-mono text-emerald-700 font-bold">
                        {benchmarkData.pid.steadyStateError.toFixed(2)} cm
                      </td>
                      <td className="py-3 px-4 font-mono">{benchmarkData.pid.iae.toFixed(2)} cm·s</td>
                      <td className="py-3 px-4">
                        <span className="px-2 py-0.5 rounded-full bg-emerald-100 text-emerald-800 text-[11px] font-bold">
                          Asymptotically Stable
                        </span>
                      </td>
                    </tr>
                    <tr>
                      <td className="py-3 px-4 font-bold text-amber-800">
                        Proportional-Only (Kp={kp.toFixed(1)}, Ki=0, Kd=0)
                      </td>
                      <td className="py-3 px-4 font-mono">
                        {benchmarkData.pOnly.riseTime ? `${benchmarkData.pOnly.riseTime.toFixed(2)} s` : '--'}
                      </td>
                      <td className="py-3 px-4 font-mono">
                        {benchmarkData.pOnly.settlingTime
                          ? `${benchmarkData.pOnly.settlingTime.toFixed(2)} s`
                          : 'Offset / Ringing'}
                      </td>
                      <td className="py-3 px-4 font-mono">
                        {benchmarkData.pOnly.peakOvershoot.toFixed(1)} %
                      </td>
                      <td className="py-3 px-4 font-mono text-amber-700">
                        {benchmarkData.pOnly.steadyStateError.toFixed(2)} cm
                      </td>
                      <td className="py-3 px-4 font-mono">{benchmarkData.pOnly.iae.toFixed(2)} cm·s</td>
                      <td className="py-3 px-4">
                        <span className="px-2 py-0.5 rounded-full bg-amber-100 text-amber-800 text-[11px] font-bold">
                          Underdamped / Offset
                        </span>
                      </td>
                    </tr>
                    <tr>
                      <td className="py-3 px-4 font-bold text-rose-700">
                        ON/OFF Bang-Bang (±35° Relay)
                      </td>
                      <td className="py-3 px-4 font-mono">
                        {benchmarkData.bangBang.riseTime
                          ? `${benchmarkData.bangBang.riseTime.toFixed(2)} s`
                          : '--'}
                      </td>
                      <td className="py-3 px-4 font-mono text-rose-600 font-bold">∞ (Limit Cycle)</td>
                      <td className="py-3 px-4 font-mono text-rose-600">
                        {benchmarkData.bangBang.peakOvershoot.toFixed(1)} %
                      </td>
                      <td className="py-3 px-4 font-mono text-rose-600">
                        {benchmarkData.bangBang.steadyStateError.toFixed(2)} cm
                      </td>
                      <td className="py-3 px-4 font-mono">{benchmarkData.bangBang.iae.toFixed(2)} cm·s</td>
                      <td className="py-3 px-4">
                        <span className="px-2 py-0.5 rounded-full bg-rose-100 text-rose-800 text-[11px] font-bold">
                          Sustained Oscillation
                        </span>
                      </td>
                    </tr>
                  </tbody>
                </table>
              </div>
            </div>
          )}
        </main>
      )}

      {/* =====================================================================
          PAGE 3: ARDUINO FIRMWARE & HARDWARE BENCH GUIDE
         ===================================================================== */}
      {activePage === 'hardware' && (
        <main className="flex-1 max-w-[1440px] w-full mx-auto px-4 lg:px-6 py-4 grid grid-cols-1 lg:grid-cols-12 gap-4 select-text">
          {/* Left Column: Pinout Diagram & Serial Protocol */}
          <div className="lg:col-span-5 flex flex-col gap-4">
            <div className="bg-white border border-slate-200 rounded-xl p-4 shadow-xs">
              <h2 className="text-sm font-extrabold text-slate-900 flex items-center gap-2">
                <Wrench className="w-4 h-4 text-sky-600" />
                ATmega328P Hardware Wiring Pinout
              </h2>
              <p className="text-xs text-slate-600 mt-1">
                Connect your HC-SR04 ultrasonic transceiver and micro-servo to the Arduino Uno/Nano as specified below, then click <strong>CONNECT USB</strong> in the top header.
              </p>

              <div className="mt-3 overflow-hidden border border-slate-200 rounded-lg">
                <table className="w-full text-left text-xs">
                  <thead className="bg-slate-50 border-b border-slate-200 text-slate-600">
                    <tr>
                      <th className="py-2 px-3 font-extrabold">Component</th>
                      <th className="py-2 px-3 font-extrabold">Module Pin</th>
                      <th className="py-2 px-3 font-extrabold">Arduino Pin</th>
                    </tr>
                  </thead>
                  <tbody className="divide-y divide-slate-100">
                    <tr>
                      <td className="py-2 px-3 font-bold text-slate-800">HC-SR04 Ultrasonic</td>
                      <td className="py-2 px-3 font-mono text-sky-700">VCC / GND</td>
                      <td className="py-2 px-3 font-mono">5V / GND</td>
                    </tr>
                    <tr>
                      <td className="py-2 px-3 font-bold text-slate-800">HC-SR04 Ultrasonic</td>
                      <td className="py-2 px-3 font-mono text-sky-700">TRIG</td>
                      <td className="py-2 px-3 font-mono font-bold">Digital D9</td>
                    </tr>
                    <tr>
                      <td className="py-2 px-3 font-bold text-slate-800">HC-SR04 Ultrasonic</td>
                      <td className="py-2 px-3 font-mono text-sky-700">ECHO</td>
                      <td className="py-2 px-3 font-mono font-bold">Digital D10</td>
                    </tr>
                    <tr>
                      <td className="py-2 px-3 font-bold text-slate-800">Micro Servo (SG90)</td>
                      <td className="py-2 px-3 font-mono text-amber-700">PWM Signal</td>
                      <td className="py-2 px-3 font-mono font-bold">Digital D6 (PWM)</td>
                    </tr>
                    <tr>
                      <td className="py-2 px-3 font-bold text-slate-800">Micro Servo (SG90)</td>
                      <td className="py-2 px-3 font-mono text-amber-700">V+ / GND</td>
                      <td className="py-2 px-3 font-mono">External 5V / Common GND</td>
                    </tr>
                  </tbody>
                </table>
              </div>
            </div>

            <div className="bg-white border border-slate-200 rounded-xl p-4 shadow-xs">
              <h3 className="text-sm font-extrabold text-slate-900">
                Web Serial Telemetry Protocol (115200 bps)
              </h3>
              <p className="text-xs text-slate-600 mt-1">
                The browser communicates bidirectionally with the microcontroller using lightweight ASCII framing:
              </p>
              <div className="mt-2.5 space-y-2 text-xs">
                <div className="p-2.5 rounded-lg bg-sky-50 border border-sky-200">
                  <div className="font-extrabold text-sky-900">Host → Arduino (Slider Updates)</div>
                  <code className="block font-mono text-[11px] text-sky-800 mt-1">
                    P2.40\n I0.05\n D0.85\n T20.0\n
                  </code>
                </div>
                <div className="p-2.5 rounded-lg bg-emerald-50 border border-emerald-200">
                  <div className="font-extrabold text-emerald-900">Arduino → Host (50 Hz Telemetry)</div>
                  <code className="block font-mono text-[11px] text-emerald-800 mt-1 break-all">
                    Target:20.00\tDistance:19.95\tServo:90.2\tOutput:0.20\tP:0.12\tI:0.08\tD:0.00
                  </code>
                </div>
              </div>
            </div>
          </div>

          {/* Right Column: Complete Copyable Arduino Sketch */}
          <div className="lg:col-span-7 bg-white border border-slate-200 rounded-xl overflow-hidden shadow-xs flex flex-col">
            <div className="px-4 py-2.5 border-b border-slate-200 bg-slate-50 flex items-center justify-between">
              <div className="flex items-center gap-2">
                <Code2 className="w-4 h-4 text-sky-600" />
                <span className="text-xs font-extrabold text-slate-800">
                  pid_control_main.ino — Complete Embedded Firmware
                </span>
              </div>
              <button
                onClick={handleCopyCode}
                className="inline-flex items-center gap-1.5 px-3 py-1 rounded-md text-xs font-bold bg-sky-600 hover:bg-sky-700 text-white transition-colors cursor-pointer"
              >
                {copiedCode ? <Check className="w-3.5 h-3.5" /> : <Copy className="w-3.5 h-3.5" />}
                {copiedCode ? 'Copied to Clipboard!' : 'Copy Sketch'}
              </button>
            </div>
            <pre className="p-4 text-xs font-mono text-slate-800 bg-slate-50/40 overflow-x-auto leading-relaxed flex-1 max-h-[540px]">
              <code>{ARDUINO_SKETCH_CODE}</code>
            </pre>
          </div>
        </main>
      )}

      {/* =====================================================================
          PAGE 4: CONTROL THEORY & MATHEMATICAL ARCHITECTURE NOTES
         ===================================================================== */}
      {activePage === 'theory' && (
        <main className="flex-1 max-w-[1440px] w-full mx-auto px-4 lg:px-6 py-4 grid grid-cols-1 md:grid-cols-2 gap-4 select-text">
          <div className="bg-white border border-slate-200 rounded-xl p-5 shadow-xs space-y-3">
            <h2 className="text-base font-extrabold text-slate-900">
              1. Second-Order Physical Plant Dynamics
            </h2>
            <p className="text-xs text-slate-600 leading-relaxed">
              The mechanical testbed models a 1-DOF translational carriage of mass <strong>m = 1.0 kg</strong> and viscous damping coefficient <strong>c = 1.8 N·s/cm</strong> driven by the sine projection of the micro-servo tilt angle:
            </p>
            <div className="p-3 rounded-lg bg-slate-50 border border-slate-200 font-mono text-xs text-slate-800">
              m · (d²x/dt²) + c · (dx/dt) = F_max · sin(θ - 90°) + F_dist
            </div>
            <p className="text-xs text-slate-600 leading-relaxed">
              Where <strong>F_max = 125.0 N</strong> and <strong>θ ∈ [20°, 160°]</strong>. Acoustic multipath jitter from the HC-SR04 transducer is modeled via zero-mean Gaussian noise sampled using the Box-Muller transform.
            </p>
          </div>

          <div className="bg-white border border-slate-200 rounded-xl p-5 shadow-xs space-y-3">
            <h2 className="text-base font-extrabold text-slate-900">
              2. Signal Conditioning &amp; Anti-Windup PID Law
            </h2>
            <p className="text-xs text-slate-600 leading-relaxed">
              To prevent high-frequency ultrasonic noise from causing servo chatter, two cascaded digital filters protect the control loop:
            </p>
            <ul className="text-xs text-slate-700 space-y-2 list-disc pl-4">
              <li>
                <strong>EMA Input Pre-Filter (α = 0.30):</strong> Smooths raw time-of-flight distance echoes before error computation.
              </li>
              <li>
                <strong>Clamped Integral Anti-Windup (±40.0 cm·s):</strong> Prevents integral accumulation runaway during large step maneuvers.
              </li>
              <li>
                <strong>Low-Pass Derivative Filter (α_d = 0.22):</strong> Attenuates high-frequency spikes on <code>de/dt</code> while preserving velocity braking.
              </li>
            </ul>
          </div>

          <div className="bg-white border border-slate-200 rounded-xl p-5 shadow-xs space-y-3 md:col-span-2">
            <h2 className="text-base font-extrabold text-slate-900">
              3. Multi-Domain Engineering Equivalence
            </h2>
            <div className="grid grid-cols-1 md:grid-cols-3 gap-3 text-xs">
              <div className="p-3.5 rounded-lg bg-emerald-50 border border-emerald-200">
                <div className="font-extrabold text-emerald-900">🌿 Smart Greenhouse Boom</div>
                <p className="text-emerald-800 mt-1 leading-relaxed">
                  Maintains uniform spray nozzle and PAR grow-light height above uneven botanical plant canopies despite structural gantry vibrations.
                </p>
              </div>
              <div className="p-3.5 rounded-lg bg-blue-50 border border-blue-200">
                <div className="font-extrabold text-blue-900">🚗 Autonomous Vehicle ACC</div>
                <p className="text-blue-800 mt-1 leading-relaxed">
                  Regulates inter-vehicle headway distance behind a lead vehicle; derivative damping provides smooth deceleration without passenger jerk.
                </p>
              </div>
              <div className="p-3.5 rounded-lg bg-violet-50 border border-violet-200">
                <div className="font-extrabold text-violet-900">🔬 1-DOF Laboratory Track</div>
                <p className="text-violet-800 mt-1 leading-relaxed">
                  Direct physical twin of the ATmega328P + HC-SR04 + micro-servo bench used for step-response verification and parameter tuning.
                </p>
              </div>
            </div>
          </div>
        </main>
      )}
    </div>
  );
}

// Reusable Telemetry Card Component
function TelemetryCard({ label, value, accentClass, valueClass }) {
  return (
    <div className="bg-white border border-slate-200 rounded-xl p-2.5 shadow-xs flex flex-col justify-between relative overflow-hidden">
      <div className={`absolute top-0 left-0 right-0 h-1 ${accentClass}`} />
      <span className="text-[10px] font-extrabold uppercase tracking-wider text-slate-500 mt-0.5">
        {label}
      </span>
      <span className={`text-base sm:text-lg font-extrabold tracking-tight mt-0.5 ${valueClass}`}>
        {value}
      </span>
    </div>
  );
}

// Reusable Bi-Directional PID Action Meter Component
function ActionMeter({ title, value, bar, barColor, textColor }) {
  const sign = value > 0 ? '+' : '';
  return (
    <div className="bg-slate-50 border border-slate-200/80 rounded-lg px-2.5 py-2">
      <div className="flex items-center justify-between text-[11px] font-bold mb-1.5">
        <span className="text-slate-600 truncate pr-1">{title}</span>
        <span className={`font-mono font-extrabold shrink-0 ${textColor}`}>
          {sign}
          {value.toFixed(1)} N
        </span>
      </div>
      <div className="h-2.5 w-full bg-white border border-slate-200 rounded-full relative overflow-hidden">
        <div className="absolute top-0 bottom-0 left-1/2 w-px bg-slate-400 z-10" />
        <div
          className={`absolute top-0.5 bottom-0.5 rounded-xs transition-all duration-75 ${barColor}`}
          style={{
            left: `${bar.leftPct}%`,
            width: `${bar.widthPct}%`
          }}
        />
      </div>
    </div>
  );
}

// Reusable Precision Tuning Slider Component
function TuningSlider({ label, value, min, max, step, accent, badgeClass, onChange }) {
  return (
    <div className="bg-slate-50 border border-slate-200/80 rounded-lg px-3 py-2 flex flex-col gap-1.5">
      <div className="flex items-center justify-between gap-1">
        <label className="text-[10px] font-extrabold uppercase tracking-wider text-slate-600 flex items-center gap-1">
          <Sliders className="w-3 h-3 text-slate-400" />
          {label}
        </label>
        <span className={`text-xs font-mono font-extrabold px-1.5 py-0.5 rounded border ${badgeClass}`}>
          {value.toFixed(2)}
        </span>
      </div>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={value}
        onChange={(e) => onChange(parseFloat(e.target.value))}
        className={`w-full h-1.5 bg-slate-200 rounded-lg cursor-pointer ${accent}`}
      />
    </div>
  );
}