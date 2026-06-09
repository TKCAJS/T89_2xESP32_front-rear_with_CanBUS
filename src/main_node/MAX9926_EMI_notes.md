# MAX9926 VR Sensor Interface — EMI Immunity Notes

Front-end for the RPM input (variable-reluctance sensor → MAX9926 → clean digital
pulse → ESP32 PCNT). Context: RPM momentarily dropped to 0 when the WiFi radio was
active under load. The firmware cause (timeout starved by loop stalls) was fixed in
`RPM.h` by driving liveness off the hardware PCNT counter. If drops persist, they are
signal-integrity / EMI, addressed by the hardware measures below.

Priority order is roughly highest-impact first for WiFi-band (RF) coupling.

## 1. Input filtering on IN+ / IN- (biggest electrical lever)
The VR inputs are differential — use that:
- Common-mode caps: small cap from each input to ground (~1–4.7 nF) shunts HF
  common-mode pickup (WiFi-band RF) before the comparator.
- Differential cap across IN+/IN- (a few nF) for differential noise.
- Series input resistors (datasheet-recommended R, often a few kΩ) form an RC
  low-pass with those caps. Set the corner ABOVE max signal frequency
  (max RPM × teeth) but well below the EMI band — typically a few kHz to ~10 kHz
  for a crank/flywheel VR.

## 2. Increase the hysteresis
Widen the programmable hysteresis band (external hysteresis resistor / config) to
raise the noise margin near the zero-crossing. Trade-off: at very low RPM the VR
output is small, so don't widen so far that cranking-speed pulses are missed. Tune
to the smallest real signal you must detect.

## 3. Use adaptive peak threshold mode
Run the threshold in adaptive peak-tracking mode (threshold follows a fraction of
signal amplitude) rather than a low fixed threshold. Adaptive tracking rejects noise
below the real pulse amplitude — ideal since VR amplitude scales with RPM.

## 4. Cabling (huge for VR sensors)
- Twisted pair from sensor to MAX9926 — cancels inductively-coupled noise.
- Shielded cable, shield grounded at the board end ONLY (single-point), not both.
- Route away from the ESP32, its antenna, and relay/servo/switching wiring.

## 5. Output side (MAX9926 → ESP32 GPIO)
The clean digital output can still pick up RF on the way to the pin:
- Keep the trace short.
- Series resistor (~100–330 Ω) at the MAX9926 output + small cap (~100 pF) at the
  GPIO: light RC that damps ringing / filters RF without smearing edges. Backed by
  the PCNT glitch filter (~1.25 µs) in firmware.
- Ferrite bead on this line near the GPIO — cheap and effective against WiFi band.

## 6. Power & layout
- Solid VCC decoupling at the MAX9926: 100 nF at the pin + a bulk cap.
- Ground plane under the IC; separate/shield the analog input section from the WiFi
  antenna/PA. The antenna is the dominant RF source — physical distance matters as
  much as filtering.

## Do-first shortlist (given WiFi correlation)
1. Twisted, shielded input pair (single-point shield ground).
2. Common-mode caps on IN+/IN-.
3. Ferrite + series-R/cap on the output line.

## Caveat
Exact R/C values and the hysteresis/threshold config are MAX9926-specific — take them
from the datasheet application circuit and size the RC corners to your actual
tooth-count × max-RPM frequency so real pulses aren't filtered out.
