#include <flutter/runtime_effect.glsl>
precision highp float;

uniform float u_time;
uniform vec2 u_resolution;
uniform vec4 u_colorA;
uniform vec4 u_colorB;
uniform float u_pulse;

out vec4 fragColor;

void main() {
  vec2 uv = FlutterFragCoord() / u_resolution;
  float base = 0.5 + 0.5 * sin(u_time * 2.0 + uv.y * 20.0 + uv.x * 10.0);
  float amp = mix(0.35, 1.0, clamp(u_pulse, 0.0, 1.0));
  float wave = clamp(base * amp, 0.0, 1.0);
  vec4 col = mix(u_colorA, u_colorB, wave);
  float glow = mix(0.85, 1.15, clamp(u_pulse, 0.0, 1.0));
  fragColor = col * glow;
}
