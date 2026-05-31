#!/usr/bin/env python3.11
import argparse
import math
import struct
import wave
from pathlib import Path


SAMPLE_RATE = 22050


def write_sine(path: Path, seconds: float, freq: float, volume: float) -> None:
    count = int(SAMPLE_RATE * seconds)
    with path.open("wb") as out:
        for i in range(count):
            sample = math.sin(2.0 * math.pi * freq * i / SAMPLE_RATE)
            out.write(struct.pack("<h", int(sample * volume * 32767)))


def convert_wav(src: Path, dst: Path, volume: float) -> None:
    with wave.open(str(src), "rb") as wav:
        channels = wav.getnchannels()
        width = wav.getsampwidth()
        rate = wav.getframerate()
        frames = wav.readframes(wav.getnframes())

    if width != 2:
        raise SystemExit("input WAV must be 16-bit PCM")

    samples = struct.iter_unpack("<" + "h" * channels, frames)
    mono = []
    for frame in samples:
        mono.append(sum(frame) / channels)

    with dst.open("wb") as out:
        for i in range(int(len(mono) * SAMPLE_RATE / rate)):
            source = i * rate / SAMPLE_RATE
            left = int(source)
            frac = source - left
            right = min(left + 1, len(mono) - 1)
            value = mono[left] * (1.0 - frac) + mono[right] * frac
            value = max(-32768, min(32767, int(value * volume)))
            out.write(struct.pack("<h", value))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path)
    parser.add_argument("--output", type=Path, default=Path("data/audio.s16"))
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--freq", type=float, default=1000.0)
    parser.add_argument("--volume", type=float, default=0.20)
    args = parser.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.input:
        convert_wav(args.input, args.output, args.volume)
    else:
        write_sine(args.output, args.seconds, args.freq, args.volume)
    print(args.output)


if __name__ == "__main__":
    main()
