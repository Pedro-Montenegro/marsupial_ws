#!/usr/bin/env python3
import subprocess
import re
import sys

# Regex para capturar VDD_IN 5210mW/3963mW
VDD_PATTERN = re.compile(r"VDD_IN\s+(\d+)mW/(\d+)mW")

def main():
    # Inicia o tegrastats com intervalo de 1s
    proc = subprocess.Popen(
        ["tegrastats", "--interval", "1000"],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        bufsize=1
    )

    print("Jetson Power Monitor (Ctrl+C para sair)\n")

    try:
        for line in proc.stdout:
            match = VDD_PATTERN.search(line)
            if match:
                inst_mw = int(match.group(1))
                avg_mw  = int(match.group(2))

                inst_w = inst_mw / 1000.0
                avg_w  = avg_mw / 1000.0

                # Atualiza sempre na mesma linha
                sys.stdout.write(
                    f"\rInstantâneo: {inst_w:5.2f} W | Médio: {avg_w:5.2f} W"
                )
                sys.stdout.flush()

    except KeyboardInterrupt:
        print("\nEncerrando...")
        proc.terminate()

if __name__ == "__main__":
    main()
