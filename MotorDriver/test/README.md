# PR2040_MotorDriver — Python Test Scripts

## Requirements

```bash
pip install pyserial
```

## monitor.py

Continuously reads and displays `RESP_STATUS` packets (100 Hz auto-send from firmware).

```bash
python monitor.py            # auto-detect RP2040 (VID=0x2E8A)
python monitor.py COM30      # specify port
```

Output columns: `ENC [counts]` / `VEL [cps]` / `VEL [mm/s]` / `ADC raw` / `CURR [A]`

## commander.py

Interactive command sender. Supports all USB Serial commands.

```bash
python commander.py          # auto-detect RP2040
python commander.py COM30    # specify port
```

### Quick reference

| Command | Example | Description |
|---------|---------|-------------|
| `stop` | `stop` | Coast all motors |
| `brake` | `brake` | Active brake all motors |
| `reset` | `reset` | Reset all encoders |
| `status` | `status` | Request one status packet |
| `ext` | `ext` | Request MCP3208 external ADC |
| `m <i> <duty>` | `m 1 500` | Motor i duty −1000..+1000 |
| `mall <d1..d4>` | `mall 200 200 200 200` | Set all motor duties |
| `mode <i> <m>` | `mode 1 1` | 0=DIRECT 1=VEL 2=POS |
| `vel <i> <cps>` | `vel 1 400` | Velocity target (counts/sec) |
| `velmm <i> <mmps>` | `velmm 1 100` | Velocity target (mm/sec) |
| `pos <i> <cnt>` | `pos 1 1654` | Position target (counts, ≈2 rev) |
| `posmm <i> <mm>` | `posmm 1 200` | Position target (mm) |
| `pid <i> kp ki kd` | `pid 1 0.8 0.1 0.0` | Velocity PID gains |
| `posgain <i> kp maxcps` | `posgain 1 2.0 800` | Position gains |
| `mon` | `mon` | Continuous monitor (Ctrl+C to stop) |
| `q` | `q` | Quit (motors stopped) |

### Typical workflow

```
# 1. Drive motor 1 in velocity mode at 100 mm/s
cmd> mode 1 1
cmd> velmm 1 100
cmd> status

# 2. Move motor 1 to 200 mm position
cmd> mode 1 2
cmd> posmm 1 200
cmd> status

# 3. Direct PWM test
cmd> mode 1 0
cmd> m 1 400
cmd> stop
```
