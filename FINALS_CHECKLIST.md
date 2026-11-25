# Finals checklist: minidrone_mine (team ENTITY)

Guidance: `perception/trackGuidance.m` (called by the `lineGuidance` block in `PerceptionAndGuidance.slx`).
Flight controller: MathWorks PX4 patch (Nov 19), same as Vishnu's `main` (61659c4 + 4e94ee6).

## Before leaving (on the laptop that will deploy)
1. Copy the whole `minidrone_mine` folder to the deploy laptop (Vishnu's has every toolbox,
   the PX4 support package, the Pi camera patch and built PX4 firmware).
2. Open `PX4MultiCopterPerceptionAndControl.prj`.
3. In `perception/trackGuidance.m` set **`HARDWARE = true;`**. This turns off the sim-only
   brick-patch red filter. (Keep `false` for simulation.)
4. Check the model still updates: open `PerceptionAndGuidance.slx` and press Ctrl+D.

## Day 1: measure on the real drone before the practice flight
| what | where | how |
|---|---|---|
| Red colour threshold | `PerceptionAndGuidance` red-score threshold (sim: 50) | live mask view over the track: line solid, floor clean |
| Camera scale | `K08 = 0.0042` (m/px at 0.8 m) | hover at 1.0 m over the 0.1 m line: it should be about 19 px wide. If it is N px: `K08 = 0.0042 * 19 / N` |
| Optical centre | `RC = 60.5; CC = 80.5;` | only if the camera is mounted off-centre |
| Altitude | `ALT = 1.0;` | the 1 m view sees 0.31 m ahead; speed adapts automatically |
| Image lag | `LAG_POS / LAG_YAW = 2` (0.2 s at 10 Hz) | if the drone brakes late at corners, try 3 |
| Frame rate | `FRAME_DT = 0.1` (10 Hz) | **must match the camera block sample time**. The Raspberry Pi capture block could not be changed on this PC (Pi blockset missing): on the deploy laptop run `tools/set_rate(0.1)` or set it by hand. If the Pi cannot keep up at 10 Hz, set `FRAME_DT = 0.2`, `LAG_POS = LAG_YAW = 1`, `IMG_AGE = 0.2`, and `set_rate(0.2)` |

## Between attempts (each of the 7)
- The guidance keeps state in persistent variables: **restart the perception model / power-cycle**
  before every attempt, or the next flight starts where the last one ended (e.g. in LANDED).

## What the drone does (so you know what is normal)
TAKEOFF (soft climb to 1 m) > FOLLOW (speed set by how much line it sees, top ~0.33 m/s at 1 m)
> CORNER (brakes onto the vertex, turns on the spot to the new leg, goes) > ... > COAST past the line end
> APPROACH the circle > DESCEND (soft, 0.2 m/s) > LANDED.
All transitions go through the output shaper: speed, turn rate and climb/sink ramp smoothly.

## Scoring reminder
Take-off 1, each section 1, soft landing 1, accurate landing (20 % of body on the circle) 1.
Best of 7 attempts counts; ties: consistency, then time. Bank a clean run first.

## If something goes wrong on Day 1
- Drone drifts off the line on straights: colour threshold too low/high (mask has gaps or noise).
- Brakes too late / overshoots corners: raise `LAG_T` (0.3 > 0.4) or `END_MARGIN` (0.10 > 0.13).
- Turns too slowly: raise `YAW_RATE` / `YAW_ACC`.
- Lands short/long of the circle: check `K08` (scale) first.
- Previous guidance versions: `perception/trackGuidance_pretune_backup.m.txt`,
  `trackGuidance_preshaper_backup.m.txt`, `trackGuidance_5hz_backup.m.txt` (rename to `.m` to use).
- Vishnu's proven guidance (`lineGuidance.m`, 1.5 m, 5 Hz) is the fallback.
