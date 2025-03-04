# 2025-Cobra

![5409](./img/garthwebbrobotics_small.jpg)

## Garth Webb Chargers - 5409

[Website](https://chargersrobotics.hdsb.ca/)

[Youtube](https://www.youtube.com/@gwssrobotics5409)

[Linkedin](https://ca.linkedin.com/company/garth-webb-robotics)

[Instagram](https://www.instagram.com/gwssrobotics/)

[The Blue Alliance](https://www.thebluealliance.com/team/5409)

## Documentation

[Button Bindings](https://docs.google.com/spreadsheets/d/1HSiYmBb3gUdGuNg3Dq-xGTUsblrMMzPB7j3w7IODIzY/edit?usp=sharing)

[CAN IDs](https://docs.google.com/spreadsheets/d/19ab2mOXPEb4q1jfBIjRCk-Qz7Qm4ZjitfzXwb1_sQiM/edit?usp=sharing)


## Button Bindings

| Button | Controller | Function | Subsystem(s)  | Action |
|--------|------------|----------|--------------------------|--------|
| LT | Primary | Rotate left | Drive | Hold |
| RT | Primary| Rotate right | Drive | Hold |
| LB | Primary | | | |
| RB | Primary | Drive-Coast | Drive | Hold |
| LS-X | Primary | Translational X | Drive | Hold |
| LS-Y | Primary | Translational Y | Drive | Hold |
| RS-X | Primary | | | |
| RS-Y | Primary | | | |
| RS-B | Primary | | | |
| X | Primary | Intake & align to station | Intake | Hold |
| Y | Primary | Intake station | End Effector | Hold |
| A | Primary | Align and score to reef | End Effector, Elevator | Hold |
| B | Primary | Remove Algae | End Effector, Elevator | Hold |
| D-PAD UP | Primary | Wait to score (auto drive) | End Effector | Hold |
| D-PAD DOWN | Primary | Wait to score (auto drive) | End Effector | Hold |
| D-PAD LEFT  | Primary | Wait to score (auto drive) | End Effector | Hold |
| D-PAD RIGHT | Primary | Wait to score (auto drive) | End Effector | Hold |
| START | Primary | Reset Field Orientation | Drive | Press |
| BACK | Primary | Start/Stop auto drive | Drive | Toggle Press |
| LT | Secondary | | | |
| RT | Secondary | | | |
| LB | Secondary | | | |
| RB | Secondary | | | |
| LS-X | Secondary | Angle for scoring | | Press |
| LS-Y | Secondary | Angle for scoring | | Press |
| RS-X | Secondary | | | |
| RS-Y | Secondary | | | |
| RS-B | Secondary | | | |
| X | Secondary | Prep Scoring L3 | Elevator, Arm | Press |
| Y | Secondary | Prep Scoring L4 | Elevator, Arm | Press |
| A | Secondary | Prep Scoring L1 | Elevator, Arm | Press |
| B | Secondary | Prep Scoring L2 | Elevator, Arm | Press |
| D-PAD UP | Secondary | Climber UP | Climber | Press |
| D-PAD DOWN | Secondary | Climber DOWN | Climber | Press |
| D-PAD LEFT | Secondary | | | |
| D-PAD RIGHT | Secondary | | | |
| START | Secondary | | | |
| BACK | Secondary | | | |

## Button Bindings Legend

| Key         | Value               |
|------------|--------------------|
| LT         | Left trigger       |
| RT         | Right trigger      |
| LB         | Left bumper        |
| RB         | Right bumper       |
| LS-X       | Left stick x-axis  |
| LS-Y       | Left stick y-axis  |
| LS-B       | Left stick button  |
| RS-X       | Right stick x-axis |
| RS-Y       | Right stick y-axis |
| RS-B       | Right stick button |
| X          | X                  |
| Y          | Y                  |
| A          | A                  |
| B          | B                  |
| D-PAD UP   | D-Pad up           |
| D-PAD DOWN | D-Pad down         |
| D-PAD LEFT | D-Pad left         |
| D-PAD RIGHT| D-Pad right        |
| START      | Start              |
| BACK       | Back               |
|            | Unbound button     |

## CAN IDs

| Device                       | CANID |
|------------------------------|-------|
| FL Drive                     | 6     |
| FL Steer                     | 7     |
| FL CANCoder                  | 12    |
| FR Drive                     | 4     |
| FR Steer                     | 5     |
| FR CANCoder                  | 11    |
| BL Drive                     | 8     |
| BL Steer                     | 9     |
| BL CANCoder                  | 13    |
| BR Drive                     | 2     |
| BR Steer                     | 3     |
| BR CANCoder                  | 10    |
| Pigeon 2.0                   | 14    |
| LEFT Elevator Kraken         | 20    |
| RIGHT Elevator Kraken        | 21    |
| Arm Pivot Kraken             | 22    |
| End Effector Rollers Falcon  | 23    |
| Arm Pivot CANCoder           | 24    |
| ToF End Effector             | 28    |
| ToF Funnel                   | 29    |


