# Project-Viikate

**Schedule**
<img width="3102" height="1524" alt="image" src="https://github.com/user-attachments/assets/f3dc2c8d-f546-4d77-93df-0deba5dccf37" />

**Critical Design Decisions**

At 28.1.2026, during the whole team meeting, was decided that the fin can design follows the leftmost design of figure 1. It was also thought of using airsoft BB's as the bearing balls and having two raceways. 

<img width="2160" height="1224" alt="image" src="https://github.com/user-attachments/assets/1ba3daca-7f52-4886-933b-9a80715c3dc0" />
Figure 1.

The GPS module was emitted at the whole team meeting at 28.1.2026. Standard sensor drift can be accounted for. We will keep it in mind for other projects.

Other decisions will be marked under each team's folder in a md file. 

# Build instructions
[Install nix](https://nixos.org/download/), then go to the root of the repo and run:
```sh
$ nix develop
$ cd firmware
$ just setup
$ just build
$ just flash
```
Afterwards you should have a fully functioning flashed teensy!
