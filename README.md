# Project-Viikate



# Phase 2 Startup (26.5.2026 Team Meeting Notes )

## Design Review
From this point onwards design review for all project subjects will be made. Another team member will review before code is rebased. For structure the current method of Rickhard looking design over with Jaakko has worked well so that is continued.

## Documentation
Documentation will created for all project subjects. Code documentation will be made to git commits. Strucuture and electronics documentations will go to an .md file in their respective folders. 

The documentation should follow the following template:
### 1. The Problem
*What are we trying to solve, and what limits (time, budget, rules) do we have?*
> 

### 2. The Chosen Solution
*What are we doing, and why is this the best choice for us right now?*
> 

### 3. Alternatives Considered
*What else did we think about doing, and why did we say no to it?*
* **Alternative A:** [Name]
  * *Why we didn't do it:* [Reason]
* **Alternative B:** [Name]
  * *Why we didn't do it:* [Reason]

### 4. Next Steps
*What needs to happen next to make this real? (CAD update, order parts, test it?)*
* [ ] Action item 1
* [ ] Action item 2

Own brain use is adviced. If there is no possible alternatives or next steps are trivial, they do not have to be marked down. Most important thing to know is what you have done and why. Commits without proper documentation are rejected in the design review process.

## Workflow adjustments
Finalized mechanical design guide will be posted under strucuture soon. A guide should be made and followed also for software.

## Next Steps
Simulated testing, creating data with openrocket or matlab, gather data from irl, drone or water rockets

Rewrite the code, old things can be used as a reference but it should be rewritten.

Finish the badge

Structure (Fusion and Openrocket) shall be made current, and documentation created

Work on a improved flight computer, while consulting structure about pins in the corner.

Try simulating tumbiling rocket for parachute sizing.

Figure out better deployment methods.

**Large picture**: first implement and fly roll control and close the second phase of the project

**Other**

Need for marketing person noted. 

New servos should be ordered.

Canards should be built to widthstand landing.

Viikate should be added as a part of ASA website.


**OLD Schedule**
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
