# Project Viikate

1. Clone this repository using `git clone https://github.com/tuomaserola/Project-Viikate.git`.
2. Open the repository in your preferred editor or just `cd` into it.
3. Run `pip install pre-commit` to install the pre-commit hook manager.
4. Run `pre-commit install --hook-type commit-msg` to activate all the local checks.

## Structure

```
project-viikate/
  cad/          -- the fusion files for the structure of the rocket
  code/         -- firmware, simulations and other adjacent software
    firmware/   -- the actual firmware that goes on the rocket
    dashboard/
  docs/         -- textual documentation related to the operations of the project
  minutes/      -- meeting notes
  pcb/          -- electronics, pinouts and the core PCB
```

## Software

## Style Choices

- For details on how to style your code refer to [RocketStyle](./code/README.md).
- For details on how to style your mechanical designs refer to [Mechanical Design Guide for Aalto Space Association Projects with Fusion](./cad/README.md).
- Folder names should follow `kebab-case` unless mandated by the style guide of the specific piece of software.
- Files should follow `snake_case` unless mandated by the style guide of a specific piece of software.

## Testing Policy

The priority of tests is strictly in this order:

0. Structural failures.
1. Serialization/deserialization.
2. Unit conversions.
3. Sensor fusion edgecases.
4. State machine transitions.
5. Numerical stability.
6. Watchdog/fault recovery.

Notice how getters and setters are not included. They are not important.

## How to make Git less painful

- Use rebases instead of merges. Merges pollute history, rebases straighten it back out.
- Don't commit stuff that is not supposed to be `diff`-ed. If you can't meaningfully interpret a file changing across time it should not be in git.
- Don't commit caches. Temporary files, output binaries, stuff like that.
- A commit message should follow the strict `type(scope): message` format. This is needed to make the scope of the changes easier to understand. For example: `feat(gnc): add the barrel roll logic`. The accepted types are as follows:
  - `feat` - a new feature, functionality that was previously missing.
  - `fix` - a fix to a bug that was causing trouble.
  - `style` - making some part of the code adhere to a style.
  - `docs` - adding documentation, amending typos.
  - `refactor` - making an old piece of code less ugly.
  - `chore` - reordering files, deleting stuff that shouldn't be tracked, etc.
- The commit text should read as what it does when applied. Not `added more tests`, but `add more tests`.
- Do not include OS-specific files in git. To exclude file locally you can add it to the `.git/info/exclude` file.
