{
  description = "Project viikate";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";

    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
      rust-overlay,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        overlays = [ (import rust-overlay) ];
        pkgs = import nixpkgs {
          inherit system overlays;
        };

        bInputs = [
          pkgs.pkg-config
          pkgs.meson
          pkgs.ninja
          pkgs.gcc-arm-embedded
          pkgs.teensy-loader-cli

          pkgs.just
          pkgs.clang-tools
          pkgs.clang
          pkgs.mold
          pkgs.sphinx

          pkgs.python3
          pkgs.python3Packages.pyserial
          pkgs.python3Packages.matplotlib
          pkgs.python3Packages.numpy

          pkgs.platformio
          pkgs.arduino-ide

          pkgs.udev
          pkgs.alsa-lib
          pkgs.vulkan-loader
          pkgs.xorg.libX11
          pkgs.xorg.libXcursor
          pkgs.xorg.libXi
          pkgs.xorg.libXrandr
          pkgs.libxkbcommon
          pkgs.wayland

          pkgs.cargo
          pkgs.rust-analyzer
          pkgs.rustc
          pkgs.clippy
          pkgs.rustfmt
        ];
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = bInputs;

          RUSTFLAGS = "-Clink-arg=-z -Clink-arg=nostart-stop-gc " + "-Clink-arg=-fuse-ld=mold";

          LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath bInputs;
        };
      }
    );
}
