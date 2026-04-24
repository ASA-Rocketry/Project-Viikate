{
  description = "Project viikate";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
      in
      {
        devShells.default = pkgs.mkShell {
          packages = [
            pkgs.pkg-config
            pkgs.meson
            pkgs.ninja
            pkgs.gcc-arm-embedded
            pkgs.teensy-loader-cli

            pkgs.just
            pkgs.clang-tools
            pkgs.sphinx

            pkgs.python3
            pkgs.python3Packages.pyserial
            pkgs.python3Packages.matplotlib
            pkgs.python3Packages.numpy

            pkgs.platformio
            pkgs.arduino-ide
          ];
        };
      }
    );
}
