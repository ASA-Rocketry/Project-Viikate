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
      ...
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        overlays = [ ];
        pkgs = import nixpkgs {
          inherit system overlays;
        };
        buildInputs = [
          pkgs.meson
          pkgs.ninja

          pkgs.clang-tools
          pkgs.clang
          pkgs.valgrind
          pkgs.gdb
        ];
      in
      {
        devShells.default = pkgs.mkShell {
          inherit buildInputs;
        };
      }
    );
}
