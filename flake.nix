{
  description = "Flake‑based NixOS config for beelink";

  inputs = {
    nixpkgs.url         = "github:NixOS/nixpkgs/nixos-25.05";
    flake-utils.url     = "github:numtide/flake-utils";
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay";
  };

  outputs = { self, nixpkgs, flake-utils, nix-ros-overlay, ... }: let
    system = "x86_64-linux";
  in {
    nixosConfigurations.beelink = nixpkgs.lib.nixosSystem {
      inherit system;

      modules = [
        ./hosts/beelink.nix
        { nixpkgs.overlays = [ nix-ros-overlay.overlays.default ]; }
      ];
    };
  };
}
