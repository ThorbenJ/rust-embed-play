
export PKG_CONFIG_PATH=$(echo /nix/store/*openssl-$(openssl --version | cut -f2 -d' ')-dev/lib/pkgconfig/)

