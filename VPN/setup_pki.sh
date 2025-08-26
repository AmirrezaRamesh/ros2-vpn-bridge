#!/usr/bin/env bash
set -e

EASYRSA_DIR="/etc/openvpn/easy-rsa"
SERVER_NAME="server"
CLIENTS=("clientA" "clientB")
CURVE="secp384r1"

if ! command -v openvpn >/dev/null; then
    echo "[*] Installing OpenVPN and easy-rsa..."
    apt-get update
    apt-get install -y openvpn easy-rsa
fi

echo "[*] Preparing easy-rsa directory: $EASYRSA_DIR"
mkdir -p "$EASYRSA_DIR"
cp -r /usr/share/easy-rsa/* "$EASYRSA_DIR"
cd "$EASYRSA_DIR"

echo "[*] Configuring easy-rsa for ECC ($CURVE)"
echo "set_var EASYRSA_ALGO ec"   >  vars
echo "set_var EASYRSA_CURVE $CURVE" >> vars

./easyrsa init-pki

echo "[*] Generating CA..."
./easyrsa build-ca nopass

echo "[*] Generating server certificate..."
./easyrsa gen-req "$SERVER_NAME" nopass
./easyrsa sign-req server "$SERVER_NAME"

for client in "${CLIENTS[@]}"; do
    echo "[*] Generating certificate for $client..."
    ./easyrsa gen-req "$client" nopass
    ./easyrsa sign-req client "$client"
done

echo "[*] Generating TLS key..."
openvpn --genkey secret /etc/openvpn/ta.key

echo ""
echo "=== PKI setup complete! ==="
echo "CA:        $EASYRSA_DIR/pki/ca.crt"
echo "Server CRT: $EASYRSA_DIR/pki/issued/${SERVER_NAME}.crt"
echo "Server KEY: $EASYRSA_DIR/pki/private/${SERVER_NAME}.key"
for client in "${CLIENTS[@]}"; do
    echo "Client $client CRT: $EASYRSA_DIR/pki/issued/${client}.crt"
    echo "Client $client KEY: $EASYRSA_DIR/pki/private/${client}.key"
done
echo "TLS Key:   /etc/openvpn/ta.key"
echo ""
echo "✅ Next: configure OpenVPN server.conf and generate .ovpn profiles."
