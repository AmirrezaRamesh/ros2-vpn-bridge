# ROS2 VPN Bridge
I was setting this up for a small robotics project, and I finally got bored enough to document it.

- [Overview](#overview)
- [General Idea](#general-idea)
- [VPN Server Config](#vpn-server-side)
- [ROS System Config](#ros-client-side)
- [Run The Nodes](#run-nodes)
- [What's next](#final-words)

## Overview 
Imagine you're working with `two ROS 2 systems` sitting behind `two separate NATs`.
As you might know, ROS 2 uses DDS for exchanging data between nodes. It relies on UDP multicast discovery to establish connections.

The problem is that this setup works only in a `LAN environment`. There are multiple ways to work around this, but before we get to them, let’s look at a simple example:
You have a robot in a robotics lab, and you want to monitor its data or actuate it remotely from anywhere on the planet (maybe from a control panel on a central host). This is when you need a `VPN` to merge the two systems onto the same virtual LAN.

There are faster and more efficient ways to solve this issue if your service provider offers static IP internet plans (which not many do, and when they do, it’s not that cheap). One alternative is an extension of this repo: [ros2-socketcomm](https://github.com/AmirrezaRamesh/ros2-socketcomm)
, combined with simple port forwarding set on the router. But in most cases, a VPN server is the practical solution.



## General Idea
to get started, you need:
- two Machines with ROS installed on them(as mentioned, on seprate NATs)

- a VPS as the VPN server

- a good reason to waste your time

steps:

1. simply set up the VPN server on the VPS(Ubuntu 22.04 server)

2. add the ROS systems into your virtual network

3. configure and modify ROS middleware(Cyclone DDS) to work over VPN

4. write two simple nodes on the machines and test if they can communicate

5. expand the project based on your needs

![pic1](1.png)

## walkthrough

### VPN server side: 

1. install packages on VPS using:
```bash 
sudo apt update
sudo apt install -y openvpn easy-rsa ufw 
```
2. move the script `setup_pki.sh` located in VPN directory onto the VPS and run the script to PKI with easy-rsa

3. move the file `server.conf` located in VPN directory onto the VPS path: `/etc/openvpn/server/server.conf`

4. run the command to define rules/IPs for clients:
```bash
sudo mkdir -p /etc/openvpn/ccd
echo "ifconfig-push 10.8.0.10 255.255.255.0" | sudo tee /etc/openvpn/ccd/clientA
echo "ifconfig-push 10.8.0.20 255.255.255.0" | sudo tee /etc/openvpn/ccd/clientB
```
5. enable tunnel and modify firewall:
```bash
echo 'net.ipv4.ip_forward=1' | sudo tee /etc/sysctl.d/99-openvpn.conf
sudo sysctl --system

sudo ufw allow 1194/udp
sudo ufw allow OpenSSH
sudo ufw enable 
```
6. start vpn server:
```bash
sudo systemctl enable --now openvpn-server@server.service
sudo systemctl status openvpn-server@server.service
```
7. create client profiles:
```bash
mkdir -p ~/ovpn_clients/clientA ~/ovpn_clients/clientB
```

8. move the files with the .opvn files from VPN directory to paths:
`~/ovpn_clients/clientA/clientA.ovpn` and `~/ovpn_clients/clientB/clientB.ovpn`
we didn't need to have the .opvn files on the server side but its a good idea to have them there and change them if needed and overwrite the client files.

### ROS client side:
1. use `scp` or any other methods to copy the each .opvn file on its target Machine(I copied on ~ directory).

2. install packages and connect
Machine A:
```bash
sudo apt update
sudo apt install -y openvpn

sudo openvpn --config ~/clientA.ovpn

sudo mkdir -p /etc/openvpn/client
sudo cp ~/clientA.ovpn /etc/openvpn/client/clientA.conf
sudo systemctl enable --now openvpn-client@clientA.service
```
Machine B:
```bash
sudo apt update
sudo apt install -y openvpn

sudo openvpn --config ~/clientA.ovpn

sudo mkdir -p /etc/openvpn/client
sudo cp ~/clientA.ovpn /etc/openvpn/client/clientA.conf
sudo systemctl enable --now openvpn-client@clientA.service
```

3. test the connection on Machine A(and Machine B if you want):
```bash
ping -c 3 10.8.0.1 #server

ping -c 10.8.0.20 #Machine B
```
if you could ping them, the VPN set up is done

for some reason, mutilcasting doesn't work over VPN, so we use unicasting meaning a peer to peer client connection(kinda embrassing?):

4. put the `DDS_client(A/B).xml` files in this `path` and `name` for both machines: `~/.ros/cyclonedds.xml`

5. put this lines inside your `.bashrc` of both machines:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.ros/cyclonedds.xml
export ROS_DOMAIN_ID=69 
```

> you can automate the process of starting server/clients on boot(on background) which is a fiarly easy task for chatgpt

### Run Nodes

0. make sure all the services mentioned earlier are running

1. use the two python files in `ROS` directory inside packages in built on each Machine and modify setup.py & package.xml

2. Machine A: `ros2 run package_name clientA`

3. Machine B: `ros2 run package_name clientB`

4. expetected output on Machine A: `Sending my regards to Machine B` every one second

5. expected outout on Machine B:`Receiving Machine A's regards`

> now you can(but you won't XD) use your nodes in this architecture

### Final Words
As you will probably notice, the connection is not very fast and there are noticeable delays, which I guess will increase as the data size grows. Therefore, this wouldn’t be the best option for lossy networks or robots operating in real time. However, it does the job for specific development purposes.

> P.S. This project was only tested once in the simplest way, and I wasn’t motivated enough to expand it. In a real project, it’s definitely buggy. So, if anyone ever wants to try this setup out (what are the chances, XD), please contact me first at `rameshamirreza3@gmail.com`. I already have some improvements in mind.